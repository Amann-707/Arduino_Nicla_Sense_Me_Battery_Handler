#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/pm/pm.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <string.h>

/* ── Register Definitions ── */
#define BQ25120_ADDR                0x6A
#define STATUS_REG                  0x00
#define FAULTS_REG                  0x01
#define FAST_CHG_REG                0x03
#define VBMON_REG                   0x0A

/* ── CD Pin ── */
#define CD_PIN                      25
#define CD_PORT                     DT_NODELABEL(gpio0)

/* ── Battery Thresholds ── */
#define CHARGE_FULL_PERCENT         98
#define BATTERY_MIN_MV              3000
#define BATTERY_MAX_MV              4200
#define SOFT_SHUTDOWN_MV            3100
#define LOW_BATTERY_MV              3300
#define RECOVERY_MV                 3300

/* ── Polling Intervals ── */
#define FAULT_POLL_INTERVAL_MS      60000
#define VBMON_POLL_INTERVAL_MS      60000
#define VBMON_LOW_POLL_INTERVAL_MS  30000

/* ── BATUVLO Recovery ── */
#define BATUVLO_MAX_POLLS           10
#define BATUVLO_POLL_INTERVAL_MS    60000
#define BATUVLO_SAFE_RAW            0x2C
#define BATUVLO_MIN_READABLE        0x04

/* ── Shutdown Loop Config ── */
#define MAX_SHUTDOWN_CHECKS         10
#define SHUTDOWN_CHECK_INTERVAL_MS  30000

/* ── Thread Stack Sizes ── */
#define POLLING_THREAD_STACK_SIZE   2048
#define FAULT_THREAD_STACK_SIZE     1024
#define BLE_THREAD_STACK_SIZE       2048
#define POLLING_THREAD_PRIORITY     5
#define FAULT_THREAD_PRIORITY       5
#define BLE_THREAD_PRIORITY         5

/* ── Global State ── */
static bool is_charging   = false;
static const struct device *gpio_dev;
static const struct device *i2c_dev;

/* ── BLE State ── */
static struct bt_conn *current_conn = NULL;
static bool ble_connected           = false;

/* ── Mutexes ── */
static K_MUTEX_DEFINE(battery_mutex);   /* Protects latest_lower/upper_percent */
static K_MUTEX_DEFINE(i2c_mutex);       /* Protects all I2C transactions        */
static K_MUTEX_DEFINE(state_mutex);     /* Protects is_charging                 */

/* ── Battery range shared between threads ── */
static uint32_t latest_lower_percent = 0;
static uint32_t latest_upper_percent = 0;


/* ════════════════════════════════════════════
   BLE: Custom Service and Characteristic UUIDs
   ════════════════════════════════════════════ */
static struct bt_uuid_128 service_uuid =
    BT_UUID_INIT_128(
        0xf0,0xde,0xbc,0x9a, 0x78,0x56, 0x34,0x12,
        0x34,0x12, 0x78,0x56,0x34,0x12,0x78,0x90);

static struct bt_uuid_128 char_uuid =
    BT_UUID_INIT_128(
        0xaa,0xaa,0xaa,0xaa, 0xbb,0xbb, 0xcc,0xcc,
        0xdd,0xdd, 0xee,0xee,0xee,0xee,0xee,0xee);

/* ── BLE Characteristic value buffer ── */
static char battery_char_value[16];   /* e.g. "72 - 74\0" */

/* ── CCCD: tracks whether phone has enabled notifications ── */
static bool notify_enabled = false;

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("BLE: Notifications %s\n", notify_enabled ? "enabled" : "disabled");
}

/* ── GATT Read callback ── */
static ssize_t battery_read(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset) {
    k_mutex_lock(&battery_mutex, K_FOREVER);
    uint32_t lo = latest_lower_percent;
    uint32_t hi = latest_upper_percent;
    k_mutex_unlock(&battery_mutex);

    snprintk(battery_char_value, sizeof(battery_char_value),
             "%u - %u", lo, hi);
    printk("BLE: Read request - battery %s%%\n", battery_char_value);
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             battery_char_value,
                             strlen(battery_char_value));
}

/* ── GATT Service Definition ── */
BT_GATT_SERVICE_DEFINE(battery_svc,
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC(&char_uuid.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            battery_read, NULL,
                            battery_char_value),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ── Advertising Data ── */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'N','i','c','l','a','B','A','T'),
};


/* ════════════════════════════════════════════
   BLE: Send battery range notification
   Called from polling thread whenever VBMON updates
   Does NOT hold i2c_mutex - BLE is not I2C
   ════════════════════════════════════════════ */
static void ble_send_battery_percent(uint32_t lower, uint32_t upper) {

    if (!ble_connected || !notify_enabled) {
        return;
    }

    snprintk(battery_char_value, sizeof(battery_char_value),
             "%u - %u", lower, upper);

    int ret = bt_gatt_notify(current_conn,
                             &battery_svc.attrs[2],
                             battery_char_value,
                             strlen(battery_char_value));
    if (ret != 0) {
        printk("BLE: Notify failed: %d\n", ret);
    } else {
        printk("BLE: Sent battery %s%%\n", battery_char_value);
    }
}


/* ════════════════════════════════════════════
   BLE: Connection callbacks
   ════════════════════════════════════════════ */
static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("BLE: Connection failed (err %d)\n", err);
        return;
    }

    current_conn   = bt_conn_ref(conn);
    ble_connected  = true;
    notify_enabled = false;

    printk("BLE: Connected\n");

    /* Send current battery level immediately on connect */
    k_mutex_lock(&battery_mutex, K_FOREVER);
    uint32_t lo = latest_lower_percent;
    uint32_t hi = latest_upper_percent;
    k_mutex_unlock(&battery_mutex);

    if (lo > 0 || hi > 0) {
        ble_send_battery_percent(lo, hi);
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("BLE: Disconnected (reason %d)\n", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    ble_connected  = false;
    notify_enabled = false;
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};


/* ════════════════════════════════════════════
   HELPER: Set CD pin state
   Does NOT need i2c_mutex - GPIO not I2C
   ════════════════════════════════════════════ */
static void set_cd_pin(int state) {
    gpio_pin_set(gpio_dev, CD_PIN, state);
    k_msleep(100);
}


/* ════════════════════════════════════════════
   FUNCTION: Configure CD pin
   ════════════════════════════════════════════ */
static int configure_cd_pin(void) {

    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return -1;
    }

    int ret = gpio_pin_configure(gpio_dev, CD_PIN, GPIO_OUTPUT_HIGH);
    if (ret != 0) {
        printk("Failed to configure CD pin: %d\n", ret);
        return ret;
    }

    printk("CD pin configured HIGH - Active Battery mode\n");
    k_msleep(100);
    return 0;
}


/* ════════════════════════════════════════════
   FORWARD DECLARATIONS
   ════════════════════════════════════════════ */
static void soft_shutdown(void);
static uint32_t read_battery_level(bool charging_active);


/* ════════════════════════════════════════════
   FUNCTION: Read raw VBMON register
   MUST be called with i2c_mutex already held
   Returns raw byte or 0x00 on failure
   ════════════════════════════════════════════ */
static uint8_t read_vbmon_raw(void) {

    uint8_t trigger[2] = {VBMON_REG, 0x80};
    int ret = i2c_write(i2c_dev, trigger, 2, BQ25120_ADDR);
    if (ret != 0) {
        printk("VBMON trigger failed: %d\n", ret);
        return 0x00;
    }

    k_msleep(2);

    uint8_t reg_addr  = VBMON_REG;
    uint8_t raw_value = 0x00;
    ret = i2c_write_read(i2c_dev,
                         BQ25120_ADDR,
                         &reg_addr, 1,
                         &raw_value, 1);
    if (ret != 0) {
        printk("VBMON read failed: %d\n", ret);
        return 0x00;
    }

    return raw_value;
}


/* ════════════════════════════════════════════
   HELPER: Decode raw VBMON byte into
   lower_percent and upper_percent
   MUST be called with i2c_mutex already held
   ════════════════════════════════════════════ */
static void decode_vbmon(uint8_t raw_value,
                         uint32_t *lower_percent,
                         uint32_t *upper_percent) {

    uint8_t range     = (raw_value >> 5) & 0x03;
    uint8_t threshold = (raw_value >> 2) & 0x07;

    uint32_t base_percent;
    switch(range) {
        case 0x03: base_percent = 90; break;
        case 0x02: base_percent = 80; break;
        case 0x01: base_percent = 70; break;
        case 0x00: base_percent = 60; break;
        default:   base_percent =  0; break;
    }

    uint32_t above_percent;
    switch(threshold) {
        case 0x07: above_percent = 8; break;
        case 0x06: above_percent = 6; break;
        case 0x03: above_percent = 4; break;
        case 0x02: above_percent = 2; break;
        case 0x01: above_percent = 0; break;
        default:   above_percent = 0; break;
    }

    *lower_percent = base_percent + above_percent;
    *upper_percent = *lower_percent + 2;
}


/* ════════════════════════════════════════════
   FUNCTION: Handle BAT_UVLO fault
   Called from read_fault_register()
   i2c_mutex is held by caller - do NOT
   re-acquire it here, just use I2C directly
   ════════════════════════════════════════════ */
static void handle_batuvlo(void) {

    printk("!! Fault: BAT_UVLO - battery below 3V\n");
    printk("!! PLEASE CONNECT CHARGER IMMEDIATELY\n");

    /* TODO: Update display "Battery Critical - Connect Charger Now" */

    /* CD LOW to enable charging if VIN present
       i2c_mutex already held by caller         */
    set_cd_pin(0);
    k_msleep(100);

    int polls_remaining = BATUVLO_MAX_POLLS - 1;
    uint8_t prev_raw    = 0x00;

    while (polls_remaining > 0) {

        printk("!! BAT_UVLO recovery: polling VBMON "
               "(%d polls remaining)\n", polls_remaining);

        /* i2c_mutex still held - call raw helper directly */
        uint8_t raw_value = read_vbmon_raw();
        printk("!! VBMON raw right now = 0x%02X\n", raw_value);

        /* Battery completely unreadable - too low to recover */
        if (!(raw_value >= BATUVLO_MIN_READABLE)) {
            printk("!! Battery unreadable - entering soft shutdown\n");

            /* Release i2c_mutex before calling soft_shutdown
               which will re-acquire it internally             */
            k_mutex_unlock(&i2c_mutex);
            soft_shutdown();
            k_mutex_lock(&i2c_mutex, K_FOREVER);
            return;
        }

        /* Battery recovered above safe threshold */
        if (raw_value > BATUVLO_SAFE_RAW) {
            printk("!! Battery recovered - resuming normal operation\n");
            k_mutex_lock(&state_mutex, K_FOREVER);
            is_charging = true;
            k_mutex_unlock(&state_mutex);
            return;
        }

        /* Check if rising */
        if (raw_value > prev_raw && prev_raw != 0x00) {
            printk("!! Battery rising (0x%02X -> 0x%02X) "
                   "- resetting timer\n", prev_raw, raw_value);
            polls_remaining = BATUVLO_MAX_POLLS;
        } else if (raw_value == prev_raw && prev_raw != 0x00) {
            printk("!! Battery not rising - ensure charger connected "
                   "(%d polls remaining)\n", polls_remaining);
        }

        printk("!! Charging but not yet safe "
               "(0x%02X < 0x%02X) - waiting\n", raw_value, BATUVLO_SAFE_RAW);

        /* TODO: Update display "Charging - Please Wait" */

        /* Decode and send over BLE */
        uint32_t lower_percent, upper_percent;
        decode_vbmon(raw_value, &lower_percent, &upper_percent);

        k_mutex_lock(&battery_mutex, K_FOREVER);
        latest_lower_percent = lower_percent;
        latest_upper_percent = upper_percent;
        k_mutex_unlock(&battery_mutex);

        /* Release i2c_mutex while sleeping and during BLE notify
           so other threads can use I2C during the wait           */
        k_mutex_unlock(&i2c_mutex);
        ble_send_battery_percent(lower_percent, upper_percent);

        prev_raw = raw_value;
        polls_remaining--;

        if (polls_remaining > 0) {
            k_msleep(BATUVLO_POLL_INTERVAL_MS);
        }

        /* Re-acquire i2c_mutex for next iteration */
        k_mutex_lock(&i2c_mutex, K_FOREVER);
    }

    printk("!! No recovery after %d polls - shutdown\n", BATUVLO_MAX_POLLS);

    /* Release before soft_shutdown which manages its own locking */
    k_mutex_unlock(&i2c_mutex);
    soft_shutdown();
    k_mutex_lock(&i2c_mutex, K_FOREVER);
}


/* ════════════════════════════════════════════
   FUNCTION: Read fault register 0x01
   Acquires i2c_mutex internally
   VIN_UV set   = no charger (discharge)
   VIN_UV clear = charger connected → CD LOW
   I2C fail during charging = charger removed
   ════════════════════════════════════════════ */
static bool read_fault_register(void) {

    k_mutex_lock(&i2c_mutex, K_FOREVER);

    uint8_t reg_addr  = FAULTS_REG;
    uint8_t reg_value;

    int ret = i2c_write_read(i2c_dev,
                             BQ25120_ADDR,
                             &reg_addr, 1,
                             &reg_value, 1);
    if (ret != 0) {

        k_mutex_lock(&state_mutex, K_FOREVER);
        bool charging = is_charging;
        k_mutex_unlock(&state_mutex);

        if (charging) {
            printk("!! I2C failed during charging - charger removed\n");
            printk(">> Pulling CD HIGH - Active Battery mode\n");
            set_cd_pin(1);
            k_mutex_lock(&state_mutex, K_FOREVER);
            is_charging = false;
            k_mutex_unlock(&state_mutex);
        } else {
            printk("Fault register read failed: %d\n", ret);
        }

        k_mutex_unlock(&i2c_mutex);
        return false;
    }

    if (reg_value != 0x00) {
        printk("\n--- Fault Register (0x01) = 0x%02X ---\n", reg_value);
    }

    bool vin_ov   = (reg_value >> 7) & 0x01;
    bool vin_uv   = (reg_value >> 6) & 0x01;
    bool bat_uvlo = (reg_value >> 5) & 0x01;
    bool bat_ocp  = (reg_value >> 4) & 0x01;

    k_mutex_lock(&state_mutex, K_FOREVER);
    bool charging = is_charging;
    k_mutex_unlock(&state_mutex);

    /* VIN_UV while discharging = no charger, normal */
    if (vin_uv && !charging) {
        k_mutex_unlock(&i2c_mutex);
        return false;
    }

    /* VIN_UV cleared while discharging = charger connected */
    if (!vin_uv && !charging && reg_value == 0x00) {
        printk(">> VIN_UV cleared - charger connected\n");
        printk(">> Pulling CD LOW - enabling charging\n");
        set_cd_pin(0);
        k_mutex_lock(&state_mutex, K_FOREVER);
        is_charging = true;
        k_mutex_unlock(&state_mutex);
        k_mutex_unlock(&i2c_mutex);
        return false;
    }

    if (vin_ov) {
        printk("!! Fault: VIN overvoltage - check charger\n");
    }
    if (bat_ocp) {
        printk("!! Fault: Battery overcurrent - check connections\n");
    }
    if (bat_uvlo) {
        /* handle_batuvlo manages i2c_mutex internally
           it expects mutex to already be held on entry  */
        handle_batuvlo();
        k_mutex_unlock(&i2c_mutex);
        return true;
    }
    if (!vin_ov && !vin_uv && !bat_uvlo && !bat_ocp && reg_value != 0x00) {
        printk("!! Unknown fault - raw: 0x%02X\n", reg_value);
    }

    k_mutex_unlock(&i2c_mutex);
    return false;
}


/* ════════════════════════════════════════════
   FUNCTION: Read status register 0x00
   Acquires i2c_mutex internally
   Called once at startup
   ════════════════════════════════════════════ */
static void read_status_register(void) {

    k_mutex_lock(&i2c_mutex, K_FOREVER);

    uint8_t reg_addr  = STATUS_REG;
    uint8_t reg_value;

    int ret = i2c_write_read(i2c_dev,
                             BQ25120_ADDR,
                             &reg_addr, 1,
                             &reg_value, 1);
    if (ret != 0) {
        printk("Status register read failed: %d\n", ret);
        k_mutex_unlock(&i2c_mutex);
        return;
    }

    printk("\n--- Status Register (0x00) = 0x%02X ---\n", reg_value);

    uint8_t stat = (reg_value >> 6) & 0x03;
    switch(stat) {
        case 0x00:
            printk("Status: Ready\n");
            break;
        case 0x01:
            printk("Status: Charge in Progress\n");
            k_mutex_lock(&state_mutex, K_FOREVER);
            is_charging = true;
            k_mutex_unlock(&state_mutex);
            break;
        case 0x02:
            printk("Status: Charge Done\n");
            k_mutex_lock(&state_mutex, K_FOREVER);
            bool was_charging = is_charging;
            k_mutex_unlock(&state_mutex);
            if (was_charging) {
                printk(">> Hardware termination - disabling charge\n");
                set_cd_pin(1);
                k_mutex_lock(&state_mutex, K_FOREVER);
                is_charging = false;
                k_mutex_unlock(&state_mutex);
            }
            break;
        case 0x03:
            printk("Status: FAULT - see fault register\n");
            break;
    }

    printk("CD Status:  %s\n", (reg_value & 0x02) ? "HIGH - IC disabled"
                                                   : "LOW  - IC enabled");
    printk("SYS Output: %s\n", (reg_value & 0x01) ? "Enabled" : "Disabled");

    k_mutex_unlock(&i2c_mutex);
}


/* ════════════════════════════════════════════
   FUNCTION: Read battery level from VBMON
   Acquires i2c_mutex internally
   charging_active = true  → CE bit disable,
                             500ms wait, read,
                             CE bit re-enable
   charging_active = false → read directly
   Returns mid_voltage_mv or 0 on failure
   ════════════════════════════════════════════ */
static uint32_t read_battery_level(bool charging_active) {

    k_mutex_lock(&i2c_mutex, K_FOREVER);

    uint8_t reg_addr  = 0;
    uint8_t reg_value = 0;
    int     ret       = 0;

    if (charging_active) {

        reg_addr = FAST_CHG_REG;
        ret = i2c_write_read(i2c_dev,
                             BQ25120_ADDR,
                             &reg_addr, 1,
                             &reg_value, 1);
        if (ret != 0) {
            printk("Failed to read fast charge register: %d\n", ret);
            k_mutex_unlock(&i2c_mutex);
            return 0;
        }

        uint8_t disable_charge[2] = {FAST_CHG_REG, reg_value | 0x02};
        ret = i2c_write(i2c_dev, disable_charge, 2, BQ25120_ADDR);
        if (ret != 0) {
            printk("Failed to disable charger: %d\n", ret);
            k_mutex_unlock(&i2c_mutex);
            return 0;
        }

        /* Release mutex during 500ms wait so fault thread
           can still poll I2C while we wait                */
        k_mutex_unlock(&i2c_mutex);
        printk("Waiting for surface charge to dissipate...\n");
        k_msleep(500);
        k_mutex_lock(&i2c_mutex, K_FOREVER);
    }

    uint8_t raw_value = read_vbmon_raw();
    if (raw_value == 0x00) {
        if (charging_active) {
            reg_addr = FAST_CHG_REG;
            i2c_write_read(i2c_dev, BQ25120_ADDR,
                           &reg_addr, 1, &reg_value, 1);
            uint8_t enable_charge[2] = {FAST_CHG_REG, reg_value & ~0x02};
            i2c_write(i2c_dev, enable_charge, 2, BQ25120_ADDR);
        }
        k_mutex_unlock(&i2c_mutex);
        return 0;
    }

    if (charging_active) {
        reg_addr = FAST_CHG_REG;
        i2c_write_read(i2c_dev, BQ25120_ADDR,
                       &reg_addr, 1, &reg_value, 1);
        uint8_t enable_charge[2] = {FAST_CHG_REG, reg_value & ~0x02};
        i2c_write(i2c_dev, enable_charge, 2, BQ25120_ADDR);
        printk("Charger re-enabled\n");
    }

    k_mutex_unlock(&i2c_mutex);

    /* Decode outside of mutex - pure math, no I2C */
    uint32_t lower_percent, upper_percent;
    decode_vbmon(raw_value, &lower_percent, &upper_percent);

    uint32_t lower_voltage_mv = (lower_percent * 4200) / 100;
    uint32_t upper_voltage_mv = (upper_percent * 4200) / 100;
    uint32_t mid_voltage_mv   = (lower_voltage_mv + upper_voltage_mv) / 2;

    printk("\n--- Battery Level (0x0A) = 0x%02X ---\n", raw_value);
    printk("VBMON Range: %d  Threshold: %d\n",
           (raw_value >> 5) & 0x03, (raw_value >> 2) & 0x07);
    printk("VBMON: %d%% to %d%% of VBATREG\n", lower_percent, upper_percent);
    printk("Voltage: ~%d.%03dV to %d.%03dV\n",
           lower_voltage_mv / 1000, lower_voltage_mv % 1000,
           upper_voltage_mv / 1000, upper_voltage_mv % 1000);

    if (mid_voltage_mv >= 4000) {
        printk("Level: FULL     [|||||]\n");
    } else if (mid_voltage_mv >= 3800) {
        printk("Level: HIGH     [||||_]\n");
    } else if (mid_voltage_mv >= 3600) {
        printk("Level: MEDIUM   [|||__]\n");
    } else if (mid_voltage_mv >= 3300) {
        printk("Level: LOW      [||___] Charge Soon\n");
    } else if (mid_voltage_mv >= 3100) {
        printk("Level: CRITICAL [|____] Charge Now\n");
    } else {
        printk("Level: EMPTY\n");
    }

    /* Update shared battery range */
    k_mutex_lock(&battery_mutex, K_FOREVER);
    latest_lower_percent = lower_percent;
    latest_upper_percent = upper_percent;
    k_mutex_unlock(&battery_mutex);

    /* Send BLE notification - no mutex needed, not I2C */
    ble_send_battery_percent(lower_percent, upper_percent);

    /* Software charge full detection */
    k_mutex_lock(&state_mutex, K_FOREVER);
    bool charging = is_charging;
    k_mutex_unlock(&state_mutex);

    if (charging && lower_percent >= CHARGE_FULL_PERCENT) {
        printk(">> Battery full - disabling charge via CD pin\n");
        set_cd_pin(1);
        k_mutex_lock(&state_mutex, K_FOREVER);
        is_charging = false;
        k_mutex_unlock(&state_mutex);
    }

    return mid_voltage_mv;
}


/* ════════════════════════════════════════════
   FUNCTION: Soft shutdown with charger detection
   Uses VIN_UV fault bit for charger detection
   Acquires i2c_mutex for each register read
   Returns if battery recovers above RECOVERY_MV
   Falls back to infinite loop if no recovery
   ════════════════════════════════════════════ */
static void soft_shutdown(void) {

    printk("\n!! SOFT SHUTDOWN - Battery critically low !!\n");
    printk("!! Please connect charger immediately       !!\n");

    /* TODO: Update display "Battery Empty - Please Charge" */
    /* TODO: Send BLE notification to user                  */
    /* TODO: Gracefully stop sensor threads                 */
    /* TODO: Save any sensor data to flash                  */

    set_cd_pin(1);
    k_mutex_lock(&state_mutex, K_FOREVER);
    is_charging = false;
    k_mutex_unlock(&state_mutex);
    k_msleep(200);

    int checks_remaining = MAX_SHUTDOWN_CHECKS - 1;

    while (checks_remaining > 0) {

        printk(">> Shutdown: checking for charger "
               "(%d checks remaining)\n", checks_remaining);

        k_mutex_lock(&i2c_mutex, K_FOREVER);

        uint8_t reg_addr  = FAULTS_REG;
        uint8_t reg_value = 0xFF;

        int ret = i2c_write_read(i2c_dev,
                                 BQ25120_ADDR,
                                 &reg_addr, 1,
                                 &reg_value, 1);

        k_mutex_unlock(&i2c_mutex);

        if (ret != 0) {
            printk(">> Fault register read failed: %d\n", ret);
            checks_remaining--;
            k_msleep(SHUTDOWN_CHECK_INTERVAL_MS);
            continue;
        }

        bool vin_uv = (reg_value >> 6) & 0x01;

        if (!vin_uv) {
            printk(">> Charger detected - enabling charging\n");

            /* TODO: Update display "Charging - Please Wait" */

            set_cd_pin(0);
            k_mutex_lock(&state_mutex, K_FOREVER);
            is_charging = true;
            k_mutex_unlock(&state_mutex);

            uint32_t recovery_voltage = 0;

            while (recovery_voltage < RECOVERY_MV) {
                recovery_voltage = read_battery_level(true);

                if (recovery_voltage == 0) {
                    printk(">> Charger removed during recovery\n");
                    set_cd_pin(1);
                    k_mutex_lock(&state_mutex, K_FOREVER);
                    is_charging = false;
                    k_mutex_unlock(&state_mutex);
                    break;
                }

                printk(">> Recovery: %d.%03dV\n",
                       recovery_voltage / 1000,
                       recovery_voltage % 1000);

                if (recovery_voltage < RECOVERY_MV) {
                    k_msleep(30000);
                }
            }

            if (recovery_voltage >= RECOVERY_MV) {
                printk(">> Recovered - resuming normal operation\n");

                /* TODO: Update display "Battery Recovered" */

                return;
            }

        } else {
            printk(">> No charger detected\n");

            /* TODO: Update display "Please Connect Charger" */
        }

        checks_remaining--;

        if (checks_remaining > 0) {
            printk(">> Light sleep for %d seconds\n",
                   SHUTDOWN_CHECK_INTERVAL_MS / 1000);
            k_msleep(SHUTDOWN_CHECK_INTERVAL_MS);
        }
    }

    printk(">> No charger after %d checks - full system off\n",
           MAX_SHUTDOWN_CHECKS);

    k_msleep(500);

    /* CD LOW so charging starts automatically if charger connected */
    while(1) {
        set_cd_pin(0);
        k_msleep(1000);
    }
}


/* ════════════════════════════════════════════
   THREAD: Fault polling thread
   Polls fault register every FAULT_POLL_INTERVAL_MS
   Detects charger connect via VIN_UV clearing
   Detects charger removal via I2C failure
   Runs independently of VBMON and BLE threads
   ════════════════════════════════════════════ */
K_THREAD_STACK_DEFINE(fault_stack, FAULT_THREAD_STACK_SIZE);
static struct k_thread fault_thread_data;

static void fault_thread(void *p1, void *p2, void *p3) {

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while(1) {
        read_fault_register();
        k_msleep(FAULT_POLL_INTERVAL_MS);
    }
}


/* ════════════════════════════════════════════
   THREAD: VBMON polling thread
   Polls battery level every 60s (30s if low)
   Triggers soft shutdown if voltage critical
   Runs independently of fault and BLE threads
   ════════════════════════════════════════════ */
K_THREAD_STACK_DEFINE(polling_stack, POLLING_THREAD_STACK_SIZE);
static struct k_thread polling_thread_data;

static void polling_thread(void *p1, void *p2, void *p3) {

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    /* Read status once at startup only */
    read_status_register();

    uint32_t vbmon_interval_ms = VBMON_POLL_INTERVAL_MS;

    while(1) {

        k_msleep(vbmon_interval_ms);

        k_mutex_lock(&state_mutex, K_FOREVER);
        bool charging = is_charging;
        k_mutex_unlock(&state_mutex);

        uint32_t voltage_mv = read_battery_level(charging);

        if (voltage_mv > 0 && voltage_mv <= SOFT_SHUTDOWN_MV) {
            printk(">> Voltage critical - soft shutdown\n");
            soft_shutdown();
            printk(">> Resuming after recovery\n");
        }

        vbmon_interval_ms = (voltage_mv > 0 && voltage_mv <= LOW_BATTERY_MV)
                            ? VBMON_LOW_POLL_INTERVAL_MS
                            : VBMON_POLL_INTERVAL_MS;
    }
}


/* ════════════════════════════════════════════
   THREAD: BLE thread
   Handles advertising and reconnection
   Runs independently of polling and fault threads
   ════════════════════════════════════════════ */
K_THREAD_STACK_DEFINE(ble_stack, BLE_THREAD_STACK_SIZE);
static struct k_thread ble_thread_data;

static void ble_thread(void *p1, void *p2, void *p3) {

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                              ad, ARRAY_SIZE(ad),
                              NULL, 0);
    if (ret != 0) {
        printk("BLE: Advertising failed to start: %d\n", ret);
        return;
    }

    printk("BLE: Advertising started - device name NiclaBAT\n");

    while(1) {

        if (!ble_connected) {
            ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
                                  ad, ARRAY_SIZE(ad),
                                  NULL, 0);
            if (ret == 0) {
                printk("BLE: Re-advertising after disconnect\n");
            }
            /* -EALREADY means already advertising - fine */
        }

        k_msleep(5000);
    }
}


/* ════════════════════════════════════════════
   MAIN
   Initialises hardware then spawns threads
   ════════════════════════════════════════════ */
void main(void) {

    gpio_dev = DEVICE_DT_GET(CD_PORT);
    i2c_dev  = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    /* Step 1: Configure CD pin */
    if (configure_cd_pin() != 0) return;

    /* Step 2: Verify I2C */
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }
    printk("I2C ready\n");

    /* Step 3: Initialise BLE */
    int ret = bt_enable(NULL);
    if (ret != 0) {
        printk("BLE: Enable failed: %d\n", ret);
        return;
    }
    printk("BLE: Enabled\n");

    bt_conn_cb_register(&conn_callbacks);

    /* Step 4: Spawn fault polling thread */
    k_thread_create(&fault_thread_data,
                    fault_stack,
                    K_THREAD_STACK_SIZEOF(fault_stack),
                    fault_thread,
                    NULL, NULL, NULL,
                    FAULT_THREAD_PRIORITY,
                    0, K_NO_WAIT);
    k_thread_name_set(&fault_thread_data, "fault");
    printk("Fault polling thread started\n");

    /* Step 5: Spawn VBMON polling thread */
    k_thread_create(&polling_thread_data,
                    polling_stack,
                    K_THREAD_STACK_SIZEOF(polling_stack),
                    polling_thread,
                    NULL, NULL, NULL,
                    POLLING_THREAD_PRIORITY,
                    0, K_NO_WAIT);
    k_thread_name_set(&polling_thread_data, "polling");
    printk("VBMON polling thread started\n");

    /* Step 6: Spawn BLE thread */
    k_thread_create(&ble_thread_data,
                    ble_stack,
                    K_THREAD_STACK_SIZEOF(ble_stack),
                    ble_thread,
                    NULL, NULL, NULL,
                    BLE_THREAD_PRIORITY,
                    0, K_NO_WAIT);
    k_thread_name_set(&ble_thread_data, "ble");
    printk("BLE thread started\n");
}