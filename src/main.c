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
#include <stdio.h>

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
#define FAULT_POLL_INTERVAL_MS      10000   /* Fault register: every 10s  */
#define VBMON_POLL_INTERVAL_MS      60000   /* VBMON: every 60s           */
#define VBMON_LOW_POLL_INTERVAL_MS  30000   /* VBMON when low: every 30s  */

/* ── BATUVLO Recovery ── */
#define BATUVLO_MAX_POLLS           10
#define BATUVLO_POLL_INTERVAL    60000
#define BATUVLO_SAFE_RAW        0x28   /* Above this = safe to resume*/
#define BATUVLO_MIN_READABLE    0x04   /* Lowest readable VBMON value*/

/* ── Shutdown Loop Config ── */
#define MAX_SHUTDOWN_CHECKS         10
#define SHUTDOWN_CHECK_INTERVAL_MS  30000

/* ── Thread Stack Sizes ── */
#define POLLING_THREAD_STACK_SIZE   2048
#define BLE_THREAD_STACK_SIZE       2048
#define POLLING_THREAD_PRIORITY     5
#define BLE_THREAD_PRIORITY         5

/* ── Global State ── */
static bool is_charging = false;
static const struct device *gpio_dev;
static const struct device *i2c_dev;

/* ── BLE State ── */
static struct bt_conn *current_conn = NULL;
static bool ble_connected           = false;

/* ── Battery percent shared between threads ── */
static uint32_t latest_lower_percent = 0;
static uint32_t latest_upper_percent = 0;
static K_MUTEX_DEFINE(battery_mutex);


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
static char battery_char_value[16];

/* ── CCCD (Client Characteristic Configuration Descriptor)
      Tracks whether phone has enabled notifications         ── */
static struct bt_gatt_ccc_cfg ccc_cfg[1] = {};
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


/* ════════════════════════════════════════════
   BLE: Send battery percent notification
   Called from polling thread whenever VBMON updates
   ════════════════════════════════════════════ */
extern const struct bt_gatt_service_static battery_svc;//Forward decleration
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
   HELPER FUNCTION: Set CD pin state
   LOW  = charging enabled (or High-Z if VIN absent)
   HIGH = Active Battery mode / charging disabled
   ════════════════════════════════════════════ */
static void set_cd_pin(int state) {
    gpio_pin_set(gpio_dev, CD_PIN, state);
    k_msleep(100);
}



/* ════════════════════════════════════════════
   HELPER: Convert mid voltage to user 0-100%
   Maps 3000mV (empty) to 4200mV (full) linearly
   ════════════════════════════════════════════ 
static uint32_t voltage_to_user_percent(uint32_t voltage_mv) {
    if (voltage_mv <= BATTERY_MIN_MV) return 0;
    if (voltage_mv >= BATTERY_MAX_MV) return 100;
    return ((voltage_mv - BATTERY_MIN_MV) * 100) /
           (BATTERY_MAX_MV - BATTERY_MIN_MV);
}*/




/* ════════════════════════════════════════════
   FUNCTION: Configure CD pin
   Must be called first in main before anything else
   Starts HIGH - guarantees Active Battery mode
   and working I2C on first boot
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
   Shared helper - no CE toggling
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
   FUNCTION: Handle BAT_UVLO fault (when battery levels too low )
   ════════════════════════════════════════════ */
static void handle_batuvlo(void) {

    printk("!! Fault: BAT_UVLO - battery below 3V\n");
    printk("!! PLEASE CONNECT CHARGER IMMEDIATELY\n");

    /* TODO: Update display "Battery Critical - Connect Charger Now" */

    set_cd_pin(0);
    k_msleep(100);

    int polls_remaining = BATUVLO_MAX_POLLS - 1;
    uint8_t prev_raw    = 0x00;

    while (polls_remaining > 0) {

        printk("!! BAT_UVLO recovery: polling VBMON "
               "(%d polls remaining)\n", polls_remaining);

        uint8_t raw_value = read_vbmon_raw();
        printk("!! Recovery VBMON raw = 0x%02X\n", raw_value);

        if (!(raw_value >= BATUVLO_MIN_READABLE)) {
            printk("!! Battery unreadable - entering soft shutdown\n");
            soft_shutdown();
            return;
        }

        if (raw_value >= BATUVLO_SAFE_RAW) {
            printk("!! Battery recovered - resuming normal operation\n");
            is_charging = true;
            return;
        }

        if (raw_value > prev_raw && prev_raw != 0x00) {
            printk("!! Battery rising (0x%02X -> 0x%02X) "
                   "- resetting timer\n", prev_raw, raw_value);
            polls_remaining = BATUVLO_MAX_POLLS;
        } else if (raw_value == prev_raw && prev_raw != 0x00) {
            printk("!! Battery not rising - ensure charger connected "
                   "(%d polls remaining)\n", polls_remaining);
        }

        /* TODO: Update display "Charging - Please Wait" */

        /* Update BLE with current percent during recovery */
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
        uint32_t lower_percent  = base_percent + above_percent;
        uint32_t lower_mv       = (lower_percent * 4200) / 100;
        uint32_t upper_mv       = ((lower_percent + 2) * 4200) / 100;
        uint32_t mid_mv         = (lower_mv + upper_mv) / 2;
        /*uint32_t user_pct       = voltage_to_user_percent(mid_mv);*/

        k_mutex_lock(&battery_mutex, K_FOREVER);
        latest_lower_percent = lower_percent;
        latest_upper_percent = lower_percent + 2;
        k_mutex_unlock(&battery_mutex);
        ble_send_battery_percent(lower_percent, lower_percent + 2);

        prev_raw = raw_value;
        polls_remaining--;

        if (polls_remaining > 0) {
            k_msleep(BATUVLO_POLL_INTERVAL);
        }
    }

    printk("!! No recovery after %d polls - shutdown\n", BATUVLO_MAX_POLLS);
    soft_shutdown();
}






/* ════════════════════════════════════════════
   FUNCTION: Read fault register 0x01
   VIN_UV set   = no charger (CD HIGH, discharge)
   VIN_UV clear = charger connected → CD LOW
   I2C fail during charging = charger removed
   ════════════════════════════════════════════ */
static bool read_fault_register(void) {

    uint8_t reg_addr  = FAULTS_REG;
    uint8_t reg_value;

    int ret = i2c_write_read(i2c_dev,
                             BQ25120_ADDR,
                             &reg_addr, 1,
                             &reg_value, 1);
    if (ret != 0) {
        if (is_charging) {
            printk("!! I2C failed during charging - charger removed\n");
            printk(">> Pulling CD HIGH - Active Battery mode\n");
            set_cd_pin(1);
            is_charging = false;
        } else {
            printk("Fault register read failed: %d\n", ret);
        }
        return false;
    }

    if (reg_value != 0x00) {
        printk("\n--- Fault Register (0x01) = 0x%02X ---\n", reg_value);
    }

    bool vin_ov   = (reg_value >> 7) & 0x01;
    bool vin_uv   = (reg_value >> 6) & 0x01;
    bool bat_uvlo = (reg_value >> 5) & 0x01;
    bool bat_ocp  = (reg_value >> 4) & 0x01;

    /* VIN_UV while discharging = no charger, normal */
    if (vin_uv && !is_charging) {
        return false;
    }

    /* VIN_UV cleared while discharging = charger connected */
    if (!vin_uv && !is_charging && reg_value == 0x00) {
        printk(">> VIN_UV cleared - charger connected\n");
        printk(">> Pulling CD LOW - enabling charging\n");
        set_cd_pin(0);
        is_charging = true;
        return false;
    }

    if (vin_ov) {
        printk("!! Fault: VIN overvoltage - check charger\n");
    }
    if (bat_ocp) {
        printk("!! Fault: Battery overcurrent - check connections\n");
    }
    if (bat_uvlo) {
        handle_batuvlo();
        return true;
    }
    if (!vin_ov && !vin_uv && !bat_uvlo && !bat_ocp && reg_value != 0x00) {
        printk("!! Unknown fault - raw: 0x%02X\n", reg_value);
    }

    return false;
}





/* ════════════════════════════════════════════
   FUNCTION: Read status register 0x00
   ════════════════════════════════════════════ */
static void read_status_register(void) {

    uint8_t reg_addr  = STATUS_REG;
    uint8_t reg_value;

    int ret = i2c_write_read(i2c_dev,
                             BQ25120_ADDR,
                             &reg_addr, 1,
                             &reg_value, 1);
    if (ret != 0) {
        printk("Status register read failed: %d\n", ret);
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
            is_charging = true;
            break;
        case 0x02:
            printk("Status: Charge Done\n");
            if (is_charging) {
                printk(">> Hardware termination - disabling charge\n");
                set_cd_pin(1);
                is_charging = false;
            }
            break;
        case 0x03:
            printk("Status: FAULT - see fault register\n");
            break;
    }

    printk("CD Status:  %s\n", (reg_value & 0x02) ? "HIGH - IC disabled"
                                                   : "LOW  - IC enabled");
    printk("SYS Output: %s\n", (reg_value & 0x01) ? "Enabled" : "Disabled");
}






/* ════════════════════════════════════════════
   FUNCTION: Read battery level from VBMON
   Returns mid_voltage_mv or 0 on failure
   Also updates latest_battery_percent and
   sends BLE notification
   ════════════════════════════════════════════ */
static uint32_t read_battery_level(bool charging_active) {

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
            return 0;
        }

        uint8_t disable_charge[2] = {FAST_CHG_REG, reg_value | 0x02};
        ret = i2c_write(i2c_dev, disable_charge, 2, BQ25120_ADDR);
        if (ret != 0) {
            printk("Failed to disable charger: %d\n", ret);
            return 0;
        }

        printk("Waiting for surface charge to dissipate...\n");
        k_msleep(500);
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

    uint32_t lower_percent    = base_percent + above_percent;
    uint32_t upper_percent    = lower_percent + 2;
    uint32_t lower_voltage_mv = (lower_percent * 4200) / 100;
    uint32_t upper_voltage_mv = (upper_percent * 4200) / 100;
    uint32_t mid_voltage_mv   = (lower_voltage_mv + upper_voltage_mv) / 2;
    /*uint32_t user_percent     = voltage_to_user_percent(mid_voltage_mv);*/

    printk("\n--- Battery Level (0x0A) = 0x%02X ---\n", raw_value);
    printk("VBMON Range: %d  Threshold: %d\n", range, threshold);
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

    /* Update shared battery percent and notify BLE */
    k_mutex_lock(&battery_mutex, K_FOREVER);
    latest_lower_percent = lower_percent;
    latest_upper_percent = upper_percent;
    k_mutex_unlock(&battery_mutex);
    ble_send_battery_percent(lower_percent, upper_percent);

    /* Software charge full detection */
    if (is_charging && lower_percent >= CHARGE_FULL_PERCENT) {
        printk(">> Battery full - disabling charge via CD pin\n");
        set_cd_pin(1);
        is_charging = false;
    }

    return mid_voltage_mv;
}



/* ════════════════════════════════════════════
      FUNCTION: Soft shutdown with charger detection
   Checks fault register for VIN_UV to detect
   charger connection passively while CD HIGH
   Returns if battery recovers above RECOVERY_MV
   Falls back to full system off after MAX checks
   ════════════════════════════════════════════ */
static void soft_shutdown(void) {

    printk("\n!! SOFT SHUTDOWN - Battery critically low !!\n");
    printk("!! Please connect charger immediately       !!\n");

    /* TODO: Update display "Battery Empty - Please Charge" */
    /* TODO: Send BLE notification to user                  */
    /* TODO: Gracefully stop sensor threads                 */
    /* TODO: Save any sensor data to flash                  */

 /* CD HIGH - Active Battery mode, I2C stable */
    set_cd_pin(1);
    is_charging = false;
    k_msleep(200);

    int checks_remaining = MAX_SHUTDOWN_CHECKS-1;

    while (checks_remaining > 0) {

        printk(">> Shutdown loop: checking for charger "
               "(%d checks remaining)\n", checks_remaining);

        
        /* Read fault register to check VIN_UV bit
           VIN_UV set   = no charger
           VIN_UV clear = charger connected          */

        uint8_t reg_addr  = FAULTS_REG;
        uint8_t reg_value = 0xFF;

        int ret = i2c_write_read(i2c_dev,
                                 BQ25120_ADDR,
                                 &reg_addr, 1,
                                 &reg_value, 1);
        if (ret != 0) {
            printk(">> Fault register read failed: %d\n", ret);
            checks_remaining--;
            k_msleep(SHUTDOWN_CHECK_INTERVAL_MS);
            continue;
        }

         bool vin_uv = (reg_value >> 6) & 0x01;

        if (!vin_uv) {
            /* VIN_UV cleared - charger connected */
            printk(">> Charger detected - enabling charging\n");

            /* TODO: Update display "Charging - Please Wait" */

            set_cd_pin(0);
            is_charging = true;

            /* Wait for battery to recover */
            uint32_t recovery_voltage = 0;

            while (recovery_voltage < RECOVERY_MV) {
                recovery_voltage = read_battery_level(true);

                if (recovery_voltage == 0) {
                    /* I2C failed - charger removed during recovery */
                    printk(">> Charger removed during recovery\n");
                    set_cd_pin(1);
                    is_charging = false;
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
                printk(">> Recovered to %d.%03dV - resuming\n",
                       recovery_voltage / 1000,
                       recovery_voltage % 1000);

                /* TODO: Update display "Battery Recovered" */
                return;  /* Back to main loop */
            }

        } else {
            /* VIN_UV still set - no charger */
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

    /* Exhausted all checks - full system off */
    printk(">> No charger after %d checks - full system off\n",
           MAX_SHUTDOWN_CHECKS);
    printk(">> Connect charger and press reset to wake\n");

    /* TODO: Update display "Connect charger and press reset" */

    k_msleep(500);
    //pm_state_force(0, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});//

    while(1) { k_msleep(1000); }
}




/* ════════════════════════════════════════════
   MAIN
   ════════════════════════════════════════════ */
void main(void) {

    gpio_dev = DEVICE_DT_GET(CD_PORT);
    i2c_dev  = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    /* Step 1: Configure CD pin - must be first */
    if (configure_cd_pin() != 0) return;

    /* Step 2: Verify I2C is ready */
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }
    printk("I2C ready\n");

    /* Step 3: Read status once at startup for initial state */
    read_status_register();

    /* Step 4: Check fault register on startup
       Handles VIN_UV → no charger → stay in Active Battery mode
       Handles VIN_UV cleared → charger present → start charging
       Handles BAT_UVLO → recovery loop                          */
    read_fault_register();

    /* ── Main Loop ── */

    uint32_t fault_poll_counter  = 0;
    uint32_t vbmon_poll_counter  = 0;

    /* Determine initial VBMON poll interval */
    uint32_t vbmon_interval_ms   = VBMON_POLL_INTERVAL_MS;

    while(1) {

        k_msleep(FAULT_POLL_INTERVAL_MS);
        fault_poll_counter += FAULT_POLL_INTERVAL_MS;
        vbmon_poll_counter += FAULT_POLL_INTERVAL_MS;

        /* ── Fault register poll every 10 seconds ──
           While discharging: detects charger connection via VIN_UV
           While charging:    detects charger removal via I2C failure  */
        read_fault_register();

        /* ── VBMON poll on its own interval ── */
        if (vbmon_poll_counter >= vbmon_interval_ms) {
            vbmon_poll_counter = 0;

            uint32_t voltage_mv = read_battery_level(is_charging);

            /* Soft shutdown if critically low */
            if (voltage_mv > 0 && voltage_mv <= SOFT_SHUTDOWN_MV) {
                printk(">> Voltage at or below %dmV - soft shutdown\n",
                       SOFT_SHUTDOWN_MV);
                soft_shutdown();
                printk(">> Resuming normal operation after recovery\n");
            }

            /* Adjust VBMON polling interval based on battery level */
            if (voltage_mv > 0 && voltage_mv <= LOW_BATTERY_MV) {
                vbmon_interval_ms = VBMON_LOW_POLL_INTERVAL_MS;
            } else {
                vbmon_interval_ms = VBMON_POLL_INTERVAL_MS;
            }
        }
    }
}