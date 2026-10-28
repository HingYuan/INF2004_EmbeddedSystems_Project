#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "IMU_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"
#include "../../device/include/dev_i2c.h"

/* No <math.h>/libm here: the header clashes with uT-Kernel's size_t typedef,
 * and libm drags in newlib syscalls (_write, _sbrk...) the kernel doesn't
 * provide. These small helpers are accurate enough for tilt/shock detection. */
#define fabsf(x)        __builtin_fabsf(x)   // inlined, no libm call
#define fabs(x)         __builtin_fabs(x)

LOCAL float imu_sqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;
    float r = (x > 1.0f) ? x : 1.0f;        // start above the root
    for (int i = 0; i < 12; i++) r = 0.5f * (r + x / r);  // Newton
    return r;
}

/* atan2 in radians, max error ~0.005 rad (~0.3 deg) */
LOCAL float imu_atan2f(float y, float x)
{
    const float PI = 3.14159265f;
    float ax = fabsf(x), ay = fabsf(y);
    if (ax == 0.0f && ay == 0.0f) return 0.0f;
    float a = (ax > ay) ? ay / ax : ax / ay;
    float r = a * (PI / 4.0f) + 0.273f * a * (1.0f - a);
    if (ay > ax) r = PI / 2.0f - r;
    if (x < 0.0f) r = PI - r;
    return (y < 0.0f) ? -r : r;
}
#define sqrtf(x)        imu_sqrtf(x)
#define atan2f(y, x)    imu_atan2f(y, x)

#if (IMU_SDA_PIN % 4) != 0 || IMU_SDA_PIN > 20 || IMU_SCL_PIN != (IMU_SDA_PIN + 1)
#error "IMU pins must be an RP2040 I2C0 SDA/SCL pair: GP0/1, GP4/5, GP8/9, GP12/13, GP16/17 or GP20/21"
#endif
#if IMU_SDA_PIN == 0 && !TM_CONSOLE_USB_CDC
#error "IMU on GP0/GP1 collides with the UART console; build with CONSOLE=usb_cdc"
#endif

extern ID navigator_mbfid;

typedef enum {
    STATE_INITIALIZE,
    STATE_CALIBRATING,
    STATE_MONITORING,
    STATE_TILT_WARNING,
    STATE_HUMP_INTEGRATING,
    STATE_SHOCK_DETECTED
} IMU_FSM_STATE;

#define CTRL_REG1_A           0x20
#define CTRL_REG4_A           0x23
#define OUT_X_L_A             0x28

LOCAL ID imu_dd = 0;    // "iica" device descriptor

/* The I2C driver routes I2C0 to GP8/GP9 at boot. Route it to the IMU pins
 * instead. GP8/GP9 are only detached if they still carry I2C (the motor task
 * may already have claimed them for PWM) - RP2040 ORs a peripheral input that
 * is selected on several pins, so an idle GP8/GP9 would corrupt SDA reads. */
#define RP2040_UART0_BASE   (0x40034000UL)

LOCAL void imu_route_pins(void)
{
#if IMU_SDA_PIN == 0
    /* If GP0/GP1 were claimed by UART0, disable the UART hardware so it releases the pins */
    out_w(RP2040_UART0_BASE + 0x030, 0); // UART0_CR = 0 (disable UART0)
#endif
#if IMU_SDA_PIN != 8
    if((in_w(GPIO_CTRL(8)) & 0x1F) == GPIO_CTRL_FUNCSEL_I2C) out_w(GPIO_CTRL(8), GPIO_CTRL_FUNCSEL_NULL);
    if((in_w(GPIO_CTRL(9)) & 0x1F) == GPIO_CTRL_FUNCSEL_I2C) out_w(GPIO_CTRL(9), GPIO_CTRL_FUNCSEL_NULL);
#endif
    out_w(GPIO(IMU_SDA_PIN), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(IMU_SCL_PIN), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(IMU_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(IMU_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
}

LOCAL ER imu_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    return i2c_write_reg(imu_dd, addr, reg, val);
}

LOCAL UW active_accel_addr = LSM303_ACCEL_ADDR;

LOCAL bool imu_read_accel_raw(int16_t *x, int16_t *y, int16_t *z)
{
    UB data[6];
    INT i;

    // Use byte-by-byte register reads (0xA8 = OUT_X_L_A with auto-increment bit)
    for(i = 0; i < 6; i++) {
        ER er = i2c_read_reg(imu_dd, active_accel_addr, (UW)(0xA8 + i), &data[i]);
        if(er < E_OK) return false;
    }
    *x = (int16_t)(((UINT)data[1] << 8) | data[0]) >> 4;
    *y = (int16_t)(((UINT)data[3] << 8) | data[2]) >> 4;
    *z = (int16_t)(((UINT)data[5] << 8) | data[4]) >> 4;
    return true;
}

static void imu_read_accel_g(float *gx, float *gy, float *gz)
{
    int16_t rx = 0, ry = 0, rz = 0;
    if (imu_read_accel_raw(&rx, &ry, &rz)) {
        *gx = (float)rx * 0.001f;
        *gy = (float)ry * 0.001f;
        *gz = (float)rz * 0.001f;
    } else {
        *gx = 0.0f;
        *gy = 0.0f;
        *gz = 1.0f; // Default baseline fallback
    }
}

LOCAL void imu_scan_bus(void)
{
    tm_printf((UB *)"[IMU] Scanning I2C bus (%s) on GP%d(SDA), GP%d(SCL)...\n",
              (UB *)IMU_I2C_DEVNAME, IMU_SDA_PIN, IMU_SCL_PIN);
    INT found = 0;
    for(UW addr = 0x08; addr <= 0x77; addr++) {
        UB dummy = 0;
        // Try reading 1 byte from reg 0x00 or 0x0F
        ER er = i2c_read_reg(imu_dd, addr, 0x0F, &dummy);
        if(er >= E_OK) {
            tm_printf((UB *)"[IMU SCAN] Found device responding at 0x%02x (reg 0x0F = 0x%02x)\n", (UINT)addr, (UINT)dummy);
            found++;
        }
    }
    if(found == 0) {
        tm_printf((UB *)"[IMU SCAN] No devices responded on I2C bus.\n");
    }
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    IMU_FSM_STATE state = STATE_INITIALIZE;
    float baseline_z = 1.0f;
    
    // Integration variables for hump height
    float vertical_velocity = 0.0f; // m/s
    float hump_peak_height = 0.0f;  // cm
    INT telemetry_timer = 0;

    // tm_printf((UB *)"[IMU] Task started on PRC%d\n", (INT)tk_get_prc());

    while (1) {
        switch (state) {
            case STATE_INITIALIZE: {
                // Open the uT-Kernel I2C0 driver (400 kHz Fast Mode, set by the driver)
                if (imu_dd <= 0) {
                    imu_dd = tk_opn_dev((const UB *)IMU_I2C_DEVNAME, TD_UPDATE);
                    if (imu_dd <= E_OK) {
                        tm_printf((UB *)"[IMU STATUS] FAILED to open I2C device %s (er=%d)\n", (UB *)IMU_I2C_DEVNAME, imu_dd);
                        imu_dd = 0;
                        tk_dly_tsk(1000);
                        break;
                    }
                    imu_route_pins();
                    tk_dly_tsk(50);
                    imu_scan_bus();
                }

                // Configure LSM303DLHC Accelerometer
                // Try configured address (0x19) or alt address (0x18)
                UW accel_addr = LSM303_ACCEL_ADDR;
                ER er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                if (er1 < E_OK) {
                    // Try alternative address 0x18
                    accel_addr = 0x18;
                    er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                }

                ER er2 = (er1 >= E_OK) ? imu_write_reg(accel_addr, CTRL_REG4_A, 0x08) : er1;

                if (er1 >= E_OK && er2 >= E_OK) {
                    active_accel_addr = accel_addr;
                    tm_printf((UB *)"====================================================\n");
                    tm_printf((UB *)"[IMU STATUS] CONNECTED! LSM303DLHC ready at 0x%02x\n", (UINT)active_accel_addr);
                    tm_printf((UB *)"[IMU STATUS] Bus: %s | SDA: GP%d | SCL: GP%d\n",
                              (UB *)IMU_I2C_DEVNAME, IMU_SDA_PIN, IMU_SCL_PIN);
                    tm_printf((UB *)"====================================================\n");
                    state = STATE_CALIBRATING;
                } else {
                    tm_printf((UB *)"[IMU STATUS] NOT CONNECTED! No response at 0x19 or 0x18 (er=%d).\n", er1);
                    tm_printf((UB *)"             Check wiring: SDA -> GP%d, SCL -> GP%d, 3.3V, GND.\n",
                              IMU_SDA_PIN, IMU_SCL_PIN);
                    tk_dly_tsk(1000);
                }
                break;
            }

            case STATE_CALIBRATING: {
                tm_printf((UB *)"[IMU STATUS] Calibrating 1g static baseline...\n");
                // Collect 50 samples to establish static flat-ground 1g reference
                float sum_z = 0.0f;
                const int SAMPLES = 50;

                for (int i = 0; i < SAMPLES; i++) {
                    float gx, gy, gz;
                    imu_read_accel_g(&gx, &gy, &gz);
                    sum_z += gz;
                    tk_dly_tsk(10);
                }
                baseline_z = sum_z / (float)SAMPLES;
                INT bz_int = (INT)baseline_z;
                INT bz_dec = (INT)(fabsf(baseline_z * 100.0f)) % 100;
                tm_printf((UB *)"[IMU STATUS] Calibration complete. Baseline Z: %d.%02d g\n", bz_int, bz_dec);

                state = STATE_MONITORING;
                break;
            }

            case STATE_MONITORING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);

                float total_g = sqrtf(gx * gx + gy * gy + gz * gz);
                float pitch = atan2f(gx, sqrtf(gy * gy + gz * gz)) * 180.0f / 3.14159f;
                float roll  = atan2f(gy, gz) * 180.0f / 3.14159f;

                // Priority 1: Emergency Shock Detection (> 2.5g)
                if (total_g > SHOCK_THRESHOLD_G) {
                    state = STATE_SHOCK_DETECTED;
                    break;
                }

                // Priority 2: Critical Tilt Warning (> 30 degrees)
                if (fabsf(pitch) > SAFE_TILT_LIMIT_DEG) {
                    state = STATE_TILT_WARNING;
                    break;
                }

                // Priority 3: Hump/Terrain Bump Detection
                if (fabsf(gz - baseline_z) > HUMP_THRESHOLD_G) {
                    vertical_velocity = 0.0f;
                    hump_peak_height = 0.0f;
                    state = STATE_HUMP_INTEGRATING;
                    break;
                }

                // Periodic telemetry display every 500 ms (50 * 10ms ticks)
                // if (++telemetry_timer >= 50) {
                //     telemetry_timer = 0;
                //     INT g_int = (INT)total_g;
                //     INT g_dec = (INT)(fabsf(total_g * 100.0f)) % 100;
                //     INT gz_int = (INT)gz;
                //     INT gz_dec = (INT)(fabsf(gz * 100.0f)) % 100;

                //     tm_printf((UB *)"[IMU READINGS] Pitch: %4d deg | Roll: %4d deg | Total: %d.%02d g | Z: %d.%02d g [OK]\n",
                //               (INT)pitch, (INT)roll, g_int, g_dec, gz_int, gz_dec);
                // }
                //Code changed to send telemetry to navigator task instead of printing to console
                /* Send normal IMU telemetry every 500 ms */
                if (++telemetry_timer >= 50) {
                    telemetry_timer = 0;
                    tm_printf((UB *)"[IMU] pitch_x100=%d roll_x100=%d total_g_x1000=%d\n",
                    (int)(pitch * 100.0f),
                    (int)(roll * 100.0f),
                    (int)(total_g * 1000.0f));
                    IMU_TELEMETRY_MSG msg = {
                        .event = 0,  /* Normal monitoring; use a named enum if available */
                        .pitch_deg = pitch,
                        .roll_deg = roll,
                        .peak_g = total_g
                    };

                    if (navigator_mbfid > E_OK) {
                        NAVIGATOR_MESSAGE nav_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg
                        };

                        ER er = tk_snd_mbf(
                            navigator_mbfid,
                            &nav_msg,
                            sizeof(NAVIGATOR_MESSAGE),
                            TMO_POL
                        );

                        if (er < E_OK) {
                            tm_printf((UB *)"[IMU] Failed to send telemetry: %d\n", er);
                        }
                    }
                }
                break;
            }

            case STATE_TILT_WARNING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);
                float pitch = atan2f(gx, sqrtf(gy * gy + gz * gz)) * 180.0f / 3.14159f;

                tm_printf((UB *)"[IMU WARNING] Safe tilt exceeded! Pitch: %d deg\n", (INT)pitch);

                IMU_TELEMETRY_MSG msg = {
                    .event = IMU_MSG_TILT_WARNING,
                    .pitch_deg = pitch,
                    .peak_g = sqrtf(gx * gx + gy * gy + gz * gz)
                };
                if (navigator_mbfid > E_OK) {
                    NAVIGATOR_MESSAGE nav_msg = {
                        .type = MSG_IMU_UPDATE,
                        .data.imu_telemetry_msg = msg
                    };
                    tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL); //
                }

                // Hysteresis recovery check
                if (fabsf(pitch) < (SAFE_TILT_LIMIT_DEG - 5.0f)) {
                    state = STATE_MONITORING;
                }
                break;
            }

            case STATE_HUMP_INTEGRATING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);

                // dt = 10 ms (0.01 s)
                const float dt = 0.01f;
                // Vertical net acceleration: a = (gz - baseline_z) * 9.81 m/s^2
                float net_accel_mps2 = (gz - baseline_z) * 9.80665f;

                vertical_velocity += net_accel_mps2 * dt;
                
                // Height using kinematic energy balance: h = v^2 / (2 * g)
                float current_height_cm = (vertical_velocity * vertical_velocity) / (2.0f * 9.80665f) * 100.0f;
                if (current_height_cm > hump_peak_height) {
                    hump_peak_height = current_height_cm;
                }

                // Check for return to flat ground
                if (fabsf(gz - baseline_z) < 0.10f) {
                    tm_printf((UB *)"[IMU HUMP] Hump Cleared. Peak height: %d.%d cm\n",
                              (INT)hump_peak_height, (INT)(hump_peak_height * 10.0f) % 10);

                    IMU_TELEMETRY_MSG msg = {
                        .event = IMU_MSG_HUMP_INTEGRATING,
                        .estimated_height_cm = hump_peak_height
                    };
                    if (navigator_mbfid > E_OK) {
                        NAVIGATOR_MESSAGE nav_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg
                        };
                        tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL); //[cite: 7]
                    }
                    state = STATE_MONITORING;
                }
                break;
            }

            case STATE_SHOCK_DETECTED: {
                tm_printf((UB *)"[IMU CRITICAL] E-STOP TRIGGERED! High-G impact detected.\n");

                IMU_TELEMETRY_MSG msg = {
                    .event = IMU_MSG_SHOCK_DETECTED,
                    .peak_g = SHOCK_THRESHOLD_G
                };
                if (navigator_mbfid > E_OK) {
                    NAVIGATOR_MESSAGE nav_msg = {
                        .type = MSG_IMU_UPDATE,
                        .data.imu_telemetry_msg = msg
                    };
                    tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL); //[cite: 7]
                }

                // Enforce safety lockdown
                while (1) {
                    tk_dly_tsk(1000);
                }
                break;
            }
        }

        // 100 Hz sampling period (10 ms)
        tk_dly_tsk(10);
    }
}


// LOCAL ER read_accel(ID iic, int16_t *x, int16_t *y, int16_t *z)
// {
// 	UB data[6];
// 	INT i;

// 	for(i = 0; i < 6; i++) {
// 		ER er = i2c_read_reg(iic, 0x19, (UW)(0xA8 + i), &data[i]);
// 		if(er < E_OK) return er;
// 	}
// 	*x = (int16_t)(((UINT)data[1] << 8) | data[0]) >> 4;
// 	*y = (int16_t)(((UINT)data[3] << 8) | data[2]) >> 4;
// 	*z = (int16_t)(((UINT)data[5] << 8) | data[4]) >> 4;
// 	return E_OK;
// }

// LOCAL ER read_magnetometer(ID iic, int16_t *x, int16_t *y, int16_t *z)
// {
// 	UB data[6];
// 	INT i;
// 	static const UB registers[6] = { 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

// 	for(i = 0; i < 6; i++) {
// 		ER er = i2c_read_reg(iic, 0x1E, registers[i], &data[i]);
// 		if(er < E_OK) return er;
// 	}
// 	*x = (int16_t)(((UINT)data[0] << 8) | data[1]);
// 	*z = (int16_t)(((UINT)data[2] << 8) | data[3]);
// 	*y = (int16_t)(((UINT)data[4] << 8) | data[5]);
// 	return E_OK;
// }

// EXPORT void imu_task(INT stacd, void *exinf)
// {
// 	ID iic;
// 	ER er;
// 	int16_t ax, ay, az;
// 	int16_t mx, my, mz;

// 	(void)stacd;
// 	(void)exinf;
// 	iic = tk_opn_dev((const UB *)"iica", TD_UPDATE);
// 	if(iic <= E_OK) {
// 		tm_printf((UB *)"[imu] cannot open iica, er=%d\n", iic);
// 		return;
// 	}

// 	er = i2c_write_reg(iic, 0x19, 0x20, 0x57);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x19, 0x23, 0x08);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x00, 0x14);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x01, 0x20);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x02, 0x00);
// 	if(er < E_OK) {
// 		tm_printf((UB *)"[imu] sensor setup failed, er=%d\n", er);
// 		(void)tk_cls_dev(iic, 0);
// 		return;
// 	}
// 	tm_printf((UB *)"[imu] LSM303DLHC at accel=0x19 mag=0x1e; raw units\n");

// 	while(1) {
// 		er = read_accel(iic, &ax, &ay, &az);
// 		if(er >= E_OK) er = read_magnetometer(iic, &mx, &my, &mz);
// 		if(er >= E_OK) {
// 			tm_printf((UB *)"[imu] accel=%d,%d,%d mag=%d,%d,%d\n",
// 				  ax, ay, az, mx, my, mz);
// 		} else {
// 			tm_printf((UB *)"[imu] I2C read failed, er=%d\n", er);
// 		}
// 		tk_dly_tsk(250);
// 	}
// }

