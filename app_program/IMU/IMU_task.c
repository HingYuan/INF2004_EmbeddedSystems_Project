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

// Fast absolute value without libm
#define fabsf(x)        __builtin_fabsf(x)
#define fabs(x)         __builtin_fabs(x)

// Square a value for fast thresholding
#define SQ(x)           ((x) * (x))

LOCAL float imu_sqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;
    float r = (x > 1.0f) ? x : 1.0f;
    for (int i = 0; i < 12; i++) r = 0.5f * (r + x / r);
    return r;
}

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

#if (IMU_SDA_PIN % 4) != 0 || IMU_SDA_PIN > 20 || IMU_SCL_PIN != (IMU_SDA_PIN + 1)
#error "IMU pins must be an RP2040 I2C0 SDA/SCL pair: GP0/1, GP4/5, GP8/9, GP12/13, GP16/17 or GP20/21"
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

LOCAL ID imu_dd = 0;

#define RP2040_UART0_BASE   (0x40034000UL)

LOCAL void imu_route_pins(void)
{
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
    SZ asz = 0;
    
    // 0xA8 is OUT_X_L_A with the MSB set for auto-increment.
    UB reg = 0xA8; 

    // Single burst write of the register address
    tk_swri_dev(imu_dd, active_accel_addr, &reg, 1, &asz);
    
    // Single burst read of all 6 bytes (saves 5 context switches into the OS kernel)
    ER er = tk_srea_dev(imu_dd, active_accel_addr, data, 6, &asz);
    
    if (er < E_OK) return false;

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
        *gz = 1.0f;
    }
}

LOCAL void imu_scan_bus(void)
{
    tm_printf((UB *)"[IMU] Scanning I2C bus on GP%d(SDA), GP%d(SCL)...\n", IMU_SDA_PIN, IMU_SCL_PIN);
    INT found = 0;
    for(UW addr = 0x08; addr <= 0x77; addr++) {
        UB dummy = 0;
        ER er = i2c_read_reg(imu_dd, addr, 0x0F, &dummy);
        if(er >= E_OK) found++;
    }
    if(found == 0) tm_printf((UB *)"[IMU SCAN] No devices responded.\n");
}

LOCAL INT iabs(INT v) { return v < 0 ? -v : v; }

EXPORT void imu_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;

    IMU_FSM_STATE state = STATE_INITIALIZE;
    float baseline_z = 1.0f;
    float baseline_pitch = 0.0f;
    float baseline_roll = 0.0f;
    float vertical_velocity = 0.0f;
    float hump_peak_height = 0.0f;
    INT telemetry_timer = 0;

    // Fast threshold constants pre-calculated for the 100Hz loop
    // SAFE_TILT_LIMIT_DEG = 30. sin(30) = 0.5. sin^2(30) = 0.25. 
    // If pitch > 30, then (gx^2) > 0.25 * (total_g^2)
    const float SHOCK_G_SQ = SQ(SHOCK_THRESHOLD_G);
    const float TILT_30_RATIO = 0.25f; 

    BOOL no_hump_sent = 0;

    while (1) {
        switch (state) {
            case STATE_INITIALIZE: {
                if (imu_dd <= 0) {
                    imu_dd = tk_opn_dev((const UB *)IMU_I2C_DEVNAME, TD_UPDATE);
                    if (imu_dd <= E_OK) {
                        imu_dd = 0;
                        tm_printf((UB *)"[IMU] I2C device open failed\n");
                        tk_dly_tsk(1000);
                        break;
                    }
                    imu_route_pins();
                    tk_dly_tsk(50);
                    imu_scan_bus();
                }

                UW accel_addr = LSM303_ACCEL_ADDR;
                ER er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                if (er1 < E_OK) {
                    accel_addr = 0x18;
                    er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                }

                ER er2 = (er1 >= E_OK) ? imu_write_reg(accel_addr, CTRL_REG4_A, 0x08) : er1;

                if (er1 >= E_OK && er2 >= E_OK) {
                    active_accel_addr = accel_addr;
                    tm_printf((UB *)"[IMU STATUS] CONNECTED!\n");
                    state = STATE_CALIBRATING;
                } else {
                    tk_dly_tsk(1000);
                }
                break;
            }

            case STATE_CALIBRATING: {
                float sum_z = 0.0f;
                float sum_pitch = 0.0f;
                float sum_roll = 0.0f;
                const int SAMPLES = 50;

                for (int i = 0; i < SAMPLES; i++) {
                    float gx, gy, gz;
                    imu_read_accel_g(&gx, &gy, &gz);
                    sum_z += gz;
                    float p = imu_atan2f(gx, imu_sqrtf(SQ(gy) + SQ(gz))) * 180.0f / 3.14159f;
                    float r = imu_atan2f(gy, gz) * 180.0f / 3.14159f;
                    sum_pitch += p;
                    sum_roll += r;
                    tk_dly_tsk(10);
                }
                baseline_z = sum_z / (float)SAMPLES;
                baseline_pitch = sum_pitch / (float)SAMPLES;
                baseline_roll = sum_roll / (float)SAMPLES;
                tm_printf((UB *)"[IMU CAL] Done! Resting baseline: Pitch=%d deg | Roll=%d deg\n",
                          (INT)baseline_pitch, (INT)baseline_roll);
                state = STATE_MONITORING;
                break;
            }

            case STATE_MONITORING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);

                // Extremely fast squared magnitudes (No floats or trigonometry)
                float total_g_sq = SQ(gx) + SQ(gy) + SQ(gz);

                float total_g = imu_sqrtf(total_g_sq);
                float raw_pitch = imu_atan2f(gx, imu_sqrtf(SQ(gy) + SQ(gz))) * 180.0f / 3.14159f;
                float raw_roll  = imu_atan2f(gy, gz) * 180.0f / 3.14159f;
                // Relative delta angles compared to resting flat baseline
                float pitch = raw_pitch - baseline_pitch;
                float roll  = raw_roll  - baseline_roll;

                static INT print_timer = 0;
                if (++print_timer >= 60) { // Print every ~1.2s to prevent serial monitor flood
                    print_timer = 0;
                    tm_printf((UB *)"[IMU] dPitch: %3d deg | dRoll: %3d deg | Total: %d.%02d g\n",
                              (INT)pitch, (INT)roll, (INT)total_g, (INT)(fabsf(total_g * 100.0f)) % 100);
                }

                // Detect mini hump when tilt exceeds baseline by >= 5 degrees
                if (iabs((INT)pitch) >= 5 || iabs((INT)roll) >= 5) {
                    float gx, gy, gz;
                    imu_read_accel_g(&gx, &gy, &gz);

                    float net_accel_mps2 = (gz - baseline_z) * 9.80665f;
                    vertical_velocity += net_accel_mps2 * 0.02f;
                    
                    // Height in mm: chassis geometric pitch lift (2.44 mm per degree tilt)
                    float current_height_mm = fabsf(pitch) * 2.44f;
                    if (current_height_mm > hump_peak_height) {
                        hump_peak_height = current_height_mm;
                    }

                    if (no_hump_sent) {
                        hump_peak_height = current_height_mm;
                        tm_printf((UB *)"[IMU] *** HUMP DETECTED! dPitch: %d deg | dRoll: %d deg ***\n", (INT)pitch, (INT)roll);
                    }

                    IMU_TELEMETRY_MSG msg = {
                        .event = IMU_MSG_HUMP_DETECTED,
                        .pitch_deg = pitch,
                        .roll_deg = roll,
                        .estimated_height_cm = hump_peak_height, // stores height in mm
                    };

                    NAVIGATOR_MESSAGE n_msg = {
                        .type = MSG_IMU_UPDATE,
                        .data.imu_telemetry_msg = msg,
                    };

                    tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_FEVR);

                    if (no_hump_sent) no_hump_sent = 0;
                } else {
                    if (!no_hump_sent) {
                        tm_printf((UB *)"[IMU HUMP] Hump Cleared! Peak height: %d mm\n",
                                  (INT)hump_peak_height);
                        
                        IMU_TELEMETRY_MSG msg = {
                            .event = IMU_MSG_NO_HUMP,
                            .estimated_height_cm = hump_peak_height, // stores height in mm
                        };

                        NAVIGATOR_MESSAGE n_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg,
                        };

                        tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_FEVR);

                        vertical_velocity = 0.0f;
                        no_hump_sent = 1;
                    }
                }

                // Record telemetry every 500ms
                if (++telemetry_timer >= 5) {
                    telemetry_timer = 0;
                    telemetry_snapshot.pitch_deg = pitch;
                    telemetry_snapshot.roll_deg = roll;
                    telemetry_snapshot.imu_event = IMU_MSG_NONE;
                }

                // Priority 1: Shock Detection 
                if (total_g_sq > SHOCK_G_SQ) {
                    state = STATE_SHOCK_DETECTED;
                    break;
                }

                // Priority 2: Tilt Warning (Pitch > 30 deg equivalent)
                if (SQ(gx) > (total_g_sq * TILT_30_RATIO)) {
                    state = STATE_TILT_WARNING;
                    break;
                }

                // // Priority 3: Hump Detection
                // if (fabsf(gz - baseline_z) > HUMP_THRESHOLD_G) {
                //     vertical_velocity = 0.0f;
                //     hump_peak_height = 0.0f;
                //     state = STATE_HUMP_INTEGRATING;
                //     break;
                // }

                break;
            }

            case STATE_TILT_WARNING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);
                
                // Allow expensive math here because the car is likely stopped/recovering anyway
                float total_g = imu_sqrtf(SQ(gx) + SQ(gy) + SQ(gz));
                float pitch = imu_atan2f(gx, imu_sqrtf(SQ(gy) + SQ(gz))) * 180.0f / 3.14159f;

                IMU_TELEMETRY_MSG msg = {
                    .event = IMU_MSG_TILT_WARNING,
                    .pitch_deg = pitch,
                    .peak_g = total_g
                };
                if (navigator_mbfid > E_OK) {
                    telemetry_snapshot.pitch_deg = pitch;
                    telemetry_snapshot.imu_event = IMU_MSG_TILT_WARNING;
                    NAVIGATOR_MESSAGE nav_msg = { .type = MSG_IMU_UPDATE, .data.imu_telemetry_msg = msg };
                    (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                }

                if (fabsf(pitch) < (SAFE_TILT_LIMIT_DEG - 5.0f)) {
                    state = STATE_MONITORING;
                }
                break;
            }

            // case STATE_HUMP_INTEGRATING: {
            //     float gx, gy, gz;
            //     imu_read_accel_g(&gx, &gy, &gz);

            //     float net_accel_mps2 = (gz - baseline_z) * 9.80665f;
            //     vertical_velocity += net_accel_mps2 * 0.01f;
                
            //     float current_height_cm = SQ(vertical_velocity) / (2.0f * 9.80665f) * 100.0f;
            //     if (current_height_cm > hump_peak_height) hump_peak_height = current_height_cm;

            //     if (fabsf(gz - baseline_z) < 0.10f) {
            //         IMU_TELEMETRY_MSG msg = {
            //             .event = IMU_MSG_HUMP_INTEGRATING,
            //             .estimated_height_cm = hump_peak_height
            //         };
            //         if (navigator_mbfid > E_OK) {
            //             telemetry_snapshot.imu_event = IMU_MSG_HUMP_INTEGRATING;
            //             NAVIGATOR_MESSAGE nav_msg = { .type = MSG_IMU_UPDATE, .data.imu_telemetry_msg = msg };
            //             (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
            //         }
            //         state = STATE_MONITORING;
            //     }
            //     break;
            // }

            case STATE_SHOCK_DETECTED: {
                IMU_TELEMETRY_MSG msg = {
                    .event = IMU_MSG_SHOCK_DETECTED,
                    .peak_g = SHOCK_THRESHOLD_G
                };
                if (navigator_mbfid > E_OK) {
                    telemetry_snapshot.imu_event = IMU_MSG_SHOCK_DETECTED;
                    NAVIGATOR_MESSAGE nav_msg = { .type = MSG_IMU_UPDATE, .data.imu_telemetry_msg = msg };
                    (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                }
                while (1) tk_dly_tsk(1000);
                break;
            }
        }

        // 20ms period (50Hz sampling)
        tk_dly_tsk(2);
    }
}