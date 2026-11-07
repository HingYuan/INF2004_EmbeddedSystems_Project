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
#include "../Motor/Motor_task.h"

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

// Swapped and inverted for physical IMU mounting:
// Pitch is along sensor Y (inverted), Roll is along sensor X (inverted)
LOCAL void imu_compute_pitch_roll(float gx, float gy, float gz, float *pitch, float *roll)
{
    float den = imu_sqrtf(SQ(gx) + SQ(gz));
    if (den < 0.001f) den = 0.001f;
    *pitch = -imu_atan2f(gy, den) * 180.0f / 3.14159265f;
    *roll  = -imu_atan2f(gx, gz) * 180.0f / 3.14159265f;
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

// --- Magnetometer (LSM303DLHC at 0x1E) ---
#define LSM303_MAG_ADDR       0x1E
#define CRA_REG_M             0x00    // Data output rate register
#define CRB_REG_M             0x01    // Gain / sensitivity register
#define MR_REG_M              0x02    // Mode register (continuous vs sleep)
#define OUT_X_H_M             0x03    // First data register (X High)

#define CTRL_REG1_A           0x20
#define CTRL_REG4_A           0x23
#define OUT_X_L_A             0x28

LOCAL ID imu_dd = 0;

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
    SZ asz = 0;
    
    // 0xA8 is OUT_X_L_A with MSB set for auto-increment.
    UB reg = 0xA8; 
    T_I2C_EXEC exec = {
        .sadr = active_accel_addr,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = 6,
        .rcv_data = data,
    };

    ER er = tk_swri_dev(imu_dd, TDN_I2C_EXEC, &exec, sizeof(T_I2C_EXEC), &asz);
    if (er < E_OK) {
        // Fallback to byte-by-byte reads
        for (int i = 0; i < 6; i++) {
            if (i2c_read_reg(imu_dd, active_accel_addr, (UW)(0xA8 + i), &data[i]) < E_OK) return false;
        }
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
        *gz = 1.0f;
    }
}

LOCAL bool imu_read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    UB data[6];
    SZ asz = 0;
    UB reg = OUT_X_H_M; // 0x03
    T_I2C_EXEC exec = {
        .sadr = LSM303_MAG_ADDR,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = 6,
        .rcv_data = data,
    };

    ER er = tk_swri_dev(imu_dd, TDN_I2C_EXEC, &exec, sizeof(T_I2C_EXEC), &asz);
    if (er < E_OK) {
        for (int i = 0; i < 6; i++) {
            if (i2c_read_reg(imu_dd, LSM303_MAG_ADDR, (UW)(OUT_X_H_M + i), &data[i]) < E_OK) return false;
        }
    }

    // On LSM303DLHC, magnetometer register order is X, Z, Y (big-endian)
    *mx = (int16_t)(((UINT)data[0] << 8) | data[1]);
    *mz = (int16_t)(((UINT)data[2] << 8) | data[3]);
    *my = (int16_t)(((UINT)data[4] << 8) | data[5]);
    return true;
}

// Returns compass heading in degrees (0 to 359)
LOCAL int16_t imu_get_heading_deg(void)
{
    int16_t mx = 0, my = 0, mz = 0;
    if (!imu_read_mag_raw(&mx, &my, &mz)) return 0;

    float rad = imu_atan2f((float)my, (float)mx);
    float deg = rad * 180.0f / 3.14159f;

    if (deg < 0.0f) deg += 360.0f;
    return (int16_t)deg;
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
                telemetry_snapshot.imu_state = (uint8_t)STATE_INITIALIZE;
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
                } else {
                    imu_route_pins();
                }

                UW accel_addr = LSM303_ACCEL_ADDR;
                ER er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                if (er1 < E_OK) {
                    accel_addr = 0x18;
                    er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                }

                ER er2 = (er1 >= E_OK) ? imu_write_reg(accel_addr, CTRL_REG4_A, 0x08) : er1;

                // Magnetometer config
                (void)imu_write_reg(LSM303_MAG_ADDR, CRA_REG_M, 0x14); // 75 Hz data rate
                (void)imu_write_reg(LSM303_MAG_ADDR, CRB_REG_M, 0x20); // +/- 1.3 Gauss range
                (void)imu_write_reg(LSM303_MAG_ADDR, MR_REG_M,  0x00); // Continuous measurement mode

                if (er1 >= E_OK && er2 >= E_OK) {
                    active_accel_addr = accel_addr;
                    tm_printf((UB *)"[IMU STATUS] CONNECTED! Accel at 0x%02x\n", (UINT)active_accel_addr);
                    state = STATE_CALIBRATING;
                    telemetry_snapshot.imu_state = (uint8_t)STATE_CALIBRATING;
                } else {
                    tm_printf((UB *)"[IMU STATUS] Device not ready (er1=%d, er2=%d). Retrying...\n", er1, er2);
                    tk_dly_tsk(1000);
                }
                break;
            }

            case STATE_CALIBRATING: {
                telemetry_snapshot.imu_state = (uint8_t)STATE_CALIBRATING;
                float sum_z = 0.0f;
                float sum_pitch = 0.0f;
                float sum_roll = 0.0f;
                const int SAMPLES = 50;

                for (int i = 0; i < SAMPLES; i++) {
                    float gx, gy, gz;
                    imu_read_accel_g(&gx, &gy, &gz);
                    sum_z += gz;

                    float p = 0.0f, r = 0.0f;
                    imu_compute_pitch_roll(gx, gy, gz, &p, &r);
                    sum_pitch += p;
                    sum_roll += r;

                    tk_dly_tsk(10);
                }
                baseline_z = sum_z / (float)SAMPLES;
                baseline_pitch = sum_pitch / (float)SAMPLES;
                baseline_roll = sum_roll / (float)SAMPLES;

                tm_printf((UB *)"[IMU CALIB] Zero offsets: Pitch=%d deg, Roll=%d deg, Z=%d.%02d g\n",
                          (INT)baseline_pitch, (INT)baseline_roll,
                          (INT)baseline_z, (INT)(fabsf(baseline_z * 100.0f)) % 100);

                state = STATE_MONITORING;
                telemetry_snapshot.imu_state = (uint8_t)STATE_MONITORING;
                break;
            }

            case STATE_MONITORING: {
                telemetry_snapshot.imu_state = (uint8_t)STATE_MONITORING;
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);

                // Extremely fast squared magnitudes (No floats or trigonometry)
                float total_g_sq = SQ(gx) + SQ(gy) + SQ(gz);

                float total_g = imu_sqrtf(total_g_sq);
                float raw_pitch = 0.0f, raw_roll = 0.0f;
                imu_compute_pitch_roll(gx, gy, gz, &raw_pitch, &raw_roll);

                // Calibrated pitch and roll (0.0 deg on flat ground)
                float pitch = raw_pitch - baseline_pitch;
                float roll  = raw_roll  - baseline_roll;

                // tm_printf((UB *)"[IMU] Pitch: %4d | Roll: %4d | Total: %d.%02d g\n",
                //             (INT)pitch, (INT)roll, (INT)total_g, (INT)(fabsf(total_g * 100.0f)) % 100);

                int16_t heading = imu_get_heading_deg();
                (void)heading;

                BOOL is_tilted = (fabsf(roll) >= HUMP_TILT_THRESHOLD_DEG || fabsf(pitch) >= HUMP_TILT_THRESHOLD_DEG);

                if (is_tilted) {
                    // Geometric height:
                    // For single-sided ramp (left/right wheel), roll is dominant -> scale by track width (100mm).
                    // For full-width ramp, pitch is dominant -> scale by chassis wheelbase (150mm).
                    const float DEG_TO_RAD = 3.14159265f / 180.0f;
                    float h_roll = HUMP_TRACK_WIDTH_MM * (fabsf(roll) * DEG_TO_RAD);
                    float h_pitch = HUMP_CHASSIS_SCALE_MM * (fabsf(pitch) * DEG_TO_RAD);
                    float current_height_mm = (h_roll > h_pitch) ? h_roll : h_pitch;

                    if (current_height_mm > hump_peak_height) {
                        hump_peak_height = current_height_mm;
                    }

                    IMU_TELEMETRY_MSG msg = {
                        .event = IMU_MSG_HUMP_DETECTED,
                        .pitch_deg = pitch,
                        .roll_deg = roll,
                        .estimated_height_mm = current_height_mm,
                    };

                    NAVIGATOR_MESSAGE n_msg = {
                        .type = MSG_IMU_UPDATE,
                        .data.imu_telemetry_msg = msg,
                    };

                    tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                    telemetry_snapshot.pitch_deg = pitch;
                    telemetry_snapshot.roll_deg = roll;
                    telemetry_snapshot.estimated_height_mm = current_height_mm;
                    telemetry_snapshot.imu_event = IMU_MSG_HUMP_DETECTED;
                    telemetry_snapshot.imu_state = (uint8_t)state;

                    if (no_hump_sent) no_hump_sent = 0;
                } else {
                    if (!no_hump_sent) {
                        // Returned back down to ground: report the final calculated peak height!
                        float final_peak_height = hump_peak_height;

                        IMU_TELEMETRY_MSG msg = {
                            .event = IMU_MSG_NO_HUMP,
                            .pitch_deg = pitch,
                            .roll_deg = roll,
                            .estimated_height_mm = final_peak_height,
                        };

                        NAVIGATOR_MESSAGE n_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg,
                        };

                        tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);

                        tm_printf((UB *)"[IMU HUMP] Returned to ground. Final Peak Height: %d mm\n", (INT)final_peak_height);

                        hump_peak_height = 0.0f;
                        vertical_velocity = 0.0f;
                        telemetry_snapshot.pitch_deg = pitch;
                        telemetry_snapshot.roll_deg = roll;
                        telemetry_snapshot.estimated_height_mm = 0.0f;
                        telemetry_snapshot.imu_event = IMU_MSG_NO_HUMP;
                        telemetry_snapshot.imu_state = (uint8_t)state;

                        no_hump_sent = 1;
                    }
                }

                // Record telemetry every 500ms
                if (++telemetry_timer >= 5) {
                    telemetry_timer = 0;
                    telemetry_snapshot.pitch_deg = pitch;
                    telemetry_snapshot.roll_deg = roll;
                    telemetry_snapshot.imu_state = (uint8_t)state;
                    if (!is_tilted) {
                        telemetry_snapshot.imu_event = IMU_MSG_NONE;
                    }
                }

                // Priority 1: Shock Detection 
                if (total_g_sq > SHOCK_G_SQ) {
                    state = STATE_SHOCK_DETECTED;
                    break;
                }

                // Priority 2: Tilt Warning (Pitch or Roll > 30 deg)
                if (fabsf(pitch) > SAFE_TILT_LIMIT_DEG || fabsf(roll) > SAFE_TILT_LIMIT_DEG) {
                    state = STATE_TILT_WARNING;
                    break;
                }

                break;
            }

            case STATE_TILT_WARNING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);
                
                // Allow expensive math here because the car is likely stopped/recovering anyway
                float total_g = imu_sqrtf(SQ(gx) + SQ(gy) + SQ(gz));
                float raw_pitch = 0.0f, raw_roll = 0.0f;
                imu_compute_pitch_roll(gx, gy, gz, &raw_pitch, &raw_roll);
                float pitch = raw_pitch - baseline_pitch;
                float roll  = raw_roll  - baseline_roll;

                IMU_TELEMETRY_MSG msg = {
                    .event = IMU_MSG_TILT_WARNING,
                    .pitch_deg = pitch,
                    .roll_deg = roll,
                    .peak_g = total_g
                };
                if (navigator_mbfid > E_OK) {
                    telemetry_snapshot.pitch_deg = pitch;
                    telemetry_snapshot.roll_deg = roll;
                    telemetry_snapshot.imu_state = (uint8_t)state;
                    telemetry_snapshot.imu_event = IMU_MSG_TILT_WARNING;
                    NAVIGATOR_MESSAGE nav_msg = { .type = MSG_IMU_UPDATE, .data.imu_telemetry_msg = msg };
                    (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                }

                if (fabsf(pitch) < (SAFE_TILT_LIMIT_DEG - 5.0f) && fabsf(roll) < (SAFE_TILT_LIMIT_DEG - 5.0f)) {
                    state = STATE_MONITORING;
                }
                break;
            }

            case STATE_SHOCK_DETECTED: {
                motor_estop = TRUE;
                telemetry_snapshot.imu_state = (uint8_t)state;
                telemetry_snapshot.imu_event = IMU_MSG_SHOCK_DETECTED;

                IMU_TELEMETRY_MSG msg = {
                    .event = IMU_MSG_SHOCK_DETECTED,
                    .peak_g = SHOCK_THRESHOLD_G
                };
                if (navigator_mbfid > E_OK) {
                    NAVIGATOR_MESSAGE nav_msg = { .type = MSG_IMU_UPDATE, .data.imu_telemetry_msg = msg };
                    (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                }
                while (1) tk_dly_tsk(1000);
                break;
            }
        }

        // Fixed from tk_dly_tsk(100) -> 10ms for true 100Hz timing
        tk_dly_tsk(100);
    }
}