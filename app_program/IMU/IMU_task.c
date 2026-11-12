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

#if (IMU_SDA_PIN % 4) != 0 || IMU_SDA_PIN > 20 || IMU_SCL_PIN != (IMU_SDA_PIN + 1)
#error "IMU pins must be an RP2040 I2C0 SDA/SCL pair: GP0/1, GP4/5, GP8/9, GP12/13, GP16/17 or GP20/21"
#endif

extern ID navigator_mbfid;

typedef enum {
    STATE_INITIALIZE,
    STATE_CALIBRATING,
    STATE_MONITORING,
    STATE_LIFTED_OVERTURNED
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

    // Correctly sign-extend 12-bit signed data from 16-bit register pairs
    int16_t raw_x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    *x = raw_x >> 4;
    *y = raw_y >> 4;
    *z = raw_z >> 4;
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

// IMU mounted 90 degrees rotated on the car:
// Sensor Y axis = car forward/backward (Pitch)
// Sensor X axis = car left/right (Roll)
LOCAL void imu_compute_pitch_roll(float gx, float gy, float gz, float *pitch_out, float *roll_out)
{
    *pitch_out = imu_atan2f(gy, imu_sqrtf(SQ(gx) + SQ(gz))) * 180.0f / 3.14159f;
    *roll_out  = imu_atan2f(gx, imu_sqrtf(SQ(gy) + SQ(gz))) * 180.0f / 3.14159f;
}

LOCAL bool imu_read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz)
{
    UB data[6];
    SZ asz = 0;
    UB reg = OUT_X_H_M; // 0x03

    if (tk_swri_dev(imu_dd, LSM303_MAG_ADDR, &reg, 1, &asz) < E_OK) return false;
    if (tk_srea_dev(imu_dd, LSM303_MAG_ADDR, data, 6, &asz) < E_OK) return false;

    // On LSM303DLHC, magnetometer register order is X, Z, Y (big-endian)
    *mx = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *mz = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    *my = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    return true;
}

// Returns compass heading in degrees (0 to 359)
LOCAL int16_t imu_get_heading_deg(void)
{
    int16_t mx = 0, my = 0, mz = 0;
    if (!imu_read_mag_raw(&mx, &my, &mz)) return telemetry_snapshot.heading_deg;

    // IMU mounted with +Y pointing backward (-Y = car forward) and +X pointing left:
    // North along forward = -my, East/90 deg CW puts North along left (+mx) -> atan2(+mx, -my)
    float rad = imu_atan2f((float)mx, -(float)my);
    float deg = rad * 180.0f / 3.14159265f;

    if (deg < 0.0f) deg += 360.0f;
    if (deg >= 360.0f) deg -= 360.0f;
    return (int16_t)deg;
}

/* Return median of 3 float samples to reject single-sample vibration/drop spikes */
LOCAL float imu_median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { b = a; }
    return b;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;

    IMU_FSM_STATE state = STATE_INITIALIZE;
    float baseline_z = 1.0f;
    float baseline_pitch = 0.0f;
    float baseline_roll = 0.0f;
    float hump_peak_height = 0.0f;
    INT telemetry_timer = 0;

    // Median-of-3 + EMA smoothing state and precomputed acceleration windows around baseline_g
    float p_hist[3] = {0.0f, 0.0f, 0.0f};
    float r_hist[3] = {0.0f, 0.0f, 0.0f};
    INT hist_idx = 0;
    float filt_pitch = 0.0f;
    float filt_roll = 0.0f;
    float lift_low_g_sq = SQ(1.0f - LIFT_ACCEL_DEV_G);
    float lift_high_g_sq = SQ(1.0f + LIFT_ACCEL_DEV_G);
    float static_low_g_sq = SQ(1.0f - 0.15f);
    float static_high_g_sq = SQ(1.0f + 0.15f);

    BOOL hump_active = FALSE;
    INT tilt_debounce = 0;
    INT safety_debounce = 0;

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
                }

                UW accel_addr = LSM303_ACCEL_ADDR;
                ER er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                if (er1 < E_OK) {
                    accel_addr = 0x18;
                    er1 = imu_write_reg(accel_addr, CTRL_REG1_A, 0x57);
                }

                ER er2 = (er1 >= E_OK) ? imu_write_reg(accel_addr, CTRL_REG4_A, 0x08) : er1;

                // Configure LSM303DLHC Magnetometer for compass heading
                (void)imu_write_reg(LSM303_MAG_ADDR, CRA_REG_M, 0x14); // 30 Hz data rate
                (void)imu_write_reg(LSM303_MAG_ADDR, CRB_REG_M, 0x20); // +/- 1.3 Gauss range
                (void)imu_write_reg(LSM303_MAG_ADDR, MR_REG_M,  0x00); // Continuous measurement mode

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
                tm_printf((UB *)"[IMU] Calibrating zero on flat ground...\n");
                tk_dly_tsk(200); // Allow sensor ADC & filters to stabilize after power-on

                float sum_z = 0.0f;
                float sum_g = 0.0f;
                float sum_pitch = 0.0f;
                float sum_roll = 0.0f;
                int valid_samples = 0;
                int attempts = 0;

                while (valid_samples < 50 && attempts < 100) {
                    int16_t rx, ry, rz;
                    if (imu_read_accel_raw(&rx, &ry, &rz)) {
                        float gx = (float)rx * 0.001f;
                        float gy = (float)ry * 0.001f;
                        float gz = (float)rz * 0.001f;
                        float raw_p, raw_r;
                        imu_compute_pitch_roll(gx, gy, gz, &raw_p, &raw_r);
                        sum_z += gz;
                        sum_g += imu_sqrtf(SQ(gx) + SQ(gy) + SQ(gz));
                        sum_pitch += raw_p;
                        sum_roll += raw_r;
                        valid_samples++;
                    }
                    attempts++;
                    tk_dly_tsk(10);
                }

                float baseline_g = 1.0f;
                if (valid_samples > 0) {
                    baseline_z = sum_z / (float)valid_samples;
                    baseline_g = sum_g / (float)valid_samples;
                    baseline_pitch = sum_pitch / (float)valid_samples;
                    baseline_roll = sum_roll / (float)valid_samples;
                } else {
                    baseline_z = 1.0f;
                    baseline_pitch = 0.0f;
                    baseline_roll = 0.0f;
                }

                float lift_low = baseline_g - LIFT_ACCEL_DEV_G;
                if (lift_low < 0.0f) lift_low = 0.0f;
                lift_low_g_sq = SQ(lift_low);
                lift_high_g_sq = SQ(baseline_g + LIFT_ACCEL_DEV_G);

                float low_g = baseline_g - 0.15f;
                if (low_g < 0.0f) low_g = 0.0f;
                static_low_g_sq = SQ(low_g);
                static_high_g_sq = SQ(baseline_g + 0.15f);

                for (INT i = 0; i < 3; i++) {
                    p_hist[i] = 0.0f;
                    r_hist[i] = 0.0f;
                }
                hist_idx = 0;
                filt_pitch = 0.0f;
                filt_roll = 0.0f;

                tm_printf((UB *)"[IMU] Calibrated! Offsets -> P:%d R:%d (samples:%d)\n",
                          (INT)baseline_pitch, (INT)baseline_roll, valid_samples);
                state = STATE_MONITORING;
                break;
            }

            case STATE_MONITORING: {
                float gx, gy, gz;
                imu_read_accel_g(&gx, &gy, &gz);

                // Extremely fast squared magnitudes
                float total_g_sq = SQ(gx) + SQ(gy) + SQ(gz);

                float raw_pitch, raw_roll;
                imu_compute_pitch_roll(gx, gy, gz, &raw_pitch, &raw_roll);

                // Zero-calibrated Pitch and Roll (reads 0 on flat ground)
                float pitch = raw_pitch - baseline_pitch;
                float roll  = raw_roll  - baseline_roll;

                // Spike rejection (median-of-3) + EMA low-pass filter for smooth height estimation
                p_hist[hist_idx] = pitch;
                r_hist[hist_idx] = roll;
                hist_idx = (hist_idx + 1) % 3;

                float med_pitch = imu_median3(p_hist[0], p_hist[1], p_hist[2]);
                float med_roll  = imu_median3(r_hist[0], r_hist[1], r_hist[2]);
                filt_pitch += 0.22f * (med_pitch - filt_pitch);
                filt_roll  += 0.22f * (med_roll  - filt_roll);

                BOOL is_quasi_static = (total_g_sq >= static_low_g_sq && total_g_sq <= static_high_g_sq);
                float current_height_mm = 0.0f;
                BOOL is_tilted = (fabsf(filt_roll) >= HUMP_TILT_THRESHOLD_DEG ||
                                  fabsf(filt_pitch) >= HUMP_TILT_THRESHOLD_DEG);

                if (is_tilted) {
                    // Geometric height using smoothed calibrated tilt angles:
                    const float DEG_TO_RAD = 3.14159265f / 180.0f;
                    float sin_roll  = fabsf(filt_roll) * DEG_TO_RAD;
                    float sin_pitch = fabsf(filt_pitch) * DEG_TO_RAD;
                    if (sin_roll > 1.0f)  sin_roll = 1.0f;
                    if (sin_pitch > 1.0f) sin_pitch = 1.0f;

                    float h_roll  = HUMP_TRACK_WIDTH_MM * sin_roll;
                    float h_pitch = HUMP_CHASSIS_SCALE_MM * sin_pitch;
                    current_height_mm = (h_roll > h_pitch) ? h_roll : h_pitch;

                    telemetry_snapshot.estimated_height_mm = current_height_mm;

                    // Debounce: require tilt to persist for 4 consecutive samples (80 ms)
                    // and notify Navigator ONCE on entering the hump (no queue flooding)
                    if (++tilt_debounce >= 4) {
                        tilt_debounce = 4;
                        telemetry_snapshot.imu_event = IMU_MSG_HUMP_DETECTED;
                        if (!hump_active) {
                            hump_active = TRUE;
                            IMU_TELEMETRY_MSG msg = {
                                .event = IMU_MSG_HUMP_DETECTED,
                                .pitch_deg = filt_pitch,
                                .roll_deg = filt_roll,
                                .estimated_height_mm = current_height_mm,
                            };
                            NAVIGATOR_MESSAGE n_msg = {
                                .type = MSG_IMU_UPDATE,
                                .data.imu_telemetry_msg = msg,
                            };
                            tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                            tm_printf((UB *)"[IMU HUMP] Detected! Tilt P:%d R:%d\n", (INT)filt_pitch, (INT)filt_roll);
                        }
                    }

                    // Update cumulative peak only after hump is confirmed and acceleration is quasi-static
                    if (hump_active && is_quasi_static && current_height_mm > hump_peak_height) {
                        hump_peak_height = current_height_mm;
                    }
                } else {
                    // Always reset live height telemetry when flat (prevents stale height after short spikes)
                    telemetry_snapshot.estimated_height_mm = 0.0f;

                    if (tilt_debounce > 0) tilt_debounce--;
                    if (tilt_debounce == 0 && hump_active) {
                        hump_active = FALSE;
                        float final_peak_height = hump_peak_height;

                        IMU_TELEMETRY_MSG msg = {
                            .event = IMU_MSG_NO_HUMP,
                            .pitch_deg = filt_pitch,
                            .roll_deg = filt_roll,
                            .estimated_height_mm = final_peak_height,
                        };
                        NAVIGATOR_MESSAGE n_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg,
                        };
                        tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                        tm_printf((UB *)"[IMU HUMP] Returned to flat. Peak:%dmm\n", (INT)final_peak_height);

                        // Keep hump_peak_height cumulative across all humps/stages; reset live height only
                        telemetry_snapshot.imu_event = IMU_MSG_NO_HUMP;
                    }
                }

                // Record telemetry every 500ms (single compact serial log <= 32B FIFO)
                if (++telemetry_timer >= 25) {
                    telemetry_timer = 0;
                    telemetry_snapshot.pitch_deg = filt_pitch;
                    telemetry_snapshot.roll_deg = filt_roll;
                    telemetry_snapshot.heading_deg = imu_get_heading_deg();
                    telemetry_snapshot.imu_state = (uint8_t)state;
                    if (!is_tilted) {
                        telemetry_snapshot.estimated_height_mm = 0.0f;
                        telemetry_snapshot.imu_event = IMU_MSG_NONE;
                    }
                    tm_printf((UB *)"[IMU] Pitch:%d Roll:%d H:%dmm\n",
                              (INT)filt_pitch, (INT)filt_roll, (INT)current_height_mm);
                }

                // Priority: Lifted or Overturned Detection -> immediate motor stop
                // Require 4 samples (80ms) so brief 1cm ramp bumps/drops (<=45ms) never trigger a false lift stop
                BOOL is_overturned = (fabsf(pitch) > OVERTURN_TILT_DEG) ||
                                     (fabsf(roll) > OVERTURN_TILT_DEG) ||
                                     ((gz * baseline_z) < (0.5f * SQ(baseline_z)));
                BOOL is_lifted = (total_g_sq > lift_high_g_sq) || (total_g_sq < lift_low_g_sq);

                if (is_overturned || is_lifted) {
                    if (++safety_debounce >= 4) {
                        motor_estop = TRUE;
                        telemetry_snapshot.pitch_deg = pitch;
                        telemetry_snapshot.roll_deg = roll;
                        state = STATE_LIFTED_OVERTURNED;
                        break;
                    }
                } else if (safety_debounce > 0) {
                    safety_debounce--;
                }

                break;
            }

            case STATE_LIFTED_OVERTURNED: {
                motor_estop = TRUE;
                telemetry_snapshot.imu_state = (uint8_t)state;
                telemetry_snapshot.imu_event = IMU_MSG_LIFTED_OVERTURNED;

                if (navigator_mbfid > E_OK) {
                    IMU_TELEMETRY_MSG msg = {
                        .event = IMU_MSG_LIFTED_OVERTURNED,
                        .pitch_deg = telemetry_snapshot.pitch_deg,
                        .roll_deg = telemetry_snapshot.roll_deg,
                    };
                    NAVIGATOR_MESSAGE nav_msg = {
                        .type = MSG_IMU_UPDATE,
                        .data.imu_telemetry_msg = msg,
                    };
                    (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                }
                tm_printf((UB *)"[IMU] LIFTED/OVERTURNED -> STOP\n");
                while (1) tk_dly_tsk(1000);
                break;
            }
        }

        // 20ms period (50Hz sampling)
        tk_dly_tsk(2);
    }
}