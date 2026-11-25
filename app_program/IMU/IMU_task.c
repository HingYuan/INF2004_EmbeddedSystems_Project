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

// --- Magnetometer (LSM303DLHC at 0x1E) ---
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

LOCAL ER imu_write_reg(UB addr, UB reg, UB val)
{
    UB data[2] = {reg, val};
    SZ asz = 0;
    ER er = tk_swri_dev(imu_dd, addr, data, sizeof(data), &asz);
    return (er < E_OK) ? er : (asz == sizeof(data) ? E_OK : E_IO);
}

LOCAL UW active_accel_addr = LSM303_ACCEL_ADDR;

LOCAL bool imu_read_accel_raw(H *x, H *y, H *z)
{
    UB data[6];
    SZ asz = 0;
    // 0xA8 is OUT_X_L_A with the MSB set for auto-increment.
    UB reg = 0xA8; 

    // Single burst write of the register address
    if (tk_swri_dev(imu_dd, active_accel_addr, &reg, 1, &asz) < E_OK || asz != 1)
        return false;
    // Single burst read of all 6 bytes (saves 5 context switches into the OS kernel)
    asz = 0;
    ER er = tk_srea_dev(imu_dd, active_accel_addr, data, 6, &asz);
    if (er < E_OK || asz != 6) return false;

    // Correctly sign-extend 12-bit signed data from 16-bit register pairs
    H raw_x = (H)(((UH)data[1] << 8) | data[0]);
    H raw_y = (H)(((UH)data[3] << 8) | data[2]);
    H raw_z = (H)(((UH)data[5] << 8) | data[4]);

    *x = raw_x >> 4;
    *y = raw_y >> 4;
    *z = raw_z >> 4;
    return true;
}

static bool imu_read_accel_g(float *gx, float *gy, float *gz)
{
    H rx = 0, ry = 0, rz = 0;
    if (imu_read_accel_raw(&rx, &ry, &rz)) {
        *gx = (float)rx * 0.001f;
        *gy = (float)ry * 0.001f;
        *gz = (float)rz * 0.001f;
        return true;
    }
    return false;
}

// IMU mounted 90 degrees rotated on the car:
// Sensor Y axis = car forward/backward (Pitch)
// Sensor X axis = car left/right (Roll)
LOCAL void imu_compute_pitch_roll(float gx, float gy, float gz, float *pitch_out, float *roll_out)
{
    *pitch_out = imu_atan2f(gy, imu_sqrtf(SQ(gx) + SQ(gz))) * 180.0f / 3.14159f;
    *roll_out  = imu_atan2f(gx, imu_sqrtf(SQ(gy) + SQ(gz))) * 180.0f / 3.14159f;
}

LOCAL bool imu_read_mag_raw(H *mx, H *my, H *mz)
{
    UB data[6];
    SZ asz = 0;
    UB reg = OUT_X_H_M; // 0x03

    if (tk_swri_dev(imu_dd, LSM303_MAG_ADDR, &reg, 1, &asz) < E_OK || asz != 1)
        return false;
    asz = 0;
    if (tk_srea_dev(imu_dd, LSM303_MAG_ADDR, data, 6, &asz) < E_OK || asz != 6)
        return false;

    // On LSM303DLHC, magnetometer register order is X, Z, Y (big-endian)
    *mx = (H)(((UH)data[0] << 8) | data[1]);
    *mz = (H)(((UH)data[2] << 8) | data[3]);
    *my = (H)(((UH)data[4] << 8) | data[5]);
    return true;
}

// Returns compass heading in degrees (0 to 359)
LOCAL H imu_get_heading_deg(void)
{
    H mx = 0, my = 0, mz = 0;
    if (!imu_read_mag_raw(&mx, &my, &mz)) return telemetry_heading();

    // IMU mounted with +Y pointing backward (-Y = car forward) and +X pointing left:
    // North along forward = -my, East/90 deg CW puts North along left (+mx) -> atan2(+mx, -my)
    float rad = imu_atan2f((float)mx, -(float)my);
    float deg = rad * 180.0f / 3.14159265f;

    if (deg < 0.0f) deg += 360.0f;
    if (deg >= 360.0f) deg -= 360.0f;
    return (H)deg;
}

/* Return median of 3 float samples to reject single-sample vibration/drop spikes */
LOCAL float imu_median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { b = a; }
    return b;
}

LOCAL BOOL imu_confirm(IMU_CONFIRM *check, BOOL condition, UW now, UW duration)
{
    if (!condition) {
        check->active = FALSE;
        return FALSE;
    }
    if (!check->active || (UW)(now - check->last_us) > IMU_SAMPLE_GAP_US) {
        check->since_us = now;
        check->active = TRUE;
    }
    check->last_us = now;
    return (UW)(now - check->since_us) >= duration;
}

LOCAL IMU_STOP_REASON imu_safety_reason(float pitch, float roll, float z,
    float total_g_sq, float baseline_z, float low_g_sq, float high_g_sq, BOOL quasi_static)
{
    if (total_g_sq < low_g_sq) return IMU_STOP_LOW_G;
    if (total_g_sq > high_g_sq) return IMU_STOP_HIGH_G;
    // Acceleration during a bump/brake is not a trustworthy gravity direction.
    if (!quasi_static) return IMU_STOP_NONE;
    if (z * baseline_z < -0.2f * SQ(baseline_z)) return IMU_STOP_INVERTED;
    if (fabsf(pitch) > OVERTURN_TILT_DEG || fabsf(roll) > OVERTURN_TILT_DEG)
        return IMU_STOP_TILT;
    return IMU_STOP_NONE;
}

LOCAL BOOL imu_safety_confirm(IMU_CONFIRM *check, IMU_STOP_REASON *previous,
                              IMU_STOP_REASON reason, UW now)
{
    // A tilt, a low-g dip and a high-g impact cannot add to each other's timer.
    if (reason != *previous) check->active = FALSE;
    *previous = reason;
    UW duration = (reason == IMU_STOP_LOW_G || reason == IMU_STOP_HIGH_G)
                  ? IMU_LIFT_CONFIRM_US : IMU_TILT_CONFIRM_US;
    return imu_confirm(check, reason != IMU_STOP_NONE, now, duration);
}

#ifndef NDEBUG
LOCAL BOOL imu_detector_self_check(void)
{
    IMU_CONFIRM check = {0};
    IMU_STOP_REASON previous = IMU_STOP_NONE;
    BOOL ok = imu_safety_reason(0, 0, 0.4f, 0.16f, 1, 0.0625f, 4, FALSE) == IMU_STOP_NONE &&
              imu_safety_reason(0, 0, -1, 1, 1, 0.0625f, 4, TRUE) == IMU_STOP_INVERTED &&
              imu_safety_reason(OVERTURN_TILT_DEG + 5, 0, 0.866f, 1, 1, 0.0625f, 4, TRUE) == IMU_STOP_TILT &&
              imu_safety_reason(OVERTURN_TILT_DEG + 5, 0, 1, 2, 1, 0.0625f, 4, FALSE) == IMU_STOP_NONE &&
              imu_safety_reason(0, 0, 0, 0, 1, 0.0625f, 4, FALSE) == IMU_STOP_LOW_G;
    for (UW t = 0; t <= IMU_TILT_CONFIRM_US; t += 20000U) {
        BOOL stop = imu_safety_confirm(&check, &previous, IMU_STOP_TILT, t);
        ok = ok && (stop == (t == IMU_TILT_CONFIRM_US));
    }
    ok = ok && !imu_safety_confirm(&check, &previous, IMU_STOP_NONE, 220000U) &&
         !imu_safety_confirm(&check, &previous, IMU_STOP_TILT, 240000U) &&
         !imu_safety_confirm(&check, &previous, IMU_STOP_INVERTED, 260000U);
    check.active = FALSE;
    ok = ok && !imu_confirm(&check, TRUE, 0xffff0000U, IMU_LIFT_CONFIRM_US) &&
         imu_confirm(&check, TRUE, (UW)(0xffff0000U + IMU_LIFT_CONFIRM_US), IMU_LIFT_CONFIRM_US);
    return ok;
}
#endif

EXPORT void imu_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;

#ifndef NDEBUG
    if (!imu_detector_self_check()) {
        robot_flag_assign(ROBOT_ESTOP_REQUEST, TRUE);
        tm_printf((UB *)"[IMU] Detector self-check failed\n");
        return;
    }
#endif

    IMU_RUNTIME_STATE imu = {.state = STATE_INITIALIZE};
    float baseline_z = 1.0f;
    float baseline_pitch = 0.0f;
    float baseline_roll = 0.0f;
    float hump_peak_height = 0.0f;
    UW telemetry_us = time_us();
    UW last_valid_us = telemetry_us;

    // Median-of-3 + EMA smoothing state and precomputed acceleration windows around baseline_g
    float p_hist[3] = {0.0f, 0.0f, 0.0f};
    float r_hist[3] = {0.0f, 0.0f, 0.0f};
    float z_hist[3] = {1.0f, 1.0f, 1.0f};
    float filt_pitch = 0.0f;
    float filt_roll = 0.0f;
    float lift_low_g_sq = SQ(IMU_LOW_G_RATIO);
    float lift_high_g_sq = SQ(1.0f + LIFT_ACCEL_DEV_G);
    float static_low_g_sq = SQ(1.0f - IMU_STATIC_DEV_G);
    float static_high_g_sq = SQ(1.0f + IMU_STATIC_DEV_G);

    IMU_CONFIRM tilt_check = {0}, flat_check = {0}, safety_check = {0}, recovery_check = {0};

    while (1) {
        UW now = time_us();
        float gx = 0, gy = 0, gz = 0, total_g_sq = 0, pitch = 0, roll = 0;
        float med_pitch = 0, med_roll = 0, med_z = baseline_z;
        if (imu.state == STATE_LIFTED_OVERTURNED) robot_flag_assign(ROBOT_ESTOP_REQUEST, TRUE);
        if (imu.state == STATE_MONITORING || imu.state == STATE_LIFTED_OVERTURNED) {
            BOOL valid = imu_read_accel_g(&gx, &gy, &gz);
            now = time_us();
            if (!valid) {
                tilt_check.active = flat_check.active = safety_check.active = recovery_check.active = FALSE;
                if ((UW)(now - last_valid_us) >= IMU_READ_FAULT_US) {
                    robot_flags_update(ROBOT_ESTOP_REQUEST | ROBOT_FAULT_IMU_READ, 0U);
                    if (!imu.read_fault) tm_printf((UB *)"[IMU] SENSOR READ FAULT -> STOP\n");
                    imu.read_fault = TRUE;
                }
                tk_dly_tsk(IMU_SAMPLE_PERIOD_MS);
                continue; // Never turn a failed read into a fictitious flat sample.
            }
            if (imu.read_fault) {
                robot_flags_update(0U, ROBOT_FAULT_IMU_READ);
                tm_printf((UB *)"[IMU] Reads recovered; stop stays latched\n");
            }
            imu.read_fault = FALSE;
            BOOL sample_gap = (UW)(now - last_valid_us) > IMU_SAMPLE_GAP_US;
            last_valid_us = now;
            total_g_sq = SQ(gx) + SQ(gy) + SQ(gz);
            imu_compute_pitch_roll(gx, gy, gz, &pitch, &roll);
            pitch -= baseline_pitch;
            roll -= baseline_roll;
            if (sample_gap) {
                for (INT i = 0; i < 3; i++) {
                    p_hist[i] = pitch;
                    r_hist[i] = roll;
                    z_hist[i] = gz;
                }
                filt_pitch = pitch;
                filt_roll = roll;
            }
            p_hist[imu.hist_idx] = pitch;
            r_hist[imu.hist_idx] = roll;
            z_hist[imu.hist_idx] = gz;
            imu.hist_idx = (imu.hist_idx + 1) % 3;
            med_pitch = imu_median3(p_hist[0], p_hist[1], p_hist[2]);
            med_roll = imu_median3(r_hist[0], r_hist[1], r_hist[2]);
            med_z = imu_median3(z_hist[0], z_hist[1], z_hist[2]);
            filt_pitch += 0.22f * (med_pitch - filt_pitch);
            filt_roll += 0.22f * (med_roll - filt_roll);
        }
        switch (imu.state) {
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

                // BDU holds each acceleration byte pair stable while the burst is read.
                ER er2 = (er1 >= E_OK) ? imu_write_reg(accel_addr, CTRL_REG4_A, 0x88) : er1;

                // Configure LSM303DLHC Magnetometer for compass heading
                (void)imu_write_reg(LSM303_MAG_ADDR, CRA_REG_M, 0x14); // 30 Hz data rate
                (void)imu_write_reg(LSM303_MAG_ADDR, CRB_REG_M, 0x20); // +/- 1.3 Gauss range
                (void)imu_write_reg(LSM303_MAG_ADDR, MR_REG_M,  0x00); // Continuous measurement mode

                if (er1 >= E_OK && er2 >= E_OK) {
                    active_accel_addr = accel_addr;
                    tm_printf((UB *)"[IMU STATUS] CONNECTED!\n");
                    imu.state = STATE_CALIBRATING;
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
                    H rx, ry, rz;
                    if (imu_read_accel_raw(&rx, &ry, &rz)) {
                        float gx = (float)rx * 0.001f;
                        float gy = (float)ry * 0.001f;
                        float gz = (float)rz * 0.001f;
                        float raw_p, raw_r;
                        imu_compute_pitch_roll(gx, gy, gz, &raw_p, &raw_r);
                        float g = imu_sqrtf(SQ(gx) + SQ(gy) + SQ(gz));
                        if (g >= 0.8f && g <= 1.2f && fabsf(raw_p) < 20.0f && fabsf(raw_r) < 20.0f) {
                            sum_z += gz;
                            sum_g += g;
                            sum_pitch += raw_p;
                            sum_roll += raw_r;
                            valid_samples++;
                        } else {
                            // Restart the baseline window if the car moves during calibration.
                            valid_samples = 0;
                            sum_z = sum_g = sum_pitch = sum_roll = 0.0f;
                        }
                    }
                    attempts++;
                    tk_dly_tsk(10);
                }

                if (valid_samples != 50) {
                    tm_printf((UB *)"[IMU] Calibration incomplete; keep flat, retrying\n");
                    break;
                }
                baseline_z = sum_z / (float)valid_samples;
                float baseline_g = sum_g / (float)valid_samples;
                baseline_pitch = sum_pitch / (float)valid_samples;
                baseline_roll = sum_roll / (float)valid_samples;

                float lift_low = baseline_g - LIFT_ACCEL_DEV_G;
                if (lift_low < baseline_g * IMU_LOW_G_RATIO) lift_low = baseline_g * IMU_LOW_G_RATIO;
                lift_low_g_sq = SQ(lift_low);
                lift_high_g_sq = SQ(baseline_g + LIFT_ACCEL_DEV_G);

                float low_g = baseline_g - IMU_STATIC_DEV_G;
                if (low_g < 0.0f) low_g = 0.0f;
                static_low_g_sq = SQ(low_g);
                static_high_g_sq = SQ(baseline_g + IMU_STATIC_DEV_G);

                for (INT i = 0; i < 3; i++) {
                    p_hist[i] = 0.0f;
                    r_hist[i] = 0.0f;
                    z_hist[i] = baseline_z;
                }
                imu.hist_idx = 0;
                filt_pitch = 0.0f;
                filt_roll = 0.0f;
                last_valid_us = telemetry_us = time_us();

                tm_printf((UB *)"[IMU] Calibrated! Offsets -> P:%d R:%d (samples:%d)\n",
                          (INT)baseline_pitch, (INT)baseline_roll, valid_samples);
                imu.state = STATE_MONITORING;
                break;
            }

            case STATE_MONITORING: {
                BOOL is_quasi_static = (total_g_sq >= static_low_g_sq && total_g_sq <= static_high_g_sq);
                float current_height_mm = 0.0f;
                BOOL is_tilted = (fabsf(filt_roll) >= HUMP_TILT_THRESHOLD_DEG ||
                                  fabsf(filt_pitch) >= HUMP_TILT_THRESHOLD_DEG);

                if (is_tilted) {
                    flat_check.active = FALSE;
                    // Geometric height using smoothed calibrated tilt angles:
                    const float DEG_TO_RAD = 3.14159265f / 180.0f;
                    float sin_roll  = fabsf(filt_roll) * DEG_TO_RAD;
                    float sin_pitch = fabsf(filt_pitch) * DEG_TO_RAD;
                    if (sin_roll > 1.0f)  sin_roll = 1.0f;
                    if (sin_pitch > 1.0f) sin_pitch = 1.0f;

                    float h_roll  = HUMP_TRACK_WIDTH_MM * sin_roll;
                    float h_pitch = HUMP_CHASSIS_SCALE_MM * sin_pitch;
                    current_height_mm = (h_roll > h_pitch) ? h_roll : h_pitch;

                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.height_dmm = robot_fixed10(current_height_mm, 0, 680);
                        robot_data_unlock(saved);
                    }

                    // Confirm continuous tilt and notify Navigator once per hump.
                    if (imu_confirm(&tilt_check, TRUE, now, IMU_HUMP_CONFIRM_US)) {
                        {
                            UINT saved = robot_data_lock();
                            telemetry_snapshot.imu_event = IMU_MSG_HUMP_DETECTED;
                            robot_data_unlock(saved);
                        }
                        if (!imu.hump_active) {
                            imu.hump_active = TRUE;
                            IMU_TELEMETRY_MSG msg = {
                                .event = IMU_MSG_HUMP_DETECTED,
                                .pitch_ddeg = robot_fixed10(filt_pitch, -1800, 1800),
                                .roll_ddeg = robot_fixed10(filt_roll, -1800, 1800),
                                .height_dmm = robot_fixed10(current_height_mm, 0, 680),
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
                    if (imu.hump_active && is_quasi_static && current_height_mm > hump_peak_height) {
                        hump_peak_height = current_height_mm;
                    }
                } else {
                    // Always reset live height telemetry when flat (prevents stale height after short spikes)
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.height_dmm = robot_fixed10(0.0f, 0, 680);
                        robot_data_unlock(saved);
                    }

                    tilt_check.active = FALSE;
                    if (imu_confirm(&flat_check, TRUE, now, IMU_HUMP_CONFIRM_US) && imu.hump_active) {
                        imu.hump_active = FALSE;
                        float final_peak_height = hump_peak_height;

                        IMU_TELEMETRY_MSG msg = {
                            .event = IMU_MSG_NO_HUMP,
                            .pitch_ddeg = robot_fixed10(filt_pitch, -1800, 1800),
                            .roll_ddeg = robot_fixed10(filt_roll, -1800, 1800),
                            .height_dmm = robot_fixed10(final_peak_height, 0, 680),
                        };
                        NAVIGATOR_MESSAGE n_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg,
                        };
                        tk_snd_mbf(navigator_mbfid, &n_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                        tm_printf((UB *)"[IMU HUMP] Returned to flat. Peak:%dmm\n", (INT)final_peak_height);

                        // Keep hump_peak_height cumulative across all humps/stages; reset live height only
                        {
                            UINT saved = robot_data_lock();
                            telemetry_snapshot.imu_event = IMU_MSG_NO_HUMP;
                            robot_data_unlock(saved);
                        }
                    }
                }

                if ((UW)(now - telemetry_us) >= IMU_TELEMETRY_US) {
                    telemetry_us = now;
                    H heading = imu_get_heading_deg();
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.pitch_ddeg = robot_fixed10(filt_pitch, -1800, 1800);
                        telemetry_snapshot.roll_ddeg = robot_fixed10(filt_roll, -1800, 1800);
                        telemetry_snapshot.heading_deg = heading;
                        telemetry_snapshot.imu_state = (UB)imu.state;
                        robot_data_unlock(saved);
                    }
                    if (!is_tilted && !imu.hump_active) {
                        {
                            UINT saved = robot_data_lock();
                            telemetry_snapshot.height_dmm = robot_fixed10(0.0f, 0, 680);
                            telemetry_snapshot.imu_event = IMU_MSG_NONE;
                            robot_data_unlock(saved);
                        }
                    }
                    tm_printf((UB *)"[IMU] Pitch:%d Roll:%d H:%dmm\n",
                              (INT)filt_pitch, (INT)filt_roll, (INT)current_height_mm);
                }

                // Do not let the EMA tail keep confirming a tilt that has already ended.
                float safety_pitch = (fabsf(med_pitch) > OVERTURN_TILT_DEG && med_pitch * filt_pitch > 0)
                                     ? filt_pitch : 0.0f;
                float safety_roll = (fabsf(med_roll) > OVERTURN_TILT_DEG && med_roll * filt_roll > 0)
                                    ? filt_roll : 0.0f;
                IMU_STOP_REASON reason = imu_safety_reason(safety_pitch, safety_roll, med_z,
                    total_g_sq, baseline_z, lift_low_g_sq, lift_high_g_sq, is_quasi_static);
                IMU_STOP_REASON previous_reason = imu.stop_reason;
                BOOL confirmed = imu_safety_confirm(&safety_check, &previous_reason, reason, now);
                imu.stop_reason = previous_reason;
                if (confirmed) {
                    static const char *const reasons[] = {"none", "tilt", "inverted", "low-g", "high-g"};
                    robot_flag_assign(ROBOT_ESTOP_REQUEST, TRUE);
                    imu.hump_active = FALSE;
                    tilt_check.active = flat_check.active = recovery_check.active = FALSE;
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.pitch_ddeg = robot_fixed10(filt_pitch, -1800, 1800);
                        telemetry_snapshot.roll_ddeg = robot_fixed10(filt_roll, -1800, 1800);
                        telemetry_snapshot.height_dmm = robot_fixed10(0.0f, 0, 680);
                        robot_data_unlock(saved);
                    }
                    robot_flags_update(ROBOT_FAULT_IMU_TILT, 0U);
                    imu.state = STATE_LIFTED_OVERTURNED;
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.imu_state = (UB)imu.state;
                        telemetry_snapshot.imu_event = IMU_MSG_LIFTED_OVERTURNED;
                        robot_data_unlock(saved);
                    }

                    if (navigator_mbfid > E_OK) {
                        IMU_TELEMETRY_MSG msg = {
                            .event = IMU_MSG_LIFTED_OVERTURNED,
                            .pitch_ddeg = robot_fixed10(filt_pitch, -1800, 1800),
                            .roll_ddeg = robot_fixed10(filt_roll, -1800, 1800),
                        };
                        NAVIGATOR_MESSAGE nav_msg = {
                            .type = MSG_IMU_UPDATE,
                            .data.imu_telemetry_msg = msg,
                        };
                        (void)tk_snd_mbf(navigator_mbfid, &nav_msg, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
                    }
                    tm_printf((UB *)"[IMU] LIFTED/OVERTURNED -> STOP\n");
                    tm_printf((UB *)"[IMU] Cause=%s held=%dms rawP=%d rawR=%d\n",
                              reasons[reason], (INT)((UW)(now - safety_check.since_us) / 1000U),
                              (INT)pitch, (INT)roll);
                    tm_printf((UB *)"[IMU] Accel mg X=%d Y=%d Z=%d G=%d\n",
                              (INT)(gx * 1000), (INT)(gy * 1000), (INT)(gz * 1000),
                              (INT)(imu_sqrtf(total_g_sq) * 1000));
                    break;
                }

                break;
            }

            case STATE_LIFTED_OVERTURNED: {
                {
                    UINT saved = robot_data_lock();
                    telemetry_snapshot.height_dmm = robot_fixed10(0.0f, 0, 680);
                    telemetry_snapshot.imu_state = (UB)imu.state;
                    telemetry_snapshot.imu_event = IMU_MSG_LIFTED_OVERTURNED;
                    robot_data_unlock(saved);
                }

                if ((UW)(now - telemetry_us) >= IMU_TELEMETRY_US) {
                    telemetry_us = now;
                    H heading = imu_get_heading_deg();
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.pitch_ddeg = robot_fixed10(filt_pitch, -1800, 1800);
                        telemetry_snapshot.roll_ddeg = robot_fixed10(filt_roll, -1800, 1800);
                        telemetry_snapshot.heading_deg = heading;
                        robot_data_unlock(saved);
                    }
                    tm_printf((UB *)"[IMU] LIFTED/OVER P:%d R:%d\n",
                              (INT)filt_pitch, (INT)filt_roll);
                }

                // Recover IMU monitoring once held upright, flat, and steady for 500 ms
                // (motor_estop / MODE_SAFE_STOP stays latched for safety)
                BOOL upright_flat = (fabsf(filt_pitch) < (OVERTURN_TILT_DEG - 5.0f)) &&
                                    (fabsf(filt_roll)  < (OVERTURN_TILT_DEG - 5.0f)) &&
                                    (fabsf(med_pitch) < (OVERTURN_TILT_DEG - 5.0f)) &&
                                    (fabsf(med_roll) < (OVERTURN_TILT_DEG - 5.0f)) &&
                                    ((med_z * baseline_z) >= (0.7f * SQ(baseline_z))) &&
                                    (total_g_sq >= static_low_g_sq) &&
                                    (total_g_sq <= static_high_g_sq);

                if (imu_confirm(&recovery_check, upright_flat, now, IMU_RECOVERY_US)) {
                    recovery_check.active = safety_check.active = tilt_check.active = flat_check.active = FALSE;
                    imu.stop_reason = IMU_STOP_NONE;
                    imu.hump_active = FALSE;
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.imu_event = IMU_MSG_NONE;
                        robot_data_unlock(saved);
                    }
                    imu.state = STATE_MONITORING;
                    {
                        UINT saved = robot_data_lock();
                        telemetry_snapshot.imu_state = (UB)imu.state;
                        robot_data_unlock(saved);
                    }
                    robot_flags_update(0U, ROBOT_FAULT_IMU_TILT);
                    tm_printf((UB *)"[IMU] Upright -> Monitoring resumed\n");
                }
                break;
            }
        }

        tk_dly_tsk(IMU_SAMPLE_PERIOD_MS);
    }
}
