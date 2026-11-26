/**
 * @file imu_task.c
 * @brief LSM303DLHC accelerometer and magnetometer monitoring task.
 *
 * NOTE: See docs/fsm_diagrams.md for the IMU FSM.
 * Reference: STMicroelectronics LSM303DLHC Datasheet (DocID018771 Rev 2):
 *   CTRL_REG1_A (0x20) = 0x57 (100 Hz normal mode, X/Y/Z enabled)
 *   CTRL_REG4_A (0x23) = 0x88 (BDU enabled, high-resolution +/-2g, 1 mg/LSB)
 *   OUT_X_L_A   (0x28 | 0x80 = 0xA8 auto-increment burst read)
 *   CRA_REG_M   (0x00) = 0x14 (30 Hz magnetometer output rate)
 *   CRB_REG_M   (0x01) = 0x20 (+/-1.3 Gauss gain)
 *   MR_REG_M    (0x02) = 0x00 (continuous-conversion mode)
 *
 * BARR-C Deviation (Rules 5.2.a, 5.4.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel device/task APIs require native RTOS integer types
 *   (INT, UINT, ER, ID, SZ, W, UB); IMU tilt and hump filtering retains
 *   bounded single-precision float32_t math validated with isfinite().
 */

#include "imu_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <bsp/libbsp.h>
#include <tk/device.h>
#include <tm/tmonitor.h>

#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"

#if (((IMU_SDA_PIN % 4u) != 0u) || (IMU_SDA_PIN > 20u) || \
     (IMU_SCL_PIN != (IMU_SDA_PIN + 1u)))
#error "IMU pins must be an RP2040 I2C0 SDA/SCL pair (GP0/1..GP20/21)."
#endif

/* LSM303DLHC register addresses and configuration bytes. */
#define IMU_REG_CRA_M                   (0x00u)
#define IMU_REG_CRB_M                   (0x01u)
#define IMU_REG_MR_M                    (0x02u)
#define IMU_REG_OUT_X_H_M               (0x03u)
#define IMU_REG_CTRL1_A                 (0x20u)
#define IMU_REG_CTRL4_A                 (0x23u)
#define IMU_REG_OUT_X_L_A_INC           (0xA8u)
#define IMU_ACCEL_ALT_ADDR              (0x18u)
#define IMU_CTRL1_100HZ_XYZ             (0x57u)
#define IMU_CTRL4_BDU_HR                (0x88u)
#define IMU_CRA_MAG_30HZ                (0x14u)
#define IMU_CRB_MAG_1P3G                (0x20u)
#define IMU_MR_MAG_CONT                 (0x00u)
#define IMU_REG_WRITE_LEN               (2u)
#define IMU_ADDR_CMD_LEN                ((SZ)1)
#define IMU_BURST_LEN_BYTES             (6u)
#define IMU_ACCEL_BIT_WIDTH             (12u)

/* Filter, math, and calibration constants. */
#define IMU_SQRT_ITERS                  (12u)
#define IMU_MEDIAN_WINDOW               (3u)
#define IMU_HIST_FIRST_IDX              (0u)
#define IMU_HIST_SECOND_IDX             (1u)
#define IMU_HIST_THIRD_IDX              (2u)
#define IMU_CALIB_TARGET_SAMPLES        (50u)
#define IMU_CALIB_MAX_ATTEMPTS          (100u)
#define IMU_INIT_RETRY_MSEC             (1000u)
#define IMU_ROUTE_SETTLE_MSEC           (50u)
#define IMU_CALIB_SETTLE_MSEC           (200u)
#define IMU_CALIB_STEP_MSEC             (10u)
#define IMU_USEC_PER_MSEC               (1000u)
#define IMU_ZERO_EPSILON                (1.0e-6f)
#define IMU_EMA_ALPHA                   (0.22f)
#define IMU_PI_F32                      (3.14159265f)
#define IMU_PITCH_PI_F32                (3.14159f)
#define IMU_RAD_TO_DEG_F32              (180.0f)
#define IMU_DEG_TO_RAD_F32              (IMU_PI_F32 / 180.0f)
#define IMU_FULL_CIRCLE_DEG             (360.0f)
#define IMU_MG_TO_G_SCALE               (0.001f)
#define IMU_G_TO_MG_SCALE               (1000.0f)
#define IMU_CALIB_MIN_G                 (0.8f)
#define IMU_CALIB_MAX_G                 (1.2f)
#define IMU_CALIB_MAX_TILT_DEG          (20.0f)
#define IMU_INVERTED_Z_FACTOR           (-0.2f)
#define IMU_RECOVERY_Z_FACTOR           (0.7f)
#define IMU_RECOVERY_TILT_MARGIN        (5.0f)
#define IMU_LOG_CLAMP_MIN_F32           (-32768.0f)
#define IMU_LOG_CLAMP_MAX_F32           (32767.0f)
#define IMU_FIXED_TILT_MIN              ((int16_t)-1800)
#define IMU_FIXED_TILT_MAX              ((int16_t)1800)
#define IMU_FIXED_HEIGHT_MIN            ((int16_t)0)
#define IMU_FIXED_HEIGHT_MAX            ((int16_t)680)

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t since_usec;
    uint32_t last_usec;
    uint32_t b_is_active : 1;
} imu_task_confirm_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t state            : 2;
    uint32_t stop_reason      : 3;
    uint32_t b_is_hump_active : 1;
    uint32_t b_has_read_fault : 1;
    uint32_t hist_idx         : 2;
} imu_task_runtime_t;

typedef struct
{
    float32_t acc_x_g;
    float32_t acc_y_g;
    float32_t acc_z_g;
    float32_t total_g_sq;
    float32_t pitch_deg;
    float32_t roll_deg;
    float32_t med_pitch;
    float32_t med_roll;
    float32_t med_z;
} imu_task_sample_t;

typedef struct
{
    imu_task_runtime_t runtime;
    float32_t          baseline_z;
    float32_t          baseline_pitch;
    float32_t          baseline_roll;
    float32_t          hump_peak_mm;
    uint32_t           telem_usec;
    uint32_t           last_valid_usec;
    float32_t          pitch_hist[IMU_MEDIAN_WINDOW];
    float32_t          roll_hist[IMU_MEDIAN_WINDOW];
    float32_t          z_hist[IMU_MEDIAN_WINDOW];
    float32_t          filt_pitch;
    float32_t          filt_roll;
    float32_t          lift_low_g_sq;
    float32_t          lift_high_g_sq;
    float32_t          static_low_g_sq;
    float32_t          static_high_g_sq;
    imu_task_confirm_t tilt_check;
    imu_task_confirm_t flat_check;
    imu_task_confirm_t safety_check;
    imu_task_confirm_t recovery_check;
} imu_task_ctx_t;

ROBOT_TASKS_STATIC_ASSERT(imu_confirm_size,
                          (9u == sizeof(imu_task_confirm_t)));
ROBOT_TASKS_STATIC_ASSERT(imu_runtime_size,
                          (2u == sizeof(imu_task_runtime_t)));

static ID       gh_imu_dev          = 0;
static uint32_t g_active_accel_addr = IMU_LSM303_ACCEL_ADDR;

static float32_t imu_abs_f32(float32_t val);
static float32_t imu_sq_f32(float32_t val);
static INT imu_trunc_to_int(float32_t val);
static const char * imu_get_reason_name(imu_task_stop_reason_t reason);
static float32_t imu_sqrt_f32(float32_t val);
static float32_t imu_atan2_f32(float32_t y_val, float32_t x_val);
static float32_t imu_median3_f32(float32_t val_a,
                                 float32_t val_b,
                                 float32_t val_c);
static void imu_route_pins(void);
static ER imu_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_val);
static bool imu_read_accel_raw(int16_t * p_raw_x,
                               int16_t * p_raw_y,
                               int16_t * p_raw_z);
static bool imu_read_accel_g(float32_t * p_acc_x,
                             float32_t * p_acc_y,
                             float32_t * p_acc_z);
static void imu_compute_pitch_roll(float32_t acc_x,
                                   float32_t acc_y,
                                   float32_t acc_z,
                                   float32_t * p_pitch_out,
                                   float32_t * p_roll_out);
static bool imu_read_mag_raw(int16_t * p_mag_x,
                             int16_t * p_mag_y,
                             int16_t * p_mag_z);
static int16_t imu_get_heading_deg(void);
static bool imu_confirm_timer(imu_task_confirm_t * p_check,
                              bool b_is_cond_met,
                              uint32_t now_usec,
                              uint32_t dur_usec);
static imu_task_stop_reason_t imu_eval_safety(float32_t pitch_deg,
                                              float32_t roll_deg,
                                              float32_t z_val,
                                              float32_t total_g_sq,
                                              float32_t baseline_z,
                                              float32_t low_g_sq,
                                              float32_t high_g_sq,
                                              bool b_is_quasi_static);
static bool imu_confirm_safety(imu_task_confirm_t * p_check,
                               imu_task_stop_reason_t * p_prev_reason,
                               imu_task_stop_reason_t cur_reason,
                               uint32_t now_usec);
static void imu_send_nav_event(navigator_task_imu_event_t evt_type,
                               float32_t pitch_deg,
                               float32_t roll_deg,
                               float32_t height_mm,
                               bool b_has_height);
static void imu_record_read_fault(imu_task_ctx_t * p_ctx, uint32_t now_usec);
static bool imu_sample_and_filter(imu_task_ctx_t * p_ctx,
                                  uint32_t * p_now_usec,
                                  imu_task_sample_t * p_sample);
static void imu_handle_init(imu_task_ctx_t * p_ctx);
static void imu_apply_calibration(imu_task_ctx_t * p_ctx,
                                  float32_t sum_z,
                                  float32_t sum_g,
                                  float32_t sum_pitch,
                                  float32_t sum_roll,
                                  uint32_t valid_samples);
static void imu_handle_calibrate(imu_task_ctx_t * p_ctx);
static void imu_handle_tilted_hump(imu_task_ctx_t * p_ctx,
                                   uint32_t now_usec,
                                   bool b_is_quasi_static,
                                   float32_t * p_height_mm);
static void imu_update_hump(imu_task_ctx_t * p_ctx,
                            uint32_t now_usec,
                            bool b_is_quasi_static,
                            float32_t * p_height_mm,
                            bool * pb_is_tilted);
static void imu_latch_safety_stop(imu_task_ctx_t * p_ctx,
                                  const imu_task_sample_t * p_sample,
                                  imu_task_stop_reason_t reason,
                                  imu_task_stop_reason_t prev_reason,
                                  uint32_t now_usec);
static void imu_handle_monitoring(imu_task_ctx_t * p_ctx,
                                  const imu_task_sample_t * p_sample,
                                  uint32_t now_usec);
static void imu_handle_lifted(imu_task_ctx_t * p_ctx,
                              const imu_task_sample_t * p_sample,
                              uint32_t now_usec);

#ifndef NDEBUG
static bool imu_detector_self_check(void);
#endif

/**
 * @brief IMU acquisition, calibration, hump, and safety monitoring task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void imu_task_run_task (INT stacd, void * p_exinf)
{
    imu_task_ctx_t ctx     = { 0 };
    bool           b_is_ok = true;

    /* Explicitly ignore unused RTOS task entry parameters. */
    (void)stacd;
    /* Explicitly ignore unused RTOS extended info pointer. */
    (void)p_exinf;

#ifndef NDEBUG
    if (!imu_detector_self_check())
    {
        robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[IMU] Detector self-check failed\n");
        b_is_ok = false;
    }
#endif

    if (b_is_ok)
    {
        uint32_t start_usec = robot_tasks_time_usec();

        /* Store initial enum state in 2-bit unsigned runtime field. */
        ctx.runtime.state    = (uint32_t)IMU_STATE_INITIALIZE;
        ctx.baseline_z       = 1.0f;
        ctx.telem_usec       = start_usec;
        ctx.last_valid_usec  = start_usec;
        ctx.lift_low_g_sq    = imu_sq_f32(IMU_LOW_G_RATIO);
        ctx.lift_high_g_sq   = imu_sq_f32(1.0f + IMU_LIFT_ACCEL_DEV_G);
        ctx.static_low_g_sq  = imu_sq_f32(1.0f - IMU_STATIC_DEV_G);
        ctx.static_high_g_sq = imu_sq_f32(1.0f + IMU_STATIC_DEV_G);

        for (uint32_t idx = 0u; idx < IMU_MEDIAN_WINDOW; idx++)
        {
            ctx.z_hist[idx] = 1.0f;
        }
    }

    while (b_is_ok)
    {
        uint32_t          now_usec      = robot_tasks_time_usec();
        imu_task_sample_t sample        = { 0 };
        bool              b_should_step = true;

        sample.med_z = ctx.baseline_z;

        /* Compare 2-bit runtime state against safety-latched enum constant. */
        if ((uint32_t)IMU_STATE_LIFTED_OVERTURNED == ctx.runtime.state)
        {
            robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
        }

        /* Sample accelerometer only in active monitoring or lifted states. */
        if (((uint32_t)IMU_STATE_MONITORING == ctx.runtime.state) ||
            ((uint32_t)IMU_STATE_LIFTED_OVERTURNED == ctx.runtime.state))
        {
            b_should_step = imu_sample_and_filter(&ctx, &now_usec, &sample);
        }

        if (b_should_step)
        {
            /* Convert 2-bit runtime state field to enum for FSM dispatch. */
            switch ((imu_task_state_t)ctx.runtime.state)
            {
                case IMU_STATE_INITIALIZE:
                    imu_handle_init(&ctx);
                break;

                case IMU_STATE_CALIBRATING:
                    imu_handle_calibrate(&ctx);
                break;

                case IMU_STATE_MONITORING:
                    imu_handle_monitoring(&ctx, &sample, now_usec);
                break;

                case IMU_STATE_LIFTED_OVERTURNED:
                    imu_handle_lifted(&ctx, &sample, now_usec);
                break;

                default:
                    /* Reset invalid bit-field state back to initialization. */
                    ctx.runtime.state = (uint32_t)IMU_STATE_INITIALIZE;
                break;
            }
        }

        /* Ignore sleep return code in periodic polling loop. */
        (void)tk_dly_tsk(IMU_SAMPLE_PERIOD_MSEC);
    }
}

/**
 * @brief Return the absolute value of a single-precision float without libm.
 * @param[in] val Input floating-point value.
 * @return Non-negative magnitude of val.
 */
static float32_t imu_abs_f32 (float32_t val)
{
    return (val < 0.0f) ? (-val) : val;
}

/**
 * @brief Return the square of a single-precision float with single evaluation.
 * @param[in] val Input floating-point value.
 * @return val squared.
 */
static float32_t imu_sq_f32 (float32_t val)
{
    return (val * val);
}

/**
 * @brief Safely truncate a finite float32_t to RTOS INT for console logging.
 * @param[in] val Input floating-point value.
 * @return Clamped integer value, or 0 if val is NaN or infinite.
 */
static INT imu_trunc_to_int (float32_t val)
{
    INT result = 0;

    if (0 != isfinite(val))
    {
        float32_t clamped = val;

        if (clamped < IMU_LOG_CLAMP_MIN_F32)
        {
            clamped = IMU_LOG_CLAMP_MIN_F32;
        }
        else if (clamped > IMU_LOG_CLAMP_MAX_F32)
        {
            clamped = IMU_LOG_CLAMP_MAX_F32;
        }
        else
        {
            /* Value already lies within [-32768.0f, 32767.0f]. */
        }

        /* Range-safe: clamped is finite and bounded within 16-bit INT range. */
        result = (INT)clamped;
    }

    return result;
}

/**
 * @brief Map a stop reason enum value to a static diagnostic label string.
 * @param[in] reason IMU stop reason code.
 * @return Null-terminated ASCII description string.
 */
static const char * imu_get_reason_name (imu_task_stop_reason_t reason)
{
    const char * p_label = "none";

    switch (reason)
    {
        case IMU_STOP_TILT:
            p_label = "tilt";
        break;

        case IMU_STOP_INVERTED:
            p_label = "inverted";
        break;

        case IMU_STOP_LOW_G:
            p_label = "low-g";
        break;

        case IMU_STOP_HIGH_G:
            p_label = "high-g";
        break;

        case IMU_STOP_NONE:
        default:
            p_label = "none";
        break;
    }

    return p_label;
}

/**
 * @brief Compute single-precision square root via 12 Newton-Raphson steps.
 * @param[in] val Non-negative input value.
 * @return Approximated square root, or 0.0f if non-positive or non-finite.
 */
static float32_t imu_sqrt_f32 (float32_t val)
{
    float32_t result = 0.0f;

    if ((0 != isfinite(val)) && (val > 0.0f))
    {
        float32_t root = (val > 1.0f) ? val : 1.0f;

        for (uint32_t iter = 0u; iter < IMU_SQRT_ITERS; iter++)
        {
            root = 0.5f * (root + (val / root));
        }

        if (0 != isfinite(root))
        {
            result = root;
        }
    }

    return result;
}

/**
 * @brief Approximate atan2(y, x) in radians without linking libm.
 *
 * Reference: Rajan et al., "Efficient Approximations for the Arctangent
 * Function," IEEE Signal Processing Magazine, May 2006 (max error ~0.0038 rad).
 *
 * @param[in] y_val Y component.
 * @param[in] x_val X component.
 * @return Angle in radians in [-pi, +pi].
 */
static float32_t imu_atan2_f32 (float32_t y_val, float32_t x_val)
{
    float32_t result  = 0.0f;
    float32_t abs_x   = imu_abs_f32(x_val);
    float32_t abs_y   = imu_abs_f32(y_val);
    bool      b_is_ok = (0 != isfinite(x_val)) && (0 != isfinite(y_val));

    if ((b_is_ok) &&
        ((abs_x >= IMU_ZERO_EPSILON) || (abs_y >= IMU_ZERO_EPSILON)))
    {
        float32_t ratio = (abs_x > abs_y) ? (abs_y / abs_x) : (abs_x / abs_y);
        float32_t angle = (ratio * (IMU_PI_F32 / 4.0f)) +
                          (0.273f * ratio * (1.0f - ratio));

        if (abs_y > abs_x)
        {
            angle = (IMU_PI_F32 / 2.0f) - angle;
        }

        if (x_val < 0.0f)
        {
            angle = IMU_PI_F32 - angle;
        }

        if (0 != isfinite(angle))
        {
            result = (y_val < 0.0f) ? (-angle) : angle;
        }
    }

    return result;
}

/**
 * @brief Compute median of three float32_t samples to reject spike outliers.
 * @param[in] val_a First sample.
 * @param[in] val_b Second sample.
 * @param[in] val_c Third sample.
 * @return Median value.
 */
static float32_t imu_median3_f32 (float32_t val_a,
                                  float32_t val_b,
                                  float32_t val_c)
{
    if (val_a > val_b)
    {
        float32_t swap_val = val_a;

        val_a = val_b;
        val_b = swap_val;
    }

    if (val_b > val_c)
    {
        float32_t swap_val = val_b;

        val_b = val_c;
        val_c = swap_val;
    }

    if (val_a > val_b)
    {
        val_b = val_a;
    }

    return val_b;
}

/**
 * @brief Route RP2040 I2C0 peripheral function to configured SDA and SCL pins.
 */
static void imu_route_pins (void)
{
    const uint32_t pad_flags =
        GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT;

    out_w(GPIO(IMU_SDA_PIN), pad_flags);
    out_w(GPIO(IMU_SCL_PIN), pad_flags);
    out_w(GPIO_CTRL(IMU_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(IMU_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
}

/**
 * @brief Write one byte to an LSM303DLHC I2C register.
 * @param[in] dev_addr 7-bit I2C device address.
 * @param[in] reg_addr Target register address.
 * @param[in] reg_val  Byte value to write.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
static ER imu_write_reg (uint8_t dev_addr, uint8_t reg_addr, uint8_t reg_val)
{
    uint8_t tx_buf[IMU_REG_WRITE_LEN] = { reg_addr, reg_val };
    SZ      actual_sz                 = 0;
    /* Cast 7-bit I2C address and buffer length to uT-Kernel W and SZ types. */
    ER      ret_err                   = tk_swri_dev(gh_imu_dev,
                                                    (W)dev_addr,
                                                    tx_buf,
                                                    (SZ)sizeof(tx_buf),
                                                    &actual_sz);

    /* Compare actual byte count transferred against 2-byte I2C payload size. */
    if ((ret_err >= E_OK) && ((SZ)sizeof(tx_buf) != actual_sz))
    {
        ret_err = E_IO;
    }

    return ret_err;
}

/**
 * @brief Read 12-bit signed accelerometer X, Y, Z counts in a 6-byte burst.
 * @param[out] p_raw_x Signed 12-bit X sample (-2048..2047).
 * @param[out] p_raw_y Signed 12-bit Y sample (-2048..2047).
 * @param[out] p_raw_z Signed 12-bit Z sample (-2048..2047).
 * @return True on valid I2C burst read, false on communication error.
 */
static bool imu_read_accel_raw (int16_t * p_raw_x,
                                int16_t * p_raw_y,
                                int16_t * p_raw_z)
{
    uint8_t rx_buf[IMU_BURST_LEN_BYTES] = { 0u };
    uint8_t reg_cmd                     = IMU_REG_OUT_X_L_A_INC;
    SZ      actual_sz                   = 0;
    ER      ret_err                     = E_OK;
    bool    b_is_ok                     = false;

    /* Cast 7-bit active accelerometer address to uT-Kernel W offset type. */
    ret_err = tk_swri_dev(gh_imu_dev,
                          (W)g_active_accel_addr,
                          &reg_cmd,
                          IMU_ADDR_CMD_LEN,
                          &actual_sz);

    if ((ret_err >= E_OK) && (IMU_ADDR_CMD_LEN == actual_sz))
    {
        actual_sz = 0;
        /* Cast 7-bit I2C address and 6-byte burst size to RTOS W and SZ. */
        ret_err = tk_srea_dev(gh_imu_dev,
                              (W)g_active_accel_addr,
                              rx_buf,
                              (SZ)IMU_BURST_LEN_BYTES,
                              &actual_sz);

        /* Verify all 6 accelerometer bytes were read from the I2C bus. */
        if ((ret_err >= E_OK) && ((SZ)IMU_BURST_LEN_BYTES == actual_sz))
        {
            /* Combine little-endian high/low register bytes into uint16_t. */
            uint16_t raw_x_u16 =
                (uint16_t)((((uint16_t)rx_buf[1]) << 8u) | (uint16_t)rx_buf[0]);
            /* Combine little-endian Y high/low register bytes into uint16_t. */
            uint16_t raw_y_u16 =
                (uint16_t)((((uint16_t)rx_buf[3]) << 8u) | (uint16_t)rx_buf[2]);
            /* Combine little-endian Z high/low register bytes into uint16_t. */
            uint16_t raw_z_u16 =
                (uint16_t)((((uint16_t)rx_buf[5]) << 8u) | (uint16_t)rx_buf[4]);

            /* Range-safe: 12-bit sign-extended values lie in [-2048, 2047]. */
            *p_raw_x = (int16_t)robot_tasks_sign_extend(
                (uint32_t)(raw_x_u16 >> 4u), IMU_ACCEL_BIT_WIDTH);
            /* Range-safe: 12-bit sign-extended Y value fits in int16_t. */
            *p_raw_y = (int16_t)robot_tasks_sign_extend(
                (uint32_t)(raw_y_u16 >> 4u), IMU_ACCEL_BIT_WIDTH);
            /* Range-safe: 12-bit sign-extended Z value fits in int16_t. */
            *p_raw_z = (int16_t)robot_tasks_sign_extend(
                (uint32_t)(raw_z_u16 >> 4u), IMU_ACCEL_BIT_WIDTH);
            b_is_ok  = true;
        }
    }

    return b_is_ok;
}

/**
 * @brief Read accelerometer axes scaled to units of g (1 mg per LSB).
 * @param[out] p_acc_x X acceleration in g.
 * @param[out] p_acc_y Y acceleration in g.
 * @param[out] p_acc_z Z acceleration in g.
 * @return True on valid sample, false on I2C read failure.
 */
static bool imu_read_accel_g (float32_t * p_acc_x,
                              float32_t * p_acc_y,
                              float32_t * p_acc_z)
{
    int16_t raw_x   = 0;
    int16_t raw_y   = 0;
    int16_t raw_z   = 0;
    bool    b_is_ok = imu_read_accel_raw(&raw_x, &raw_y, &raw_z);

    if (b_is_ok)
    {
        /* Convert exact 12-bit signed integer X count to float32_t g units. */
        *p_acc_x = ((float32_t)raw_x) * IMU_MG_TO_G_SCALE;
        /* Convert exact 12-bit signed integer Y count to float32_t g units. */
        *p_acc_y = ((float32_t)raw_y) * IMU_MG_TO_G_SCALE;
        /* Convert exact 12-bit signed integer Z count to float32_t g units. */
        *p_acc_z = ((float32_t)raw_z) * IMU_MG_TO_G_SCALE;
    }

    return b_is_ok;
}

/**
 * @brief Compute vehicle pitch and roll in degrees from mounted IMU axes.
 *
 * NOTE: Sensor is rotated 90 degrees on the chassis: sensor Y maps to pitch
 * (forward/backward) and sensor X maps to roll (left/right).
 *
 * @param[in]  acc_x       X acceleration in g.
 * @param[in]  acc_y       Y acceleration in g.
 * @param[in]  acc_z       Z acceleration in g.
 * @param[out] p_pitch_out Computed pitch angle in degrees.
 * @param[out] p_roll_out  Computed roll angle in degrees.
 */
static void imu_compute_pitch_roll (float32_t acc_x,
                                    float32_t acc_y,
                                    float32_t acc_z,
                                    float32_t * p_pitch_out,
                                    float32_t * p_roll_out)
{
    float32_t pitch_rad = imu_atan2_f32(
        acc_y, imu_sqrt_f32(imu_sq_f32(acc_x) + imu_sq_f32(acc_z)));
    float32_t roll_rad  = imu_atan2_f32(
        acc_x, imu_sqrt_f32(imu_sq_f32(acc_y) + imu_sq_f32(acc_z)));
    float32_t pitch_deg = (pitch_rad * IMU_RAD_TO_DEG_F32) / IMU_PITCH_PI_F32;
    float32_t roll_deg  = (roll_rad * IMU_RAD_TO_DEG_F32) / IMU_PITCH_PI_F32;

    *p_pitch_out = (0 != isfinite(pitch_deg)) ? pitch_deg : 0.0f;
    *p_roll_out  = (0 != isfinite(roll_deg)) ? roll_deg : 0.0f;
}

/**
 * @brief Read raw big-endian LSM303DLHC magnetometer X, Z, Y registers.
 * @param[out] p_mag_x Signed 16-bit X magnetic count.
 * @param[out] p_mag_y Signed 16-bit Y magnetic count.
 * @param[out] p_mag_z Signed 16-bit Z magnetic count.
 * @return True on valid I2C burst read, false on failure.
 */
static bool imu_read_mag_raw (int16_t * p_mag_x,
                              int16_t * p_mag_y,
                              int16_t * p_mag_z)
{
    uint8_t rx_buf[IMU_BURST_LEN_BYTES] = { 0u };
    uint8_t reg_cmd                     = IMU_REG_OUT_X_H_M;
    SZ      actual_sz                   = 0;
    ER      ret_err                     = E_OK;
    bool    b_is_ok                     = false;

    /* Cast 7-bit magnetometer I2C address to uT-Kernel W offset parameter. */
    ret_err = tk_swri_dev(gh_imu_dev,
                          (W)IMU_LSM303_MAG_ADDR,
                          &reg_cmd,
                          IMU_ADDR_CMD_LEN,
                          &actual_sz);

    if ((ret_err >= E_OK) && (IMU_ADDR_CMD_LEN == actual_sz))
    {
        actual_sz = 0;
        /* Cast 7-bit magnetometer address and burst size to RTOS W and SZ. */
        ret_err = tk_srea_dev(gh_imu_dev,
                              (W)IMU_LSM303_MAG_ADDR,
                              rx_buf,
                              (SZ)IMU_BURST_LEN_BYTES,
                              &actual_sz);

        /* Verify all 6 magnetometer bytes were received over I2C. */
        if ((ret_err >= E_OK) && ((SZ)IMU_BURST_LEN_BYTES == actual_sz))
        {
            /* LSM303DLHC mag register order is X, Z, Y (big-endian). */
            uint16_t raw_x =
                (uint16_t)((((uint16_t)rx_buf[0]) << 8u) | (uint16_t)rx_buf[1]);
            /* Assemble big-endian Z magnetometer register pair. */
            uint16_t raw_z =
                (uint16_t)((((uint16_t)rx_buf[2]) << 8u) | (uint16_t)rx_buf[3]);
            /* Assemble big-endian Y magnetometer register pair. */
            uint16_t raw_y =
                (uint16_t)((((uint16_t)rx_buf[4]) << 8u) | (uint16_t)rx_buf[5]);

            /* Reinterpret 16-bit two's-complement X sensor word as int16_t. */
            *p_mag_x = (int16_t)raw_x;
            /* Reinterpret 16-bit two's-complement Z sensor word as int16_t. */
            *p_mag_z = (int16_t)raw_z;
            /* Reinterpret 16-bit two's-complement Y sensor word as int16_t. */
            *p_mag_y = (int16_t)raw_y;
            b_is_ok  = true;
        }
    }

    return b_is_ok;
}

/**
 * @brief Compute compass heading in degrees (0..359) from magnetometer axes.
 * @return Heading angle in degrees, or last cached heading on I2C failure.
 */
static int16_t imu_get_heading_deg (void)
{
    int16_t heading = navigator_task_get_heading();
    int16_t mag_x   = 0;
    int16_t mag_y   = 0;
    int16_t mag_z   = 0;

    if (imu_read_mag_raw(&mag_x, &mag_y, &mag_z))
    {
        /* Convert 16-bit signed magnetometer counts to float32_t for atan2. */
        float32_t rad = imu_atan2_f32((float32_t)mag_x, -((float32_t)mag_y));
        float32_t deg = (rad * IMU_RAD_TO_DEG_F32) / IMU_PI_F32;

        if (0 != isfinite(deg))
        {
            deg = (deg < 0.0f) ? (deg + IMU_FULL_CIRCLE_DEG) : deg;
            deg = (deg >= IMU_FULL_CIRCLE_DEG)
                  ? (deg - IMU_FULL_CIRCLE_DEG)
                  : deg;
            /* Range-safe: deg is finite and normalized into [0.0f, 360.0f). */
            heading = (int16_t)deg;
        }
    }

    return heading;
}

/**
 * @brief Track continuous condition duration independent of scheduling jitter.
 * @param[in,out] p_check       Confirmation timer state.
 * @param[in]     b_is_cond_met Current boolean condition state.
 * @param[in]     now_usec      Current 32-bit microsecond timestamp.
 * @param[in]     dur_usec      Required continuous duration in microseconds.
 * @return True once condition has remained continuously true for dur_usec.
 */
static bool imu_confirm_timer (imu_task_confirm_t * p_check,
                               bool b_is_cond_met,
                               uint32_t now_usec,
                               uint32_t dur_usec)
{
    bool b_is_confirmed = false;

    if (!b_is_cond_met)
    {
        p_check->b_is_active = 0u;
    }
    else
    {
        if ((0u == p_check->b_is_active) ||
            ((now_usec - p_check->last_usec) > IMU_SAMPLE_GAP_USEC))
        {
            p_check->since_usec  = now_usec;
            p_check->b_is_active = 1u;
        }

        p_check->last_usec = now_usec;
        b_is_confirmed     = ((now_usec - p_check->since_usec) >= dur_usec);
    }

    return b_is_confirmed;
}

/**
 * @brief Classify instantaneous lift, impact, inversion, or tilt stop cause.
 * @param[in] pitch_deg         Filtered pitch angle in degrees.
 * @param[in] roll_deg          Filtered roll angle in degrees.
 * @param[in] z_val             Median Z acceleration in g.
 * @param[in] total_g_sq        Squared total acceleration magnitude.
 * @param[in] baseline_z        Calibrated flat-ground Z acceleration in g.
 * @param[in] low_g_sq          Free-fall squared g threshold.
 * @param[in] high_g_sq         High-g impact squared g threshold.
 * @param[in] b_is_quasi_static True when total g is within static window.
 * @return Instantaneous stop reason classification.
 */
static imu_task_stop_reason_t imu_eval_safety (float32_t pitch_deg,
                                               float32_t roll_deg,
                                               float32_t z_val,
                                               float32_t total_g_sq,
                                               float32_t baseline_z,
                                               float32_t low_g_sq,
                                               float32_t high_g_sq,
                                               bool b_is_quasi_static)
{
    imu_task_stop_reason_t reason = IMU_STOP_NONE;

    if (total_g_sq < low_g_sq)
    {
        reason = IMU_STOP_LOW_G;
    }
    else if (total_g_sq > high_g_sq)
    {
        reason = IMU_STOP_HIGH_G;
    }
    else if (!b_is_quasi_static)
    {
        reason = IMU_STOP_NONE;
    }
    else if ((z_val * baseline_z) <
             (IMU_INVERTED_Z_FACTOR * imu_sq_f32(baseline_z)))
    {
        reason = IMU_STOP_INVERTED;
    }
    else if ((imu_abs_f32(pitch_deg) > IMU_OVERTURN_TILT_DEG) ||
             (imu_abs_f32(roll_deg) > IMU_OVERTURN_TILT_DEG))
    {
        reason = IMU_STOP_TILT;
    }
    else
    {
        reason = IMU_STOP_NONE;
    }

    return reason;
}

/**
 * @brief Require continuous confirmation of a single safety stop cause.
 * @param[in,out] p_check       Confirmation timer state.
 * @param[in,out] p_prev_reason Previously tracked stop cause.
 * @param[in]     cur_reason    Current instantaneous stop cause.
 * @param[in]     now_usec      Current microsecond timestamp.
 * @return True if cur_reason persisted for its confirmation duration.
 */
static bool imu_confirm_safety (imu_task_confirm_t * p_check,
                                imu_task_stop_reason_t * p_prev_reason,
                                imu_task_stop_reason_t cur_reason,
                                uint32_t now_usec)
{
    uint32_t dur_usec = IMU_TILT_CONFIRM_USEC;

    if (cur_reason != (*p_prev_reason))
    {
        p_check->b_is_active = 0u;
    }

    *p_prev_reason = cur_reason;

    if ((IMU_STOP_LOW_G == cur_reason) || (IMU_STOP_HIGH_G == cur_reason))
    {
        dur_usec = IMU_LIFT_CONFIRM_USEC;
    }

    return imu_confirm_timer(p_check,
                             (IMU_STOP_NONE != cur_reason),
                             now_usec,
                             dur_usec);
}

/**
 * @brief Format and send an IMU telemetry event message to Navigator.
 * @param[in] evt_type     IMU event enum value.
 * @param[in] pitch_deg    Filtered pitch in degrees.
 * @param[in] roll_deg     Filtered roll in degrees.
 * @param[in] height_mm    Hump height in mm (stored when b_has_height).
 * @param[in] b_has_height True to populate height_dmm field.
 */
static void imu_send_nav_event (navigator_task_imu_event_t evt_type,
                                float32_t pitch_deg,
                                float32_t roll_deg,
                                float32_t height_mm,
                                bool b_has_height)
{
    ID h_nav_mbf = robot_tasks_get_nav_mbf();

    if (h_nav_mbf > E_OK)
    {
        int16_t              pitch_ddeg = robot_tasks_scale_tenths(
            pitch_deg, IMU_FIXED_TILT_MIN, IMU_FIXED_TILT_MAX);
        int16_t              roll_ddeg  = robot_tasks_scale_tenths(
            roll_deg, IMU_FIXED_TILT_MIN, IMU_FIXED_TILT_MAX);
        navigator_task_msg_t nav_msg    = { 0 };

        /* Pack message type and event enums into unsigned bit-fields. */
        nav_msg.type               = (uint32_t)MSG_IMU_UPDATE;
        /* Store 2-bit IMU event code in packed message structure. */
        nav_msg.data.imu_msg.event = (uint32_t)evt_type;
        /* Narrow 12-bit masked two's-complement pitch into uint16_t field. */
        nav_msg.data.imu_msg.pitch_ddeg_raw =
            (uint16_t)robot_tasks_pack_signed((int32_t)pitch_ddeg,
                                              NAV_TILT_BIT_WIDTH);
        /* Narrow 12-bit masked two's-complement roll into uint16_t field. */
        nav_msg.data.imu_msg.roll_ddeg_raw =
            (uint16_t)robot_tasks_pack_signed((int32_t)roll_ddeg,
                                              NAV_TILT_BIT_WIDTH);

        if (b_has_height)
        {
            int16_t h_dmm = robot_tasks_scale_tenths(
                height_mm, IMU_FIXED_HEIGHT_MIN, IMU_FIXED_HEIGHT_MAX);

            /* Range-safe: clamped to [0, 680] matching 10-bit field. */
            nav_msg.data.imu_msg.height_dmm = (uint16_t)h_dmm;
        }

        /* Non-blocking send; cast struct size to RTOS INT message size. */
        (void)tk_snd_mbf(h_nav_mbf,
                         &nav_msg,
                         (INT)sizeof(navigator_task_msg_t),
                         TMO_POL);
    }
}

/**
 * @brief Handle sustained I2C read failure and latch E-stop on timeout.
 * @param[in,out] p_ctx    IMU task context.
 * @param[in]     now_usec Current microsecond timestamp.
 */
static void imu_record_read_fault (imu_task_ctx_t * p_ctx, uint32_t now_usec)
{
    p_ctx->tilt_check.b_is_active     = 0u;
    p_ctx->flat_check.b_is_active     = 0u;
    p_ctx->safety_check.b_is_active   = 0u;
    p_ctx->recovery_check.b_is_active = 0u;

    if ((now_usec - p_ctx->last_valid_usec) >= IMU_READ_FAULT_USEC)
    {
        robot_tasks_flags_update(
            ROBOT_ESTOP_REQUEST | ROBOT_FAULT_IMU_READ, 0u);

        if (0u == p_ctx->runtime.b_has_read_fault)
        {
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[IMU] SENSOR READ FAULT -> STOP\n");
        }

        p_ctx->runtime.b_has_read_fault = 1u;
    }
}

/**
 * @brief Acquire one accelerometer reading, check read faults, and filter.
 * @param[in,out] p_ctx      IMU task context.
 * @param[out]    p_now_usec Updated post-read timestamp.
 * @param[out]    p_sample   Populated raw and median-filtered sample.
 * @return True if sample is valid, false if I2C read failed.
 */
static bool imu_sample_and_filter (imu_task_ctx_t * p_ctx,
                                   uint32_t * p_now_usec,
                                   imu_task_sample_t * p_sample)
{
    bool b_is_valid = imu_read_accel_g(&p_sample->acc_x_g,
                                       &p_sample->acc_y_g,
                                       &p_sample->acc_z_g);

    *p_now_usec = robot_tasks_time_usec();

    if (!b_is_valid)
    {
        imu_record_read_fault(p_ctx, *p_now_usec);
    }
    else
    {
        bool b_has_sample_gap = false;

        if (0u != p_ctx->runtime.b_has_read_fault)
        {
            robot_tasks_flags_update(0u, ROBOT_FAULT_IMU_READ);
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[IMU] Reads recovered; stop stays latched\n");
        }

        p_ctx->runtime.b_has_read_fault = 0u;
        b_has_sample_gap =
            ((*p_now_usec - p_ctx->last_valid_usec) > IMU_SAMPLE_GAP_USEC);
        p_ctx->last_valid_usec = *p_now_usec;

        p_sample->total_g_sq = imu_sq_f32(p_sample->acc_x_g) +
                               imu_sq_f32(p_sample->acc_y_g) +
                               imu_sq_f32(p_sample->acc_z_g);
        imu_compute_pitch_roll(p_sample->acc_x_g,
                               p_sample->acc_y_g,
                               p_sample->acc_z_g,
                               &p_sample->pitch_deg,
                               &p_sample->roll_deg);
        p_sample->pitch_deg -= p_ctx->baseline_pitch;
        p_sample->roll_deg  -= p_ctx->baseline_roll;

        if (b_has_sample_gap)
        {
            for (uint32_t idx = 0u; idx < IMU_MEDIAN_WINDOW; idx++)
            {
                p_ctx->pitch_hist[idx] = p_sample->pitch_deg;
                p_ctx->roll_hist[idx]  = p_sample->roll_deg;
                p_ctx->z_hist[idx]     = p_sample->acc_z_g;
            }

            p_ctx->filt_pitch = p_sample->pitch_deg;
            p_ctx->filt_roll  = p_sample->roll_deg;
        }

        p_ctx->pitch_hist[p_ctx->runtime.hist_idx] = p_sample->pitch_deg;
        p_ctx->roll_hist[p_ctx->runtime.hist_idx]  = p_sample->roll_deg;
        p_ctx->z_hist[p_ctx->runtime.hist_idx]     = p_sample->acc_z_g;
        p_ctx->runtime.hist_idx =
            (p_ctx->runtime.hist_idx + 1u) % IMU_MEDIAN_WINDOW;

        p_sample->med_pitch = imu_median3_f32(
            p_ctx->pitch_hist[IMU_HIST_FIRST_IDX],
            p_ctx->pitch_hist[IMU_HIST_SECOND_IDX],
            p_ctx->pitch_hist[IMU_HIST_THIRD_IDX]);
        p_sample->med_roll  = imu_median3_f32(
            p_ctx->roll_hist[IMU_HIST_FIRST_IDX],
            p_ctx->roll_hist[IMU_HIST_SECOND_IDX],
            p_ctx->roll_hist[IMU_HIST_THIRD_IDX]);
        p_sample->med_z     = imu_median3_f32(
            p_ctx->z_hist[IMU_HIST_FIRST_IDX],
            p_ctx->z_hist[IMU_HIST_SECOND_IDX],
            p_ctx->z_hist[IMU_HIST_THIRD_IDX]);

        p_ctx->filt_pitch +=
            IMU_EMA_ALPHA * (p_sample->med_pitch - p_ctx->filt_pitch);
        p_ctx->filt_roll  +=
            IMU_EMA_ALPHA * (p_sample->med_roll - p_ctx->filt_roll);
    }

    return b_is_valid;
}

/**
 * @brief Open I2C0, re-route pins, and configure LSM303DLHC registers.
 * @param[in,out] p_ctx IMU task context.
 */
static void imu_handle_init (imu_task_ctx_t * p_ctx)
{
    bool b_is_dev_ready = true;

    if (gh_imu_dev <= 0)
    {
        /* Cast ASCII device name to uT-Kernel const UB pointer. */
        gh_imu_dev = tk_opn_dev((const UB *)IMU_I2C_DEVNAME, TD_UPDATE);

        if (gh_imu_dev <= E_OK)
        {
            gh_imu_dev = 0;
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[IMU] I2C device open failed\n");
            /* Ignore delay return value while backing off before retry. */
            (void)tk_dly_tsk(IMU_INIT_RETRY_MSEC);
            b_is_dev_ready = false;
        }
        else
        {
            imu_route_pins();
            /* Ignore delay return value during pin pull-up settle window. */
            (void)tk_dly_tsk(IMU_ROUTE_SETTLE_MSEC);
        }
    }

    if (b_is_dev_ready)
    {
        /* Fit 7-bit default accelerometer address into uint8_t variable. */
        uint8_t accel_addr = (uint8_t)IMU_LSM303_ACCEL_ADDR;
        ER      err_ctrl1  = imu_write_reg(accel_addr,
                                           IMU_REG_CTRL1_A,
                                           IMU_CTRL1_100HZ_XYZ);
        ER      err_ctrl4  = err_ctrl1;

        if (err_ctrl1 < E_OK)
        {
            accel_addr = IMU_ACCEL_ALT_ADDR;
            err_ctrl1  = imu_write_reg(accel_addr,
                                       IMU_REG_CTRL1_A,
                                       IMU_CTRL1_100HZ_XYZ);
        }

        if (err_ctrl1 >= E_OK)
        {
            err_ctrl4 = imu_write_reg(accel_addr,
                                      IMU_REG_CTRL4_A,
                                      IMU_CTRL4_BDU_HR);
        }

        /* Best-effort magnetometer rate init; heading is optional telemetry. */
        (void)imu_write_reg((uint8_t)IMU_LSM303_MAG_ADDR,
                            IMU_REG_CRA_M,
                            IMU_CRA_MAG_30HZ);
        /* Best-effort magnetometer gain init. */
        (void)imu_write_reg((uint8_t)IMU_LSM303_MAG_ADDR,
                            IMU_REG_CRB_M,
                            IMU_CRB_MAG_1P3G);
        /* Best-effort magnetometer continuous mode init. */
        (void)imu_write_reg((uint8_t)IMU_LSM303_MAG_ADDR,
                            IMU_REG_MR_M,
                            IMU_MR_MAG_CONT);

        if ((err_ctrl1 >= E_OK) && (err_ctrl4 >= E_OK))
        {
            /* Widen 7-bit active accelerometer address to uint32_t state. */
            g_active_accel_addr  = (uint32_t)accel_addr;
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[IMU STATUS] CONNECTED!\n");
            /* Advance 2-bit runtime state to calibration phase. */
            p_ctx->runtime.state = (uint32_t)IMU_STATE_CALIBRATING;
        }
        else
        {
            /* Ignore delay return value before next I2C probe attempt. */
            (void)tk_dly_tsk(IMU_INIT_RETRY_MSEC);
        }
    }
}

/**
 * @brief Compute and store calibrated baselines and g-window thresholds.
 * @param[in,out] p_ctx         IMU task context.
 * @param[in]     sum_z         Summed flat-ground Z samples.
 * @param[in]     sum_g         Summed total g magnitude samples.
 * @param[in]     sum_pitch     Summed raw pitch angles.
 * @param[in]     sum_roll      Summed raw roll angles.
 * @param[in]     valid_samples Number of valid samples averaged.
 */
static void imu_apply_calibration (imu_task_ctx_t * p_ctx,
                                   float32_t sum_z,
                                   float32_t sum_g,
                                   float32_t sum_pitch,
                                   float32_t sum_roll,
                                   uint32_t valid_samples)
{
    /* Exact conversion: valid_samples is 50, well within float32_t mantissa. */
    float32_t count_f    = (float32_t)valid_samples;
    float32_t baseline_g = sum_g / count_f;
    float32_t lift_low   = baseline_g - IMU_LIFT_ACCEL_DEV_G;
    float32_t static_low = baseline_g - IMU_STATIC_DEV_G;
    uint32_t  now_usec   = robot_tasks_time_usec();

    p_ctx->baseline_z     = sum_z / count_f;
    p_ctx->baseline_pitch = sum_pitch / count_f;
    p_ctx->baseline_roll  = sum_roll / count_f;

    if (lift_low < (baseline_g * IMU_LOW_G_RATIO))
    {
        lift_low = baseline_g * IMU_LOW_G_RATIO;
    }

    if (static_low < 0.0f)
    {
        static_low = 0.0f;
    }

    p_ctx->lift_low_g_sq    = imu_sq_f32(lift_low);
    p_ctx->lift_high_g_sq   = imu_sq_f32(baseline_g + IMU_LIFT_ACCEL_DEV_G);
    p_ctx->static_low_g_sq  = imu_sq_f32(static_low);
    p_ctx->static_high_g_sq = imu_sq_f32(baseline_g + IMU_STATIC_DEV_G);

    for (uint32_t idx = 0u; idx < IMU_MEDIAN_WINDOW; idx++)
    {
        p_ctx->pitch_hist[idx] = 0.0f;
        p_ctx->roll_hist[idx]  = 0.0f;
        p_ctx->z_hist[idx]     = p_ctx->baseline_z;
    }

    p_ctx->runtime.hist_idx = 0u;
    p_ctx->filt_pitch       = 0.0f;
    p_ctx->filt_roll        = 0.0f;
    p_ctx->last_valid_usec  = now_usec;
    p_ctx->telem_usec       = now_usec;

    /* Cast string literal and sample count to uT-Kernel UB* and UINT types. */
    tm_printf((UB *)"[IMU] Calibrated! Offsets -> P:%d R:%d (samples:%u)\n",
              imu_trunc_to_int(p_ctx->baseline_pitch),
              imu_trunc_to_int(p_ctx->baseline_roll),
              (UINT)valid_samples);
    /* Store MONITORING enum state in 2-bit unsigned runtime field. */
    p_ctx->runtime.state = (uint32_t)IMU_STATE_MONITORING;
}

/**
 * @brief Collect 50 consecutive stationary flat-ground samples for zeroing.
 * @param[in,out] p_ctx IMU task context.
 */
static void imu_handle_calibrate (imu_task_ctx_t * p_ctx)
{
    float32_t sum_z         = 0.0f;
    float32_t sum_g         = 0.0f;
    float32_t sum_pitch     = 0.0f;
    float32_t sum_roll      = 0.0f;
    uint32_t  valid_samples = 0u;
    uint32_t  attempts      = 0u;

    /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
    tm_printf((UB *)"[IMU] Calibrating zero on flat ground...\n");
    /* Ignore delay return value during pre-calibration settle period. */
    (void)tk_dly_tsk(IMU_CALIB_SETTLE_MSEC);

    while ((valid_samples < IMU_CALIB_TARGET_SAMPLES) &&
           (attempts < IMU_CALIB_MAX_ATTEMPTS))
    {
        float32_t acc_x = 0.0f;
        float32_t acc_y = 0.0f;
        float32_t acc_z = 0.0f;

        if (imu_read_accel_g(&acc_x, &acc_y, &acc_z))
        {
            float32_t raw_pitch = 0.0f;
            float32_t raw_roll  = 0.0f;
            float32_t mag_g     = imu_sqrt_f32(imu_sq_f32(acc_x) +
                                               imu_sq_f32(acc_y) +
                                               imu_sq_f32(acc_z));

            imu_compute_pitch_roll(acc_x, acc_y, acc_z, &raw_pitch, &raw_roll);

            if ((mag_g >= IMU_CALIB_MIN_G) &&
                (mag_g <= IMU_CALIB_MAX_G) &&
                (imu_abs_f32(raw_pitch) < IMU_CALIB_MAX_TILT_DEG) &&
                (imu_abs_f32(raw_roll) < IMU_CALIB_MAX_TILT_DEG))
            {
                sum_z     += acc_z;
                sum_g     += mag_g;
                sum_pitch += raw_pitch;
                sum_roll  += raw_roll;
                valid_samples++;
            }
            else
            {
                valid_samples = 0u;
                sum_z         = 0.0f;
                sum_g         = 0.0f;
                sum_pitch     = 0.0f;
                sum_roll      = 0.0f;
            }
        }

        attempts++;
        /* Ignore delay return value between 10 ms calibration samples. */
        (void)tk_dly_tsk(IMU_CALIB_STEP_MSEC);
    }

    if (IMU_CALIB_TARGET_SAMPLES != valid_samples)
    {
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[IMU] Calibration incomplete; keep flat, retrying\n");
    }
    else
    {
        imu_apply_calibration(p_ctx,
                              sum_z,
                              sum_g,
                              sum_pitch,
                              sum_roll,
                              valid_samples);
    }
}

/**
 * @brief Process active tilt during hump traversal and update peak height.
 * @param[in,out] p_ctx             IMU task context.
 * @param[in]     now_usec          Current microsecond timestamp.
 * @param[in]     b_is_quasi_static True when acceleration is near 1 g.
 * @param[out]    p_height_mm       Live computed hump height in mm.
 */
static void imu_handle_tilted_hump (imu_task_ctx_t * p_ctx,
                                    uint32_t now_usec,
                                    bool b_is_quasi_static,
                                    float32_t * p_height_mm)
{
    float32_t sin_roll  = imu_abs_f32(p_ctx->filt_roll) *
                          IMU_DEG_TO_RAD_F32;
    float32_t sin_pitch = imu_abs_f32(p_ctx->filt_pitch) *
                          IMU_DEG_TO_RAD_F32;
    float32_t h_roll    = 0.0f;
    float32_t h_pitch   = 0.0f;
    float32_t height_mm = 0.0f;

    p_ctx->flat_check.b_is_active = 0u;

    if (sin_roll > 1.0f)
    {
        sin_roll = 1.0f;
    }

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }

    h_roll       = IMU_HUMP_TRACK_WIDTH_MM * sin_roll;
    h_pitch      = IMU_HUMP_CHASSIS_SCALE_MM * sin_pitch;
    height_mm    = (h_roll > h_pitch) ? h_roll : h_pitch;
    *p_height_mm = height_mm;

    navigator_task_set_imu_height(robot_tasks_scale_tenths(
        height_mm, IMU_FIXED_HEIGHT_MIN, IMU_FIXED_HEIGHT_MAX));

    if (imu_confirm_timer(&p_ctx->tilt_check,
                          true,
                          now_usec,
                          IMU_HUMP_CONFIRM_USEC))
    {
        /* Fit 2-bit hump detected enum code into uint8_t setter parameter. */
        navigator_task_set_imu_event((uint8_t)IMU_MSG_HUMP_DETECTED);

        if (0u == p_ctx->runtime.b_is_hump_active)
        {
            p_ctx->runtime.b_is_hump_active = 1u;
            imu_send_nav_event(IMU_MSG_HUMP_DETECTED,
                               p_ctx->filt_pitch,
                               p_ctx->filt_roll,
                               height_mm,
                               true);
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[IMU HUMP] Detected! Tilt P:%d R:%d\n",
                      imu_trunc_to_int(p_ctx->filt_pitch),
                      imu_trunc_to_int(p_ctx->filt_roll));
        }
    }

    if ((0u != p_ctx->runtime.b_is_hump_active) &&
        (b_is_quasi_static) &&
        (height_mm > p_ctx->hump_peak_mm))
    {
        p_ctx->hump_peak_mm = height_mm;
    }
}

/**
 * @brief Update hump tilt detection, peak height estimate, and Navigator msgs.
 * @param[in,out] p_ctx             IMU task context.
 * @param[in]     now_usec          Current microsecond timestamp.
 * @param[in]     b_is_quasi_static True when acceleration is near 1 g.
 * @param[out]    p_height_mm       Live computed hump height in mm.
 * @param[out]    pb_is_tilted      True if filtered tilt exceeds threshold.
 */
static void imu_update_hump (imu_task_ctx_t * p_ctx,
                             uint32_t now_usec,
                             bool b_is_quasi_static,
                             float32_t * p_height_mm,
                             bool * pb_is_tilted)
{
    bool b_is_tilted =
        ((imu_abs_f32(p_ctx->filt_roll) >= IMU_HUMP_TILT_THRESH_DEG) ||
         (imu_abs_f32(p_ctx->filt_pitch) >= IMU_HUMP_TILT_THRESH_DEG));

    *p_height_mm  = 0.0f;
    *pb_is_tilted = b_is_tilted;

    if (b_is_tilted)
    {
        imu_handle_tilted_hump(p_ctx,
                               now_usec,
                               b_is_quasi_static,
                               p_height_mm);
    }
    else
    {
        navigator_task_set_imu_height(IMU_FIXED_HEIGHT_MIN);
        p_ctx->tilt_check.b_is_active = 0u;

        if ((imu_confirm_timer(&p_ctx->flat_check,
                               true,
                               now_usec,
                               IMU_HUMP_CONFIRM_USEC)) &&
            (0u != p_ctx->runtime.b_is_hump_active))
        {
            float32_t peak_mm = p_ctx->hump_peak_mm;

            p_ctx->runtime.b_is_hump_active = 0u;
            imu_send_nav_event(IMU_MSG_NO_HUMP,
                               p_ctx->filt_pitch,
                               p_ctx->filt_roll,
                               peak_mm,
                               true);
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[IMU HUMP] Returned to flat. Peak:%dmm\n",
                      imu_trunc_to_int(peak_mm));
            /* Fit 2-bit hump cleared enum code into uint8_t setter argument. */
            navigator_task_set_imu_event((uint8_t)IMU_MSG_NO_HUMP);
        }
    }
}

/**
 * @brief Latch E-stop, publish telemetry, and log confirmed safety stop cause.
 * @param[in,out] p_ctx       IMU task context.
 * @param[in]     p_sample    Latest filtered IMU sample.
 * @param[in]     reason      Confirmed instantaneous stop reason.
 * @param[in]     prev_reason Persisted stop reason enum value.
 * @param[in]     now_usec    Current microsecond timestamp.
 */
static void imu_latch_safety_stop (imu_task_ctx_t * p_ctx,
                                   const imu_task_sample_t * p_sample,
                                   imu_task_stop_reason_t reason,
                                   imu_task_stop_reason_t prev_reason,
                                   uint32_t now_usec)
{
    uint32_t    held_msec =
        (now_usec - p_ctx->safety_check.since_usec) / IMU_USEC_PER_MSEC;
    float32_t   total_mg  =
        imu_sqrt_f32(p_sample->total_g_sq) * IMU_G_TO_MG_SCALE;

    /* Store confirmed stop reason enum in 3-bit unsigned runtime field. */
    p_ctx->runtime.stop_reason        = (uint32_t)prev_reason;
    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    p_ctx->runtime.b_is_hump_active   = 0u;
    p_ctx->tilt_check.b_is_active     = 0u;
    p_ctx->flat_check.b_is_active     = 0u;
    p_ctx->recovery_check.b_is_active = 0u;

    robot_tasks_flags_update(ROBOT_FAULT_IMU_TILT, 0u);
    /* Advance 2-bit runtime state to safety-latched lifted/overturned state. */
    p_ctx->runtime.state = (uint32_t)IMU_STATE_LIFTED_OVERTURNED;

    /* Narrow 2-bit runtime state and event enum to uint8_t setter arguments. */
    navigator_task_set_imu_stop(
        robot_tasks_scale_tenths(p_ctx->filt_pitch,
                                 IMU_FIXED_TILT_MIN,
                                 IMU_FIXED_TILT_MAX),
        robot_tasks_scale_tenths(p_ctx->filt_roll,
                                 IMU_FIXED_TILT_MIN,
                                 IMU_FIXED_TILT_MAX),
        (uint8_t)p_ctx->runtime.state,
        (uint8_t)IMU_MSG_LIFTED_OVERTURNED);

    imu_send_nav_event(IMU_MSG_LIFTED_OVERTURNED,
                       p_ctx->filt_pitch,
                       p_ctx->filt_roll,
                       0.0f,
                       false);

    /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
    tm_printf((UB *)"[IMU] LIFTED/OVERTURNED -> STOP\n");
    /* Cast format literal and held duration to uT-Kernel UB* and UINT. */
    tm_printf((UB *)"[IMU] Cause=%s held=%ums rawP=%d rawR=%d\n",
              imu_get_reason_name(reason),
              (UINT)held_msec,
              imu_trunc_to_int(p_sample->pitch_deg),
              imu_trunc_to_int(p_sample->roll_deg));
    /* Cast format literal to uT-Kernel UB pointer for acceleration dump. */
    tm_printf((UB *)"[IMU] Accel mg X=%d Y=%d Z=%d G=%d\n",
              imu_trunc_to_int(p_sample->acc_x_g * IMU_G_TO_MG_SCALE),
              imu_trunc_to_int(p_sample->acc_y_g * IMU_G_TO_MG_SCALE),
              imu_trunc_to_int(p_sample->acc_z_g * IMU_G_TO_MG_SCALE),
              imu_trunc_to_int(total_mg));
}

/**
 * @brief Execute normal monitoring: hump tracking, telemetry, and safety stop.
 * @param[in,out] p_ctx    IMU task context.
 * @param[in]     p_sample Latest filtered IMU sample.
 * @param[in]     now_usec Current microsecond timestamp.
 */
static void imu_handle_monitoring (imu_task_ctx_t * p_ctx,
                                   const imu_task_sample_t * p_sample,
                                   uint32_t now_usec)
{
    bool                   b_is_quasi_static =
        ((p_sample->total_g_sq >= p_ctx->static_low_g_sq) &&
         (p_sample->total_g_sq <= p_ctx->static_high_g_sq));
    float32_t              height_mm         = 0.0f;
    bool                   b_is_tilted       = false;
    float32_t              safe_pitch        = 0.0f;
    float32_t              safe_roll         = 0.0f;
    imu_task_stop_reason_t reason            = IMU_STOP_NONE;
    /* Convert 3-bit runtime stop reason field to enum for confirmation. */
    imu_task_stop_reason_t prev_reason       =
        (imu_task_stop_reason_t)p_ctx->runtime.stop_reason;

    imu_update_hump(p_ctx,
                    now_usec,
                    b_is_quasi_static,
                    &height_mm,
                    &b_is_tilted);

    if ((now_usec - p_ctx->telem_usec) >= IMU_TELEMETRY_USEC)
    {
        int16_t heading = imu_get_heading_deg();

        p_ctx->telem_usec = now_usec;
        /* Narrow 2-bit runtime state field into uint8_t pose setter param. */
        navigator_task_set_imu_pose(
            robot_tasks_scale_tenths(p_ctx->filt_pitch,
                                     IMU_FIXED_TILT_MIN,
                                     IMU_FIXED_TILT_MAX),
            robot_tasks_scale_tenths(p_ctx->filt_roll,
                                     IMU_FIXED_TILT_MIN,
                                     IMU_FIXED_TILT_MAX),
            heading,
            true,
            (uint8_t)p_ctx->runtime.state);

        if ((!b_is_tilted) && (0u == p_ctx->runtime.b_is_hump_active))
        {
            navigator_task_set_imu_height(IMU_FIXED_HEIGHT_MIN);
            /* Fit 2-bit idle IMU event code into uint8_t setter argument. */
            navigator_task_set_imu_event((uint8_t)IMU_MSG_NONE);
        }

        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[IMU] Pitch:%d Roll:%d H:%dmm\n",
                  imu_trunc_to_int(p_ctx->filt_pitch),
                  imu_trunc_to_int(p_ctx->filt_roll),
                  imu_trunc_to_int(height_mm));
    }

    if ((imu_abs_f32(p_sample->med_pitch) > IMU_OVERTURN_TILT_DEG) &&
        ((p_sample->med_pitch * p_ctx->filt_pitch) > 0.0f))
    {
        safe_pitch = p_ctx->filt_pitch;
    }

    if ((imu_abs_f32(p_sample->med_roll) > IMU_OVERTURN_TILT_DEG) &&
        ((p_sample->med_roll * p_ctx->filt_roll) > 0.0f))
    {
        safe_roll = p_ctx->filt_roll;
    }

    reason = imu_eval_safety(safe_pitch,
                             safe_roll,
                             p_sample->med_z,
                             p_sample->total_g_sq,
                             p_ctx->baseline_z,
                             p_ctx->lift_low_g_sq,
                             p_ctx->lift_high_g_sq,
                             b_is_quasi_static);

    if (imu_confirm_safety(&p_ctx->safety_check,
                           &prev_reason,
                           reason,
                           now_usec))
    {
        imu_latch_safety_stop(p_ctx, p_sample, reason, prev_reason, now_usec);
    }
    else
    {
        /* Persist updated stop reason enum in 3-bit unsigned runtime field. */
        p_ctx->runtime.stop_reason = (uint32_t)prev_reason;
    }
}

/**
 * @brief Monitor upright stability after lift/overturn until resume.
 * @param[in,out] p_ctx    IMU task context.
 * @param[in]     p_sample Latest filtered IMU sample.
 * @param[in]     now_usec Current microsecond timestamp.
 */
static void imu_handle_lifted (imu_task_ctx_t * p_ctx,
                               const imu_task_sample_t * p_sample,
                               uint32_t now_usec)
{
    const float32_t flat_limit        = IMU_OVERTURN_TILT_DEG -
                                        IMU_RECOVERY_TILT_MARGIN;
    bool            b_is_upright_flat = false;

    navigator_task_set_imu_height(IMU_FIXED_HEIGHT_MIN);
    /* Narrow 2-bit runtime state field to uint8_t setter parameter. */
    navigator_task_set_imu_state((uint8_t)p_ctx->runtime.state);
    /* Fit 2-bit lifted/overturned event code into uint8_t setter parameter. */
    navigator_task_set_imu_event((uint8_t)IMU_MSG_LIFTED_OVERTURNED);

    if ((now_usec - p_ctx->telem_usec) >= IMU_TELEMETRY_USEC)
    {
        int16_t heading = imu_get_heading_deg();

        p_ctx->telem_usec = now_usec;
        navigator_task_set_imu_pose(
            robot_tasks_scale_tenths(p_ctx->filt_pitch,
                                     IMU_FIXED_TILT_MIN,
                                     IMU_FIXED_TILT_MAX),
            robot_tasks_scale_tenths(p_ctx->filt_roll,
                                     IMU_FIXED_TILT_MIN,
                                     IMU_FIXED_TILT_MAX),
            heading,
            false,
            0u);
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[IMU] LIFTED/OVER P:%d R:%d\n",
                  imu_trunc_to_int(p_ctx->filt_pitch),
                  imu_trunc_to_int(p_ctx->filt_roll));
    }

    b_is_upright_flat =
        ((imu_abs_f32(p_ctx->filt_pitch) < flat_limit) &&
         (imu_abs_f32(p_ctx->filt_roll) < flat_limit) &&
         (imu_abs_f32(p_sample->med_pitch) < flat_limit) &&
         (imu_abs_f32(p_sample->med_roll) < flat_limit) &&
         ((p_sample->med_z * p_ctx->baseline_z) >=
          (IMU_RECOVERY_Z_FACTOR * imu_sq_f32(p_ctx->baseline_z))) &&
         (p_sample->total_g_sq >= p_ctx->static_low_g_sq) &&
         (p_sample->total_g_sq <= p_ctx->static_high_g_sq));

    if (imu_confirm_timer(&p_ctx->recovery_check,
                          b_is_upright_flat,
                          now_usec,
                          IMU_RECOVERY_USEC))
    {
        p_ctx->recovery_check.b_is_active = 0u;
        p_ctx->safety_check.b_is_active   = 0u;
        p_ctx->tilt_check.b_is_active     = 0u;
        p_ctx->flat_check.b_is_active     = 0u;
        /* Reset 3-bit runtime stop reason field to IMU_STOP_NONE. */
        p_ctx->runtime.stop_reason        = (uint32_t)IMU_STOP_NONE;
        p_ctx->runtime.b_is_hump_active   = 0u;
        /* Restore 2-bit runtime state field to normal monitoring. */
        p_ctx->runtime.state              = (uint32_t)IMU_STATE_MONITORING;

        /* Fit 2-bit idle event code into uint8_t setter parameter. */
        navigator_task_set_imu_event((uint8_t)IMU_MSG_NONE);
        /* Narrow 2-bit runtime state field to uint8_t setter parameter. */
        navigator_task_set_imu_state((uint8_t)p_ctx->runtime.state);
        robot_tasks_flags_update(0u, ROBOT_FAULT_IMU_TILT);
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[IMU] Upright -> Monitoring resumed\n");
    }
}

#ifndef NDEBUG
/**
 * @brief Debug-only self-check verifying IMU safety detector and wrap timer.
 * @return True if all checks pass, false otherwise.
 */
static bool imu_detector_self_check (void)
{
    const uint32_t         step_usec = 20000u;
    const uint32_t         wrap_base = 0xFFFF0000u;
    imu_task_confirm_t     check     = { 0u, 0u, 0u };
    imu_task_stop_reason_t prev      = IMU_STOP_NONE;
    bool                   b_is_ok   = false;

    b_is_ok =
        ((IMU_STOP_NONE == imu_eval_safety(
              0.0f, 0.0f, 0.4f, 0.16f, 1.0f, 0.0625f, 4.0f, false)) &&
         (IMU_STOP_INVERTED == imu_eval_safety(
              0.0f, 0.0f, -1.0f, 1.0f, 1.0f, 0.0625f, 4.0f, true)) &&
         (IMU_STOP_TILT == imu_eval_safety(
              IMU_OVERTURN_TILT_DEG + 5.0f, 0.0f, 0.866f, 1.0f, 1.0f,
              0.0625f, 4.0f, true)) &&
         (IMU_STOP_NONE == imu_eval_safety(
              IMU_OVERTURN_TILT_DEG + 5.0f, 0.0f, 1.0f, 2.0f, 1.0f,
              0.0625f, 4.0f, false)) &&
         (IMU_STOP_LOW_G == imu_eval_safety(
              0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0625f, 4.0f, false)));

    for (uint32_t tick_usec = 0u;
         tick_usec <= IMU_TILT_CONFIRM_USEC;
         tick_usec += step_usec)
    {
        bool b_should_stop = imu_confirm_safety(&check,
                                                &prev,
                                                IMU_STOP_TILT,
                                                tick_usec);
        bool b_is_expected = (IMU_TILT_CONFIRM_USEC == tick_usec);

        b_is_ok = ((b_is_ok) && (b_should_stop == b_is_expected));
    }

    b_is_ok = ((b_is_ok) &&
               (!imu_confirm_safety(&check, &prev, IMU_STOP_NONE, 220000u)) &&
               (!imu_confirm_safety(&check, &prev, IMU_STOP_TILT, 240000u)) &&
               (!imu_confirm_safety(
                   &check, &prev, IMU_STOP_INVERTED, 260000u)));

    check.b_is_active = 0u;
    b_is_ok = ((b_is_ok) &&
               (!imu_confirm_timer(&check,
                                   true,
                                   wrap_base,
                                   IMU_LIFT_CONFIRM_USEC)) &&
               (imu_confirm_timer(&check,
                                  true,
                                  wrap_base + IMU_LIFT_CONFIRM_USEC,
                                  IMU_LIFT_CONFIRM_USEC)));

    return b_is_ok;
}
#endif /* NDEBUG */

/*** end of file ***/
