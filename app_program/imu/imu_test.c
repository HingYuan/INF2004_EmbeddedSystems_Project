/**
 * @file imu_test.c
 * @brief Subsystem 4 (IMU Attitude, Hump Height & Safety) unit tests.
 *
 * BARR-C Deviation (Rules 5.2.a, 5.4.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Unit tests exercise robot_tasks_is_finite_f32() and
 *   robot_tasks_scale_tenths() single-precision float32_t boundary conversions
 *   and uT-Kernel message-buffer IPC APIs (ID, INT).
 */

#include "imu_test.h"

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../motor/motor_task.h"
#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"
#include "../test_report.h"
#include "imu_task.h"

#define IMU_TEST_POS_INF_BITS           (0x7F800000u)
#define IMU_TEST_NEG_INF_BITS           (0xFF800000u)
#define IMU_TEST_QNAN_BITS              (0x7FC00001u)
#define IMU_TEST_SNAN_BITS              (0x7F800001u)
#define IMU_TEST_ALL_FLAGS_MASK         (0xFFFFFFFFu)
#define IMU_TEST_IMU_MSG_BYTES          (6u)
#define IMU_TEST_TIMER_DELAY_USEC       (50u)

typedef union
{
    float32_t f32_val;
    uint32_t  u32_bits;
} imu_test_f32_bits_t;

static float32_t imu_test_bits_to_f32(uint32_t raw_bits);
static void imu_test_cleanup(void);
static bool imu_test_check_finite_f32(void);
static bool imu_test_check_scale_tenths(void);
static bool imu_test_check_pose_and_heading(void);
static bool imu_test_check_hump_telem(void);
static bool imu_test_check_stop_latch(void);
static bool imu_test_check_ipc_msg_pack(void);
static bool imu_test_check_fault_flags(void);
static bool imu_test_check_thresholds_timer(void);

/**
 * @brief Execute all automated unit tests (`S4-U01`..`S4-U08`) for Subsystem 4.
 */
void imu_test_run (void)
{
    imu_test_cleanup();

    TEST_CHECK("S4-U01", imu_test_check_finite_f32());
    TEST_CHECK("S4-U02", imu_test_check_scale_tenths());
    TEST_CHECK("S4-U03", imu_test_check_pose_and_heading());
    TEST_CHECK("S4-U04", imu_test_check_hump_telem());
    TEST_CHECK("S4-U05", imu_test_check_stop_latch());
    TEST_CHECK("S4-U06", imu_test_check_ipc_msg_pack());
    TEST_CHECK("S4-U07", imu_test_check_fault_flags());
    TEST_CHECK("S4-U08", imu_test_check_thresholds_timer());

    imu_test_cleanup();
}

/**
 * @brief Reinterpret a raw 32-bit IEEE-754 word as float32_t.
 * @param[in] raw_bits Raw IEEE-754 bit pattern.
 * @return Corresponding single-precision float value.
 */
static float32_t imu_test_bits_to_f32 (uint32_t raw_bits)
{
    imu_test_f32_bits_t conv = { 0.0f };

    conv.u32_bits = raw_bits;

    return conv.f32_val;
}

/**
 * @brief Reset IMU telemetry fields, Navigator queue, and shared flags.
 */
static void imu_test_cleanup (void)
{
    navigator_task_msg_t msg = { 0 };

    while (tk_rcv_mbf(robot_tasks_get_nav_mbf(), &msg, TMO_POL) >= E_OK)
    {
        /* Drain queued test messages. */
    }

    robot_tasks_flags_update(0u, IMU_TEST_ALL_FLAGS_MASK);
    /* Fit 2-bit IMU_STATE_INITIALIZE enum into uint8_t pose setter argument. */
    navigator_task_set_imu_pose(0, 0, 0, true, (uint8_t)IMU_STATE_INITIALIZE);
    navigator_task_set_imu_height(0);
    /* Fit 2-bit IMU_MSG_NONE enum into uint8_t event setter argument. */
    navigator_task_set_imu_event((uint8_t)IMU_MSG_NONE);
}

/**
 * @brief S4-U01: Verify IEEE-754 finiteness check on normal and Inf/NaN bits.
 * @return True if finite values pass and +/-Inf and quiet/signaling NaN fail.
 */
static bool imu_test_check_finite_f32 (void)
{
    float32_t pos_inf = imu_test_bits_to_f32(IMU_TEST_POS_INF_BITS);
    float32_t neg_inf = imu_test_bits_to_f32(IMU_TEST_NEG_INF_BITS);
    float32_t q_nan   = imu_test_bits_to_f32(IMU_TEST_QNAN_BITS);
    float32_t s_nan   = imu_test_bits_to_f32(IMU_TEST_SNAN_BITS);

    return ((robot_tasks_is_finite_f32(0.0f)) &&
            (robot_tasks_is_finite_f32(-0.0f)) &&
            (robot_tasks_is_finite_f32(1.0f)) &&
            (robot_tasks_is_finite_f32(-981.25f)) &&
            (!robot_tasks_is_finite_f32(pos_inf)) &&
            (!robot_tasks_is_finite_f32(neg_inf)) &&
            (!robot_tasks_is_finite_f32(q_nan)) &&
            (!robot_tasks_is_finite_f32(s_nan)));
}

/**
 * @brief S4-U02: Verify tenths fixed-point rounding, clamping, and Inf/NaN.
 * @return True if tilt [-1800, 1800] and height [0, 680] conversions match.
 */
static bool imu_test_check_scale_tenths (void)
{
    float32_t pos_inf = imu_test_bits_to_f32(IMU_TEST_POS_INF_BITS);
    float32_t neg_inf = imu_test_bits_to_f32(IMU_TEST_NEG_INF_BITS);
    float32_t q_nan   = imu_test_bits_to_f32(IMU_TEST_QNAN_BITS);

    return ((0 == robot_tasks_scale_tenths(0.0f, -1800, 1800)) &&
            (55 == robot_tasks_scale_tenths(5.54f, -1800, 1800)) &&
            (56 == robot_tasks_scale_tenths(5.56f, -1800, 1800)) &&
            (-251 == robot_tasks_scale_tenths(-25.06f, -1800, 1800)) &&
            (1800 == robot_tasks_scale_tenths(210.0f, -1800, 1800)) &&
            (-1800 == robot_tasks_scale_tenths(-210.0f, -1800, 1800)) &&
            (0 == robot_tasks_scale_tenths(-3.2f, 0, 680)) &&
            (185 == robot_tasks_scale_tenths(18.46f, 0, 680)) &&
            (680 == robot_tasks_scale_tenths(95.0f, 0, 680)) &&
            (1800 == robot_tasks_scale_tenths(pos_inf, -1800, 1800)) &&
            (-1800 == robot_tasks_scale_tenths(neg_inf, -1800, 1800)) &&
            (0 == robot_tasks_scale_tenths(q_nan, 0, 680)));
}

/**
 * @brief S4-U03: Verify 12-bit signed pose, heading, and optional state flag.
 * @return True if pitch/roll/heading and b_should_set_state behave atomically.
 */
static bool imu_test_check_pose_and_heading (void)
{
    navigator_task_telemetry_t snap    = { 0u };
    bool                       b_is_ok = false;

    /* Fit 2-bit IMU_STATE_MONITORING enum into uint8_t pose setter argument. */
    navigator_task_set_imu_pose(-1800,
                                1800,
                                359,
                                true,
                                (uint8_t)IMU_STATE_MONITORING);
    b_is_ok = (359 == navigator_task_get_heading());

    /* Update angles with b_should_set_state=false; state stays MONITORING. */
    navigator_task_set_imu_pose(-62,
                                145,
                                180,
                                false,
                                (uint8_t)IMU_STATE_INITIALIZE);
    navigator_task_read_telem(&snap);

    /* Compare 2-bit snapshot state field against IMU_STATE_MONITORING enum. */
    return ((b_is_ok) &&
            (180 == navigator_task_get_heading()) &&
            (-62 ==
             robot_tasks_sign_extend(snap.pitch_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            (145 ==
             robot_tasks_sign_extend(snap.roll_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            ((uint32_t)IMU_STATE_MONITORING == snap.imu_state));
}

/**
 * @brief S4-U04: Verify hump height (dmm) and hump event transitions.
 * @return True if height and event getters/snapshots track hump lifecycle.
 */
static bool imu_test_check_hump_telem (void)
{
    navigator_task_telemetry_t snap    = { 0u };
    bool                       b_is_ok = false;

    navigator_task_set_imu_height(315);
    /* Fit 2-bit IMU_MSG_HUMP_DETECTED enum into uint8_t setter argument. */
    navigator_task_set_imu_event((uint8_t)IMU_MSG_HUMP_DETECTED);
    /* Compare uint8_t getter return against IMU_MSG_HUMP_DETECTED enum. */
    b_is_ok = ((uint8_t)IMU_MSG_HUMP_DETECTED ==
               navigator_task_get_imu_event());
    navigator_task_read_telem(&snap);

    /* Compare 2-bit snapshot event field against IMU_MSG_HUMP_DETECTED enum. */
    b_is_ok = ((b_is_ok) &&
               (315u == snap.height_dmm) &&
               ((uint32_t)IMU_MSG_HUMP_DETECTED == snap.imu_event));

    navigator_task_set_imu_height(0);
    /* Fit 2-bit IMU_MSG_NO_HUMP enum into uint8_t setter argument. */
    navigator_task_set_imu_event((uint8_t)IMU_MSG_NO_HUMP);
    /* Compare uint8_t getter return against IMU_MSG_NO_HUMP enum. */
    b_is_ok = ((b_is_ok) &&
               ((uint8_t)IMU_MSG_NO_HUMP == navigator_task_get_imu_event()));

    return b_is_ok;
}

/**
 * @brief S4-U05: Verify navigator_task_set_imu_stop zeroes height and latches.
 * @return True if stop setter clears hump height and records tilt/state/event.
 */
static bool imu_test_check_stop_latch (void)
{
    navigator_task_telemetry_t snap = { 0u };

    navigator_task_set_imu_height(420);
    /* Fit 2-bit lifted/overturned state and event enums into uint8_t args. */
    navigator_task_set_imu_stop(-312,
                                285,
                                (uint8_t)IMU_STATE_LIFTED_OVERTURNED,
                                (uint8_t)IMU_MSG_LIFTED_OVERTURNED);
    navigator_task_read_telem(&snap);

    /* Verify 2-bit snapshot state and event fields match lifted/overturned. */
    return ((0u == snap.height_dmm) &&
            ((uint32_t)IMU_STATE_LIFTED_OVERTURNED == snap.imu_state) &&
            ((uint32_t)IMU_MSG_LIFTED_OVERTURNED == snap.imu_event) &&
            (-312 ==
             robot_tasks_sign_extend(snap.pitch_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            (285 ==
             robot_tasks_sign_extend(snap.roll_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)));
}

/**
 * @brief S4-U06: Verify 6-byte packed navigator_task_imu_msg_t MBF delivery.
 * @return True if signed pitch/roll, peak_mg, and height_dmm round-trip.
 */
static bool imu_test_check_ipc_msg_pack (void)
{
    ID                   h_nav = robot_tasks_get_nav_mbf();
    navigator_task_msg_t tx    = { 0 };
    navigator_task_msg_t rx    = { 0 };
    INT                  rx_sz = 0;

    /* Store MSG_IMU_UPDATE enum in 4-bit Navigator message type field. */
    tx.type                        = (uint32_t)MSG_IMU_UPDATE;
    /* Store IMU_MSG_HUMP_DETECTED enum in 2-bit IMU event field. */
    tx.data.imu_msg.event          = (uint32_t)IMU_MSG_HUMP_DETECTED;
    /* Narrow 12-bit packed signed pitch (-95) into uint16_t bit-field. */
    tx.data.imu_msg.pitch_ddeg_raw =
        (uint16_t)robot_tasks_pack_signed(-95, NAV_TILT_BIT_WIDTH);
    /* Narrow 12-bit packed signed roll (+72) into uint16_t bit-field. */
    tx.data.imu_msg.roll_ddeg_raw  =
        (uint16_t)robot_tasks_pack_signed(72, NAV_TILT_BIT_WIDTH);
    tx.data.imu_msg.peak_mg        = 1240u;
    tx.data.imu_msg.height_dmm     = 265u;

    /* Cast Navigator message size to RTOS INT parameter for MBF send. */
    if (E_OK == tk_snd_mbf(h_nav, &tx, (INT)sizeof(tx), TMO_POL))
    {
        rx_sz = tk_rcv_mbf(h_nav, &rx, TMO_POL);
    }

    /* Compare received INT size and unsigned bit-fields against expected. */
    return ((IMU_TEST_IMU_MSG_BYTES == sizeof(navigator_task_imu_msg_t)) &&
            ((INT)sizeof(rx) == rx_sz) &&
            ((uint32_t)MSG_IMU_UPDATE == rx.type) &&
            ((uint32_t)IMU_MSG_HUMP_DETECTED == rx.data.imu_msg.event) &&
            (-95 ==
             robot_tasks_sign_extend(rx.data.imu_msg.pitch_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            (72 ==
             robot_tasks_sign_extend(rx.data.imu_msg.roll_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            (1240u == rx.data.imu_msg.peak_mg) &&
            (265u == rx.data.imu_msg.height_dmm));
}

/**
 * @brief S4-U07: Verify IMU read/tilt fault flags and latched E-stop recovery.
 * @return True if clearing ROBOT_FAULT_IMU_READ leaves E-stop latched.
 */
static bool imu_test_check_fault_flags (void)
{
    bool b_is_ok = false;

    robot_tasks_flags_update(0u, IMU_TEST_ALL_FLAGS_MASK);

    /* Simulate 500 ms I2C read fault -> sets IMU_READ + ESTOP_REQUEST. */
    robot_tasks_flags_update(ROBOT_FAULT_IMU_READ | ROBOT_ESTOP_REQUEST, 0u);
    b_is_ok = ((motor_task_is_estop()) &&
               (0u != (robot_tasks_flags_get() & ROBOT_FAULT_IMU_READ)));

    /* Simulate I2C recovery -> clears IMU_READ while E-stop stays latched. */
    robot_tasks_flags_update(0u, ROBOT_FAULT_IMU_READ);
    b_is_ok = ((b_is_ok) &&
               (motor_task_is_estop()) &&
               (0u == (robot_tasks_flags_get() & ROBOT_FAULT_IMU_READ)));

    /* Tilt safety stop sets ROBOT_FAULT_IMU_TILT; recovery clears it. */
    robot_tasks_flags_update(ROBOT_FAULT_IMU_TILT, 0u);
    b_is_ok = ((b_is_ok) &&
               (0u != (robot_tasks_flags_get() & ROBOT_FAULT_IMU_TILT)));
    robot_tasks_flags_update(0u, ROBOT_FAULT_IMU_TILT);
    b_is_ok = ((b_is_ok) &&
               (0u == (robot_tasks_flags_get() & ROBOT_FAULT_IMU_TILT)));

    robot_tasks_flags_update(0u, IMU_TEST_ALL_FLAGS_MASK);

    return b_is_ok;
}

/**
 * @brief S4-U08: Verify LSM303 constants, windows, and hardware timer tick.
 * @return True if threshold hierarchy and 1-usec hardware timer advance hold.
 */
static bool imu_test_check_thresholds_timer (void)
{
    uint32_t t0_usec = robot_tasks_time_usec();
    uint32_t elapsed = 0u;

    robot_tasks_delay_usec(IMU_TEST_TIMER_DELAY_USEC);
    elapsed = robot_tasks_time_usec() - t0_usec;

    return ((0x19u == IMU_LSM303_ACCEL_ADDR) &&
            (0x1Eu == IMU_LSM303_MAG_ADDR) &&
            (IMU_HUMP_TILT_THRESH_DEG < IMU_OVERTURN_TILT_DEG) &&
            (80000u == IMU_HUMP_CONFIRM_USEC) &&
            (200000u == IMU_TILT_CONFIRM_USEC) &&
            (80000u == IMU_LIFT_CONFIRM_USEC) &&
            (500000u == IMU_RECOVERY_USEC) &&
            (500000u == IMU_READ_FAULT_USEC) &&
            (elapsed >= IMU_TEST_TIMER_DELAY_USEC));
}

/*** end of file ***/
