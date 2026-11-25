/** @file motor_test.c
 *
 * @brief Subsystem 1 (Motor Control & Wheel Encoders) automated unit tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#include "motor_test.h"

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../robot_tasks.h"
#include "../test_report.h"
#include "motor_task.h"

#define MOTOR_TEST_QUEUE_CAP            (10u)
#define MOTOR_TEST_DRIVE_PCT            (25)
#define MOTOR_TEST_DIST_50MM_TICKS      (151u)
#define MOTOR_TEST_PIVOT_90DEG_TICKS    (237u)
#define MOTOR_TEST_PIVOT_45DEG_TICKS    (118u)

static void motor_test_drain_queue(void);
static bool motor_test_pop_cmd(motor_task_cmd_t * p_cmd);
static bool motor_test_check_clamp(void);
static bool motor_test_check_cmd_layout(void);
static bool motor_test_check_fault_mask(void);
static bool motor_test_check_predicates(void);
static bool motor_test_check_odometry_init(void);
static bool motor_test_check_linear_moves(void);
static bool motor_test_check_pivots(void);
static bool motor_test_check_stop_and_cap(void);

/**
 * @brief Execute all automated unit tests (`S1-U01`..`S1-U08`) for Subsystem 1.
 */
void motor_test_run (void)
{
    TEST_CHECK("S1-U01", motor_test_check_clamp());
    TEST_CHECK("S1-U02", motor_test_check_cmd_layout());
    TEST_CHECK("S1-U03", motor_test_check_fault_mask());
    TEST_CHECK("S1-U04", motor_test_check_predicates());
    TEST_CHECK("S1-U05", motor_test_check_odometry_init());
    TEST_CHECK("S1-U06", motor_test_check_linear_moves());
    TEST_CHECK("S1-U07", motor_test_check_pivots());
    TEST_CHECK("S1-U08", motor_test_check_stop_and_cap());

    motor_test_drain_queue();
    robot_tasks_flags_update(0u, 0xFFFFFFFFu);
}

/**
 * @brief Drain all pending commands from the motor message buffer.
 */
static void motor_test_drain_queue (void)
{
    motor_task_cmd_t cmd = { 0u };

    while (tk_rcv_mbf(robot_tasks_get_motor_mbf(), &cmd, TMO_POL) >= E_OK)
    {
        /* Discard queued test commands. */
    }
}

/**
 * @brief Pop one queued motor command without blocking.
 * @param[out] p_cmd Output motor command structure.
 * @return True if a full 4-byte command was dequeued, false otherwise.
 */
static bool motor_test_pop_cmd (motor_task_cmd_t * p_cmd)
{
    INT rx_sz = tk_rcv_mbf(robot_tasks_get_motor_mbf(), p_cmd, TMO_POL);

    return ((INT)sizeof(motor_task_cmd_t) == rx_sz);
}

/**
 * @brief S1-U01: Verify signed PWM clamping across bounds and extremes.
 * @return True if all clamp cases match expected bounds.
 */
static bool motor_test_check_clamp (void)
{
    return ((0 == motor_task_clamp(0, -100, 100)) &&
            (-25 == motor_task_clamp(-25, -100, 100)) &&
            (100 == motor_task_clamp(100, -100, 100)) &&
            (-100 == motor_task_clamp(-100, -100, 100)) &&
            (100 == motor_task_clamp(101, -100, 100)) &&
            (-100 == motor_task_clamp(-101, -100, 100)) &&
            (100 == motor_task_clamp(INT32_MAX, -100, 100)) &&
            (-100 == motor_task_clamp(INT32_MIN, -100, 100)) &&
            (16 == motor_task_clamp(5, 16, 100)) &&
            (-8 == motor_task_clamp(-20, -8, 8)));
}

/**
 * @brief S1-U02: Verify 4-byte motor_task_cmd_t union packing and sign bits.
 * @return True if union fields pack without overlap or sign loss.
 */
static bool motor_test_check_cmd_layout (void)
{
    motor_task_cmd_t cmd = { 0u };

    cmd.fields.target_speed_left  = -100;
    cmd.fields.target_speed_right = 100;
    cmd.fields.turn_ticks_left    = 255u;
    cmd.fields.turn_ticks_right   = 42u;

    return ((4u == sizeof(motor_task_cmd_t)) &&
            (0u != cmd.raw) &&
            (-100 == cmd.fields.target_speed_left) &&
            (100 == cmd.fields.target_speed_right) &&
            (255u == cmd.fields.turn_ticks_left) &&
            (42u == cmd.fields.turn_ticks_right));
}

/**
 * @brief S1-U03: Verify motor fault bitmask isolation from non-motor flags.
 * @return True if only stall, encoder, and E-stop fault bits are returned.
 */
static bool motor_test_check_fault_mask (void)
{
    const uint32_t all_motor_faults =
        MOTOR_FAULT_BIT_STALL |
        MOTOR_FAULT_BIT_ENCODER |
        MOTOR_FAULT_BIT_ESTOP;
    const uint32_t other_flags =
        ROBOT_FAULT_SONAR |
        ROBOT_FAULT_WIFI |
        ROBOT_FAULT_IMU_TILT |
        ROBOT_ESTOP_REQUEST;
    bool b_is_ok = false;

    robot_tasks_flags_update(0u, 0xFFFFFFFFu);
    b_is_ok = (0u == motor_task_get_fault_word());

    robot_tasks_flags_update(other_flags, 0u);
    b_is_ok = (b_is_ok && (0u == motor_task_get_fault_word()));

    robot_tasks_flags_update(MOTOR_FAULT_BIT_STALL, 0u);
    b_is_ok = (b_is_ok && (MOTOR_FAULT_BIT_STALL == motor_task_get_fault_word()));

    robot_tasks_flags_update(all_motor_faults, 0u);
    b_is_ok = (b_is_ok && (all_motor_faults == motor_task_get_fault_word()));

    robot_tasks_flags_update(0u, MOTOR_FAULT_BIT_ENCODER);
    b_is_ok = (b_is_ok &&
               ((MOTOR_FAULT_BIT_STALL | MOTOR_FAULT_BIT_ESTOP) ==
                motor_task_get_fault_word()));

    robot_tasks_flags_update(0u, 0xFFFFFFFFu);

    return b_is_ok;
}

/**
 * @brief S1-U04: Verify E-stop, stop-ack, and encoder-degraded predicates.
 * @return True if each inline predicate tracks its exact flag bit.
 */
static bool motor_test_check_predicates (void)
{
    bool b_is_ok = false;

    robot_tasks_flags_update(0u, 0xFFFFFFFFu);
    b_is_ok = ((!motor_task_is_estop()) &&
               (!motor_task_is_stop_ack_req()) &&
               (!motor_task_is_enc_degraded()));

    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    b_is_ok = (b_is_ok &&
               motor_task_is_estop() &&
               (!motor_task_is_stop_ack_req()) &&
               (!motor_task_is_enc_degraded()));

    robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, true);
    robot_tasks_flag_assign(ROBOT_ENCODER_DEGRADED, true);
    b_is_ok = (b_is_ok &&
               motor_task_is_estop() &&
               motor_task_is_stop_ack_req() &&
               motor_task_is_enc_degraded());

    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, false);
    b_is_ok = (b_is_ok &&
               (!motor_task_is_estop()) &&
               motor_task_is_stop_ack_req() &&
               motor_task_is_enc_degraded());

    robot_tasks_flags_update(0u, 0xFFFFFFFFu);

    return b_is_ok;
}

/**
 * @brief S1-U05: Verify initial encoder/odometry baseline and scale constants.
 * @return True if initial odometry getters and calibration constants hold.
 */
static bool motor_test_check_odometry_init (void)
{
    return ((0 == motor_task_get_enc_left()) &&
            (0 == motor_task_get_enc_right()) &&
            (0 == motor_task_get_left_dist_mm()) &&
            (0 == motor_task_get_right_dist_mm()) &&
            (0 == motor_task_get_avg_dist_mm()) &&
            (0 == motor_task_get_avg_dist_cm()) &&
            (0 == motor_task_get_speed()) &&
            (0u == motor_task_get_drop_count()) &&
            (60 == MOTOR_WHEEL_DIAMETER_MM) &&
            (20 == MOTOR_ENCODER_SLOTS) &&
            (33 == MOTOR_MM_PER_TICK_X100) &&
            (100 == MOTOR_WHEELBASE_MM));
}

/**
 * @brief S1-U06: Verify forward and backward bounded translation commands.
 * @return True if queued speeds, polarity, and tick counts match specification.
 */
static bool motor_test_check_linear_moves (void)
{
    motor_task_cmd_t cmd     = { 0u };
    bool             b_is_ok = true;

    motor_test_drain_queue();

    /* 50 mm forward -> (50 * 100) / 33 = 151 ticks at (+25, -25). */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_move_forward(50)) &&
               motor_test_pop_cmd(&cmd) &&
               (MOTOR_TEST_DRIVE_PCT == cmd.fields.target_speed_left) &&
               ((-MOTOR_TEST_DRIVE_PCT) == cmd.fields.target_speed_right) &&
               (MOTOR_TEST_DIST_50MM_TICKS == cmd.fields.turn_ticks_left) &&
               (MOTOR_TEST_DIST_50MM_TICKS == cmd.fields.turn_ticks_right));

    /* 500 mm forward -> 1515 ticks; 8-bit wire field saturates at 255. */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_move_forward(500)) &&
               motor_test_pop_cmd(&cmd) &&
               (255u == cmd.fields.turn_ticks_left) &&
               (255u == cmd.fields.turn_ticks_right));

    /* 50 mm backward -> (-25, +25) with 151 ticks. */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_move_backward(50)) &&
               motor_test_pop_cmd(&cmd) &&
               ((-MOTOR_TEST_DRIVE_PCT) == cmd.fields.target_speed_left) &&
               (MOTOR_TEST_DRIVE_PCT == cmd.fields.target_speed_right) &&
               (MOTOR_TEST_DIST_50MM_TICKS == cmd.fields.turn_ticks_left));

    /* Non-positive distance -> 0 ticks. */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_move_forward(-10)) &&
               motor_test_pop_cmd(&cmd) &&
               (0u == cmd.fields.turn_ticks_left) &&
               (0u == cmd.fields.turn_ticks_right));

    return b_is_ok;
}

/**
 * @brief S1-U07: Verify left and right in-place pivot arc tick calculation.
 * @return True if 90/45/negative degree pivots produce exact tick goals.
 */
static bool motor_test_check_pivots (void)
{
    motor_task_cmd_t cmd     = { 0u };
    bool             b_is_ok = true;

    motor_test_drain_queue();

    /* 90 deg left -> ((314 * 100 * 90) / 360) / 33 = 237 ticks, (-25, -25). */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_turn_left(90)) &&
               motor_test_pop_cmd(&cmd) &&
               ((-MOTOR_TEST_DRIVE_PCT) == cmd.fields.target_speed_left) &&
               ((-MOTOR_TEST_DRIVE_PCT) == cmd.fields.target_speed_right) &&
               (MOTOR_TEST_PIVOT_90DEG_TICKS == cmd.fields.turn_ticks_left) &&
               (MOTOR_TEST_PIVOT_90DEG_TICKS == cmd.fields.turn_ticks_right));

    /* 90 deg right -> (+25, +25) with 237 ticks; negative angle uses abs(). */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_turn_right(-90)) &&
               motor_test_pop_cmd(&cmd) &&
               (MOTOR_TEST_DRIVE_PCT == cmd.fields.target_speed_left) &&
               (MOTOR_TEST_DRIVE_PCT == cmd.fields.target_speed_right) &&
               (MOTOR_TEST_PIVOT_90DEG_TICKS == cmd.fields.turn_ticks_left));

    /* 45 deg left -> 118 ticks. */
    b_is_ok = (b_is_ok &&
               (E_OK == motor_task_turn_left(45)) &&
               motor_test_pop_cmd(&cmd) &&
               (MOTOR_TEST_PIVOT_45DEG_TICKS == cmd.fields.turn_ticks_left));

    return b_is_ok;
}

/**
 * @brief S1-U08: Verify immediate stop command and 10-slot MBF capacity.
 * @return True if stop zeroes command fields and queue bounds hold.
 */
static bool motor_test_check_stop_and_cap (void)
{
    motor_task_cmd_t cmd     = { 0u };
    bool             b_is_ok = true;

    motor_test_drain_queue();

    b_is_ok = ((E_OK == motor_task_stop()) &&
               motor_test_pop_cmd(&cmd) &&
               (0u == cmd.raw));

    for (uint32_t idx = 0u; (idx < MOTOR_TEST_QUEUE_CAP) && b_is_ok; idx++)
    {
        b_is_ok = (E_OK == motor_task_stop());
    }

    /* 11th non-blocking send on full 10-entry queue must time out cleanly. */
    b_is_ok = (b_is_ok && (E_TMOUT == motor_task_stop()));

    motor_test_drain_queue();

    return b_is_ok;
}

/*** end of file ***/
