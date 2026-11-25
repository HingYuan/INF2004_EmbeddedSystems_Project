/** @file ultrasonic_test.c
 *
 * @brief Subsystem 3 (Ultrasonic, Pan Servo & Obstacle Bypass) unit tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#include "ultrasonic_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../motor/motor_task.h"
#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"
#include "../test_report.h"
#include "bypass_helpers.h"
#include "ultrasonic_task.h"

static void ultrasonic_test_cleanup(void);
static bool ultrasonic_test_check_samples_valid(void);
static bool ultrasonic_test_check_ray_pack(void);
static bool ultrasonic_test_check_classify_ray(void);
static bool ultrasonic_test_check_width_offset(void);
static bool ultrasonic_test_check_estimate_width(void);
static bool ultrasonic_test_check_plan_width(void);
static bool ultrasonic_test_check_bypass_moves(void);
static bool ultrasonic_test_check_fault_and_leg(void);

/**
 * @brief Execute all automated unit tests (`S3-U01`..`S3-U08`) for Subsystem 3.
 */
void ultrasonic_test_run (void)
{
    ultrasonic_test_cleanup();

    TEST_CHECK("S3-U01", ultrasonic_test_check_samples_valid());
    TEST_CHECK("S3-U02", ultrasonic_test_check_ray_pack());
    TEST_CHECK("S3-U03", ultrasonic_test_check_classify_ray());
    TEST_CHECK("S3-U04", ultrasonic_test_check_width_offset());
    TEST_CHECK("S3-U05", ultrasonic_test_check_estimate_width());
    TEST_CHECK("S3-U06", ultrasonic_test_check_plan_width());
    TEST_CHECK("S3-U07", ultrasonic_test_check_bypass_moves());
    TEST_CHECK("S3-U08", ultrasonic_test_check_fault_and_leg());

    ultrasonic_test_cleanup();
}

/**
 * @brief Drain motor queue and clear ranger flags and shared fault bits.
 */
static void ultrasonic_test_cleanup (void)
{
    motor_task_cmd_t cmd = { 0u };

    while (tk_rcv_mbf(robot_tasks_get_motor_mbf(), &cmd, TMO_POL) >= E_OK)
    {
        /* Discard queued bypass test commands. */
    }

    (void)tk_clr_flg(robot_tasks_get_ranger_flg(), 0u);
    robot_tasks_flags_update(0u, 0xFFFFFFFFu);
}

/**
 * @brief S3-U01: Verify 3-ping sorted sample spread validation.
 * @return True if sorted <=2 cm adjacent pairs pass and noisy/unsorted fail.
 */
static bool ultrasonic_test_check_samples_valid (void)
{
    return (bypass_helpers_samples_valid(20u, 20u, 20u) &&
            bypass_helpers_samples_valid(20u, 22u, 60u) &&
            bypass_helpers_samples_valid(10u, 48u, 50u) &&
            (!bypass_helpers_samples_valid(20u, 25u, 30u)) &&
            (!bypass_helpers_samples_valid(25u, 20u, 21u)));
}

/**
 * @brief S3-U02: Verify 7-ray 12-bit packed profile distance & status storage.
 * @return True if all 7 rays round-trip without cross-slot bit bleed.
 */
static bool ultrasonic_test_check_ray_pack (void)
{
    navigator_task_obs_profile_t prof    = { 0u };
    bool                         b_is_ok =
        (33u == sizeof(navigator_task_obs_profile_t));

    for (uint32_t idx = 0u; idx < SONAR_PROFILE_RAYS; idx++)
    {
        uint32_t                 dist_cm = (idx == 6u) ? 511u : (idx * 65u);
        ultrasonic_task_status_t st      = (ultrasonic_task_status_t)idx;

        navigator_task_set_ray(&prof, idx, dist_cm, st);
    }

    for (uint32_t idx = 0u; (idx < SONAR_PROFILE_RAYS) && b_is_ok; idx++)
    {
        uint32_t                 exp_cm = (idx == 6u) ? 511u : (idx * 65u);
        ultrasonic_task_status_t exp_st = (ultrasonic_task_status_t)idx;

        b_is_ok = ((exp_cm == navigator_task_get_ray_cm(&prof, idx)) &&
                   (exp_st == navigator_task_get_ray_status(&prof, idx)));
    }

    return b_is_ok;
}

/**
 * @brief S3-U03: Verify front-face vs clear vs uncertain ray classification.
 * @return True if plane tolerance, no-echo clear, and fault guards match.
 */
static bool ultrasonic_test_check_classify_ray (void)
{
    return ((WIDTH_RAY_FACE ==
             bypass_helpers_classify_ray(20u, 0, 20u, SONAR_OK)) &&
            (WIDTH_RAY_FACE ==
             bypass_helpers_classify_ray(20u, -30, 23u, SONAR_OK)) &&
            (WIDTH_RAY_CLEAR ==
             bypass_helpers_classify_ray(20u, 0, 30u, SONAR_OK)) &&
            (WIDTH_RAY_CLEAR ==
             bypass_helpers_classify_ray(20u, 30, 0u, SONAR_NO_ECHO)) &&
            (WIDTH_RAY_UNCERTAIN ==
             bypass_helpers_classify_ray(20u, 15, 0u, SONAR_UNSTABLE)) &&
            (WIDTH_RAY_UNCERTAIN ==
             bypass_helpers_classify_ray(20u, 0, 1u, SONAR_TOO_CLOSE)) &&
            (WIDTH_RAY_UNCERTAIN ==
             bypass_helpers_classify_ray(20u, 60, 20u, SONAR_OK)) &&
            (WIDTH_RAY_UNCERTAIN ==
             bypass_helpers_classify_ray(1u, 0, 20u, SONAR_OK)));
}

/**
 * @brief S3-U04: Verify 7.5-deg half-cone compensated lateral width offset.
 * @return True if <=7.5 deg clamps to 0, monotonic up to 45 deg, bounds safe.
 */
static bool ultrasonic_test_check_width_offset (void)
{
    uint32_t off_0  = bypass_helpers_width_offset_mm(20u, 0u);
    uint32_t off_7  = bypass_helpers_width_offset_mm(20u, 7u);
    uint32_t off_15 = bypass_helpers_width_offset_mm(20u, 15u);
    uint32_t off_30 = bypass_helpers_width_offset_mm(20u, 30u);
    uint32_t off_45 = bypass_helpers_width_offset_mm(20u, 45u);

    return ((0u == off_0) &&
            (0u == off_7) &&
            (off_15 > 0u) &&
            (off_30 > off_15) &&
            (off_45 > off_30) &&
            (off_45 <= 201u) &&
            (0u == bypass_helpers_width_offset_mm(20u, 46u)) &&
            (0u == bypass_helpers_width_offset_mm(401u, 30u)));
}

/**
 * @brief S3-U05: Verify dual-scan width merge, conflict, and span rejection.
 * @return True if estimated, unavailable, and inconsistent cases all hold.
 */
static bool ultrasonic_test_check_estimate_width (void)
{
    navigator_task_obs_profile_t first   = { 0u };
    navigator_task_obs_profile_t second  = { 0u };
    navigator_task_width_t       width   = { 0u, 0u };
    bool                         b_is_ok = true;

    first.front_cm                        = 20u;
    second.front_cm                       = 22u;
    first.measured_width.left_min_mm      = 80u;
    first.measured_width.left_max_mm      = 100u;
    first.measured_width.b_is_left_valid  = 1u;
    second.measured_width.right_min_mm    = 50u;
    second.measured_width.right_max_mm    = 70u;
    second.measured_width.b_is_right_valid = 1u;

    /* Complementary left+right edges across 2 cm reverse -> (90+60)/10=15 cm. */
    width   = bypass_helpers_estimate_width(&first, &second);
    b_is_ok = (((uint32_t)WIDTH_ESTIMATED == width.status) &&
               (15u == width.width_cm) &&
               ((uint32_t)WIDTH_UNAVAILABLE ==
                bypass_helpers_estimate_width(&first, NULL).status));

    /* Conflicting left edge brackets >20 mm apart -> WIDTH_INCONSISTENT. */
    first.measured_width.right_min_mm     = 50u;
    first.measured_width.right_max_mm     = 70u;
    first.measured_width.b_is_right_valid = 1u;
    second.measured_width.left_min_mm     = 180u;
    second.measured_width.left_max_mm     = 200u;
    second.measured_width.b_is_left_valid = 1u;
    b_is_ok = (b_is_ok &&
               ((uint32_t)WIDTH_INCONSISTENT ==
                bypass_helpers_estimate_width(&first, &second).status));

    /* Broad bracket >20 mm span on both scans -> WIDTH_UNAVAILABLE. */
    first.measured_width.left_min_mm  = 50u;
    first.measured_width.left_max_mm  = 100u;
    second.measured_width.left_min_mm = 50u;
    second.measured_width.left_max_mm = 100u;
    b_is_ok = (b_is_ok &&
               ((uint32_t)WIDTH_UNAVAILABLE ==
                bypass_helpers_estimate_width(&first, &second).status));

    return b_is_ok;
}

/**
 * @brief S3-U06: Verify detour side selection and outward shift planning.
 * @return True if bounded narrow side and unbounded fallback plan properly.
 */
static bool ultrasonic_test_check_plan_width (void)
{
    navigator_task_bypass_ctx_t  bypass  = { 0u };
    navigator_task_obs_profile_t second  = { 0u };
    bool                         b_is_ok = false;

    /* Both unbounded, right outer ray has NO_ECHO (401) vs left 50 cm -> R. */
    bypass.front_cm              = 20u;
    bypass.first_scan.front_cm   = 20u;
    second.front_cm              = 22u;
    second.left_scan_status      = (uint32_t)SONAR_OK;
    second.left_scan_cm          = 50u;
    second.right_scan_status     = (uint32_t)SONAR_NO_ECHO;
    bypass_helpers_plan_width(&bypass, &second);

    b_is_ok = ((-1 == navigator_task_get_side(&bypass)) &&
               (BYPASS_WIDTH_STEP_MM == bypass.planned_shift_mm));

    /* Left ray +15 deg (idx 4) open on both scans at 30 cm -> left 190 mm. */
    bypass                       = (navigator_task_bypass_ctx_t){ 0u };
    second                       = (navigator_task_obs_profile_t){ 0u };
    bypass.front_cm              = 30u;
    bypass.first_scan.front_cm   = 30u;
    second.front_cm              = 32u;
    navigator_task_set_ray(&bypass.first_scan, 4u, 0u, SONAR_NO_ECHO);
    navigator_task_set_ray(&second, 4u, 0u, SONAR_NO_ECHO);
    bypass_helpers_plan_width(&bypass, &second);

    return (b_is_ok &&
            (1 == navigator_task_get_side(&bypass)) &&
            ((90u + BYPASS_WIDTH_CLEAR_MM) == bypass.planned_shift_mm));
}

/**
 * @brief S3-U07: Verify bypass release, 20 mm scan-back, pivot, and limit abort.
 * @return True if queued commands and over-limit safety abort match FSM spec.
 */
static bool ultrasonic_test_check_bypass_moves (void)
{
    navigator_task_bypass_ctx_t bypass  = { 0u };
    motor_task_cmd_t            mot_cmd = { 0u };
    bool                        b_is_ok = true;

    ultrasonic_test_cleanup();

    /* Own obstacle E-stop then release via BYPASS_WAIT_RELEASE. */
    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, true);
    bypass.b_owns_obs_stop = 1u;
    bypass_helpers_begin(&bypass, &mot_cmd, BYPASS_WAIT_RELEASE, 0, 0, 0u);
    b_is_ok = (((uint32_t)BYPASS_WAIT_RELEASE == bypass.phase) &&
               (!motor_task_is_estop()) &&
               (!motor_task_is_stop_ack_req()));

    /* 20 mm reverse scan step + 90 deg left pivot + follow_next advance. */
    navigator_task_set_side(&bypass, 1);
    bypass.front_cm = 20u;
    bypass_helpers_begin(&bypass,
                         &mot_cmd,
                         BYPASS_SCAN_BACK,
                         -BYPASS_API_POWER,
                         BYPASS_API_POWER,
                         BYPASS_SCAN_BACK_MM);
    b_is_ok = (b_is_ok &&
               ((uint32_t)BYPASS_SCAN_BACK == bypass.phase) &&
               (1u == bypass.b_has_move_pending));
    ultrasonic_test_cleanup();

    bypass_helpers_pivot(&bypass, &mot_cmd, BYPASS_TURN_OUT, 1);
    b_is_ok = (b_is_ok && ((uint32_t)BYPASS_TURN_OUT == bypass.phase));
    ultrasonic_test_cleanup();

    bypass_helpers_follow_next(&bypass, &mot_cmd);
    b_is_ok = (b_is_ok &&
               ((uint32_t)BYPASS_ADVANCE == bypass.phase) &&
               (300u == bypass.leg_mm));
    ultrasonic_test_cleanup();

    /* Exceeding BYPASS_MAX_SIDE_MM (1500 mm) must abort and latch E-stop. */
    bypass.shift_mm = 1450u;
    bypass_helpers_begin(&bypass,
                         &mot_cmd,
                         BYPASS_SHIFT_OUT,
                         BYPASS_API_POWER,
                         -BYPASS_API_POWER,
                         100u);
    b_is_ok = (b_is_ok &&
               ((uint32_t)BYPASS_ABORT == bypass.phase) &&
               motor_task_is_estop());

    ultrasonic_test_cleanup();

    return b_is_ok;
}

/**
 * @brief S3-U08: Verify sonar/motor hardware fault flags and zero-leg abort.
 * @return True if fault predicates and zero-odometry completion abort work.
 */
static bool ultrasonic_test_check_fault_and_leg (void)
{
    navigator_task_bypass_ctx_t bypass  = { 0u };
    motor_task_cmd_t            mot_cmd = { 0u };
    bool                        b_is_ok = false;

    ultrasonic_test_cleanup();
    b_is_ok = ((!ultrasonic_task_has_fault()) &&
               (!bypass_helpers_has_hw_fault()));

    robot_tasks_flags_update(ROBOT_FAULT_SONAR, 0u);
    b_is_ok = (b_is_ok &&
               ultrasonic_task_has_fault() &&
               bypass_helpers_has_hw_fault());

    robot_tasks_flags_update(ROBOT_FAULT_SERVO, ROBOT_FAULT_SONAR);
    b_is_ok = (b_is_ok && ultrasonic_task_has_fault());

    robot_tasks_flags_update(MOTOR_FAULT_BIT_STALL, ROBOT_FAULT_SERVO);
    b_is_ok = (b_is_ok &&
               (!ultrasonic_task_has_fault()) &&
               bypass_helpers_has_hw_fault());

    ultrasonic_test_cleanup();

    /* Completing a translation leg with 0 mm encoder progress must abort. */
    bypass.phase        = (uint32_t)BYPASS_SHIFT_OUT;
    bypass.leg_mm       = 150u;
    bypass.leg_start_mm = motor_task_get_avg_dist_mm();
    b_is_ok = (b_is_ok &&
               (!bypass_helpers_complete_leg(&bypass, &mot_cmd)) &&
               ((uint32_t)BYPASS_ABORT == bypass.phase) &&
               motor_task_is_estop());

    ultrasonic_test_cleanup();

    return b_is_ok;
}

/*** end of file ***/
