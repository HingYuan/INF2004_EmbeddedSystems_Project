/**
 * @file bypass_helpers.c
 * @brief Rectangular obstacle bypass and front-face width estimation helpers.
 */

#include "bypass_helpers.h"

#include <stdbool.h>
#include <stdint.h>

#include <tm/tmonitor.h>

#include "../robot_tasks.h"

#define BYPASS_MIN_SONAR_CM             (2u)
#define BYPASS_MAX_SONAR_CM             (400u)
#define BYPASS_NO_ECHO_ROOM_CM          (401u)
#define BYPASS_MAX_ANGLE_DEG            (45)
#define BYPASS_MAX_WIDTH_CM             (800u)
#define BYPASS_MAX_LEG_MM               (4095u)
#define BYPASS_MAX_SHIFT_MM_STORE       (2047u)
#define BYPASS_PLANE_DELTA_MAX_MM       (30)
#define BYPASS_SCAN_PAIR_COUNT          (2u)
#define BYPASS_HALF_RAY_COUNT           (3)
#define BYPASS_CENTER_RAY_IDX           (3)
#define BYPASS_RAY_STEP_DEG             (15)
#define BYPASS_PROBE_ATTEMPTS_MAX       (3u)
#define BYPASS_TRIG_TABLE_LEN           (4u)
#define BYPASS_SIDE_RIGHT               (-1)
#define BYPASS_SIDE_LEFT                (1)

/* Compile-time geometric and configuration invariants (Rule 5.5.a). */
ROBOT_TASKS_STATIC_ASSERT(bypass_turn_angles,
                          ((BYPASS_LEFT_TURN_DEG > 0u) &&
                           (BYPASS_RIGHT_TURN_DEG > 0u)));
ROBOT_TASKS_STATIC_ASSERT(bypass_pivot_clear,
                          ((BYPASS_PIVOT_CLEAR_MM > 0u) &&
                           (BYPASS_PIVOT_CLEAR_MM <= BYPASS_MAX_SIDE_MM)));
ROBOT_TASKS_STATIC_ASSERT(bypass_depth_clear,
                          ((BYPASS_DEPTH_CLEAR_MM > 0u) &&
                           (BYPASS_DEPTH_CLEAR_MM <= BYPASS_MAX_FORWARD_MM)));
ROBOT_TASKS_STATIC_ASSERT(bypass_pivot_pos,
                          (BYPASS_PIVOT_FROM_FRONT_CM <=
                           SONAR_ROBOT_LENGTH_CM));
ROBOT_TASKS_STATIC_ASSERT(
    bypass_swept_radius,
    (((BYPASS_SWEPT_RADIUS_CM * BYPASS_SWEPT_RADIUS_CM) >=
      ((BYPASS_PIVOT_FROM_FRONT_CM * BYPASS_PIVOT_FROM_FRONT_CM) +
       (SONAR_ROBOT_HALF_WIDTH_CM * SONAR_ROBOT_HALF_WIDTH_CM))) &&
     ((BYPASS_SWEPT_RADIUS_CM * BYPASS_SWEPT_RADIUS_CM) >=
      (((SONAR_ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM) *
        (SONAR_ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM)) +
       (SONAR_ROBOT_HALF_WIDTH_CM * SONAR_ROBOT_HALF_WIDTH_CM)))));
ROBOT_TASKS_STATIC_ASSERT(bypass_beam_half,
                          (BYPASS_WIDTH_BEAM_HALF_DDEG < 450u));

static uint32_t bypass_width_ray_rad (uint32_t angle_deg);
static uint32_t bypass_width_cosine (uint32_t angle_deg);
static bool bypass_width_merge_edge (bool b_valid_a,
                                     uint32_t min_a_mm,
                                     uint32_t max_a_mm,
                                     bool b_valid_b,
                                     uint32_t min_b_mm,
                                     uint32_t max_b_mm,
                                     uint32_t * p_midpoint,
                                     bool * p_b_conflict);
static bypass_helpers_ray_class_t bypass_classify_ok_ray (uint32_t front_cm,
                                                          uint32_t abs_ang,
                                                          uint32_t dist_cm);
static bool bypass_align_width_scans (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second,
    navigator_task_width_scan_t * p_scan_a,
    navigator_task_width_scan_t * p_scan_b);
static void bypass_apply_probe_decision (
    navigator_task_bypass_ctx_t * p_bypass,
    motor_task_cmd_t * p_cmd,
    bool b_clear,
    const navigator_task_sonar_probe_t * p_probe);
static void bypass_eval_fresh_probe (
    navigator_task_bypass_ctx_t * p_bypass,
    motor_task_cmd_t * p_cmd,
    const navigator_task_sonar_probe_t * p_probe);
static uint32_t bypass_refine_fine_offset (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second,
    int32_t side_sign,
    int32_t occupied,
    uint32_t edge_cm);
static uint32_t bypass_side_room (ultrasonic_task_status_t status,
                                  uint32_t dist_cm);
static uint32_t bypass_side_offset (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second,
    int32_t side_sign,
    uint32_t front_cm,
    bool * p_b_known);
static motor_task_cmd_t bypass_pivot_cmd (int32_t dir_sign);
static const char * bypass_phase_name (navigator_task_bypass_phase_t phase);

/**
 * @brief Classify one sonar ray relative to the obstacle front face plane.
 * @param[in] front_cm  Measured center standoff in cm.
 * @param[in] angle_deg Signed servo angle in degrees (-45..45).
 * @param[in] dist_cm   Measured ray distance in cm.
 * @param[in] status    Ultrasonic status code.
 * @return Classification of ray as face, clear, or uncertain.
 */
bypass_helpers_ray_class_t bypass_helpers_classify_ray (
    uint32_t front_cm,
    int32_t angle_deg,
    uint32_t dist_cm,
    ultrasonic_task_status_t status)
{
    bypass_helpers_ray_class_t ray_cls = WIDTH_RAY_UNCERTAIN;

    if ((front_cm >= BYPASS_MIN_SONAR_CM) &&
        (front_cm <= BYPASS_MAX_SONAR_CM) &&
        (angle_deg >= (-BYPASS_MAX_ANGLE_DEG)) &&
        (angle_deg <= BYPASS_MAX_ANGLE_DEG))
    {
        uint32_t abs_ang = (angle_deg < 0)
                           ? ((uint32_t)(-angle_deg))
                           : ((uint32_t)angle_deg);

        if ((SONAR_NO_ECHO == status) &&
            ((front_cm * 1000u) <=
             (BYPASS_MAX_SONAR_CM * bypass_width_cosine(abs_ang))))
        {
            ray_cls = WIDTH_RAY_CLEAR;
        }
        else if ((SONAR_OK == status) &&
                 (dist_cm >= BYPASS_MIN_SONAR_CM) &&
                 (dist_cm <= BYPASS_MAX_SONAR_CM))
        {
            ray_cls = bypass_classify_ok_ray(front_cm, abs_ang, dist_cm);
        }
        else
        {
            ray_cls = WIDTH_RAY_UNCERTAIN;
        }
    }

    return ray_cls;
}

/**
 * @brief Convert a beam-compensated ray angle at front_cm to lateral mm offset.
 * @param[in] front_cm  Front standoff distance in cm.
 * @param[in] angle_deg Unsigned ray angle magnitude in degrees (0..45).
 * @return Lateral offset in millimetres.
 */
uint32_t bypass_helpers_width_offset_mm (uint32_t front_cm, uint32_t angle_deg)
{
    uint32_t offset_mm = 0u;

    if ((front_cm <= BYPASS_MAX_SONAR_CM) &&
        (angle_deg <= (uint32_t)BYPASS_MAX_ANGLE_DEG))
    {
        uint32_t rad_val = bypass_width_ray_rad(angle_deg);
        uint32_t sq_val  = (rad_val * rad_val) / 1000u;
        uint32_t sin_val = (rad_val - ((rad_val * sq_val) / 6000u)) +
                           (((rad_val * sq_val) * sq_val) / 120000000u);
        uint32_t cos_val = bypass_width_cosine(angle_deg);

        offset_mm = (((front_cm * 10u) * sin_val) + (cos_val / 2u)) / cos_val;
    }

    return offset_mm;
}

/**
 * @brief Check whether three sorted sonar distance samples are consistent.
 * @param[in] low_cm    Smallest sample in cm.
 * @param[in] middle_cm Median sample in cm.
 * @param[in] high_cm   Largest sample in cm.
 * @return True if sorted and at least one adjacent gap is within tolerance.
 */
bool bypass_helpers_samples_valid (uint32_t low_cm,
                                   uint32_t middle_cm,
                                   uint32_t high_cm)
{
    return ((low_cm <= middle_cm) &&
            (middle_cm <= high_cm) &&
            (((middle_cm - low_cm) <= BYPASS_WIDTH_SAMPLE_SPREAD_CM) ||
             ((high_cm - middle_cm) <= BYPASS_WIDTH_SAMPLE_SPREAD_CM)));
}

/**
 * @brief Combine one or two obstacle scans into a dashboard width estimate.
 * @param[in] p_first  First stationary obstacle profile.
 * @param[in] p_second Optional second profile taken after 2 cm reverse.
 * @return Packed obstacle width and status result.
 */
navigator_task_width_t bypass_helpers_estimate_width (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second)
{
    navigator_task_width_t      result   = { 0u, (uint32_t)WIDTH_UNAVAILABLE };
    navigator_task_width_scan_t scan_a   = p_first->measured_width;
    navigator_task_width_scan_t scan_b   = { 0u };
    bool                        b_usable =
        bypass_align_width_scans(p_first, p_second, &scan_a, &scan_b);

    if (b_usable)
    {
        uint32_t left_mm    = 0u;
        uint32_t right_mm   = 0u;
        bool     b_conflict = false;
        bool     b_left_ok  = bypass_width_merge_edge(
            (0u != scan_a.left_valid),
            scan_a.left_min_mm,
            scan_a.left_max_mm,
            (0u != scan_b.left_valid),
            scan_b.left_min_mm,
            scan_b.left_max_mm,
            &left_mm,
            &b_conflict);
        bool     b_right_ok = bypass_width_merge_edge(
            (0u != scan_a.right_valid),
            scan_a.right_min_mm,
            scan_a.right_max_mm,
            (0u != scan_b.right_valid),
            scan_b.right_min_mm,
            scan_b.right_max_mm,
            &right_mm,
            &b_conflict);

        if (b_conflict)
        {
            result.status = (uint32_t)WIDTH_INCONSISTENT;
        }
        else if (b_left_ok && b_right_ok)
        {
            uint32_t total_cm = ((left_mm + right_mm) + 5u) / 10u;
            bool     b_in_rng = ((total_cm > 0u) &&
                                 (total_cm <= BYPASS_MAX_WIDTH_CM));

            result.width_cm = b_in_rng ? total_cm : 0u;
            result.status   = b_in_rng
                              ? (uint32_t)WIDTH_ESTIMATED
                              : (uint32_t)WIDTH_UNAVAILABLE;
        }
        else
        {
            /* Keep default WIDTH_UNAVAILABLE status. */
        }
    }

    return result;
}

/**
 * @brief Print the obstacle width result to the serial console.
 * @param[in] width Packed obstacle width and status structure.
 */
void bypass_helpers_report_width (navigator_task_width_t width)
{
    switch ((navigator_task_width_status_t)width.status)
    {
        case WIDTH_ESTIMATED:
            tm_printf((UB *)"[width] Dashboard: %u cm (estimated)\n",
                      (UINT)width.width_cm);
        break;

        case WIDTH_SCANNING:
            tm_printf((UB *)"[width] Dashboard: Scanning... "
                      "waiting for the second scan\n");
        break;

        case WIDTH_INCONSISTENT:
            tm_printf((UB *)"[width] Dashboard: Inconsistent scans; "
                      "edge positions disagree\n");
        break;

        default:
            tm_printf((UB *)"[width] Dashboard: Unavailable; "
                      "both edges could not be established\n");
        break;
    }
}

#ifndef NDEBUG
/**
 * @brief Run debug-only unit checks for obstacle width estimation.
 * @return True if all assertions pass, false otherwise.
 */
bool bypass_helpers_width_check (void)
{
    navigator_task_obs_profile_t first  = { 0u };
    navigator_task_obs_profile_t second = { 0u };
    navigator_task_width_t       width  = { 0u, 0u };
    bool                         b_ok   = true;

    first.front_cm                    = 20u;
    second.front_cm                   = 22u;
    first.measured_width.left_min_mm  = 80u;
    first.measured_width.left_max_mm  = 100u;
    first.measured_width.left_valid   = 1u;
    second.measured_width.right_min_mm = 50u;
    second.measured_width.right_max_mm = 70u;
    second.measured_width.right_valid  = 1u;

    width = bypass_helpers_estimate_width(&first, &second);
    b_ok  = (b_ok &&
             ((uint32_t)WIDTH_ESTIMATED == width.status) &&
             (15u == width.width_cm) &&
             ((uint32_t)WIDTH_UNAVAILABLE ==
              bypass_helpers_estimate_width(&first, NULL).status));

    first.measured_width.right_valid  = 1u;
    first.measured_width.right_min_mm = 50u;
    first.measured_width.right_max_mm = 70u;
    second.measured_width             = (navigator_task_width_scan_t){ 0u };
    width = bypass_helpers_estimate_width(&first, &second);
    b_ok  = (b_ok &&
             ((uint32_t)WIDTH_ESTIMATED == width.status) &&
             (15u == width.width_cm));

    second.measured_width.left_min_mm = 180u;
    second.measured_width.left_max_mm = 200u;
    second.measured_width.left_valid  = 1u;
    b_ok = (b_ok &&
            ((uint32_t)WIDTH_INCONSISTENT ==
             bypass_helpers_estimate_width(&first, &second).status));

    second.measured_width.left_min_mm = 50u;
    second.measured_width.left_max_mm = 200u;
    width = bypass_helpers_estimate_width(&first, &second);
    b_ok  = (b_ok &&
             ((uint32_t)WIDTH_ESTIMATED == width.status) &&
             (15u == width.width_cm));

    /* Regression check: broad spans must not inflate a 12 cm box to 23 cm. */
    first                              = (navigator_task_obs_profile_t){ 0u };
    second                             = (navigator_task_obs_profile_t){ 0u };
    first.front_cm                     = 18u;
    second.front_cm                    = 22u;
    first.measured_width.left_min_mm   = 135u;
    first.measured_width.left_max_mm   = 180u;
    first.measured_width.left_valid    = 1u;
    second.measured_width.left_min_mm  = 89u;
    second.measured_width.left_max_mm  = 127u;
    second.measured_width.left_valid   = 1u;
    second.measured_width.right_min_mm = 89u;
    second.measured_width.right_max_mm = 107u;
    second.measured_width.right_valid  = 1u;
    b_ok = (b_ok &&
            ((uint32_t)WIDTH_UNAVAILABLE ==
             bypass_helpers_estimate_width(&first, &second).status));

    b_ok = (b_ok &&
            (WIDTH_RAY_CLEAR ==
             bypass_helpers_classify_ray(20u, 0, 30u, SONAR_OK)) &&
            (WIDTH_RAY_FACE ==
             bypass_helpers_classify_ray(20u, 0, 20u, SONAR_OK)) &&
            (WIDTH_RAY_UNCERTAIN ==
             bypass_helpers_classify_ray(20u, 30, 0u, SONAR_UNSTABLE)) &&
            (0u == bypass_helpers_width_offset_mm(20u, 0u)) &&
            (bypass_helpers_width_offset_mm(20u, 45u) <= 201u) &&
            bypass_helpers_samples_valid(20u, 21u, 60u) &&
            (!bypass_helpers_samples_valid(20u, 30u, 60u)));

    return b_ok;
}
#endif /* NDEBUG */

/**
 * @brief Choose detour side and initial outward shift from two sonar profiles.
 * @param[in,out] p_bypass Bypass state context.
 * @param[in]     p_second Second obstacle profile after 2 cm reverse.
 */
void bypass_helpers_plan_width (navigator_task_bypass_ctx_t * p_bypass,
                                const navigator_task_obs_profile_t * p_second)
{
    const uint32_t back_cm    = BYPASS_SCAN_BACK_MM / 10u;
    bool           b_left_ok  = false;
    bool           b_right_ok = false;
    uint32_t       left_mm    = 0u;
    uint32_t       right_mm   = 0u;
    uint32_t       left_room  = 0u;
    uint32_t       right_room = 0u;
    int32_t        chosen     = BYPASS_SIDE_RIGHT;
    bool           b_known    = false;

    if ((p_second->front_cm >= (back_cm + BYPASS_MIN_SONAR_CM)) &&
        (p_second->front_cm <=
         ((p_bypass->front_cm + BYPASS_PROFILE_DEPTH_CM) + back_cm)))
    {
        uint32_t orig_front = p_second->front_cm - back_cm;

        if (orig_front > p_bypass->front_cm)
        {
            p_bypass->front_cm = orig_front;
        }
    }

    left_mm    = bypass_side_offset(&p_bypass->first_scan,
                                    p_second,
                                    BYPASS_SIDE_LEFT,
                                    p_bypass->front_cm,
                                    &b_left_ok);
    right_mm   = bypass_side_offset(&p_bypass->first_scan,
                                    p_second,
                                    BYPASS_SIDE_RIGHT,
                                    p_bypass->front_cm,
                                    &b_right_ok);
    left_room  = bypass_side_room(
        (ultrasonic_task_status_t)p_second->left_scan_status,
        p_second->left_scan_cm);
    right_room = bypass_side_room(
        (ultrasonic_task_status_t)p_second->right_scan_status,
        p_second->right_scan_cm);

    if (b_left_ok != b_right_ok)
    {
        chosen = b_left_ok ? BYPASS_SIDE_LEFT : BYPASS_SIDE_RIGHT;
    }
    else if ((left_mm < right_mm) ||
             ((left_mm == right_mm) && (left_room >= right_room)))
    {
        chosen = BYPASS_SIDE_LEFT;
    }
    else
    {
        chosen = BYPASS_SIDE_RIGHT;
    }

    navigator_task_set_side(p_bypass, chosen);
    b_known = (chosen > 0) ? b_left_ok : b_right_ok;
    p_bypass->planned_shift_mm = b_known
                                 ? ((chosen > 0) ? left_mm : right_mm)
                                 : BYPASS_WIDTH_STEP_MM;
    p_bypass->width_clr_pend   = 0u;
    p_bypass->depth_clr_pend   = 0u;

    tm_printf((UB *)"[bypass] Dynamic offsets L=%u R=%u mm (0=unknown); "
              "pass %s, edge=%s, front=%u cm, first step=%u mm\n",
              (UINT)left_mm,
              (UINT)right_mm,
              (chosen > 0) ? "left" : "right",
              b_known ? "bounded by both scans" : "unmeasured: verify outward",
              (UINT)p_bypass->front_cm,
              (UINT)p_bypass->planned_shift_mm);
}

#ifndef NDEBUG
/**
 * @brief Run debug-only unit checks for bypass offset and pivot helpers.
 * @return True if all assertions pass, false otherwise.
 */
bool bypass_helpers_self_check (void)
{
    static const int32_t sides[BYPASS_SCAN_PAIR_COUNT] =
    {
        BYPASS_SIDE_RIGHT,
        BYPASS_SIDE_LEFT
    };
    bool b_ok = (0u == bypass_pivot_cmd(0).raw);

    for (uint32_t idx = 0u; idx < BYPASS_SCAN_PAIR_COUNT; idx++)
    {
        int32_t                      side   = sides[idx];
        navigator_task_obs_profile_t first  = { 0u };
        navigator_task_obs_profile_t second = { 0u };
        bool                         b_kn   = false;
        motor_task_cmd_t             pivot  = { 0u };
        uint32_t                     ray1   = (uint32_t)(BYPASS_CENTER_RAY_IDX +
                                                         side);
        uint32_t                     ray3   = (uint32_t)(BYPASS_CENTER_RAY_IDX +
                                                         (side * 3));

        first.front_cm  = 30u;
        second.front_cm = 32u;

        b_ok = (b_ok &&
                (0u == bypass_side_offset(&first, &second, side, 30u, &b_kn)) &&
                (!b_kn));

        navigator_task_ray_set(&first, ray1, 0u, SONAR_NO_ECHO);
        navigator_task_ray_set(&second, ray1, 0u, SONAR_NO_ECHO);
        b_ok = (b_ok &&
                ((90u + BYPASS_WIDTH_CLEAR_MM) ==
                 bypass_side_offset(&first, &second, side, 30u, &b_kn)) &&
                b_kn);

        navigator_task_ray_set(&first, ray3, 43u, SONAR_OK);
        navigator_task_ray_set(&second, ray3, 45u, SONAR_OK);
        b_ok = (b_ok &&
                (0u == bypass_side_offset(&first, &second, side, 30u, &b_kn)) &&
                (!b_kn));

        pivot = bypass_pivot_cmd(side);
        b_ok  = (b_ok &&
                 (((-side) * BYPASS_API_POWER) ==
                  (int32_t)pivot.fields.target_speed_left) &&
                 (pivot.fields.target_speed_right ==
                  pivot.fields.target_speed_left) &&
                 (0u == pivot.fields.turn_ticks_left) &&
                 (0u == pivot.fields.turn_ticks_right));
    }

    return b_ok;
}
#endif /* NDEBUG */

/**
 * @brief Check whether a motor stall, encoder, or sonar fault is active.
 * @return True if hardware fault requires bypass abort.
 */
bool bypass_helpers_has_hw_fault (void)
{
    const uint32_t motor_mask = MOTOR_FAULT_BIT_STALL | MOTOR_FAULT_BIT_ENCODER;

    return (ultrasonic_task_has_fault() ||
            (0u != (motor_task_get_fault_word() & motor_mask)));
}

/**
 * @brief Abort bypass detour, latch E-stop, and halt motors safely.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer zeroed on abort.
 * @param[in]     p_reason Diagnostic abort reason string.
 */
void bypass_helpers_abort (navigator_task_bypass_ctx_t * p_bypass,
                           motor_task_cmd_t * p_cmd,
                           const char * p_reason)
{
    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, false);
    p_bypass->owns_obs_stop = 0u;
    p_bypass->phase         = (uint32_t)BYPASS_ABORT;
    p_bypass->move_pending  = 0u;
    p_bypass->deadline_usec = 0u;
    p_cmd->raw              = 0u;
    (void)motor_task_stop();

    tm_printf((UB *)"[bypass] Fault: %s (motor=%u sonar=%d IMU=%u)\n",
              p_reason,
              (UINT)motor_task_get_fault_word(),
              ultrasonic_task_has_fault() ? 1 : 0,
              (UINT)navigator_task_get_imu_event());
}

/**
 * @brief Queue one measured bypass movement and arm its fault deadline.
 * @param[in,out] p_bypass   Bypass state context.
 * @param[out]    p_cmd      Motor command tracking structure.
 * @param[in]     phase      Target bypass phase.
 * @param[in]     left_spd   Left motor power percentage (-100..100).
 * @param[in]     right_spd  Right motor power percentage (-100..100).
 * @param[in]     amount_val Distance (mm) or pivot angle (deg).
 */
void bypass_helpers_begin (navigator_task_bypass_ctx_t * p_bypass,
                           motor_task_cmd_t * p_cmd,
                           navigator_task_bypass_phase_t phase,
                           int32_t left_spd,
                           int32_t right_spd,
                           uint32_t amount_val)
{
    bool b_aborted = false;

    if (bypass_helpers_has_hw_fault() ||
        (motor_task_is_estop() && (0u == p_bypass->owns_obs_stop)))
    {
        bypass_helpers_abort(p_bypass, p_cmd, "motor stop or sonar fault");
        b_aborted = true;
    }

    if (!b_aborted)
    {
        ER           result     = E_OK;
        uint32_t     tmo_msec   = 0u;
        const char * p_unit     = "mm";

        p_bypass->phase            = (uint32_t)phase;
        p_bypass->phase_start_usec = robot_tasks_time_usec();
        p_bypass->move_pending     = 0u;
        p_bypass->deadline_usec    = 0u;
        p_bypass->leg_mm           = 0u;
        p_bypass->leg_start_mm     = motor_task_get_avg_dist_mm();
        p_cmd->raw                 = 0u;
        /* Range-safe: callers pass speed percentages in [-100, 100]. */
        p_cmd->fields.target_speed_left  = (int8_t)left_spd;
        p_cmd->fields.target_speed_right = (int8_t)right_spd;

        switch (phase)
        {
            case BYPASS_WAIT_RELEASE:
                if (0u == p_bypass->owns_obs_stop)
                {
                    bypass_helpers_abort(p_bypass,
                                         p_cmd,
                                         "scan stop not owned");
                    b_aborted = true;
                }
                else
                {
                    p_bypass->owns_obs_stop = 0u;
                    robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, false);
                    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, false);
                    result   = E_OK;
                    tmo_msec = BYPASS_RESPONSE_MSEC;
                }
            break;

            case BYPASS_TURN_OUT:
            case BYPASS_TURN_PARALLEL:
            case BYPASS_TURN_IN:
            case BYPASS_WIDTH_TURN_OUT:
            case BYPASS_WIDTH_ALIGN:
            case BYPASS_DEPTH_TURN_BACK:
                /* Range-safe: turn angles are 90 degrees. */
                result = (left_spd < 0)
                         ? motor_task_turn_left((int32_t)amount_val)
                         : motor_task_turn_right((int32_t)amount_val);
                tmo_msec = BYPASS_TURN_TIMEOUT_MSEC;
                p_unit   = "deg";
            break;

            case BYPASS_SHIFT_OUT:
            case BYPASS_ADVANCE:
            case BYPASS_FIND_LINE:
            case BYPASS_WIDTH_EXTEND:
            case BYPASS_SCAN_RETURN:
            {
                bool b_side_phase = ((BYPASS_SHIFT_OUT == phase) ||
                                     (BYPASS_WIDTH_EXTEND == phase));
                bool b_over_limit =
                    ((0u == amount_val) ||
                     (amount_val > BYPASS_MAX_LEG_MM) ||
                     (b_side_phase &&
                      ((amount_val + p_bypass->shift_mm) >
                       BYPASS_MAX_SIDE_MM)) ||
                     ((BYPASS_ADVANCE == phase) &&
                      ((amount_val + p_bypass->forward_mm) >
                       BYPASS_MAX_FORWARD_MM)));

                if (b_over_limit)
                {
                    bypass_helpers_abort(p_bypass,
                                         p_cmd,
                                         "exploration distance limit");
                    b_aborted = true;
                }
                else
                {
                    p_bypass->leg_mm = amount_val;
                    /* Range-safe: amount_val is in [1, 4095]. */
                    result   = motor_task_move_forward((int32_t)amount_val);
                    tmo_msec = BYPASS_MOVE_MARGIN_MSEC +
                               (((amount_val + 9u) / 10u) *
                                BYPASS_MOVE_TMO_MSEC_CM);
                }
            }
            break;

            case BYPASS_SCAN_BACK:
                p_bypass->leg_mm = BYPASS_SCAN_BACK_MM;
                result = motor_task_move_backward((int32_t)BYPASS_SCAN_BACK_MM);
                tmo_msec = BYPASS_MOVE_MARGIN_MSEC +
                           ((BYPASS_SCAN_BACK_MM / 10u) *
                            BYPASS_MOVE_TMO_MSEC_CM);
            break;

            case BYPASS_LINE_FOUND:
                result = motor_task_stop();
            break;

            default:
                bypass_helpers_abort(p_bypass, p_cmd, "invalid movement phase");
                b_aborted = true;
            break;
        }

        if ((!b_aborted) && (result < E_OK))
        {
            bypass_helpers_abort(p_bypass, p_cmd, "motor command delivery");
            b_aborted = true;
        }

        if ((!b_aborted) && (0u != tmo_msec))
        {
            p_bypass->deadline_usec =
                p_bypass->phase_start_usec + (tmo_msec * 1000u);
        }

        if (!b_aborted)
        {
            p_bypass->move_pending = ((BYPASS_LINE_FOUND != phase) &&
                                      (BYPASS_WAIT_RELEASE != phase))
                                     ? 1u
                                     : 0u;

            tm_printf((UB *)"[bypass] %s side=%d target=%u %s L=%d R=%d\n",
                      bypass_phase_name(phase),
                      (INT)navigator_task_bypass_side(p_bypass),
                      (UINT)amount_val,
                      p_unit,
                      (INT)left_spd,
                      (INT)right_spd);
        }
    }
}

/**
 * @brief Queue an in-place 90-degree pivot in the requested direction.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 * @param[in]     phase    Target bypass phase.
 * @param[in]     dir_sign Pivot direction (+1 left, -1 right).
 */
void bypass_helpers_pivot (navigator_task_bypass_ctx_t * p_bypass,
                           motor_task_cmd_t * p_cmd,
                           navigator_task_bypass_phase_t phase,
                           int32_t dir_sign)
{
    if ((BYPASS_SIDE_LEFT != dir_sign) && (BYPASS_SIDE_RIGHT != dir_sign))
    {
        bypass_helpers_abort(p_bypass, p_cmd, "invalid pivot direction");
    }
    else
    {
        motor_task_cmd_t pivot = bypass_pivot_cmd(dir_sign);
        uint32_t         deg   = (dir_sign > 0)
                                 ? BYPASS_LEFT_TURN_DEG
                                 : BYPASS_RIGHT_TURN_DEG;

        bypass_helpers_begin(p_bypass,
                             p_cmd,
                             phase,
                             (int32_t)pivot.fields.target_speed_left,
                             (int32_t)pivot.fields.target_speed_right,
                             deg);
    }
}

/**
 * @brief Request side-facing ultrasonic tracking on the obstacle side.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer used if abort occurs.
 */
void bypass_helpers_track_side (navigator_task_bypass_ctx_t * p_bypass,
                                motor_task_cmd_t * p_cmd)
{
    if ((uint32_t)BYPASS_ABORT != p_bypass->phase)
    {
        uint32_t now_usec = robot_tasks_time_usec();
        UINT     flg_bit  = (navigator_task_bypass_side(p_bypass) > 0)
                            ? SONAR_TRACK_RIGHT_FLAG
                            : SONAR_TRACK_LEFT_FLAG;

        p_bypass->requested_usec   = now_usec;
        p_bypass->last_track_usec  = now_usec;
        p_bypass->last_sample_usec = now_usec;
        p_bypass->tracking_active  = 1u;

        if (tk_set_flg(robot_tasks_get_ranger_flg(), flg_bit) < E_OK)
        {
            bypass_helpers_abort(p_bypass, p_cmd, "side tracking request");
        }
    }
}

/**
 * @brief Request a stationary front sonar probe for clearance confirmation.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer zeroed while stationary.
 * @param[in]     phase    Probe phase (width check or return check).
 */
void bypass_helpers_probe (navigator_task_bypass_ctx_t * p_bypass,
                           motor_task_cmd_t * p_cmd,
                           navigator_task_bypass_phase_t phase)
{
    if ((uint32_t)BYPASS_ABORT != p_bypass->phase)
    {
        uint32_t now_usec = robot_tasks_time_usec();

        if ((uint32_t)phase != p_bypass->phase)
        {
            p_bypass->probe_clear_cnt = 0u;
            p_bypass->probe_attempts  = 0u;
        }

        p_bypass->phase            = (uint32_t)phase;
        p_bypass->tracking_active  = 0u;
        p_bypass->requested_usec   = now_usec;
        p_bypass->last_sample_usec = now_usec;
        p_bypass->deadline_usec    = now_usec + (BYPASS_PROBE_MSEC * 1000u);
        p_cmd->raw                 = 0u;

        if (tk_set_flg(robot_tasks_get_ranger_flg(),
                       SONAR_PROBE_FRONT_FLAG) < E_OK)
        {
            bypass_helpers_abort(p_bypass, p_cmd, "stationary probe request");
        }
    }
}

/**
 * @brief Advance past obstacle depth or pivot inward to probe return path.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 */
void bypass_helpers_follow_next (navigator_task_bypass_ctx_t * p_bypass,
                                 motor_task_cmd_t * p_cmd)
{
    if ((uint32_t)BYPASS_ABORT != p_bypass->phase)
    {
        uint32_t min_mm =
            (p_bypass->front_cm + BYPASS_PIVOT_FROM_FRONT_CM) * 10u;

        if (p_bypass->forward_mm >= min_mm)
        {
            bypass_helpers_pivot(p_bypass,
                                 p_cmd,
                                 BYPASS_TURN_IN,
                                 -navigator_task_bypass_side(p_bypass));
        }
        else
        {
            bypass_helpers_begin(p_bypass,
                                 p_cmd,
                                 BYPASS_ADVANCE,
                                 BYPASS_API_POWER,
                                 -BYPASS_API_POWER,
                                 min_mm - p_bypass->forward_mm);
        }
    }
}

/**
 * @brief Validate settled odometry after a bounded translation completes.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer used if abort occurs.
 * @return True if odometry is valid, false if bypass was aborted.
 */
bool bypass_helpers_complete_leg (navigator_task_bypass_ctx_t * p_bypass,
                                  motor_task_cmd_t * p_cmd)
{
    bool b_ok = true;

    if (0u != p_bypass->leg_mm)
    {
        int32_t moved_mm =
            motor_task_get_avg_dist_mm() - p_bypass->leg_start_mm;
        bool b_side_phase =
            (((uint32_t)BYPASS_SHIFT_OUT == p_bypass->phase) ||
             ((uint32_t)BYPASS_WIDTH_EXTEND == p_bypass->phase));
        bool b_invalid =
            ((moved_mm <= 0) ||
             (moved_mm > (int32_t)BYPASS_MAX_LEG_MM) ||
             (b_side_phase &&
              ((((uint32_t)moved_mm) + p_bypass->shift_mm) >
               BYPASS_MAX_SHIFT_MM_STORE)) ||
             (((uint32_t)BYPASS_ADVANCE == p_bypass->phase) &&
              ((((uint32_t)moved_mm) + p_bypass->forward_mm) >
               BYPASS_MAX_LEG_MM)));

        if (b_invalid)
        {
            bypass_helpers_abort(p_bypass,
                                 p_cmd,
                                 "invalid or excessive odometry at movement "
                                 "completion");
            b_ok = false;
        }
        else
        {
            tm_printf((UB *)"[bypass] %s travelled=%d mm (requested=%u mm)\n",
                      bypass_phase_name(
                          (navigator_task_bypass_phase_t)p_bypass->phase),
                      (INT)moved_mm,
                      (UINT)p_bypass->leg_mm);
            /* Range-safe: moved_mm is validated in [1, 4095]. */
            p_bypass->leg_mm = (uint32_t)moved_mm;
        }
    }

    return b_ok;
}

/**
 * @brief Process a fresh stationary front sonar probe reply during bypass.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 * @param[in]     p_probe  Latest sonar probe message payload.
 */
void bypass_helpers_probe_result (navigator_task_bypass_ctx_t * p_bypass,
                                  motor_task_cmd_t * p_cmd,
                                  const navigator_task_sonar_probe_t * p_probe)
{
    bool b_relevant = (((uint32_t)BYPASS_WIDTH_CHECK == p_bypass->phase) ||
                       ((uint32_t)BYPASS_RETURN_CHECK == p_bypass->phase));

    if (b_relevant)
    {
        int32_t angle_deg = robot_tasks_sign_extend(p_probe->angle_deg_raw,
                                                    NAV_ANGLE_BIT_WIDTH);
        /* Modular subtraction preserves 32-bit wraparound ordering. */
        int32_t since_req  = (int32_t)(p_probe->sampled_at_usec -
                                       p_bypass->requested_usec);
        int32_t since_last = (int32_t)(p_probe->sampled_at_usec -
                                       p_bypass->last_sample_usec);
        uint32_t age_usec  = robot_tasks_time_usec() - p_probe->sampled_at_usec;
        bool b_fresh = ((0 == angle_deg) &&
                        (since_req >= 0) &&
                        (since_last > 0) &&
                        (age_usec < (BYPASS_PROBE_MSEC * 1000u)));

        if (b_fresh)
        {
            bypass_eval_fresh_probe(p_bypass, p_cmd, p_probe);
        }
    }
}

/**
 * @brief Classify one valid SONAR_OK ray against the obstacle front face plane.
 * @param[in] front_cm Measured center standoff in cm.
 * @param[in] abs_ang  Unsigned ray angle magnitude in degrees.
 * @param[in] dist_cm  Measured ray distance in cm.
 * @return WIDTH_RAY_FACE, WIDTH_RAY_CLEAR, or WIDTH_RAY_UNCERTAIN.
 */
static bypass_helpers_ray_class_t bypass_classify_ok_ray (uint32_t front_cm,
                                                          uint32_t abs_ang,
                                                          uint32_t dist_cm)
{
    bypass_helpers_ray_class_t ray_cls  = WIDTH_RAY_FACE;
    uint32_t                   depth_mm =
        ((dist_cm * 10u) * bypass_width_cosine(abs_ang)) / 1000u;
    uint32_t                   front_mm = front_cm * 10u;
    uint32_t                   tol_mm   = BYPASS_WIDTH_DEPTH_TOL_CM * 10u;

    if (depth_mm > (front_mm + tol_mm))
    {
        ray_cls = WIDTH_RAY_CLEAR;
    }
    else if ((depth_mm + tol_mm) < front_mm)
    {
        ray_cls = WIDTH_RAY_UNCERTAIN;
    }
    else
    {
        ray_cls = WIDTH_RAY_FACE;
    }

    return ray_cls;
}

/**
 * @brief Align two obstacle width scans when a second scan is present.
 * @param[in]     p_first  First stationary obstacle profile.
 * @param[in]     p_second Optional second profile taken after 2 cm reverse.
 * @param[in,out] p_scan_a First width scan (cleared if plane conflicts).
 * @param[out]    p_scan_b Second width scan (cleared if plane conflicts).
 * @return True if at least one consistent scan remains usable.
 */
static bool bypass_align_width_scans (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second,
    navigator_task_width_scan_t * p_scan_a,
    navigator_task_width_scan_t * p_scan_b)
{
    bool b_usable    = true;
    bool b_planes_ok = true;

    if (NULL != p_second)
    {
        *p_scan_b = p_second->measured_width;

        if ((p_first->front_cm >= BYPASS_MIN_SONAR_CM) &&
            (p_second->front_cm >= BYPASS_MIN_SONAR_CM))
        {
            /* Range-safe: front_cm is bounded to [2, 400]. */
            int32_t delta_mm =
                ((((int32_t)p_second->front_cm) -
                  ((int32_t)p_first->front_cm)) * 10) -
                ((int32_t)BYPASS_SCAN_BACK_MM);

            b_planes_ok = ((delta_mm >= (-BYPASS_PLANE_DELTA_MAX_MM)) &&
                           (delta_mm <= BYPASS_PLANE_DELTA_MAX_MM));
        }
    }

    if (!b_planes_ok)
    {
        bool b_a_full = ((0u != p_scan_a->left_valid) &&
                         (0u != p_scan_a->right_valid));
        bool b_b_full = ((0u != p_scan_b->left_valid) &&
                         (0u != p_scan_b->right_valid));

        if (b_a_full)
        {
            *p_scan_b = (navigator_task_width_scan_t){ 0u };
        }
        else if (b_b_full)
        {
            *p_scan_a = (navigator_task_width_scan_t){ 0u };
        }
        else
        {
            b_usable = false;
        }
    }

    return b_usable;
}

/**
 * @brief Apply the confirmed clear or blocked decision for a front probe.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 * @param[in]     b_clear  True if the front probe confirmed sufficient gap.
 * @param[in]     p_probe  Latest sonar probe message payload.
 */
static void bypass_apply_probe_decision (
    navigator_task_bypass_ctx_t * p_bypass,
    motor_task_cmd_t * p_cmd,
    bool b_clear,
    const navigator_task_sonar_probe_t * p_probe)
{
    int32_t side_sign = navigator_task_bypass_side(p_bypass);

    tm_printf((UB *)"[bypass] %s front %s: %u cm/status%u, "
              "sideways=%u along=%u mm\n",
              ((uint32_t)BYPASS_WIDTH_CHECK == p_bypass->phase)
                  ? "Width"
                  : "Return",
              b_clear ? "clear" : "blocked/uncertain",
              (UINT)p_probe->dist_cm,
              (UINT)p_probe->status,
              (UINT)p_bypass->shift_mm,
              (UINT)p_bypass->forward_mm);

    switch ((navigator_task_bypass_phase_t)p_bypass->phase)
    {
        case BYPASS_WIDTH_CHECK:
            p_bypass->width_clr_pend = b_clear ? 1u : 0u;

            if (b_clear)
            {
                tm_printf((UB *)"[bypass] Width clear: add %u mm outward once "
                          "for pivot clearance\n",
                          (UINT)BYPASS_PIVOT_CLEAR_MM);
            }

            bypass_helpers_track_side(p_bypass, p_cmd);
            bypass_helpers_pivot(p_bypass,
                                 p_cmd,
                                 BYPASS_WIDTH_TURN_OUT,
                                 side_sign);
        break;

        case BYPASS_RETURN_CHECK:
            p_bypass->depth_clr_pend = b_clear ? 1u : 0u;

            if (b_clear)
            {
                tm_printf((UB *)"[bypass] Depth clear: add %u mm parallel to "
                          "box once before returning\n",
                          (UINT)BYPASS_DEPTH_CLEAR_MM);
            }

            bypass_helpers_track_side(p_bypass, p_cmd);
            bypass_helpers_pivot(p_bypass,
                                 p_cmd,
                                 BYPASS_DEPTH_TURN_BACK,
                                 side_sign);
        break;

        default:
        break;
    }
}

/**
 * @brief Evaluate one validated fresh stationary front probe sample.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 * @param[in]     p_probe  Fresh sonar probe message payload.
 */
static void bypass_eval_fresh_probe (
    navigator_task_bypass_ctx_t * p_bypass,
    motor_task_cmd_t * p_cmd,
    const navigator_task_sonar_probe_t * p_probe)
{
    p_bypass->last_sample_usec = p_probe->sampled_at_usec;

    if (((uint32_t)SONAR_SENSOR_FAULT == p_probe->status) ||
        ((uint32_t)SONAR_SERVO_FAULT == p_probe->status))
    {
        bypass_helpers_abort(p_bypass, p_cmd, "front probe hardware");
    }
    else
    {
        uint32_t return_pad_mm =
            (SONAR_ROBOT_LENGTH_CM + SONAR_WALL_BODY_GAP_CM) * 10u;
        uint32_t req_mm =
            ((uint32_t)BYPASS_WIDTH_CHECK == p_bypass->phase)
            ? ((p_bypass->front_cm + SONAR_ROBOT_LENGTH_CM) * 10u)
            : (p_bypass->shift_mm + return_pad_mm);
        bool b_valid =
            (((uint32_t)SONAR_OK == p_probe->status) &&
             (0u != p_probe->b_valid) &&
             (p_probe->dist_cm >= BYPASS_MIN_SONAR_CM) &&
             (p_probe->dist_cm <= BYPASS_MAX_SONAR_CM));
        bool b_clear =
            (((uint32_t)SONAR_NO_ECHO == p_probe->status) ||
             (b_valid && ((((uint32_t)p_probe->dist_cm) * 10u) > req_mm)));
        bool b_blocked =
            (((uint32_t)SONAR_TOO_CLOSE == p_probe->status) ||
             (b_valid && (!b_clear)));
        bool b_retry = false;

        if (p_bypass->probe_attempts < BYPASS_PROBE_ATTEMPTS_MAX)
        {
            p_bypass->probe_attempts++;
        }

        p_bypass->probe_clear_cnt =
            b_clear ? (p_bypass->probe_clear_cnt + 1u) : 0u;
        b_retry = ((b_clear &&
                    (p_bypass->probe_clear_cnt < BYPASS_CLEAR_CONFIRM)) ||
                   ((!b_clear) && (!b_blocked) &&
                    (p_bypass->probe_attempts < BYPASS_CLEAR_CONFIRM)));

        if (b_retry)
        {
            bypass_helpers_probe(
                p_bypass,
                p_cmd,
                (navigator_task_bypass_phase_t)p_bypass->phase);
        }
        else
        {
            bypass_apply_probe_decision(p_bypass, p_cmd, b_clear, p_probe);
        }
    }
}

/**
 * @brief Convert beam-compensated angle in degrees to milliradians.
 * @param[in] angle_deg Unsigned ray angle in degrees.
 * @return Beam-compensated angle in milliradians.
 */
static uint32_t bypass_width_ray_rad (uint32_t angle_deg)
{
    uint32_t ddeg   = angle_deg * 10u;
    uint32_t result = 0u;

    if (ddeg > BYPASS_WIDTH_BEAM_HALF_DDEG)
    {
        result = ((ddeg - BYPASS_WIDTH_BEAM_HALF_DDEG) * 17453u) / 10000u;
    }

    return result;
}

/**
 * @brief Compute 4th-order Taylor cosine (x1000) of beam-compensated angle.
 * @param[in] angle_deg Unsigned ray angle in degrees.
 * @return Scaled cosine value in [0, 1000].
 */
static uint32_t bypass_width_cosine (uint32_t angle_deg)
{
    uint32_t rad_val = bypass_width_ray_rad(angle_deg);
    uint32_t sq_val  = (rad_val * rad_val) / 1000u;

    return ((1000u - (sq_val / 2u)) + ((sq_val * sq_val) / 24000u));
}

/**
 * @brief Merge one edge bracket across two scans and detect conflicts.
 * @param[in]  b_valid_a    True if first scan bracket is valid.
 * @param[in]  min_a_mm     First scan minimum mm.
 * @param[in]  max_a_mm     First scan maximum mm.
 * @param[in]  b_valid_b    True if second scan bracket is valid.
 * @param[in]  min_b_mm     Second scan minimum mm.
 * @param[in]  max_b_mm     Second scan maximum mm.
 * @param[out] p_midpoint   Merged edge midpoint in mm.
 * @param[out] p_b_conflict Set true if both valid brackets disagree beyond tol.
 * @return True if a merged midpoint was produced.
 */
static bool bypass_width_merge_edge (bool b_valid_a,
                                     uint32_t min_a_mm,
                                     uint32_t max_a_mm,
                                     bool b_valid_b,
                                     uint32_t min_b_mm,
                                     uint32_t max_b_mm,
                                     uint32_t * p_midpoint,
                                     bool * p_b_conflict)
{
    bool b_ok_a = (b_valid_a &&
                   (min_a_mm <= max_a_mm) &&
                   ((max_a_mm - min_a_mm) <= BYPASS_WIDTH_MAX_SPAN_MM));
    bool b_ok_b = (b_valid_b &&
                   (min_b_mm <= max_b_mm) &&
                   ((max_b_mm - min_b_mm) <= BYPASS_WIDTH_MAX_SPAN_MM));
    bool b_done = false;

    if (b_ok_a && b_ok_b)
    {
        uint32_t low_mm  = (min_a_mm > min_b_mm) ? min_a_mm : min_b_mm;
        uint32_t high_mm = (max_a_mm < max_b_mm) ? max_a_mm : max_b_mm;

        if ((low_mm > high_mm) &&
            ((low_mm - high_mm) > BYPASS_WIDTH_REPEAT_TOL_MM))
        {
            *p_b_conflict = true;
        }
        else
        {
            *p_midpoint = (low_mm + high_mm) / 2u;
            b_done      = true;
        }
    }
    else if (b_ok_a || b_ok_b)
    {
        *p_midpoint = b_ok_a
                      ? ((min_a_mm + max_a_mm) / 2u)
                      : ((min_b_mm + max_b_mm) / 2u);
        b_done      = true;
    }
    else
    {
        b_done = false;
    }

    return b_done;
}

/**
 * @brief Score open room on one outer ray for tie-breaking side selection.
 * @param[in] status Outer ray ultrasonic status.
 * @param[in] dist_cm Outer ray measured distance in cm.
 * @return Room score (401 for no-echo, dist_cm for valid hit, 0 otherwise).
 */
static uint32_t bypass_side_room (ultrasonic_task_status_t status,
                                  uint32_t dist_cm)
{
    uint32_t room_cm = 0u;

    if (SONAR_NO_ECHO == status)
    {
        room_cm = BYPASS_NO_ECHO_ROOM_CM;
    }
    else if ((SONAR_OK == status) &&
             (dist_cm >= BYPASS_MIN_SONAR_CM) &&
             (dist_cm <= BYPASS_MAX_SONAR_CM))
    {
        room_cm = dist_cm;
    }
    else
    {
        room_cm = 0u;
    }

    return room_cm;
}

/**
 * @brief Compute conservative lateral bypass offset on one side from 2 scans.
 * @param[in]  p_first   First obstacle profile.
 * @param[in]  p_second  Second obstacle profile after 2 cm reverse.
 * @param[in]  side_sign +1 for left side, -1 for right side.
 * @param[in]  front_cm  Baseline standoff distance in cm.
 * @param[out] p_b_known True if an open ray beyond hits bounded the edge.
 * @return Required outward shift in mm (0 if edge is unbounded).
 */
static uint32_t bypass_side_offset (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second,
    int32_t side_sign,
    uint32_t front_cm,
    bool * p_b_known)
{
    static const uint32_t sin_tbl[BYPASS_TRIG_TABLE_LEN] =
    {
        0u, 259u, 500u, 707u
    };
    static const uint32_t cos_tbl[BYPASS_TRIG_TABLE_LEN] =
    {
        1000u, 966u, 866u, 707u
    };
    static const uint32_t tan_tbl[BYPASS_TRIG_TABLE_LEN] =
    {
        0u, 268u, 577u, 1000u
    };
    const navigator_task_obs_profile_t * p_scans[BYPASS_SCAN_PAIR_COUNT] =
    {
        p_first,
        p_second
    };
    uint32_t ref_cm[BYPASS_SCAN_PAIR_COUNT] =
    {
        p_first->front_cm,
        p_second->front_cm
    };
    uint32_t depth_cm   = 0u;
    uint32_t observed_x = 0u;
    int32_t  occupied   = 0;
    uint32_t edge_cm    = 0u;
    uint32_t result_mm  = 0u;

    *p_b_known = false;

    for (uint32_t scan_idx = 0u; scan_idx < BYPASS_SCAN_PAIR_COUNT; scan_idx++)
    {
        uint32_t back_add = (0u != scan_idx)
                            ? (BYPASS_SCAN_BACK_MM / 10u)
                            : 0u;

        if ((ref_cm[scan_idx] < BYPASS_MIN_SONAR_CM) ||
            (ref_cm[scan_idx] >
             ((front_cm + BYPASS_PROFILE_DEPTH_CM) + back_add)))
        {
            ref_cm[scan_idx] = front_cm + back_add;
        }
    }

    depth_cm = (ref_cm[0] > ref_cm[1]) ? ref_cm[0] : ref_cm[1];

    for (int32_t ray_step = 1; ray_step <= BYPASS_HALF_RAY_COUNT; ray_step++)
    {
        uint32_t ray_idx =
            (uint32_t)(BYPASS_CENTER_RAY_IDX + (side_sign * ray_step));

        for (uint32_t scan_idx = 0u;
             scan_idx < BYPASS_SCAN_PAIR_COUNT;
             scan_idx++)
        {
            uint32_t                 ray_cm = navigator_task_ray_cm(
                p_scans[scan_idx], ray_idx);
            ultrasonic_task_status_t ray_st = navigator_task_ray_status(
                p_scans[scan_idx], ray_idx);
            bool b_hit =
                ((SONAR_TOO_CLOSE == ray_st) ||
                 ((SONAR_OK == ray_st) &&
                  (ray_cm >= BYPASS_MIN_SONAR_CM) &&
                  (ray_cm <= BYPASS_MAX_SONAR_CM) &&
                  (((ray_cm * cos_tbl[ray_step]) / 1000u) <=
                   (ref_cm[scan_idx] + BYPASS_PROFILE_DEPTH_CM))));

            if (b_hit)
            {
                uint32_t lat_x = (SONAR_TOO_CLOSE == ray_st)
                                 ? ((depth_cm * tan_tbl[ray_step]) / 1000u)
                                 : (((ray_cm * sin_tbl[ray_step]) + 999u) /
                                    1000u);

                occupied = ray_step;

                if (lat_x > observed_x)
                {
                    observed_x = lat_x;
                }
            }
        }
    }

    for (int32_t ray_step = occupied + 1;
         (ray_step <= BYPASS_HALF_RAY_COUNT) && (!(*p_b_known));
         ray_step++)
    {
        uint32_t ray_idx =
            (uint32_t)(BYPASS_CENTER_RAY_IDX + (side_sign * ray_step));
        bool b_open = true;

        for (uint32_t scan_idx = 0u;
             scan_idx < BYPASS_SCAN_PAIR_COUNT;
             scan_idx++)
        {
            ultrasonic_task_status_t ray_st = navigator_task_ray_status(
                p_scans[scan_idx], ray_idx);
            uint32_t ray_cm = navigator_task_ray_cm(p_scans[scan_idx], ray_idx);
            bool b_clear_ray =
                ((SONAR_NO_ECHO == ray_st) ||
                 ((SONAR_OK == ray_st) &&
                  (ray_cm >= BYPASS_MIN_SONAR_CM) &&
                  (ray_cm <= BYPASS_MAX_SONAR_CM) &&
                  (((ray_cm * cos_tbl[ray_step]) / 1000u) >
                   (ref_cm[scan_idx] + BYPASS_PROFILE_DEPTH_CM))));

            b_open = (b_open && b_clear_ray);
        }

        if (b_open)
        {
            edge_cm    = ((depth_cm * tan_tbl[ray_step]) + 999u) / 1000u;
            *p_b_known = true;
        }
    }

    if (*p_b_known)
    {
        edge_cm = bypass_refine_fine_offset(p_first,
                                            p_second,
                                            side_sign,
                                            occupied,
                                            edge_cm);

        if (observed_x > edge_cm)
        {
            edge_cm = observed_x;
        }

        result_mm = (edge_cm * 10u) + BYPASS_WIDTH_CLEAR_MM;
    }

    return result_mm;
}

/**
 * @brief Trim coarse side-offset estimate using refined edge measurements.
 * @param[in] p_first   First obstacle scan profile.
 * @param[in] p_second  Second obstacle scan profile.
 * @param[in] side_sign Side being evaluated (+1 left, -1 right).
 * @param[in] occupied  Highest occupied coarse ray index (0..3).
 * @param[in] edge_cm   Coarse lateral edge estimate in cm.
 * @return Refined lateral edge estimate in cm.
 */
static uint32_t bypass_refine_fine_offset (
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second,
    int32_t side_sign,
    int32_t occupied,
    uint32_t edge_cm)
{
    uint32_t refined_cm = edge_cm;
    bool     b_fine     = (side_sign > 0)
                          ? ((0u != p_first->left_offset_valid) &&
                             (0u != p_second->left_offset_valid))
                          : ((0u != p_first->right_offset_valid) &&
                             (0u != p_second->right_offset_valid));
    int32_t  ang_a      = (side_sign > 0)
                          ? robot_tasks_sign_extend(p_first->left_edge_deg_raw,
                                                    NAV_ANGLE_BIT_WIDTH)
                          : (-robot_tasks_sign_extend(
                                 p_first->right_edge_deg_raw,
                                 NAV_ANGLE_BIT_WIDTH));
    int32_t  ang_b      = (side_sign > 0)
                          ? robot_tasks_sign_extend(p_second->left_edge_deg_raw,
                                                    NAV_ANGLE_BIT_WIDTH)
                          : (-robot_tasks_sign_extend(
                                 p_second->right_edge_deg_raw,
                                 NAV_ANGLE_BIT_WIDTH));

    if (b_fine &&
        (ang_a >= (occupied * BYPASS_RAY_STEP_DEG)) &&
        (ang_b >= (occupied * BYPASS_RAY_STEP_DEG)))
    {
        uint32_t off_a    = (side_sign > 0)
                            ? p_first->left_offset_cm
                            : p_first->right_offset_cm;
        uint32_t off_b    = (side_sign > 0)
                            ? p_second->left_offset_cm
                            : p_second->right_offset_cm;
        uint32_t fine_cm  = (off_a > off_b) ? off_a : off_b;
        uint32_t body_pad = SONAR_ROBOT_HALF_WIDTH_CM + SONAR_WALL_BODY_GAP_CM;

        if ((fine_cm >= body_pad) && ((fine_cm - body_pad) < refined_cm))
        {
            refined_cm = fine_cm - body_pad;
        }
    }

    return refined_cm;
}

/**
 * @brief Build in-place pivot motor speed signs (+1 left, -1 right).
 * @param[in] dir_sign Pivot direction (+1 left, -1 right).
 * @return Populated motor command structure (zeroed if dir_sign invalid).
 */
static motor_task_cmd_t bypass_pivot_cmd (int32_t dir_sign)
{
    motor_task_cmd_t cmd = { 0u };

    if ((BYPASS_SIDE_LEFT == dir_sign) || (BYPASS_SIDE_RIGHT == dir_sign))
    {
        /* Range-safe: +/-BYPASS_API_POWER (-25 or +25) fits in int8_t. */
        int8_t wheel_spd = (int8_t)((-dir_sign) * BYPASS_API_POWER);

        cmd.fields.target_speed_left  = wheel_spd;
        cmd.fields.target_speed_right = wheel_spd;
    }

    return cmd;
}

/**
 * @brief Return a human-readable serial log label for a bypass phase.
 * @param[in] phase Bypass phase enum value.
 * @return Null-terminated phase name string.
 */
static const char * bypass_phase_name (navigator_task_bypass_phase_t phase)
{
    const char * p_name = "IDLE";

    switch (phase)
    {
        case BYPASS_WAIT_RELEASE:
            p_name = "WAIT_MOTOR_RELEASE";
        break;

        case BYPASS_SCAN_BACK:
            p_name = "SCAN_BACK_2CM";
        break;

        case BYPASS_SCAN_RETURN:
            p_name = "RESTORE_2CM";
        break;

        case BYPASS_WIDTH_TURN_OUT:
            p_name = "WIDTH_TURN_OUT";
        break;

        case BYPASS_WIDTH_EXTEND:
            p_name = "WIDTH_EXTEND";
        break;

        case BYPASS_WIDTH_ALIGN:
            p_name = "WIDTH_ALIGN";
        break;

        case BYPASS_DEPTH_TURN_BACK:
            p_name = "FACE_PARALLEL";
        break;

        case BYPASS_TURN_OUT:
            p_name = "TURN_OUT_90";
        break;

        case BYPASS_SHIFT_OUT:
            p_name = "SIDE_STEP";
        break;

        case BYPASS_TURN_PARALLEL:
            p_name = "TURN_BACK_90";
        break;

        case BYPASS_ADVANCE:
            p_name = "PASS_BOX";
        break;

        case BYPASS_TURN_IN:
            p_name = "TURN_TO_LINE_90";
        break;

        case BYPASS_FIND_LINE:
            p_name = "FIND_LINE";
        break;

        case BYPASS_LINE_FOUND:
            p_name = "LINE_FOUND";
        break;

        case BYPASS_ABORT:
            p_name = "FAULT";
        break;

        default:
            p_name = "IDLE";
        break;
    }

    return p_name;
}

/*** end of file ***/
