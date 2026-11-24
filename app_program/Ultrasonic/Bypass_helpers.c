#include <tm/tmonitor.h>

#include "Bypass_helpers.h"
#include "../robot_tasks.h"

// Called by Navigator; the bypass state machine and context remain owned by Navigator.
_Static_assert(BYPASS_LEFT_TURN_DEG > 0 && BYPASS_RIGHT_TURN_DEG > 0, "invalid bypass turn angle");
_Static_assert(BYPASS_PIVOT_CLEAR_MM > 0 && BYPASS_PIVOT_CLEAR_MM <= BYPASS_MAX_SIDE_MM, "invalid pivot clearance move");
_Static_assert(BYPASS_DEPTH_CLEAR_MM > 0 && BYPASS_DEPTH_CLEAR_MM <= BYPASS_MAX_FORWARD_MM, "invalid depth clearance move");
_Static_assert(BYPASS_PIVOT_FROM_FRONT_CM <= ROBOT_LENGTH_CM, "invalid pivot position");
_Static_assert(BYPASS_SWEPT_RADIUS_CM * BYPASS_SWEPT_RADIUS_CM >=
               BYPASS_PIVOT_FROM_FRONT_CM * BYPASS_PIVOT_FROM_FRONT_CM + ROBOT_HALF_WIDTH_CM * ROBOT_HALF_WIDTH_CM &&
               BYPASS_SWEPT_RADIUS_CM * BYPASS_SWEPT_RADIUS_CM >=
               (ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM) * (ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM) +
               ROBOT_HALF_WIDTH_CM * ROBOT_HALF_WIDTH_CM, "swept radius must cover the whole body");
/* Fourth-order cosine is sufficient for the integer -45..45 degree scan. */
LOCAL UW width_cosine(UW angle)
{
    UW rad = angle * 17453U / 1000U;
    UW square = rad * rad / 1000U;
    return 1000U - square / 2U + square * square / 24000U;
}

EXPORT WIDTH_RAY_CLASS obstacle_width_classify(UW front_cm, W angle, UW cm, SONAR_STATUS status)
{
    if (front_cm < 2U || front_cm > 400U || angle < -45 || angle > 45)
        return WIDTH_RAY_UNCERTAIN;
    if (status == SONAR_NO_ECHO) {
        UW cosine = width_cosine(angle < 0 ? -angle : angle);
        return front_cm * 1000U <= 400U * cosine ? WIDTH_RAY_CLEAR : WIDTH_RAY_UNCERTAIN;
    }
    if (status != SONAR_OK || cm < 2U || cm > 400U) return WIDTH_RAY_UNCERTAIN;
    UW depth_mm = cm * 10U * width_cosine(angle < 0 ? -angle : angle) / 1000U;
    UW front_mm = front_cm * 10U;
    UW tolerance_mm = WIDTH_DEPTH_TOLERANCE_CM * 10U;
    if (depth_mm > front_mm + tolerance_mm) return WIDTH_RAY_CLEAR;
    if (depth_mm + tolerance_mm < front_mm) return WIDTH_RAY_UNCERTAIN;
    return WIDTH_RAY_FACE;
}

EXPORT UW obstacle_width_offset_mm(UW front_cm, UW angle)
{
    if (front_cm > 400U || angle > 45U) return 0U;
    UW rad = angle * 17453U / 1000U;
    UW square = rad * rad / 1000U;
    UW sine = rad - rad * square / 6000U + rad * square * square / 120000000U;
    UW cosine = width_cosine(angle);
    return (front_cm * 10U * sine + cosine / 2U) / cosine;
}

/* Both scans share the same lateral origin; the 2 cm reverse changes only depth. */
LOCAL BOOL width_merge_edge(BOOL av, UW amin, UW amax, BOOL bv, UW bmin, UW bmax,
                            UW *midpoint, BOOL *conflict)
{
    if ((!av && !bv) || (av && amin > amax) || (bv && bmin > bmax)) return FALSE;
    if (av && bv) {
        UW lo = amin > bmin ? amin : bmin;
        UW hi = amax < bmax ? amax : bmax;
        if (lo > hi && lo - hi > WIDTH_REPEAT_TOLERANCE_MM) {
            *conflict = TRUE;
            return FALSE;
        }
        *midpoint = (lo + hi) / 2U;
    } else {
        *midpoint = av ? (amin + amax) / 2U : (bmin + bmax) / 2U;
    }
    return TRUE;
}

EXPORT OBSTACLE_WIDTH obstacle_width_estimate(const OBSTACLE_PROFILE *first,
                                             const OBSTACLE_PROFILE *second)
{
    OBSTACLE_WIDTH result = {0};
    OBSTACLE_WIDTH_SCAN a = first->measured_width;
    OBSTACLE_WIDTH_SCAN b = second ? second->measured_width : (OBSTACLE_WIDTH_SCAN){0};
    /* Reject merging a different front plane after the short reverse. */
    if (second && first->front_cm >= 2U && second->front_cm >= 2U) {
        W delta_mm = ((W)second->front_cm - first->front_cm) * 10 - (W)BYPASS_SCAN_BACK_MM;
        if (delta_mm < -30 || delta_mm > 30) {
            if (a.left_valid && a.right_valid) b = (OBSTACLE_WIDTH_SCAN){0};
            else if (b.left_valid && b.right_valid) a = (OBSTACLE_WIDTH_SCAN){0};
            else return result;
        }
    }
    UW left = 0, right = 0;
    BOOL conflict = FALSE;
    BOOL left_ok = width_merge_edge(a.left_valid, a.left_min_mm, a.left_max_mm,
        b.left_valid, b.left_min_mm, b.left_max_mm, &left, &conflict);
    BOOL right_ok = width_merge_edge(a.right_valid, a.right_min_mm, a.right_max_mm,
        b.right_valid, b.right_min_mm, b.right_max_mm, &right, &conflict);
    if (conflict) result.status = WIDTH_INCONSISTENT;
    else if (left_ok && right_ok) {
        UW cm = (left + right + 5U) / 10U;
        if (cm > 0U && cm <= 800U) {
            result.cm = cm;
            result.status = WIDTH_ESTIMATED;
        }
    }
    return result;
}

#ifndef NDEBUG
EXPORT BOOL obstacle_width_self_check(void)
{
    OBSTACLE_PROFILE first = {.front_cm = 20}, second = {.front_cm = 22};
    first.measured_width = (OBSTACLE_WIDTH_SCAN){.left_min_mm = 80, .left_max_mm = 100, .left_valid = 1};
    second.measured_width = (OBSTACLE_WIDTH_SCAN){.right_min_mm = 50, .right_max_mm = 70, .right_valid = 1};
    OBSTACLE_WIDTH width = obstacle_width_estimate(&first, &second);
    if (width.status != WIDTH_ESTIMATED || width.cm != 15 ||
        obstacle_width_estimate(&first, NULL).status != WIDTH_UNAVAILABLE) return FALSE;
    first.measured_width.right_valid = 1;
    first.measured_width.right_min_mm = 50;
    first.measured_width.right_max_mm = 70;
    second.measured_width = (OBSTACLE_WIDTH_SCAN){0};
    width = obstacle_width_estimate(&first, &second);
    if (width.status != WIDTH_ESTIMATED || width.cm != 15) return FALSE;
    second.measured_width = (OBSTACLE_WIDTH_SCAN){.left_min_mm = 180, .left_max_mm = 200, .left_valid = 1};
    if (obstacle_width_estimate(&first, &second).status != WIDTH_INCONSISTENT) return FALSE;
    return obstacle_width_classify(20, 30, 30, SONAR_OK) == WIDTH_RAY_CLEAR &&
           obstacle_width_classify(20, 30, 23, SONAR_OK) == WIDTH_RAY_FACE &&
           obstacle_width_classify(20, 30, 0, SONAR_UNSTABLE) == WIDTH_RAY_UNCERTAIN &&
           obstacle_width_offset_mm(20, 45) >= 199U && obstacle_width_offset_mm(20, 45) <= 201U;
}
#endif

// No echo is open space, not a failed scan. Other unavailable readings rank below usable rays.
LOCAL UW bypass_side_room(SONAR_STATUS status, UW cm)
{
    if (status == SONAR_NO_ECHO) return 401U;
    if (status == SONAR_OK && cm >= 2U && cm <= 400U) return cm;
    return 0U;
}

LOCAL UW bypass_side_offset(const OBSTACLE_PROFILE *first, const OBSTACLE_PROFILE *second,
                            INT side, UW front_cm, BOOL *known)
{
    static const UW sine[4] = {0, 259, 500, 707};
    static const UW cosine[4] = {1000, 966, 866, 707};
    static const UW tangent[4] = {0, 268, 577, 1000};
    const OBSTACLE_PROFILE *scans[2] = {first, second};
    UW reference[2] = {first->front_cm, second->front_cm};
    for (INT scan = 0; scan < 2; scan++)
        if (reference[scan] < 2U || reference[scan] > front_cm + BYPASS_PROFILE_DEPTH_CM +
            (scan ? BYPASS_SCAN_BACK_MM / 10U : 0U))
            reference[scan] = front_cm + (scan ? BYPASS_SCAN_BACK_MM / 10U : 0U);
    UW depth = reference[0] > reference[1] ? reference[0] : reference[1];
    UW observed_x = 0;
    INT occupied = 0;
    *known = FALSE;

    // Inspect all outer rays: an isolated missing inner ray must not hide a farther occupied ray.
    for (INT ray = 1; ray <= 3; ray++) {
        INT index = 3 + side * ray;
        for (INT scan = 0; scan < 2; scan++) {
            UW cm = sonar_ray_cm(scans[scan], index);
            SONAR_STATUS status = sonar_ray_status(scans[scan], index);
            if (status == SONAR_TOO_CLOSE || (status == SONAR_OK && cm >= 2U && cm <= 400U &&
                cm * cosine[ray] / 1000U <= reference[scan] + BYPASS_PROFILE_DEPTH_CM)) {
                occupied = ray;
                UW x = status == SONAR_TOO_CLOSE ? depth * tangent[ray] / 1000U :
                       (cm * sine[ray] + 999U) / 1000U;
                if (x > observed_x) observed_x = x;
            }
        }
    }

    // The first ray beyond every hit must be open in BOTH scans before it bounds the edge.
    UW edge_cm = 0;
    for (INT ray = occupied + 1; ray <= 3; ray++) {
        INT index = 3 + side * ray;
        BOOL open = TRUE;
        for (INT scan = 0; scan < 2; scan++) {
            SONAR_STATUS status = sonar_ray_status(scans[scan], index);
            UW cm = sonar_ray_cm(scans[scan], index);
            open = open && (status == SONAR_NO_ECHO || (status == SONAR_OK && cm >= 2U && cm <= 400U &&
                   cm * cosine[ray] / 1000U > reference[scan] + BYPASS_PROFILE_DEPTH_CM));
        }
        if (open) {
            edge_cm = (depth * tangent[ray] + 999U) / 1000U;
            *known = TRUE;
            break;
        }
    }

    // An unbounded scan gives no width: find the edge with short stationary forward checks.
    if (!*known) return 0;

    BOOL fine = side > 0 ? first->left_offset_valid && second->left_offset_valid :
                          first->right_offset_valid && second->right_offset_valid;
    W a = side > 0 ? first->left_edge_deg : -first->right_edge_deg;
    W b = side > 0 ? second->left_edge_deg : -second->right_edge_deg;
    if (*known && fine && a >= occupied * 15 && b >= occupied * 15) {
        UW x = side > 0 ? first->left_offset_cm : first->right_offset_cm;
        UW y = side > 0 ? second->left_offset_cm : second->right_offset_cm;
        UW fine_cm = x > y ? x : y;
        if (fine_cm >= ROBOT_HALF_WIDTH_CM + WALL_BODY_GAP_CM) {
            fine_cm -= ROBOT_HALF_WIDTH_CM + WALL_BODY_GAP_CM;
            if (fine_cm < edge_cm) edge_cm = fine_cm;
        }
    }
    if (observed_x > edge_cm) edge_cm = observed_x;
    return edge_cm * 10U + BYPASS_WIDTH_CLEAR_MM;
}

EXPORT void bypass_plan_width(BYPASS_CONTEXT *bypass, const OBSTACLE_PROFILE *second)
{
    if (second->front_cm >= BYPASS_SCAN_BACK_MM / 10U + 2U &&
        second->front_cm <= bypass->front_cm + BYPASS_PROFILE_DEPTH_CM + BYPASS_SCAN_BACK_MM / 10U) {
        UW original_front = second->front_cm - BYPASS_SCAN_BACK_MM / 10U;
        if (original_front > bypass->front_cm) bypass->front_cm = original_front;
    }
    BOOL left_known, right_known;
    UW left = bypass_side_offset(&bypass->first_scan, second, 1, bypass->front_cm, &left_known);
    UW right = bypass_side_offset(&bypass->first_scan, second, -1, bypass->front_cm, &right_known);
    UW left_room = bypass_side_room(second->left_scan_status, second->left_scan_cm);
    UW right_room = bypass_side_room(second->right_scan_status, second->right_scan_cm);
    if (left_known != right_known)
        bypass->side = left_known ? 1 : -1;
    else
        bypass->side = left < right || (left == right && left_room >= right_room) ? 1 : -1;
    BOOL known = bypass->side > 0 ? left_known : right_known;
    bypass->planned_shift_mm = known ? (bypass->side > 0 ? left : right) : BYPASS_WIDTH_STEP_MM;
    bypass->width_clearance_pending = FALSE;
    bypass->depth_clearance_pending = FALSE;
    tm_printf((UB *)"[bypass] Dynamic offsets L=%u R=%u mm (0=unknown); pass %s, edge=%s, front=%u cm, first step=%u mm\n",
              left, right, bypass->side > 0 ? "left" : "right",
              known ? "bounded by both scans" : "unmeasured: verify outward",
              bypass->front_cm, bypass->planned_shift_mm);
}

// Wiring uses opposite signs for forward, but the SAME signs for an in-place pivot.
LOCAL MOTOR_COMMAND bypass_pivot_command(INT direction)
{
    MOTOR_COMMAND command = {0};
    if (direction == 1 || direction == -1) {
        command.target_speed_left = -direction * BYPASS_API_POWER;
        command.target_speed_right = -direction * BYPASS_API_POWER;
    }
    return command;
}

#ifndef NDEBUG
#define BYPASS_CHECK(condition) do { if (!(condition)) return FALSE; } while (0)
// Debug builds check edge offsets with the width margin and mirrored motor signs.
EXPORT BOOL bypass_self_check(void)
{
    BYPASS_CHECK(bypass_pivot_command(0).raw == 0U);
    for (INT side = -1; side <= 1; side += 2) {
        OBSTACLE_PROFILE first = {.front_cm = 30U}, second = {.front_cm = 32U};
        BOOL known;
        BYPASS_CHECK(bypass_side_offset(&first, &second, side, 30U, &known) == 0U && !known);
        sonar_ray_set(&first, 3 + side, 0U, SONAR_NO_ECHO);
        sonar_ray_set(&second, 3 + side, 0U, SONAR_NO_ECHO);
        BYPASS_CHECK(bypass_side_offset(&first, &second, side, 30U, &known) == 90U + BYPASS_WIDTH_CLEAR_MM && known);

        sonar_ray_set(&first, 3 + side * 3, 43U, SONAR_OK);
        sonar_ray_set(&second, 3 + side * 3, 45U, SONAR_OK);
        BYPASS_CHECK(bypass_side_offset(&first, &second, side, 30U, &known) == 0U && !known);
        MOTOR_COMMAND pivot = bypass_pivot_command(side);
        BYPASS_CHECK(pivot.target_speed_left == -side * BYPASS_API_POWER);
        BYPASS_CHECK(pivot.target_speed_right == pivot.target_speed_left);
        BYPASS_CHECK(pivot.turn_ticks_left == 0 && pivot.turn_ticks_right == 0);
    }
    return TRUE;
}
#undef BYPASS_CHECK
#endif

// Bypass ignores IMU events; motor and sonar faults still abort.
EXPORT BOOL bypass_hardware_fault(void)
{
    return sonar_fault ||
           (motor_fault_word & (FAULT_BIT_STALL | FAULT_BIT_ENCODER)) != 0U;
}

// Ordinary no-echo replies do not abort; hardware, delivery and movement faults do.
EXPORT void bypass_abort(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, const char *reason)
{
    robot_flag_assign(ROBOT_ESTOP_REQUEST, TRUE);
    robot_flag_assign(ROBOT_STOP_ACK_REQUEST, FALSE);
    bypass->owns_obstacle_stop = FALSE;
    bypass->phase = BYPASS_ABORT;
    bypass->move_pending = FALSE;
    bypass->deadline_us = 0;
    *command = (MOTOR_COMMAND){0};
    (void)stop();
    tm_printf((UB *)"[bypass] Fault: %s (motor=%u sonar=%d IMU=%u)\n",
              reason, motor_fault_word, sonar_fault, telemetry_imu_event());
}

// Name each step in serial logs so it can be matched to the physical movement.
LOCAL const char *bypass_phase_name(BYPASS_PHASE phase)
{
    switch (phase) {
        case BYPASS_WAIT_RELEASE:  return "WAIT_MOTOR_RELEASE";
        case BYPASS_SCAN_BACK:     return "SCAN_BACK_2CM";
        case BYPASS_SCAN_RETURN:   return "RESTORE_2CM";
        case BYPASS_WIDTH_TURN_OUT:return "WIDTH_TURN_OUT";
        case BYPASS_WIDTH_EXTEND:  return "WIDTH_EXTEND";
        case BYPASS_WIDTH_ALIGN:   return "WIDTH_ALIGN";
        case BYPASS_DEPTH_TURN_BACK:return "FACE_PARALLEL";
        case BYPASS_TURN_OUT:      return "TURN_OUT_90";
        case BYPASS_SHIFT_OUT:     return "SIDE_STEP";
        case BYPASS_TURN_PARALLEL: return "TURN_BACK_90";
        case BYPASS_ADVANCE:       return "PASS_BOX";
        case BYPASS_TURN_IN:       return "TURN_TO_LINE_90";
        case BYPASS_FIND_LINE:     return "FIND_LINE";
        case BYPASS_LINE_FOUND:    return "LINE_FOUND";
        case BYPASS_ABORT:         return "FAULT";
        default:                  return "IDLE";
    }
}

// Queue one measured movement. Deadlines only detect faults; MOVE_DONE advances the switch.
EXPORT void bypass_begin(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT left, INT right, UW amount)
{
    if (bypass_hardware_fault() || (motor_estop && !bypass->owns_obstacle_stop)) {
        bypass_abort(bypass, command, "motor stop or sonar fault");
        return;
    }
    bypass->phase = phase;
    bypass->phase_started_us = time_us();
    bypass->move_pending = FALSE;
    bypass->deadline_us = 0;
    bypass->leg_mm = 0;
    bypass->leg_start_mm = motor_get_avg_dist_mm();
    *command = (MOTOR_COMMAND){0};
    command->target_speed_left = left;
    command->target_speed_right = right;
    ER result;
    UW timeout_ms = 0;
    const char *unit = "mm";
    switch (phase) {
        case BYPASS_WAIT_RELEASE:
            if (!bypass->owns_obstacle_stop) {
                bypass_abort(bypass, command, "scan stop not owned");
                return;
            }
            bypass->owns_obstacle_stop = FALSE;
            robot_flag_assign(ROBOT_STOP_ACK_REQUEST, FALSE);
            robot_flag_assign(ROBOT_ESTOP_REQUEST, FALSE);
            result = E_OK;
            timeout_ms = BYPASS_RESPONSE_MS;
            break;
        case BYPASS_TURN_OUT:
        case BYPASS_TURN_PARALLEL:
        case BYPASS_TURN_IN:
        case BYPASS_WIDTH_TURN_OUT:
        case BYPASS_WIDTH_ALIGN:
        case BYPASS_DEPTH_TURN_BACK:
            result = left < 0 ? turnLeft((INT)amount) : turnRight((INT)amount);
            timeout_ms = BYPASS_TURN_TIMEOUT_MS;
            unit = "deg";
            break;
        case BYPASS_SHIFT_OUT:
        case BYPASS_ADVANCE:
        case BYPASS_FIND_LINE:
        case BYPASS_WIDTH_EXTEND:
        case BYPASS_SCAN_RETURN:
            if (!amount || amount > 4095U ||
                ((phase == BYPASS_SHIFT_OUT || phase == BYPASS_WIDTH_EXTEND) &&
                 amount + bypass->shift_mm > BYPASS_MAX_SIDE_MM) ||
                (phase == BYPASS_ADVANCE &&
                 amount + bypass->forward_mm > BYPASS_MAX_FORWARD_MM)) {
                bypass_abort(bypass, command, "exploration distance limit");
                return;
            }
            bypass->leg_mm = amount;
            result = moveForward((INT)amount);
            timeout_ms = BYPASS_MOVE_MARGIN_MS + ((amount + 9U) / 10U) * BYPASS_MOVE_TIMEOUT_MS_CM;
            break;
        case BYPASS_SCAN_BACK:
            bypass->leg_mm = BYPASS_SCAN_BACK_MM;
            result = moveBackward(BYPASS_SCAN_BACK_MM);
            timeout_ms = BYPASS_MOVE_MARGIN_MS + BYPASS_SCAN_BACK_MM / 10U * BYPASS_MOVE_TIMEOUT_MS_CM;
            break;
        case BYPASS_LINE_FOUND:
            result = stop();
            break;
        default:
            bypass_abort(bypass, command, "invalid movement phase");
            return;
    }
    if (result < E_OK) {
        bypass_abort(bypass, command, "motor command delivery");
        return;
    }
    bypass->move_pending = phase != BYPASS_LINE_FOUND && phase != BYPASS_WAIT_RELEASE;
    if (timeout_ms) bypass->deadline_us = bypass->phase_started_us + timeout_ms * 1000U;
    tm_printf((UB *)"[bypass] %s side=%d target=%u %s L=%d R=%d\n",
              bypass_phase_name(phase), bypass->side, amount, unit, left, right);
}

EXPORT void bypass_pivot(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT direction)
{
    if (direction != 1 && direction != -1) {
        bypass_abort(bypass, command, "invalid pivot direction");
        return;
    }
    MOTOR_COMMAND pivot = bypass_pivot_command(direction);
    UW degrees = direction > 0 ? BYPASS_LEFT_TURN_DEG : BYPASS_RIGHT_TURN_DEG;
    bypass_begin(bypass, command, phase, pivot.target_speed_left, pivot.target_speed_right, degrees);
}

EXPORT void bypass_track_side(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command)
{
    if (bypass->phase == BYPASS_ABORT) return;
    bypass->requested_us = time_us();
    bypass->last_tracking_us = bypass->last_sample_us = bypass->requested_us;
    bypass->tracking_active = TRUE;
    if (tk_set_flg(ranger_flgid, bypass->side > 0 ? SONAR_TRACK_RIGHT_FLAG : SONAR_TRACK_LEFT_FLAG) < E_OK)
        bypass_abort(bypass, command, "side tracking request");
}

EXPORT void bypass_probe(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, BYPASS_PHASE phase)
{
    if (bypass->phase == BYPASS_ABORT) return;
    if (bypass->phase != phase) bypass->probe_clear_count = bypass->probe_attempts = 0;
    bypass->phase = phase;
    bypass->tracking_active = FALSE;
    bypass->requested_us = bypass->last_sample_us = time_us();
    bypass->deadline_us = bypass->requested_us + BYPASS_PROBE_MS * 1000U;
    *command = (MOTOR_COMMAND){0}; // Motor already reported completion; do not overwrite its next target.
    if (tk_set_flg(ranger_flgid, SONAR_PROBE_FRONT_FLAG) < E_OK)
        bypass_abort(bypass, command, "stationary probe request");
}

// MOVE_DONE arrives after the wheels settle, so include braking/coast in the route position.
EXPORT BOOL bypass_complete_leg(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command)
{
    if (!bypass->leg_mm) return TRUE;
    INT moved = motor_get_avg_dist_mm() - bypass->leg_start_mm;
    if (moved <= 0 || moved > 4095 ||
        ((bypass->phase == BYPASS_SHIFT_OUT || bypass->phase == BYPASS_WIDTH_EXTEND) &&
         (UW)moved + bypass->shift_mm > 2047U) ||
        (bypass->phase == BYPASS_ADVANCE && (UW)moved + bypass->forward_mm > 4095U)) {
        bypass_abort(bypass, command, "invalid or excessive odometry at movement completion");
        return FALSE;
    }
    tm_printf((UB *)"[bypass] %s travelled=%d mm (requested=%u mm)\n",
              bypass_phase_name(bypass->phase), moved, bypass->leg_mm);
    bypass->leg_mm = (UW)moved;
    return TRUE;
}

EXPORT void bypass_follow_next(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command)
{
    if (bypass->phase == BYPASS_ABORT) return;
    UW minimum = (bypass->front_cm + BYPASS_PIVOT_FROM_FRONT_CM) * 10U;
    if (bypass->forward_mm >= minimum) {
        // Check the actual cross-section; a diagonal echo is not a measurement of box depth.
        bypass_pivot(bypass, command, BYPASS_TURN_IN, -bypass->side);
    } else {
        bypass_begin(bypass, command, BYPASS_ADVANCE,
                     BYPASS_API_POWER, -BYPASS_API_POWER, minimum - bypass->forward_mm);
    }
}

EXPORT void bypass_probe_result(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                               const SONAR_PROBE_RESULT *probe)
{
    if (bypass->phase != BYPASS_WIDTH_CHECK && bypass->phase != BYPASS_RETURN_CHECK) return;
    if (probe->angle_deg != 0 || (W)(probe->sampled_at_us - bypass->requested_us) < 0 ||
        (W)(probe->sampled_at_us - bypass->last_sample_us) <= 0 ||
        (UW)(time_us() - probe->sampled_at_us) >= BYPASS_PROBE_MS * 1000U) return;
    bypass->last_sample_us = probe->sampled_at_us;
    if (probe->status == SONAR_SENSOR_FAULT || probe->status == SONAR_SERVO_FAULT) {
        bypass_abort(bypass, command, "front probe hardware");
        return;
    }
    UW required_mm = bypass->phase == BYPASS_WIDTH_CHECK ?
                     (bypass->front_cm + ROBOT_LENGTH_CM) * 10U :
                     bypass->shift_mm + (ROBOT_LENGTH_CM + WALL_BODY_GAP_CM) * 10U;
    BOOL valid = probe->status == SONAR_OK && probe->valid &&
                 probe->distance_cm >= 2U && probe->distance_cm <= 400U;
    BOOL clear = probe->status == SONAR_NO_ECHO || (valid && probe->distance_cm * 10U > required_mm);
    BOOL blocked = probe->status == SONAR_TOO_CLOSE || (valid && !clear);
    if (bypass->probe_attempts < 3U) bypass->probe_attempts++;
    bypass->probe_clear_count = clear ? bypass->probe_clear_count + 1U : 0U;
    if ((clear && bypass->probe_clear_count < BYPASS_CLEAR_CONFIRM) ||
        (!clear && !blocked && bypass->probe_attempts < BYPASS_CLEAR_CONFIRM)) {
        bypass_probe(bypass, command, bypass->phase);
        return;
    }
    tm_printf((UB *)"[bypass] %s front %s: %u cm/status%d, sideways=%u along=%u mm\n",
              bypass->phase == BYPASS_WIDTH_CHECK ? "Width" : "Return", clear ? "clear" : "blocked/uncertain",
              probe->distance_cm, probe->status, bypass->shift_mm, bypass->forward_mm);
    switch (bypass->phase) {
        case BYPASS_WIDTH_CHECK:
            bypass->width_clearance_pending = clear;
            if (clear)
                tm_printf((UB *)"[bypass] Width clear: add %u mm outward once for pivot clearance\n", BYPASS_PIVOT_CLEAR_MM);
            bypass_track_side(bypass, command);
            bypass_pivot(bypass, command, BYPASS_WIDTH_TURN_OUT, bypass->side);
            break;
        case BYPASS_RETURN_CHECK:
            bypass->depth_clearance_pending = clear;
            if (clear)
                tm_printf((UB *)"[bypass] Depth clear: add %u mm parallel to box once before returning\n", BYPASS_DEPTH_CLEAR_MM);
            bypass_track_side(bypass, command);
            bypass_pivot(bypass, command, BYPASS_DEPTH_TURN_BACK, bypass->side);
            break;
        default:
            break;
    }
}
