
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Ultrasonic_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"
#include "../Motor/Motor_task.h"

// Servo settings
#define SERVO_PWM_WRAP          19999U
#define SERVO_CENTER_US          1695U
#define SERVO_SETTLE_MS           150U

// Ultrasonic settings
#define SONAR_TIMEOUT_US        26000U
#define SONAR_ECHO_END_US       60000U  // Allow a long pulse to finish; a permanently HIGH echo is a fault.
#define SONAR_PING_DELAY_MS        60U
#define OBSTACLE_TRIGGER_CM        30U
#define OBSTACLE_CONFIRM_COUNT      2U

// Coarse/fine profiling
#define COARSE_COUNT                7
#define MIN_VALID_COARSE_COUNT     5
#define FINE_PASSES                 2
#define EDGE_DEPTH_GAP_CM         10U

static const W coarse_angles[COARSE_COUNT] = {
    -45, -30, -15, 0, 15, 30, 45
};

static const char *const sonar_status_names[] = {
    "invalid", "ok", "no-echo", "too-close", "sensor-fault", "servo-fault", "unstable"
};

// Measure one ultrasonic echo
LOCAL SONAR_STATUS read_distance_cm(UW *distance_cm)
{
    UW start, pulse_us, measured_cm;
    *distance_cm = 0;

    if (gpio_get_val(SONAR_ECHO_PIN) != 0)
        return SONAR_SENSOR_FAULT;

    if (gpio_set_val(SONAR_TRIG_PIN, 0) < E_OK)
        return SONAR_SENSOR_FAULT;
    delay_us(2);
    if (gpio_set_val(SONAR_TRIG_PIN, 1) < E_OK)
        return SONAR_SENSOR_FAULT;
    delay_us(10);
    if (gpio_set_val(SONAR_TRIG_PIN, 0) < E_OK)
        return SONAR_SENSOR_FAULT;

    // Wait for echo to start
    start = time_us();

    while (gpio_get_val(SONAR_ECHO_PIN) == 0) {
        if ((UW)(time_us() - start) >= SONAR_TIMEOUT_US)
            return SONAR_NO_ECHO;
    }

    // Measure echo HIGH duration
    start = time_us();

    while (gpio_get_val(SONAR_ECHO_PIN) != 0) {
        if ((UW)(time_us() - start) >= SONAR_ECHO_END_US)
            return SONAR_SENSOR_FAULT;
    }

    pulse_us = (UW)(time_us() - start);
    measured_cm = (pulse_us + 29U) / 58U;

    if (measured_cm < 2U)
        return SONAR_TOO_CLOSE;
    if (measured_cm > 400U)
        return SONAR_NO_ECHO;

    *distance_cm = measured_cm;
    return SONAR_OK;
}

// Convert -45 to +45 degrees into servo pulse width
LOCAL UW servo_pulse_from_angle(W angle)
{
    if (angle < -45) angle = -45;
    if (angle > 45) angle = 45;

    // Positive angle is left
    return (UW)((W)SERVO_CENTER_US +
                angle * 350 / 45);
}

// Move servo and obtain a stable distance measurement
LOCAL SONAR_STATUS scan_angle(W angle, UW *distance_cm)
{
    UW readings[3];
    INT count = 0;
    INT no_echo_count = 0;
    *distance_cm = 0;

    if (pwm_set_cc(
            SERVO_PIN,
            servo_pulse_from_angle(angle)) < E_OK)
        return SONAR_SERVO_FAULT;

    tk_dly_tsk(SERVO_SETTLE_MS);

    for (INT i = 0; i < 3; i++) {
        UW cm = 0;

        if (i > 0)
            tk_dly_tsk(SONAR_PING_DELAY_MS);

        SONAR_STATUS status = read_distance_cm(&cm);
        if (status == SONAR_OK)
            readings[count++] = cm;
        else if (status == SONAR_NO_ECHO)
            no_echo_count++;
        else
            return status;
    }

    if (no_echo_count == 3)
        return SONAR_NO_ECHO;
    if (count < 2)
        return SONAR_UNSTABLE;

    if (count == 2) {
        UW gap = readings[0] > readings[1]
            ? readings[0] - readings[1]
            : readings[1] - readings[0];

        if (gap > 15U)
            return SONAR_UNSTABLE;

        *distance_cm = (readings[0] + readings[1]) / 2U;
        return SONAR_OK;
    }

    // Sort three samples and select median
    for (INT i = 0; i < 3; i++) {
        for (INT j = i + 1; j < 3; j++) {
            if (readings[j] < readings[i]) {
                UW temp = readings[i];
                readings[i] = readings[j];
                readings[j] = temp;
            }
        }
    }

    *distance_cm = readings[1];
    return SONAR_OK;
}

// Keep the closest observed point
LOCAL void update_closest(
    OBSTACLE_PROFILE *profile,
    W angle,
    UW cm)
{
    if (profile->closest_cm == 0U ||
        cm < profile->closest_cm) {

        profile->closest_cm = cm;
        profile->closest_angle_deg = angle;
    }
}

// Fixed-point sideways position
LOCAL W lateral_cm(W angle, UW distance_cm)
{
    W rad_mrad = angle * 17453 / 1000;

    W sin_scaled = rad_mrad -
        (rad_mrad * rad_mrad * rad_mrad) / 6000000;

    return ((W)distance_cm * sin_scaled) / 1000;
}

// Approximate forward depth of a ray
LOCAL UW forward_depth_cm(W angle, UW distance_cm)
{
    W rad_mrad = angle * 17453 / 1000;

    W cos_scaled = 1000 -
        (rad_mrad * rad_mrad) / 2000;

    return distance_cm * (UW)cos_scaled / 1000U;
}

// Classify possible obstacle-side returns
LOCAL BOOL same_obstacle_depth(
    W angle,
    UW distance_cm,
    UW center_cm)
{
    return forward_depth_cm(angle, distance_cm) <=
           center_cm + EDGE_DEPTH_GAP_CM;
}

// Refine a candidate boundary with two midpoint samples
LOCAL BOOL refine_edge(
    W near_angle,
    UW near_distance,
    W far_angle,
    UW center_cm,
    W *edge_angle,
    UW *edge_distance,
    OBSTACLE_PROFILE *profile)
{
    for (INT pass = 0; pass < FINE_PASSES; pass++) {
        W middle = (near_angle + far_angle) / 2;
        UW cm = 0;

        if (middle == near_angle || middle == far_angle)
            return FALSE;

        if (scan_angle(middle, &cm) != SONAR_OK)
            return FALSE;

        update_closest(profile, middle, cm);

        tm_printf(
            (UB *)"[sonar] fine %d: %u cm\n",
            middle, cm);

        if (same_obstacle_depth(middle, cm, center_cm)) {
            near_angle = middle;
            near_distance = cm;
        } else {
            far_angle = middle;
        }
    }

    *edge_angle = near_angle;
    *edge_distance = near_distance;
    return TRUE;
}

// Initial coarse and fine obstacle profile
LOCAL BOOL scan_obstacle(OBSTACLE_PROFILE *profile)
{
    UW cm[COARSE_COUNT] = {0};
    BOOL valid[COARSE_COUNT] = {0};

    UW left_cm = 0;
    UW right_cm = 0;

    INT valid_count = 0;
    BOOL center_ok;
    ER servo_er;

    *profile = (OBSTACLE_PROFILE){0};

    // Coarse scan: 7 directions, 15 degrees apart
    for (INT i = 0; i < COARSE_COUNT; i++) {

        valid[i] = scan_angle(coarse_angles[i], &cm[i]) == SONAR_OK;

        // Retry an invalid coarse angle once
        if (!valid[i]) {
            tk_dly_tsk(SONAR_PING_DELAY_MS);
            valid[i] = scan_angle(
                coarse_angles[i], &cm[i]) == SONAR_OK;
        }

        if (valid[i]) {
            valid_count++;
            update_closest(profile, coarse_angles[i], cm[i]);
        }

        tm_printf(
            (UB *)"[sonar] coarse %d: %u cm valid=%d\n",
            coarse_angles[i], cm[i], valid[i]);
    }

    center_ok = valid[3] &&
                cm[3] <= OBSTACLE_TRIGGER_CM + 20U;
    profile->front_cm = valid[3] ? cm[3] : 0U;
    profile->left_scan_cm = valid[COARSE_COUNT - 1] ? cm[COARSE_COUNT - 1] : 0U;
    profile->right_scan_cm = valid[0] ? cm[0] : 0U;

    if (center_ok) {

        // Find possible right boundary
        for (INT i = 3; i > 0; i--) {

            if (!valid[i] || !valid[i - 1])
                break;

            if (!same_obstacle_depth(
                    coarse_angles[i], cm[i], cm[3]))
                break;

            if (!same_obstacle_depth(
                    coarse_angles[i - 1], cm[i - 1], cm[3])) {

                profile->right_edge_valid = refine_edge(
                    coarse_angles[i], cm[i],
                    coarse_angles[i - 1], cm[3],
                    &profile->right_edge_deg,
                    &right_cm, profile);

                break;
            }
        }

        // Find possible left boundary
        for (INT i = 3; i < COARSE_COUNT - 1; i++) {

            if (!valid[i] || !valid[i + 1])
                break;

            if (!same_obstacle_depth(
                    coarse_angles[i], cm[i], cm[3]))
                break;

            if (!same_obstacle_depth(
                    coarse_angles[i + 1], cm[i + 1], cm[3])) {

                profile->left_edge_valid = refine_edge(
                    coarse_angles[i], cm[i],
                    coarse_angles[i + 1], cm[3],
                    &profile->left_edge_deg,
                    &left_cm, profile);

                break;
            }
        }
    }

    // Estimate left-side centre offset
    if (profile->left_edge_valid) {
        W x = lateral_cm(
            profile->left_edge_deg, left_cm);

        profile->left_offset_cm =
            (UW)(x > 0 ? x : 0) +
            ROBOT_HALF_WIDTH_CM +
            WALL_BODY_GAP_CM;

        profile->left_offset_valid = TRUE;
    }

    // Estimate right-side centre offset
    if (profile->right_edge_valid) {
        W x = lateral_cm(
            profile->right_edge_deg, right_cm);

        profile->right_offset_cm =
            (UW)(x < 0 ? -x : 0) +
            ROBOT_HALF_WIDTH_CM +
            WALL_BODY_GAP_CM;

        profile->right_offset_valid = TRUE;
    }

    // Estimate apparent obstacle width
    if (profile->left_edge_valid &&
        profile->right_edge_valid) {

        W left_x = lateral_cm(
            profile->left_edge_deg, left_cm);

        W right_x = lateral_cm(
            profile->right_edge_deg, right_cm);

        if (left_x > right_x) {
            profile->estimated_width_cm =
                (UW)(left_x - right_x);

            profile->width_valid = TRUE;
        }
    }

    // Return servo to forward position
    servo_er = pwm_set_cc(
        SERVO_PIN, SERVO_CENTER_US);

    if (servo_er >= E_OK)
        tk_dly_tsk(SERVO_SETTLE_MS);

    // A valid scan does not guarantee bypass clearance
    profile->profile_valid =
        center_ok &&
        valid_count >= MIN_VALID_COARSE_COUNT &&
        servo_er >= E_OK;

    profile->left_clearance_valid = FALSE;
    profile->right_clearance_valid = FALSE;

    tm_printf(
        (UB *)"[sonar] Coarse valid: %d/%d\n",
        valid_count, COARSE_COUNT);

    tm_printf(
        (UB *)"[sonar] Profile=%d Right=%d Left=%d Width=%d\n",
        profile->profile_valid,
        profile->right_edge_valid,
        profile->left_edge_valid,
        profile->width_valid);

    if (profile->width_valid) {
        tm_printf(
            (UB *)"[sonar] Approx width: %u cm\n",
            profile->estimated_width_cm);
    } else {
        tm_printf((UB *)"[sonar] Width unknown\n");
    }

    return profile->profile_valid;
}

// Send one Navigator message
LOCAL BOOL send_to_navigator(
    const NAVIGATOR_MESSAGE *message)
{
    ER er = tk_snd_mbf(
        navigator_mbfid,
        (void *)message,
        sizeof(*message),
        TMO_POL);
    if (er < E_OK) {
        motor_estop = TRUE;
        tm_printf((UB *)"[sonar] Failed to send Navigator event: %d\n", er);
        return FALSE;
    }
    return TRUE;
}

// Send live front distance without blocking the sonar task
LOCAL void send_sonar_distance(UW distance_cm, BOOL valid)
{
    NAVIGATOR_MESSAGE sonar_msg = {0};

    sonar_msg.type = MSG_SONAR_DISTANCE;
    sonar_msg.sampled_at_us = time_us();
    sonar_msg.data.sonar_distance.distance_cm = distance_cm;
    sonar_msg.data.sonar_distance.valid = valid;

    telemetry_snapshot.obstacle_distance_valid = valid;
    if (valid) {
        telemetry_snapshot.obstacle_distance_cm = distance_cm;
    }

    // Drop this telemetry update if the message buffer is full.
    // Never block the safety-monitoring loop for dashboard telemetry.
    (void)tk_snd_mbf(
        navigator_mbfid,
        (void *)&sonar_msg,
        sizeof(sonar_msg),
        TMO_POL
    );
}

// Perform one requested stationary measurement
LOCAL BOOL handle_probe(UINT flags)
{
    W angle = 0;
    UW cm = 0;

    if (flags & SONAR_PROBE_LEFT_FLAG)
        angle = 45;
    else if (flags & SONAR_PROBE_RIGHT_FLAG)
        angle = -45;

    UINT probes = SONAR_PROBE_FRONT_FLAG | SONAR_PROBE_LEFT_FLAG |
                  SONAR_PROBE_RIGHT_FLAG;
    BOOL valid = motor_estop && (flags & probes) != 0U &&
                 (flags & ~probes) == 0U && (flags & (flags - 1U)) == 0U;
    SONAR_STATUS status = SONAR_INVALID;
    if (valid)
        status = scan_angle(angle, &cm);
    else
        motor_estop = TRUE;

    UW sampled_at = time_us();

    ER er = pwm_set_cc(
        SERVO_PIN, SERVO_CENTER_US);

    if (er < E_OK)
        status = SONAR_SERVO_FAULT;
    else
        tk_dly_tsk(SERVO_SETTLE_MS);

    NAVIGATOR_MESSAGE message = {0};
    valid = status == SONAR_OK;
    message.type = MSG_SONAR_PROBED;
    message.sampled_at_us = sampled_at;

    message.data.sonar_probe.angle_deg = angle;
    message.data.sonar_probe.distance_cm =
        valid ? cm : 0U;
    message.data.sonar_probe.valid = valid;
    message.data.sonar_probe.status = status;

    tm_printf(
        (UB *)"[sonar] probe %d: %u cm valid=%d status=%s\n",
        angle,
        message.data.sonar_probe.distance_cm,
        valid, sonar_status_names[status]);

    return send_to_navigator(&message);
}

EXPORT void ultrasonic_task(INT stacd, void *exinf)
{
    UW front_cm = 0;
    UW confirmations = 0;
    UINT pattern;

    NAVIGATOR_MESSAGE message = {0};

    (void)stacd;
    (void)exinf;

    // Configure TRIG and ECHO
    if (gpio_set_pin(
            SONAR_TRIG_PIN, GPIO_MODE_OUT) < E_OK ||
        gpio_set_pin(
            SONAR_ECHO_PIN, GPIO_MODE_IN) < E_OK) {

        motor_estop = TRUE;
        tm_printf((UB *)"[sonar] GPIO setup failed\n");
        return;
    }

    gpio_set_val(SONAR_TRIG_PIN, 0);

    // Configure 50 Hz servo PWM
    if (pwm_set_divider(SERVO_PIN, 125, 0) < E_OK ||
        pwm_set_wrap(SERVO_PIN, SERVO_PWM_WRAP) < E_OK ||
        pwm_set_pin(SERVO_PIN) < E_OK ||
        pwm_set_cc(
            SERVO_PIN, SERVO_CENTER_US) < E_OK ||
        pwm_set_enabled(
            SERVO_PIN, TRUE) < E_OK) {

        motor_estop = TRUE;
        tm_printf((UB *)"[sonar] Servo setup failed\n");
        return;
    }

    tk_dly_tsk(SERVO_SETTLE_MS);

    tm_printf(
        (UB *)"[sonar] Forward monitoring at %u cm\n",
        OBSTACLE_TRIGGER_CM);

    while (1) {

        // Forward monitoring during line following
        BOOL valid = read_distance_cm(&front_cm) == SONAR_OK;

        // Send live distance to Navigator on every measurement
        send_sonar_distance(front_cm, valid);

        // Preserve existing obstacle confirmation logic
        if (valid && front_cm <= OBSTACLE_TRIGGER_CM)
            confirmations++;
        else
            confirmations = 0;

        if (confirmations < OBSTACLE_CONFIRM_COUNT) {
            tk_dly_tsk(SONAR_PING_DELAY_MS);
            continue;
        }

        // Stop before sending the detection event
        motor_estop = TRUE;

        message.type = MSG_OBSTACLE_DETECTED;
        message.sampled_at_us = time_us();
        message.data.obstacle_distance_cm = front_cm;

        tm_printf(
            (UB *)"[sonar] Obstacle at %u cm\n",
            front_cm);

        if (!send_to_navigator(&message))
            return;

        // Wait until Motor stopping is acknowledged
        if (tk_wai_flg(
                ranger_flgid,
                SONAR_SCAN_REQUEST,
                TWF_ORW | TWF_BITCLR,
                &pattern,
                TMO_FEVR) < E_OK)
            return;

        // Coarse/fine scan of stationary obstacle
        OBSTACLE_PROFILE profile = {0};
        scan_obstacle(&profile);

        message.type = MSG_OBSTACLE_PROFILED;
        message.sampled_at_us = time_us();
        message.data.obstacle_profile = profile;

        if (!send_to_navigator(&message))
            return;

        // Tracking keeps the servo parked; directional scans still require a stop.
        BOOL tracking = FALSE;
        W tracking_angle = 0;
        while (1) {
            UINT requests =
                SONAR_RESUME_FLAG |
                SONAR_SCAN_REQUEST |
                SONAR_PROBE_FRONT_FLAG |
                SONAR_PROBE_LEFT_FLAG |
                SONAR_PROBE_RIGHT_FLAG |
                SONAR_TRACK_LEFT_FLAG |
                SONAR_TRACK_RIGHT_FLAG |
                SONAR_PAUSE_FLAG;

            ER er = tk_wai_flg(
                    ranger_flgid,
                    requests,
                    TWF_ORW | TWF_BITCLR,
                    &pattern,
                    tracking ? (TMO)SONAR_PING_DELAY_MS : TMO_FEVR);

            if (er == E_TMOUT && tracking) {
                UW cm = 0;
                message = (NAVIGATOR_MESSAGE){0};
                message.type = MSG_TRACKING_OBSTACLE;
                message.data.sonar_probe.status = read_distance_cm(&cm);
                message.data.sonar_probe.valid = message.data.sonar_probe.status == SONAR_OK;
                message.sampled_at_us = time_us();
                message.data.sonar_probe.angle_deg = tracking_angle;
                message.data.sonar_probe.distance_cm = cm;
                if (!send_to_navigator(&message))
                    return;
                if (!message.data.sonar_probe.valid)
                    tracking = FALSE;
                continue;
            }
            if (er < E_OK) {

                motor_estop = TRUE;
                return;
            }

            pattern &= requests;
            tracking = FALSE;
            if (pattern == 0U || (pattern & (pattern - 1U)) != 0U) {
                (void)handle_probe(pattern);
                motor_estop = TRUE;
                continue;
            }
            if (pattern == SONAR_RESUME_FLAG) {
                if (pwm_set_cc(SERVO_PIN, SERVO_CENTER_US) < E_OK) {
                    motor_estop = TRUE;
                    return;
                }
                tk_dly_tsk(SERVO_SETTLE_MS);
                break;
            }
            if (pattern == SONAR_PAUSE_FLAG)
                continue;

            if (pattern == SONAR_SCAN_REQUEST) {
                message = (NAVIGATOR_MESSAGE){0};
                message.type = MSG_OBSTACLE_PROFILED;
                if (motor_estop)
                    scan_obstacle(&message.data.obstacle_profile);
                else
                    motor_estop = TRUE;
                message.sampled_at_us = time_us();
                if (!send_to_navigator(&message))
                    return;
                continue;
            }

            if (pattern == SONAR_TRACK_LEFT_FLAG ||
                pattern == SONAR_TRACK_RIGHT_FLAG) {
                UW cm = 0;
                tracking_angle = pattern == SONAR_TRACK_LEFT_FLAG ? 45 : -45;
                message = (NAVIGATOR_MESSAGE){0};
                message.type = MSG_TRACKING_OBSTACLE;
                message.data.sonar_probe.status = motor_estop ?
                    scan_angle(tracking_angle, &cm) : SONAR_INVALID;
                message.data.sonar_probe.valid = message.data.sonar_probe.status == SONAR_OK;
                message.sampled_at_us = time_us();
                message.data.sonar_probe.angle_deg = tracking_angle;
                message.data.sonar_probe.distance_cm = cm;
                tracking = message.data.sonar_probe.valid;
                if (!send_to_navigator(&message))
                    return;
                continue;
            }

            if (!handle_probe(pattern)) {
                motor_estop = TRUE;
                tm_printf((UB *)"[sonar] Probe failed\n");
                return;
            }
        }

        confirmations = 0;
        tk_dly_tsk(SONAR_PING_DELAY_MS);
    }
}
