
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
#define SONAR_ECHO_END_US       60000U
#define SONAR_PING_DELAY_MS        60U
#define OBSTACLE_CONFIRM_COUNT      2U

// Coarse/fine profiling
#define COARSE_COUNT                SONAR_PROFILE_RAYS
#define FINE_PASSES                 2
#define EDGE_DEPTH_GAP_CM         10U

EXPORT volatile BOOL sonar_fault = FALSE;

LOCAL BOOL hardware_fault(SONAR_STATUS status)
{
    if (status != SONAR_SENSOR_FAULT && status != SONAR_SERVO_FAULT)
        return FALSE;
    sonar_fault = TRUE;
    motor_estop = TRUE;
    return TRUE;
}

static const W coarse_angles[COARSE_COUNT] = {
    -45, -30, -15, 0, 15, 30, 45
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

        SONAR_STATUS status = scan_angle(middle, &cm);
        if (hardware_fault(status))
            return FALSE;
        if (status == SONAR_NO_ECHO) {
            far_angle = middle;
            continue;
        }
        if (status != SONAR_OK)
            return FALSE;

        update_closest(profile, middle, cm);

        tm_printf(
            (UB *)"[sonar] fine %d: %u cm\n",
            middle, cm);

        if (same_obstacle_depth(middle, cm, center_cm)) {
            near_angle = middle;
        } else {
            far_angle = middle;
        }
    }

    // Use the outer bound, so angular scan resolution cannot understate the width.
    *edge_angle = far_angle;
    UW cosine = forward_depth_cm(far_angle, 1000U);
    *edge_distance = (center_cm * 1000U + cosine - 1U) / cosine;
    return TRUE;
}

// Initial coarse and fine obstacle profile
LOCAL BOOL scan_obstacle(OBSTACLE_PROFILE *profile)
{
    UW cm[COARSE_COUNT] = {0};
    BOOL valid[COARSE_COUNT] = {0};
    SONAR_STATUS status[COARSE_COUNT] = {0};

    UW left_cm = 0;
    UW right_cm = 0;

    INT valid_count = 0;
    BOOL center_ok;
    ER servo_er;

    *profile = (OBSTACLE_PROFILE){0};

    // Coarse scan: 7 directions, 15 degrees apart
    for (INT i = 0; i < COARSE_COUNT; i++) {

        status[i] = scan_angle(coarse_angles[i], &cm[i]);
        if (hardware_fault(status[i]))
            return FALSE;

        // Open space is a usable edge candidate, not a failed scan.
        if (status[i] == SONAR_UNSTABLE) {
            tk_dly_tsk(SONAR_PING_DELAY_MS);
            status[i] = scan_angle(
                coarse_angles[i], &cm[i]);
            if (hardware_fault(status[i]))
                return FALSE;
        }
        valid[i] = status[i] == SONAR_OK;
        profile->ray_cm[i] = cm[i];
        profile->ray_status[i] = status[i];

        if (valid[i]) {
            valid_count++;
            update_closest(profile, coarse_angles[i], cm[i]);
        }

        tm_printf(
            (UB *)"[sonar] coarse %d: %u cm valid=%d\n",
            coarse_angles[i], cm[i], valid[i]);
    }

    center_ok = valid[3];
    profile->front_cm = valid[3] ? cm[3] : 0U;
    // Compare the actual outer rays when choosing a side; missing edges must not leave side=0.
    profile->left_scan_cm = cm[6];
    profile->right_scan_cm = cm[0];
    profile->left_scan_status = status[6];
    profile->right_scan_status = status[0];

    if (center_ok) {

        // Find possible right boundary
        for (INT i = 3; i > 0; i--) {

            if (!valid[i] || (!valid[i - 1] && status[i - 1] != SONAR_NO_ECHO))
                break;

            if (!same_obstacle_depth(
                    coarse_angles[i], cm[i], cm[3]))
                break;

            if (status[i - 1] == SONAR_NO_ECHO || !same_obstacle_depth(
                    coarse_angles[i - 1], cm[i - 1], cm[3])) {

                profile->right_edge_valid = refine_edge(
                    coarse_angles[i],
                    coarse_angles[i - 1], cm[3],
                    &profile->right_edge_deg,
                    &right_cm, profile);

                break;
            }
        }

        // Find possible left boundary
        for (INT i = 3; i < COARSE_COUNT - 1; i++) {

            if (!valid[i] || (!valid[i + 1] && status[i + 1] != SONAR_NO_ECHO))
                break;

            if (!same_obstacle_depth(
                    coarse_angles[i], cm[i], cm[3]))
                break;

            if (status[i + 1] == SONAR_NO_ECHO || !same_obstacle_depth(
                    coarse_angles[i + 1], cm[i + 1], cm[3])) {

                profile->left_edge_valid = refine_edge(
                    coarse_angles[i],
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
    else
        hardware_fault(SONAR_SERVO_FAULT);

    // A valid scan does not guarantee bypass clearance
    profile->profile_valid =
        center_ok &&
        !sonar_fault &&
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
        100);
    if (er < E_OK) {
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

    // Navigator requests probes only after MOVE_DONE; a normal stopped motor needs no estop latch.

    SONAR_STATUS status = scan_angle(angle, &cm);

    ER er = pwm_set_cc(
        SERVO_PIN, SERVO_CENTER_US);

    if (er < E_OK)
        status = SONAR_SERVO_FAULT;
    else
        tk_dly_tsk(SERVO_SETTLE_MS);

    NAVIGATOR_MESSAGE message = {0};
    message.type = MSG_SONAR_PROBED;

    message.data.sonar_probe.angle_deg = angle;
    message.data.sonar_probe.distance_cm =
        status == SONAR_OK ? cm : 0U;
    message.data.sonar_probe.valid = status == SONAR_OK;
    message.data.sonar_probe.status = status;
    message.data.sonar_probe.sampled_at_us = time_us();
    hardware_fault(status);

    tm_printf(
        (UB *)"[sonar] probe %d: %u cm valid=%d\n",
        angle,
        message.data.sonar_probe.distance_cm,
        message.data.sonar_probe.valid);

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

        hardware_fault(SONAR_SENSOR_FAULT);
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

        hardware_fault(SONAR_SERVO_FAULT);
        tm_printf((UB *)"[sonar] Servo setup failed\n");
        return;
    }

    tk_dly_tsk(SERVO_SETTLE_MS);

    tm_printf(
        (UB *)"[sonar] Forward monitoring at %u cm\n",
        OBSTACLE_TRIGGER_CM);

    while (1) {

        // Forward monitoring during line following
        SONAR_STATUS status = read_distance_cm(&front_cm);
        if (hardware_fault(status))
            return;
        BOOL valid = status == SONAR_OK || status == SONAR_TOO_CLOSE;
        if (status == SONAR_TOO_CLOSE)
            front_cm = 1U;

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
        message.data.obstacle_profile = profile;

        if (!send_to_navigator(&message))
            return;

        // A tracking request parks the servo once; missed echoes do not end tracking.
        BOOL tracking = FALSE;
        W tracking_angle = 0;
        while (1) {
            UINT requests =
                SONAR_RESUME_FLAG |
                SONAR_SCAN_REQUEST |
                SONAR_TRACK_LEFT_FLAG |
                SONAR_TRACK_RIGHT_FLAG |
                SONAR_PROBE_FRONT_FLAG |
                SONAR_PROBE_LEFT_FLAG |
                SONAR_PROBE_RIGHT_FLAG;

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
                message.data.sonar_probe.distance_cm = cm;
                message.data.sonar_probe.angle_deg = tracking_angle;
                message.data.sonar_probe.sampled_at_us = time_us();
                BOOL failed = hardware_fault(message.data.sonar_probe.status);
                // Drop a streaming sample rather than stall the sonar task behind telemetry.
                (void)tk_snd_mbf(navigator_mbfid, &message, sizeof(message), TMO_POL);
                if (failed)
                    return;
                continue;
            }
            if (er < E_OK) {
                hardware_fault(SONAR_SENSOR_FAULT);
                return;
            }

            if (pattern & SONAR_RESUME_FLAG) {
                if (pwm_set_cc(SERVO_PIN, SERVO_CENTER_US) < E_OK) {
                    hardware_fault(SONAR_SERVO_FAULT);
                    return;
                }
                tk_dly_tsk(SERVO_SETTLE_MS);
                break;
            }

            // Stationary requests end streaming. Navigator waits for the reply before moving again.
            if (pattern & SONAR_SCAN_REQUEST) {
                tracking = FALSE;
                message = (NAVIGATOR_MESSAGE){0};
                message.type = MSG_OBSTACLE_PROFILED;
                scan_obstacle(&message.data.obstacle_profile);
                if (!send_to_navigator(&message)) {
                    hardware_fault(SONAR_SENSOR_FAULT);
                    return;
                }
                continue;
            }

            if (pattern & (SONAR_PROBE_FRONT_FLAG | SONAR_PROBE_LEFT_FLAG | SONAR_PROBE_RIGHT_FLAG)) {
                tracking = FALSE;
                if (!handle_probe(pattern)) {
                    hardware_fault(SONAR_SENSOR_FAULT);
                    return;
                }
                continue;
            }

            if (pattern & (SONAR_TRACK_LEFT_FLAG | SONAR_TRACK_RIGHT_FLAG)) {
                W angle = (pattern & SONAR_TRACK_LEFT_FLAG) ? 45 : -45;
                if (!tracking || angle != tracking_angle) {
                    if (pwm_set_cc(SERVO_PIN, servo_pulse_from_angle(angle)) < E_OK) {
                        hardware_fault(SONAR_SERVO_FAULT);
                        return;
                    }
                    tk_dly_tsk(SERVO_SETTLE_MS);
                }
                tracking_angle = angle;
                tracking = TRUE;
                continue;
            }

        }

        confirmations = 0;
        tk_dly_tsk(SONAR_PING_DELAY_MS);
    }
}
