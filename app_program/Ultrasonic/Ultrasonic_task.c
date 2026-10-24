#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Ultrasonic_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"
#include "../Motor/Motor_task.h"

#define SERVO_PWM_WRAP          19999U
#define SERVO_CENTER_US          1695U
#define SERVO_SETTLE_MS           300U

#define SONAR_TIMEOUT_US        26000U
#define SONAR_PING_DELAY_MS        60U
#define OBSTACLE_TRIGGER_CM        30U
#define OBSTACLE_CONFIRM_COUNT      2U

#define COARSE_COUNT                7
#define FINE_PASSES                 2
#define EDGE_DEPTH_GAP_CM         10U  // Provisional depth-change threshold

#define ROBOT_HALF_WIDTH_CM        7U  // Round up from 6.25 cm
#define SAFETY_MARGIN_CM           2U  // Test value
static const W coarse_angles[COARSE_COUNT] = {
    -45, -30, -15, 0, 15, 30, 45
};

// Read one ultrasonic ECHO pulse.
LOCAL BOOL read_distance_cm(UW *distance_cm)
{
    UW start, pulse_us, measured_cm;

    if (gpio_get_val(SONAR_ECHO_PIN) != 0)
        return FALSE;

    gpio_set_val(SONAR_TRIG_PIN, 0);
    delay_us(2);
    gpio_set_val(SONAR_TRIG_PIN, 1);
    delay_us(10);
    gpio_set_val(SONAR_TRIG_PIN, 0);

    start = time_us();
    while (gpio_get_val(SONAR_ECHO_PIN) == 0) {
        if ((UW)(time_us() - start) >= SONAR_TIMEOUT_US)
            return FALSE;
    }

    start = time_us();
    while (gpio_get_val(SONAR_ECHO_PIN) != 0) {
        if ((UW)(time_us() - start) >= SONAR_TIMEOUT_US)
            return FALSE;
    }

    pulse_us = (UW)(time_us() - start);
    measured_cm = (pulse_us + 29U) / 58U;

    if (measured_cm < 2U || measured_cm > 400U)
        return FALSE;

    *distance_cm = measured_cm;
    return TRUE;
}

// +45 is left, -45 is right; servo pulse calibration is physical.
LOCAL UW servo_pulse_from_angle(W angle_deg)
{
    if (angle_deg < -45) angle_deg = -45;
    if (angle_deg > 45) angle_deg = 45;
    return (UW)((W)SERVO_CENTER_US + angle_deg * 350 / 45);
}

// Take a median of three, or two agreeing samples.
LOCAL BOOL scan_angle(W angle_deg, UW *distance_cm)
{
    UW readings[3];
    INT count = 0;

    if (pwm_set_cc(SERVO_PIN, servo_pulse_from_angle(angle_deg)) < E_OK)
        return FALSE;

    tk_dly_tsk(SERVO_SETTLE_MS);

    for (INT i = 0; i < 3; i++) {
        UW distance;

        if (i != 0)
            tk_dly_tsk(SONAR_PING_DELAY_MS);

        if (read_distance_cm(&distance))
            readings[count++] = distance;
    }

    if (count < 2)
        return FALSE;

    if (count == 2) {
        UW gap = readings[0] > readings[1]
            ? readings[0] - readings[1]
            : readings[1] - readings[0];

        if (gap > 15U)
            return FALSE;

        *distance_cm = (readings[0] + readings[1]) / 2U;
        return TRUE;
    }

    // Sort only three values to obtain the median.
    for (INT i = 0; i < 3; i++) {
        for (INT j = i + 1; j < 3; j++) {
            if (readings[j] < readings[i]) {
                UW tmp = readings[i];
                readings[i] = readings[j];
                readings[j] = tmp;
            }
        }
    }

    *distance_cm = readings[1];
    return TRUE;
}

LOCAL void update_closest(OBSTACLE_PROFILE *p, W angle_deg, UW cm)
{
    if (p->closest_cm == 0U || cm < p->closest_cm) {
        p->closest_cm = cm;
        p->closest_angle_deg = angle_deg;
    }
}

// Fixed-point sine approximation valid over +/-45 degrees.
LOCAL W lateral_cm(W angle_deg, UW distance_cm)
{
    W rad_mrad = angle_deg * 17453 / 1000;
    W sin_scaled = rad_mrad -
        (rad_mrad * rad_mrad * rad_mrad) / 6000000;

    return ((W)distance_cm * sin_scaled) / 1000;
}

// Approximate forward depth to compare rays at different angles.
LOCAL UW forward_depth_cm(W angle_deg, UW distance_cm)
{
    W rad_mrad = angle_deg * 17453 / 1000;
    W cos_scaled = 1000 - (rad_mrad * rad_mrad) / 2000;

    return distance_cm * (UW)cos_scaled / 1000U;
}

// 'Near' means consistent with the observed front obstacle depth.
LOCAL BOOL same_obstacle_depth(W angle_deg, UW cm, UW centre_cm)
{
    return forward_depth_cm(angle_deg, cm) <=
           centre_cm + EDGE_DEPTH_GAP_CM;
}

// Two adaptive midpoints narrow a 15-degree near/far bracket.
LOCAL BOOL refine_edge(W near_deg, UW near_cm, W far_deg,
                       UW centre_cm, W *result_deg, UW *result_cm,
                       OBSTACLE_PROFILE *profile)
{
    for (INT pass = 0; pass < FINE_PASSES; pass++) {
        W middle = (near_deg + far_deg) / 2;
        UW cm = 0;

        if (middle == near_deg || middle == far_deg)
            return FALSE;

        if (!scan_angle(middle, &cm))
            return FALSE;  // Unknown echo is never assumed clear.

        update_closest(profile, middle, cm);

        tm_printf((UB *)"[sonar] fine %d deg: %u cm\n", middle, cm);

        if (same_obstacle_depth(middle, cm, centre_cm)) {
            near_deg = middle;
            near_cm = cm;
        } else {
            far_deg = middle;
        }
    }

    *result_deg = near_deg;  // Measured point on obstacle side
    *result_cm = near_cm;
    return TRUE;
}

// Coarse map, boundary refinement, and conservative geometry.
LOCAL BOOL scan_obstacle(OBSTACLE_PROFILE *profile)
{
    UW cm[COARSE_COUNT] = {0};
    BOOL valid[COARSE_COUNT] = {0};
    UW left_cm = 0, right_cm = 0;
    INT valid_count = 0;
    BOOL centre_ok;
    ER servo_er;

    *profile = (OBSTACLE_PROFILE){0};

    // Stage 1: seven coarse directions, 15 degrees apart.
    for (INT i = 0; i < COARSE_COUNT; i++) {
        valid[i] = scan_angle(coarse_angles[i], &cm[i]);

        if (valid[i]) {
            valid_count++;
            update_closest(profile, coarse_angles[i], cm[i]);
        }

        tm_printf((UB *)"[sonar] coarse %d: %u cm valid=%d\n",
                  coarse_angles[i], cm[i], valid[i]);
    }

    // A stable forward obstacle should still be visible after stopping.
    centre_ok = valid[3] && cm[3] <=
                OBSTACLE_TRIGGER_CM + 20U;

    if (centre_ok) {
        // Stage 2a: right boundary, scanning from centre outwards.
        for (INT i = 3; i > 0; i--) {
            if (!valid[i] || !valid[i - 1])
                break;

            if (!same_obstacle_depth(coarse_angles[i], cm[i], cm[3]))
                break;

            if (!same_obstacle_depth(coarse_angles[i - 1], cm[i - 1], cm[3])) {
                profile->right_edge_valid = refine_edge(
                    coarse_angles[i], cm[i], coarse_angles[i - 1],
                    cm[3], &profile->right_edge_deg, &right_cm, profile);
                break;
            }
        }

        // Stage 2b: left boundary, scanning from centre outwards.
        for (INT i = 3; i < COARSE_COUNT - 1; i++) {
            if (!valid[i] || !valid[i + 1])
                break;

            if (!same_obstacle_depth(coarse_angles[i], cm[i], cm[3]))
                break;

            if (!same_obstacle_depth(coarse_angles[i + 1], cm[i + 1], cm[3])) {
                profile->left_edge_valid = refine_edge(
                    coarse_angles[i], cm[i], coarse_angles[i + 1],
                    cm[3], &profile->left_edge_deg, &left_cm, profile);
                break;
            }
        }
    }

    // Stage 3: estimate visible lateral extent and minimum centre offsets.
    if (profile->left_edge_valid) {
        W left_x = lateral_cm(profile->left_edge_deg, left_cm);
        profile->left_offset_cm = (UW)(left_x > 0 ? left_x : 0) +
            ROBOT_HALF_WIDTH_CM + SAFETY_MARGIN_CM;
        profile->left_offset_valid = TRUE;
    }

    if (profile->right_edge_valid) {
        W right_x = lateral_cm(profile->right_edge_deg, right_cm);
        profile->right_offset_cm = (UW)(right_x < 0 ? -right_x : 0) +
            ROBOT_HALF_WIDTH_CM + SAFETY_MARGIN_CM;
        profile->right_offset_valid = TRUE;
    }

    if (profile->left_edge_valid && profile->right_edge_valid) {
        W left_x = lateral_cm(profile->left_edge_deg, left_cm);
        W right_x = lateral_cm(profile->right_edge_deg, right_cm);

        if (left_x > right_x) {
            profile->estimated_width_cm = (UW)(left_x - right_x);
            profile->width_valid = TRUE;
        }
    }

    servo_er = pwm_set_cc(SERVO_PIN, SERVO_CENTER_US);
    if (servo_er >= E_OK)
        tk_dly_tsk(SERVO_SETTLE_MS);

    // This reports sensing quality; it does NOT authorise movement.
    profile->profile_valid = centre_ok && servo_er >= E_OK;
    profile->left_clearance_valid = FALSE;
    profile->right_clearance_valid = FALSE;

    tm_printf((UB *)"[sonar] samples=%d centre=%d edges R=%d L=%d width=%d\n",
              valid_count, centre_ok, profile->right_edge_valid,
              profile->left_edge_valid, profile->width_valid);

    return profile->profile_valid;
}

LOCAL BOOL send_to_navigator(const NAVIGATOR_MESSAGE *message)
{
    ER er = tk_snd_mbf(navigator_mbfid, (void *)message,
                       sizeof(*message), TMO_FEVR);
    return er >= E_OK;
}

EXPORT void ultrasonic_task(INT stacd, void *exinf)
{
    UW front_cm = 0;
    UW confirmations = 0;
    UINT pattern;
    NAVIGATOR_MESSAGE message = {0};

    (void)stacd;
    (void)exinf;

    if (gpio_set_pin(SONAR_TRIG_PIN, GPIO_MODE_OUT) < E_OK ||
        gpio_set_pin(SONAR_ECHO_PIN, GPIO_MODE_IN) < E_OK) {
        motor_estop = TRUE;
        tm_printf((UB *)"[sonar] GPIO setup failed\n");
        return;
    }
    gpio_set_val(SONAR_TRIG_PIN, 0);

    if (pwm_set_divider(SERVO_PIN, 125, 0) < E_OK ||
        pwm_set_wrap(SERVO_PIN, SERVO_PWM_WRAP) < E_OK ||
        pwm_set_pin(SERVO_PIN) < E_OK ||
        pwm_set_cc(SERVO_PIN, SERVO_CENTER_US) < E_OK ||
        pwm_set_enabled(SERVO_PIN, TRUE) < E_OK) {
        motor_estop = TRUE;
        tm_printf((UB *)"[sonar] Servo setup failed\n");
        return;
    }

    tk_dly_tsk(SERVO_SETTLE_MS);
    tm_printf((UB *)"[sonar] Forward monitoring at %u cm\n",
              OBSTACLE_TRIGGER_CM);

    while (1) {
        BOOL valid = read_distance_cm(&front_cm);

        if (valid && front_cm <= OBSTACLE_TRIGGER_CM)
            confirmations++;
        else
            confirmations = 0;

        if (confirmations < OBSTACLE_CONFIRM_COUNT) {
            tk_dly_tsk(SONAR_PING_DELAY_MS);
            continue;
        }

        // Stop and inform Navigator; do not scan while moving.
        motor_estop = TRUE;
        message.type = MSG_OBSTACLE_DETECTED;
        message.data.obstacle_distance_cm = front_cm;
        tm_printf((UB *)"[sonar] Obstacle at %u cm\n", front_cm);

        if (!send_to_navigator(&message))
            return;

        // Navigator sets this only after MSG_MOTOR_STOPPED.
        if (tk_wai_flg(ranger_flgid, SONAR_SCAN_REQUEST,
                       TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR) < E_OK)
            return;

        OBSTACLE_PROFILE profile = {0};
        scan_obstacle(&profile);

        message.type = MSG_OBSTACLE_PROFILED;
        message.data.obstacle_profile = profile;
        if (!send_to_navigator(&message))
            return;

        // Bypass and line recovery are not enabled in this build.
        if (tk_wai_flg(ranger_flgid, SONAR_RESUME_FLAG,
                       TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR) < E_OK)
            return;

        confirmations = 0;
        tk_dly_tsk(SONAR_PING_DELAY_MS);
    }
}
