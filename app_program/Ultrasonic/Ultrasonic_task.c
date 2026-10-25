
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Ultrasonic_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"
#include "../Motor/Motor_task.h"

// Servo calibration
#define SERVO_PWM_WRAP          19999U
#define SERVO_CENTER_US          1695U
#define SERVO_SETTLE_MS           300U

// Ultrasonic configuration
#define SONAR_TIMEOUT_US        26000U
#define SONAR_PING_DELAY_MS        60U
#define OBSTACLE_TRIGGER_CM       20U
#define OBSTACLE_CONFIRM_COUNT      2U

// Obstacle profiling
#define COARSE_COUNT              7
#define PROFILE_NEAR_CM          50U
#define FINE_PASSES               2

static const W coarse_angles[COARSE_COUNT] = {
    -45, -30, -15, 0, 15, 30, 45
};

// Measure distance using TRIG and ECHO
LOCAL BOOL read_distance_cm(UW *distance_cm)
{
    UW start;
    UW pulse_us;
    UW measured_cm;

    if (gpio_get_val(SONAR_ECHO_PIN) != 0)
        return FALSE;

    // Send the trigger pulse
    gpio_set_val(SONAR_TRIG_PIN, 0);
    delay_us(2);
    gpio_set_val(SONAR_TRIG_PIN, 1);
    delay_us(10);
    gpio_set_val(SONAR_TRIG_PIN, 0);

    // Wait for ECHO HIGH
    start = time_us();

    while (gpio_get_val(SONAR_ECHO_PIN) == 0) {
        if ((UW)(time_us() - start) >= SONAR_TIMEOUT_US)
            return FALSE;
    }

    // Measure ECHO HIGH duration
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

// Convert angle to calibrated servo pulse width
LOCAL UW servo_pulse_from_angle(W angle_deg)
{
    if (angle_deg < -45)
        angle_deg = -45;

    if (angle_deg > 45)
        angle_deg = 45;

    W pulse = (W)SERVO_CENTER_US +
              angle_deg * 350 / 45;

    return (UW)pulse;
}

// Rotate servo and take repeated readings
LOCAL BOOL scan_angle(W angle_deg, UW *distance_cm)
{
    UW pulse = servo_pulse_from_angle(angle_deg);
    UW readings[3];
    INT count = 0;

    if (pwm_set_cc(SERVO_PIN, pulse) < E_OK)
        return FALSE;

    tk_dly_tsk(SERVO_SETTLE_MS);

    for (INT i = 0; i < 3; i++) {
        UW distance = 0;

        if (i > 0)
            tk_dly_tsk(SONAR_PING_DELAY_MS);

        if (read_distance_cm(&distance))
            readings[count++] = distance;
    }

    // Require at least two valid measurements
    if (count < 2)
        return FALSE;

    if (count == 2) {
        UW difference = readings[0] > readings[1]
            ? readings[0] - readings[1]
            : readings[1] - readings[0];

        if (difference > 15U)
            return FALSE;

        *distance_cm = (readings[0] + readings[1]) / 2U;
        return TRUE;
    }

    // Median of three valid measurements
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
    return TRUE;
}

// Record the closest point from coarse or fine scans
LOCAL void update_closest(
    OBSTACLE_PROFILE *profile,
    W angle_deg,
    UW distance_cm)
{
    if (profile->closest_cm == 0U ||
        distance_cm < profile->closest_cm) {

        profile->closest_cm = distance_cm;
        profile->closest_angle_deg = angle_deg;
    }
}

// Refine a nearby-to-farther transition
LOCAL BOOL refine_edge(
    W near_angle,
    UW near_distance,
    W far_angle,
    W *edge_angle,
    UW *edge_distance,
    OBSTACLE_PROFILE *profile)
{
    for (INT i = 0; i < FINE_PASSES; i++) {
        W middle = (near_angle + far_angle) / 2;
        UW distance = 0;

        if (!scan_angle(middle, &distance))
            return FALSE;

        update_closest(profile, middle, distance);

        if (distance <= PROFILE_NEAR_CM) {
            near_angle = middle;
            near_distance = distance;
        } else {
            far_angle = middle;
        }
    }

    // Retain the last nearby measured point
    *edge_angle = near_angle;
    *edge_distance = near_distance;
    return TRUE;
}

// Approximate lateral position without requiring libm
LOCAL W lateral_position_cm(W angle_deg, UW distance_cm)
{
    W angle_mrad = angle_deg * 17453 / 1000;
    W sin_scaled = angle_mrad -
        (angle_mrad * angle_mrad * angle_mrad) / 6000000;

    return ((W)distance_cm * sin_scaled) / 1000;
}

// Perform coarse and fine obstacle profiling
LOCAL BOOL scan_obstacle(OBSTACLE_PROFILE *profile)
{
    UW distances[COARSE_COUNT] = {0};
    BOOL valid[COARSE_COUNT] = {0};

    UW right_edge_distance = 0;
    UW left_edge_distance = 0;

    *profile = (OBSTACLE_PROFILE){0};

    // Coarse scan across seven directions
    for (INT i = 0; i < COARSE_COUNT; i++) {
        valid[i] = scan_angle(coarse_angles[i], &distances[i]);

        if (valid[i])
            update_closest(profile, coarse_angles[i],
                           distances[i]);

        // Show each coarse measurement
        tm_printf((UB *)"[sonar] %d deg: %u cm, valid=%d\n",
              coarse_angles[i], distances[i], valid[i]);
    }

    // A nearby forward obstacle must be present
    BOOL center_detected =
        valid[3] && distances[3] <= PROFILE_NEAR_CM;

    if (center_detected) {

        // Find the right obstacle boundary
        for (INT i = 3; i > 0; i--) {
            if (!valid[i] || !valid[i - 1])
                break;

            if (distances[i] > PROFILE_NEAR_CM)
                break;

            if (distances[i - 1] > PROFILE_NEAR_CM) {
                profile->right_edge_valid = refine_edge(
                    coarse_angles[i], distances[i],
                    coarse_angles[i - 1],
                    &profile->right_edge_deg,
                    &right_edge_distance,
                    profile
                );
                break;
            }
        }

        // Find the left obstacle boundary
        for (INT i = 3; i < COARSE_COUNT - 1; i++) {
            if (!valid[i] || !valid[i + 1])
                break;

            if (distances[i] > PROFILE_NEAR_CM)
                break;

            if (distances[i + 1] > PROFILE_NEAR_CM) {
                profile->left_edge_valid = refine_edge(
                    coarse_angles[i], distances[i],
                    coarse_angles[i + 1],
                    &profile->left_edge_deg,
                    &left_edge_distance,
                    profile
                );
                break;
            }
        }
    }

    // Estimate the obstacle's lateral span
    if (profile->left_edge_valid &&
        profile->right_edge_valid) {

        W left_x = lateral_position_cm(
            profile->left_edge_deg, left_edge_distance);

        W right_x = lateral_position_cm(
            profile->right_edge_deg, right_edge_distance);

        if (left_x > right_x) {
            profile->estimated_width_cm = (UW)(left_x - right_x);
            profile->width_valid = TRUE;
        }
    }

    // Restore the forward-facing position
    ER servo_er = pwm_set_cc(
        SERVO_PIN, SERVO_CENTER_US);

    if (servo_er >= E_OK)
        tk_dly_tsk(SERVO_SETTLE_MS);

    // Scan quality, not bypass safety
    profile->profile_valid =
        center_detected &&
        profile->left_edge_valid &&
        profile->right_edge_valid &&
        profile->width_valid &&
        servo_er >= E_OK;

    tm_printf((UB *)"[sonar] Center: %d\n", center_detected);

    tm_printf((UB *)"[sonar] Left edge: %d, angle: %d\n",
            profile->left_edge_valid,
            profile->left_edge_deg);

    tm_printf((UB *)"[sonar] Right edge: %d, angle: %d\n",
            profile->right_edge_valid,
            profile->right_edge_deg);

    tm_printf((UB *)"[sonar] Width: %u cm, valid: %d\n",
            profile->estimated_width_cm,
            profile->width_valid);

    tm_printf((UB *)"[sonar] Servo return: %d\n",
            servo_er >= E_OK);

    // Side clearance needs additional path information
    profile->left_clearance_valid = FALSE;
    profile->right_clearance_valid = FALSE;

    return profile->profile_valid;
}

// Send ultrasonic information to Navigator
LOCAL BOOL send_to_navigator(const NAVIGATOR_MESSAGE *message)
{
    ER er = tk_snd_mbf(
        navigator_mbfid,
        (void *)message,
        sizeof(*message),
        TMO_FEVR
    );

    return er >= E_OK;
}

// Main ultrasonic RTOS task
EXPORT void ultrasonic_task(INT stacd, void *exinf)
{
    UW front_cm = 0;
    UW confirmations = 0;
    UINT pattern;

    NAVIGATOR_MESSAGE message = {0};

    (void)stacd;
    (void)exinf;

    // Initialise ultrasonic GPIO
    if (gpio_set_pin(SONAR_TRIG_PIN, GPIO_MODE_OUT) < E_OK ||
        gpio_set_pin(SONAR_ECHO_PIN, GPIO_MODE_IN) < E_OK) {

        tm_printf((UB *)"[sonar] GPIO error\n");
        return;
    }

    gpio_set_val(SONAR_TRIG_PIN, 0);

    // Initialise servo at 50 Hz
    if (pwm_set_divider(SERVO_PIN, 125, 0) < E_OK ||
        pwm_set_wrap(SERVO_PIN, SERVO_PWM_WRAP) < E_OK ||
        pwm_set_pin(SERVO_PIN) < E_OK ||
        pwm_set_cc(SERVO_PIN, SERVO_CENTER_US) < E_OK ||
        pwm_set_enabled(SERVO_PIN, TRUE) < E_OK) {

        tm_printf((UB *)"[sonar] Servo error\n");
        return;
    }

    tk_dly_tsk(SERVO_SETTLE_MS);

    while (1) {
        BOOL valid = read_distance_cm(&front_cm);

        // Confirm nearby obstacles
        if (valid && front_cm <= OBSTACLE_TRIGGER_CM)
            confirmations++;
        else
            confirmations = 0;

        if (confirmations < OBSTACLE_CONFIRM_COUNT) {
            tk_dly_tsk(SONAR_PING_DELAY_MS);
            continue;
        }

        // Stop before handing control to Navigator
        motor_estop = TRUE;

        message.type = MSG_OBSTACLE_DETECTED;
        message.data.obstacle_distance_cm = front_cm;

        if (!send_to_navigator(&message))
            return;

        // Wait for motor-stop confirmation via Navigator
        if (tk_wai_flg(
                ranger_flgid,
                SONAR_SCAN_REQUEST,
                TWF_ORW | TWF_BITCLR,
                &pattern,
                TMO_FEVR) < E_OK)
            return;

        OBSTACLE_PROFILE profile = {0};

        scan_obstacle(&profile);

        message.type = MSG_OBSTACLE_PROFILED;
        message.data.obstacle_profile = profile;

        if (!send_to_navigator(&message))
            return;

        // Resume monitoring only after successful line recovery
        if (tk_wai_flg(
                ranger_flgid,
                SONAR_RESUME_FLAG,
                TWF_ORW | TWF_BITCLR,
                &pattern,
                TMO_FEVR) < E_OK)
            return;

        confirmations = 0;
        tk_dly_tsk(SONAR_PING_DELAY_MS);
    }
}
