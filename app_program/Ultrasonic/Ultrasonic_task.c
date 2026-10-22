#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Ultrasonic_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"
#include "../Motor/Motor_task.h"

#define SERVO_PWM_WRAP          19999U
#define SERVO_LEFT_US            2045U
#define SERVO_CENTER_US          1695U
#define SERVO_RIGHT_US           1345U
#define SERVO_SETTLE_MS           300U

#define SONAR_TIMEOUT_US        26000U
#define SONAR_PING_DELAY_MS        60U
#define OBSTACLE_TRIGGER_CM        25U
#define OBSTACLE_CONFIRM_COUNT      2U

LOCAL BOOL read_distance_cm(UW *distance_cm)
{
    UW start;
    UW pulse_us;
    UW measured_cm;

    if (gpio_get_val(SONAR_ECHO_PIN) != 0)
        return FALSE;

    // Send a 10 us trigger pulse
    gpio_set_val(SONAR_TRIG_PIN, 0);
    delay_us(2);
    gpio_set_val(SONAR_TRIG_PIN, 1);
    delay_us(10);
    gpio_set_val(SONAR_TRIG_PIN, 0);

    // Wait for echo HIGH, with timeout
    start = time_us();
    while (gpio_get_val(SONAR_ECHO_PIN) == 0) {
        if ((UW)(time_us() - start) >= SONAR_TIMEOUT_US)
            return FALSE;
    }

    // Measure the echo HIGH duration
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

LOCAL BOOL scan_at(UW servo_pulse_us, UW *distance_cm)
{
    if (pwm_set_cc(SERVO_PIN, servo_pulse_us) < E_OK)
        return FALSE;

    tk_dly_tsk(SERVO_SETTLE_MS);

    return read_distance_cm(distance_cm);
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

    // Initialise ultrasonic GPIO
    if (gpio_set_pin(SONAR_TRIG_PIN, GPIO_MODE_OUT) < E_OK ||
        gpio_set_pin(SONAR_ECHO_PIN, GPIO_MODE_IN) < E_OK) {
        tm_printf((UB *)"[sonar] GPIO setup failed\n");
        tk_ext_tsk();
    }

    gpio_set_val(SONAR_TRIG_PIN, 0);

    // Set up 50 Hz servo PWM and face forward
    if (pwm_set_divider(SERVO_PIN, 125, 0) < E_OK ||
        pwm_set_wrap(SERVO_PIN, SERVO_PWM_WRAP) < E_OK ||
        pwm_set_pin(SERVO_PIN) < E_OK ||
        pwm_set_cc(SERVO_PIN, SERVO_CENTER_US) < E_OK ||
        pwm_set_enabled(SERVO_PIN, TRUE) < E_OK) {
        tm_printf((UB *)"[sonar] Servo setup failed\n");
        tk_ext_tsk();
    }

    tk_dly_tsk(SERVO_SETTLE_MS);
    tm_printf((UB *)"[sonar] Forward monitoring started\n");

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

        // Stop before scanning
        motor_estop = TRUE;
        tm_printf((UB *)"[sonar] Obstacle at %u cm\n", front_cm);

        message.type = MSG_OBSTACLE_DETECTED;
        message.data.obstacle_distance_cm = front_cm;

        if (!send_to_navigator(&message))
            tk_ext_tsk();

        OBSTACLE_PROFILE profile = {0};

        profile.left_valid =
            scan_at(SERVO_LEFT_US, &profile.left_cm);

        profile.center_valid =
            scan_at(SERVO_CENTER_US, &profile.center_cm);

        profile.right_valid =
            scan_at(SERVO_RIGHT_US, &profile.right_cm);

        if (pwm_set_cc(SERVO_PIN, SERVO_CENTER_US) < E_OK)
            tk_ext_tsk();

        message.type = MSG_OBSTACLE_PROFILED;
        message.data.obstacle_profile = profile;

        if (!send_to_navigator(&message))
            tk_ext_tsk();

        tm_printf((UB *)"[sonar] L=%u(%d) C=%u(%d) R=%u(%d)\n",
                  profile.left_cm, profile.left_valid,
                  profile.center_cm, profile.center_valid,
                  profile.right_cm, profile.right_valid);

        // Wait for avoidance and line recovery to finish
        if (tk_wai_flg(ranger_flgid, SONAR_RESUME_FLAG,
                       TWF_ORW | TWF_BITCLR,
                       &pattern, TMO_FEVR) < E_OK)
            tk_ext_tsk();

        confirmations = 0;
        tk_dly_tsk(SONAR_PING_DELAY_MS);
    }
}
