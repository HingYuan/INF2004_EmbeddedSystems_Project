/**
 * @file sonar_servo_main.c
 * @brief Standalone Pico SDK HC-SR04 and pan-servo sweep test utility.
 *
 * Reference: HC-SR04 Ultrasonic Ranging Module Datasheet (speed of sound
 * 0.0343 cm per usec round trip divided by 2 -> 343 / 2000 tenths of a cm).
 *
 * BARR-C Deviation (Rules 2.1.c.ii, 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Interactive hardware bring-up utility prints live range scans
 *   over USB stdio; ISO C main() returns int.
 */

#include "sonar_servo_main.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

#define SONAR_TEST_SERVO_PIN            (15u)
#define SONAR_TEST_TRIG_PIN             (1u)
#define SONAR_TEST_ECHO_PIN             (0u)
#define SONAR_TEST_POS_LEFT_USEC        (2045u)
#define SONAR_TEST_POS_CENTER_USEC      (1695u)
#define SONAR_TEST_POS_RIGHT_USEC       (1345u)
#define SONAR_TEST_SERVO_CLKDIV         (125u)
#define SONAR_TEST_SERVO_WRAP_TICKS     (20000u)
#define SONAR_TEST_SERVO_SETTLE_MSEC    (300u)
#define SONAR_TEST_TRIG_PULSE_USEC      (10u)
#define SONAR_TEST_ECHO_TIMEOUT_USEC    (26000u)
#define SONAR_TEST_STARTUP_DELAY_MSEC   (2000u)
#define SONAR_TEST_SWEEP_DELAY_MSEC     (1000u)
#define SONAR_TEST_SOUND_NUM            (343u)
#define SONAR_TEST_SOUND_DEN            (2000u)
#define SONAR_TEST_SOUND_HALF_DEN       (1000u)
#define SONAR_TEST_TENTHS_PER_CM        (10)
#define SONAR_TEST_INVALID_TENTHS_CM    (-10)

static void sonar_servo_setup(void);
static void sonar_servo_set_pos(uint32_t pulse_width_usec);
static int32_t sonar_servo_read_tenths_cm(void);
static void sonar_servo_print_line(const char * p_label, int32_t tenths_cm);

/**
 * @brief Standalone Pico SDK entry point sweeping servo and printing range.
 * @return Zero on exit (never returns).
 */
int main (void)
{
    stdio_init_all();
    sonar_servo_setup();

    gpio_init(SONAR_TEST_TRIG_PIN);
    gpio_set_dir(SONAR_TEST_TRIG_PIN, GPIO_OUT);
    gpio_put(SONAR_TEST_TRIG_PIN, false);

    gpio_init(SONAR_TEST_ECHO_PIN);
    gpio_set_dir(SONAR_TEST_ECHO_PIN, GPIO_IN);

    sleep_ms(SONAR_TEST_STARTUP_DELAY_MSEC);

    for (;;)
    {
        int32_t left_tenths   = 0;
        int32_t center_tenths = 0;
        int32_t right_tenths  = 0;

        sonar_servo_set_pos(SONAR_TEST_POS_LEFT_USEC);
        left_tenths = sonar_servo_read_tenths_cm();

        sonar_servo_set_pos(SONAR_TEST_POS_CENTER_USEC);
        center_tenths = sonar_servo_read_tenths_cm();

        sonar_servo_set_pos(SONAR_TEST_POS_RIGHT_USEC);
        right_tenths = sonar_servo_read_tenths_cm();

        sonar_servo_set_pos(SONAR_TEST_POS_CENTER_USEC);

        printf("--- Radar Scan Complete ---\n");
        sonar_servo_print_line("Left   (120 deg)", left_tenths);
        sonar_servo_print_line("Center ( 90 deg)", center_tenths);
        sonar_servo_print_line("Right  ( 60 deg)", right_tenths);
        printf("\n");

        sleep_ms(SONAR_TEST_SWEEP_DELAY_MSEC);
    }
}

/**
 * @brief Configure 50 Hz hardware PWM (1 usec tick) on the servo pin.
 */
static void sonar_servo_setup (void)
{
    pwm_config cfg       = pwm_get_default_config();
    uint32_t   slice_num = 0u;

    gpio_set_function(SONAR_TEST_SERVO_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SONAR_TEST_SERVO_PIN);
    pwm_config_set_clkdiv_int(&cfg, SONAR_TEST_SERVO_CLKDIV);
    pwm_init(slice_num, &cfg, true);
    pwm_set_wrap(slice_num, SONAR_TEST_SERVO_WRAP_TICKS);
}

/**
 * @brief Command servo pulse width in microseconds and wait for settle.
 * @param[in] pulse_width_usec Target pulse duration in microseconds.
 */
static void sonar_servo_set_pos (uint32_t pulse_width_usec)
{
    /* Range-safe: servo pulse widths are in [1345, 2045] usec (uint16_t). */
    pwm_set_gpio_level(SONAR_TEST_SERVO_PIN, (uint16_t)pulse_width_usec);
    sleep_ms(SONAR_TEST_SERVO_SETTLE_MSEC);
}

/**
 * @brief Trigger HC-SR04 and return measured distance in tenths of cm or -10.
 * @return Distance in tenths of a centimetre, or -10 on echo timeout.
 */
static int32_t sonar_servo_read_tenths_cm (void)
{
    int32_t  dist_tenths     = SONAR_TEST_INVALID_TENTHS_CM;
    uint32_t start_ref       = 0u;
    uint32_t echo_start      = 0u;
    bool     b_has_timed_out = false;

    gpio_put(SONAR_TEST_TRIG_PIN, true);
    sleep_us(SONAR_TEST_TRIG_PULSE_USEC);
    gpio_put(SONAR_TEST_TRIG_PIN, false);

    start_ref = time_us_32();

    while ((!gpio_get(SONAR_TEST_ECHO_PIN)) && (!b_has_timed_out))
    {
        if ((time_us_32() - start_ref) > SONAR_TEST_ECHO_TIMEOUT_USEC)
        {
            b_has_timed_out = true;
        }
    }

    if (!b_has_timed_out)
    {
        echo_start = time_us_32();

        while ((gpio_get(SONAR_TEST_ECHO_PIN)) && (!b_has_timed_out))
        {
            if ((time_us_32() - start_ref) > SONAR_TEST_ECHO_TIMEOUT_USEC)
            {
                b_has_timed_out = true;
            }
        }
    }

    if (!b_has_timed_out)
    {
        uint32_t dur_usec    = time_us_32() - echo_start;
        uint32_t calc_tenths =
            ((dur_usec * SONAR_TEST_SOUND_NUM) + SONAR_TEST_SOUND_HALF_DEN) /
            SONAR_TEST_SOUND_DEN;

        /* Range-safe: dur_usec <= 26000 yields calc_tenths <= 4459. */
        dist_tenths = (int32_t)calc_tenths;
    }

    return dist_tenths;
}

/**
 * @brief Print one formatted fixed-point tenths-of-cm scan line.
 * @param[in] p_label   Angle label prefix string.
 * @param[in] tenths_cm Distance in tenths of a cm, or negative on timeout.
 */
static void sonar_servo_print_line (const char * p_label, int32_t tenths_cm)
{
    if (tenths_cm < 0)
    {
        printf("%s : -1.0 cm\n", p_label);
    }
    else
    {
        int32_t whole_cm = tenths_cm / SONAR_TEST_TENTHS_PER_CM;
        int32_t frac_cm  = tenths_cm % SONAR_TEST_TENTHS_PER_CM;

        printf("%s : %" PRId32 ".%" PRId32 " cm\n",
               p_label,
               whole_cm,
               frac_cm);
    }
}

/*** end of file ***/
