/**
 * @file ultrasonic_servo_main.c
 * @brief Standalone Pico SDK HC-SR04 and pan-servo sweep test utility.
 *
 * Reference: HC-SR04 Ultrasonic Ranging Module Datasheet (speed of sound
 * 0.0343 cm per usec round trip divided by 2).
 */

#include "ultrasonic_servo_main.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"

#define US_SERVO_PIN                    (15u)
#define US_TRIG_PIN                     (1u)
#define US_ECHO_PIN                     (0u)
#define US_POS_LEFT_USEC                (2045u)
#define US_POS_CENTER_USEC              (1695u)
#define US_POS_RIGHT_USEC               (1345u)
#define US_SERVO_CLKDIV_F32             (125.0f)
#define US_SERVO_WRAP_TICKS             (20000u)
#define US_SERVO_SETTLE_MSEC            (300u)
#define US_TRIG_PULSE_USEC              (10u)
#define US_ECHO_TIMEOUT_USEC            (26000u)
#define US_STARTUP_DELAY_MSEC           (2000u)
#define US_SWEEP_DELAY_MSEC             (1000u)
#define US_CM_PER_USEC_F32              (0.0343f)
#define US_ROUND_TRIP_DIV_F32           (2.0f)
#define US_INVALID_DIST_CM_F32          (-1.0f)

typedef float float32_t;

static void us_setup_servo (void);
static void us_set_servo_pos (uint32_t pulse_width_usec);
static float32_t us_get_distance_cm (void);

/**
 * @brief Standalone Pico SDK entry point sweeping servo and printing range.
 * @return Zero on exit (never returns).
 */
int main (void)
{
    stdio_init_all();
    us_setup_servo();

    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);

    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);

    sleep_ms(US_STARTUP_DELAY_MSEC);

    for (;;)
    {
        float32_t dist_left   = 0.0f;
        float32_t dist_center = 0.0f;
        float32_t dist_right  = 0.0f;

        us_set_servo_pos(US_POS_LEFT_USEC);
        dist_left = us_get_distance_cm();

        us_set_servo_pos(US_POS_CENTER_USEC);
        dist_center = us_get_distance_cm();

        us_set_servo_pos(US_POS_RIGHT_USEC);
        dist_right = us_get_distance_cm();

        us_set_servo_pos(US_POS_CENTER_USEC);

        printf("--- Radar Scan Complete ---\n");
        /* C99 default argument promotion widens float32_t for printf %.1f. */
        printf("Left   (120 deg) : %.1f cm\n", dist_left);
        printf("Center ( 90 deg) : %.1f cm\n", dist_center);
        printf("Right  ( 60 deg) : %.1f cm\n\n", dist_right);

        sleep_ms(US_SWEEP_DELAY_MSEC);
    }

    return 0;
}

/**
 * @brief Configure 50 Hz hardware PWM (1 usec tick) on the servo pin.
 */
static void us_setup_servo (void)
{
    uint32_t   slice_num = 0u;
    pwm_config cfg       = pwm_get_default_config();

    gpio_set_function(US_SERVO_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(US_SERVO_PIN);
    pwm_config_set_clkdiv(&cfg, US_SERVO_CLKDIV_F32);
    pwm_init(slice_num, &cfg, true);
    pwm_set_wrap(slice_num, US_SERVO_WRAP_TICKS);
}

/**
 * @brief Command servo pulse width in microseconds and wait for settle.
 * @param[in] pulse_width_usec Target pulse duration in microseconds.
 */
static void us_set_servo_pos (uint32_t pulse_width_usec)
{
    /* Range-safe: servo pulse widths are in [1345, 2045] usec (uint16_t). */
    pwm_set_gpio_level(US_SERVO_PIN, (uint16_t)pulse_width_usec);
    sleep_ms(US_SERVO_SETTLE_MSEC);
}

/**
 * @brief Trigger HC-SR04 and return measured distance in cm or -1.0f.
 * @return Distance in centimetres, or -1.0f on echo timeout or non-finite.
 */
static float32_t us_get_distance_cm (void)
{
    float32_t dist_cm    = US_INVALID_DIST_CM_F32;
    uint32_t  start_ref  = 0u;
    uint32_t  echo_start = 0u;
    uint32_t  echo_end   = 0u;
    bool      b_timedout = false;

    gpio_put(US_TRIG_PIN, 1);
    sleep_us(US_TRIG_PULSE_USEC);
    gpio_put(US_TRIG_PIN, 0);

    start_ref = time_us_32();

    while ((!gpio_get(US_ECHO_PIN)) && (!b_timedout))
    {
        if ((time_us_32() - start_ref) > US_ECHO_TIMEOUT_USEC)
        {
            b_timedout = true;
        }
    }

    if (!b_timedout)
    {
        echo_start = time_us_32();

        while (gpio_get(US_ECHO_PIN) && (!b_timedout))
        {
            if ((time_us_32() - start_ref) > US_ECHO_TIMEOUT_USEC)
            {
                b_timedout = true;
            }
        }
    }

    if (!b_timedout)
    {
        uint32_t  dur_usec = 0u;
        float32_t calc_cm  = 0.0f;

        echo_end = time_us_32();
        dur_usec = echo_end - echo_start;
        calc_cm  = (((float32_t)dur_usec) * US_CM_PER_USEC_F32) /
                   US_ROUND_TRIP_DIV_F32;

        if (isfinite(calc_cm))
        {
            dist_cm = calc_cm;
        }
    }

    return dist_cm;
}

/*** end of file ***/
