/**
 * @file motors_main.c
 * @brief Standalone Pico SDK motor PWM and encoder test sketch.
 */

#include "motors_main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define MOTORS_PIN_M1A                  (8u)
#define MOTORS_PIN_M1B                  (9u)
#define MOTORS_PIN_M2A                  (10u)
#define MOTORS_PIN_M2B                  (11u)
#define MOTORS_PIN_BTN_1                (20u)
#define MOTORS_PIN_BTN_2                (21u)
#define MOTORS_PIN_ENC_A                (2u)
#define MOTORS_PIN_ENC_B                (3u)
#define MOTORS_PWM_COUNT                (4u)
#define MOTORS_SPEED_MAX                (65535u)
#define MOTORS_SPEED_HALF               (32767u)
#define MOTORS_SPEED_SLOW               (16383u)
#define MOTORS_LOOP_DELAY_MSEC          (100u)

static volatile int32_t g_encoder_ticks = 0;

static void motors_setup_pwm_pin (uint32_t pin_num);
static void motors_encoder_isr (uint gpio_pin, uint32_t events);

/**
 * @brief Standalone Pico SDK entry point testing button-driven motors.
 * @return Zero on exit (never returns).
 */
int main (void)
{
    static const uint32_t motor_pins[MOTORS_PWM_COUNT] =
    {
        MOTORS_PIN_M1A,
        MOTORS_PIN_M1B,
        MOTORS_PIN_M2A,
        MOTORS_PIN_M2B
    };

    stdio_init_all();

    gpio_init(MOTORS_PIN_BTN_1);
    gpio_set_dir(MOTORS_PIN_BTN_1, GPIO_IN);
    gpio_pull_up(MOTORS_PIN_BTN_1);

    gpio_init(MOTORS_PIN_BTN_2);
    gpio_set_dir(MOTORS_PIN_BTN_2, GPIO_IN);
    gpio_pull_up(MOTORS_PIN_BTN_2);

    gpio_init(MOTORS_PIN_ENC_A);
    gpio_set_dir(MOTORS_PIN_ENC_A, GPIO_IN);
    gpio_pull_up(MOTORS_PIN_ENC_A);

    gpio_init(MOTORS_PIN_ENC_B);
    gpio_set_dir(MOTORS_PIN_ENC_B, GPIO_IN);
    gpio_pull_up(MOTORS_PIN_ENC_B);

    gpio_set_irq_enabled_with_callback(MOTORS_PIN_ENC_A,
                                       GPIO_IRQ_EDGE_RISE,
                                       true,
                                       &motors_encoder_isr);

    for (uint32_t pin_idx = 0u; pin_idx < MOTORS_PWM_COUNT; pin_idx++)
    {
        motors_setup_pwm_pin(motor_pins[pin_idx]);
        pwm_set_gpio_level(motor_pins[pin_idx], 0u);
    }

    for (;;)
    {
        bool b_btn1_pressed = (!gpio_get(MOTORS_PIN_BTN_1));
        bool b_btn2_pressed = (!gpio_get(MOTORS_PIN_BTN_2));

        if (b_btn1_pressed)
        {
            pwm_set_gpio_level(MOTORS_PIN_M1A, MOTORS_SPEED_HALF);
            pwm_set_gpio_level(MOTORS_PIN_M1B, 0u);
            pwm_set_gpio_level(MOTORS_PIN_M2A, 0u);
            pwm_set_gpio_level(MOTORS_PIN_M2B, MOTORS_SPEED_HALF);
        }
        else if (b_btn2_pressed)
        {
            pwm_set_gpio_level(MOTORS_PIN_M1A, 0u);
            pwm_set_gpio_level(MOTORS_PIN_M1B, MOTORS_SPEED_SLOW);
            pwm_set_gpio_level(MOTORS_PIN_M2A, MOTORS_SPEED_SLOW);
            pwm_set_gpio_level(MOTORS_PIN_M2B, 0u);
        }
        else
        {
            pwm_set_gpio_level(MOTORS_PIN_M1A, 0u);
            pwm_set_gpio_level(MOTORS_PIN_M1B, 0u);
            pwm_set_gpio_level(MOTORS_PIN_M2A, 0u);
            pwm_set_gpio_level(MOTORS_PIN_M2B, 0u);
        }

        /* Range-safe: g_encoder_ticks fits in standard 32-bit int on RP2040. */
        printf("Encoder Ticks: %d\n", (int)g_encoder_ticks);
        sleep_ms(MOTORS_LOOP_DELAY_MSEC);
    }

    return 0;
}

/**
 * @brief Configure one RP2040 GPIO pin as a 16-bit hardware PWM output.
 * @param[in] pin_num GPIO pin index.
 */
static void motors_setup_pwm_pin (uint32_t pin_num)
{
    uint32_t slice_num = 0u;

    gpio_set_function(pin_num, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(pin_num);
    pwm_set_wrap(slice_num, MOTORS_SPEED_MAX);
    pwm_set_enabled(slice_num, true);
}

/**
 * @brief Pico SDK GPIO interrupt callback counting directional encoder ticks.
 *
 * BARR-C Deviation (Rules 5.2.a, 6.5.a):
 *   Approver: Project Team (INF2004).
 *   Rationale: Pico SDK gpio_irq_callback_t requires standard C signature
 *   void (*)(uint, uint32_t) dispatched by SDK IRQ machinery.
 *
 * @param[in] gpio_pin Triggering GPIO pin index.
 * @param[in] events   Edge event bitmask (unused).
 */
static void motors_encoder_isr (uint gpio_pin, uint32_t events)
{
    (void)events;

    if (MOTORS_PIN_ENC_A == gpio_pin)
    {
        if (gpio_get(MOTORS_PIN_ENC_B))
        {
            g_encoder_ticks++;
        }
        else
        {
            g_encoder_ticks--;
        }
    }
}

/*** end of file ***/
