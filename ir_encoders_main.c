/**
 * @file ir_encoders_main.c
 * @brief Standalone Pico SDK test utility monitoring three IR GPIO interrupts.
 */

#include "ir_encoders_main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define IR_TEST_SENSOR_1_PIN            (5u)
#define IR_TEST_SENSOR_2_PIN            (17u)
#define IR_TEST_SENSOR_3_PIN            (26u)
#define IR_TEST_STARTUP_DELAY_MSEC      (2000u)
#define IR_TEST_POLL_DELAY_MSEC         (10u)

static volatile bool gb_sensor1_trig = false;
static volatile bool gb_sensor2_trig = false;
static volatile bool gb_sensor3_trig = false;

static void ir_encoders_gpio_isr (uint gpio_pin, uint32_t events);

/**
 * @brief Standalone Pico SDK entry point monitoring three IR GPIO interrupts.
 * @return Zero on exit (never returns).
 */
int main (void)
{
    const uint32_t irq_edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    stdio_init_all();
    sleep_ms(IR_TEST_STARTUP_DELAY_MSEC);
    printf("IR Sensor Interrupt Monitor Started\n");

    gpio_init(IR_TEST_SENSOR_1_PIN);
    gpio_init(IR_TEST_SENSOR_2_PIN);
    gpio_init(IR_TEST_SENSOR_3_PIN);

    gpio_set_dir(IR_TEST_SENSOR_1_PIN, GPIO_IN);
    gpio_set_dir(IR_TEST_SENSOR_2_PIN, GPIO_IN);
    gpio_set_dir(IR_TEST_SENSOR_3_PIN, GPIO_IN);

    gpio_set_irq_enabled_with_callback(IR_TEST_SENSOR_1_PIN,
                                       irq_edges,
                                       true,
                                       &ir_encoders_gpio_isr);
    gpio_set_irq_enabled(IR_TEST_SENSOR_2_PIN, irq_edges, true);
    gpio_set_irq_enabled(IR_TEST_SENSOR_3_PIN, irq_edges, true);

    for (;;)
    {
        if (gb_sensor1_trig)
        {
            gb_sensor1_trig = false;
            printf("Sensor 1 (Pin 5) changed! Current state: %d\n",
                   gpio_get(IR_TEST_SENSOR_1_PIN) ? 1 : 0);
        }

        if (gb_sensor2_trig)
        {
            gb_sensor2_trig = false;
            printf("Sensor 2 (Pin 17) changed! Current state: %d\n",
                   gpio_get(IR_TEST_SENSOR_2_PIN) ? 1 : 0);
        }

        if (gb_sensor3_trig)
        {
            gb_sensor3_trig = false;
            printf("Sensor 3 (Pin 26) changed! Current state: %d\n",
                   gpio_get(IR_TEST_SENSOR_3_PIN) ? 1 : 0);
        }

        sleep_ms(IR_TEST_POLL_DELAY_MSEC);
    }

    return 0;
}

/**
 * @brief Pico SDK GPIO interrupt callback latching per-sensor trigger flags.
 *
 * BARR-C Deviation (Rules 5.2.a, 6.5.a):
 *   Approver: Project Team (INF2004).
 *   Rationale: Pico SDK gpio_irq_callback_t requires standard C signature
 *   void (*)(uint, uint32_t) dispatched by SDK IRQ machinery.
 *
 * @param[in] gpio_pin Triggering GPIO pin index.
 * @param[in] events   Edge event bitmask (unused).
 */
static void ir_encoders_gpio_isr (uint gpio_pin, uint32_t events)
{
    (void)events;

    if (IR_TEST_SENSOR_1_PIN == gpio_pin)
    {
        gb_sensor1_trig = true;
    }
    else if (IR_TEST_SENSOR_2_PIN == gpio_pin)
    {
        gb_sensor2_trig = true;
    }
    else if (IR_TEST_SENSOR_3_PIN == gpio_pin)
    {
        gb_sensor3_trig = true;
    }
    else
    {
        /* Ignore interrupts from unconfigured GPIO pins. */
    }
}

/*** end of file ***/
