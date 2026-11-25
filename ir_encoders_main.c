/**
 * @file ir_encoders_main.c
 * @brief Standalone Pico SDK test utility monitoring three IR GPIO interrupts.
 *
 * BARR-C Deviation (Rules 2.1.c.ii, 5.2.a, 6.5.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Interactive hardware bring-up utility prints live sensor transitions
 *   over USB stdio; ISO C main() returns int and Pico SDK gpio_irq_callback_t
 *   requires signature void (*)(uint, uint32_t) dispatched by SDK IRQ glue.
 */

#include "ir_encoders_main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define IR_TEST_SENSOR_1_PIN            (5u)
#define IR_TEST_SENSOR_2_PIN            (17u)
#define IR_TEST_SENSOR_3_PIN            (26u)
#define IR_TEST_STARTUP_DELAY_MSEC      (2000u)
#define IR_TEST_POLL_DELAY_MSEC         (10u)

static volatile bool gb_has_sensor1_edge = false;
static volatile bool gb_has_sensor2_edge = false;
static volatile bool gb_has_sensor3_edge = false;

static void ir_encoders_gpio_isr(uint gpio_pin, uint32_t events);

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
        uint32_t irq_state  = save_and_disable_interrupts();
        bool     b_has_s1   = gb_has_sensor1_edge;
        bool     b_has_s2   = gb_has_sensor2_edge;
        bool     b_has_s3   = gb_has_sensor3_edge;

        gb_has_sensor1_edge = false;
        gb_has_sensor2_edge = false;
        gb_has_sensor3_edge = false;
        restore_interrupts(irq_state);

        if (b_has_s1)
        {
            printf("Sensor 1 (Pin 5) changed! Current state: %u\n",
                   gpio_get(IR_TEST_SENSOR_1_PIN) ? 1u : 0u);
        }

        if (b_has_s2)
        {
            printf("Sensor 2 (Pin 17) changed! Current state: %u\n",
                   gpio_get(IR_TEST_SENSOR_2_PIN) ? 1u : 0u);
        }

        if (b_has_s3)
        {
            printf("Sensor 3 (Pin 26) changed! Current state: %u\n",
                   gpio_get(IR_TEST_SENSOR_3_PIN) ? 1u : 0u);
        }

        sleep_ms(IR_TEST_POLL_DELAY_MSEC);
    }
}

/**
 * @brief Pico SDK GPIO interrupt callback latching per-sensor trigger flags.
 * @param[in] gpio_pin Triggering GPIO pin index.
 * @param[in] events   Edge event bitmask (unused).
 */
static void ir_encoders_gpio_isr (uint gpio_pin, uint32_t events)
{
    /* Range-safe: intentionally unused SDK callback parameter. */
    (void)events;

    if (IR_TEST_SENSOR_1_PIN == gpio_pin)
    {
        gb_has_sensor1_edge = true;
    }
    else if (IR_TEST_SENSOR_2_PIN == gpio_pin)
    {
        gb_has_sensor2_edge = true;
    }
    else if (IR_TEST_SENSOR_3_PIN == gpio_pin)
    {
        gb_has_sensor3_edge = true;
    }
    else
    {
        /* Ignore interrupts from unconfigured GPIO pins. */
    }
}

/*** end of file ***/
