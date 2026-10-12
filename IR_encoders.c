#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define the sensor pins
#define SENSOR_1_PIN 5
#define SENSOR_2_PIN 17
#define SENSOR_3_PIN 26

// Volatile flags to let the main loop know an interrupt fired
volatile bool s1_triggered = false;
volatile bool s2_triggered = false;
volatile bool s3_triggered = false;

// The Interrupt Service Routine (ISR)
// This fires instantly in the background whenever a pin changes state.
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == SENSOR_1_PIN) {
        s1_triggered = true;
    } else if (gpio == SENSOR_2_PIN) {
        s2_triggered = true;
    } else if (gpio == SENSOR_3_PIN) {
        s3_triggered = true;
    }
}

int main() {
    // Initialize standard I/O for UART/USB serial monitor
    stdio_init_all();

    // Give you a moment to open the serial monitor after flashing
    sleep_ms(2000); 
    printf("IR Sensor Interrupt Monitor Started\n");

    // Initialize the GPIO pins
    gpio_init(SENSOR_1_PIN);
    gpio_init(SENSOR_2_PIN);
    gpio_init(SENSOR_3_PIN);

    // Set them as inputs
    gpio_set_dir(SENSOR_1_PIN, GPIO_IN);
    gpio_set_dir(SENSOR_2_PIN, GPIO_IN);
    gpio_set_dir(SENSOR_3_PIN, GPIO_IN);

    // Optional: Enable pull-up or pull-down resistors if your IR sensor floats
    // gpio_pull_down(SENSOR_1_PIN); 
    
    // Attach the interrupt to Pin 5 (Rising and Falling edges)
    gpio_set_irq_enabled_with_callback(SENSOR_1_PIN, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
        true, 
        &gpio_callback);

    // Attach the same interrupt callback to pins 17 and 26
    gpio_set_irq_enabled(SENSOR_2_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(SENSOR_3_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    while (true) {
        // We print in the main loop instead of the ISR.
        // Printing inside a hardware interrupt can crash the Pico if data piles up.
        
        if (s1_triggered) {
            s1_triggered = false;
            printf("Sensor 1 (Pin 5) changed! Current state: %d\n", gpio_get(SENSOR_1_PIN));
        }
        
        if (s2_triggered) {
            s2_triggered = false;
            printf("Sensor 2 (Pin 17) changed! Current state: %d\n", gpio_get(SENSOR_2_PIN));
        }
        
        if (s3_triggered) {
            s3_triggered = false;
            printf("Sensor 3 (Pin 26) changed! Current state: %d\n", gpio_get(SENSOR_3_PIN));
        }

        // Sleep briefly to prevent the while loop from maxing out the CPU
        sleep_ms(10);
    }

    return 0;
}