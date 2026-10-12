#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Motor Pin Definitions (from motor_pwm.c)
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

// Button Pin Definitions
#define BTN_1 20
#define BTN_2 21

// Encoder Pin Definitions (Choose two free GPIO pins)
#define ENC_A 2
#define ENC_B 3

// PWM Speeds
#define SPEED_MAX 65535
#define SPEED_HALF 32767
#define SPEED_SLOW 16383

// Volatile variable because it is modified inside an interrupt
volatile int encoder_ticks = 0;

// Interrupt Service Routine (ISR) for the encoder
void encoder_isr(uint gpio, uint32_t events) {
    // When Phase A triggers, check Phase B to determine direction
    if (gpio == ENC_A) {
        if (gpio_get(ENC_B)) {
            encoder_ticks++; // Forward
        } else {
            encoder_ticks--; // Backward
        }
    }
}

// Helper function to initialize a pin for PWM
void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);       
    uint slice_num = pwm_gpio_to_slice_num(pin); 
    pwm_set_wrap(slice_num, 65535);              
    pwm_set_enabled(slice_num, true);            
}

int main() {
    // Initializes UART so printf() goes to your serial monitor
    stdio_init_all();

    // Initialize Buttons
    gpio_init(BTN_1);
    gpio_set_dir(BTN_1, GPIO_IN);
    gpio_pull_up(BTN_1);

    gpio_init(BTN_2);
    gpio_set_dir(BTN_2, GPIO_IN);
    gpio_pull_up(BTN_2);

    // Initialize Encoder Pins
    gpio_init(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN);
    gpio_pull_up(ENC_A);
    
    gpio_init(ENC_B);
    gpio_set_dir(ENC_B, GPIO_IN);
    gpio_pull_up(ENC_B);

    // Attach hardware interrupt to Encoder Phase A on rising edge
    gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_RISE, true, &encoder_isr);

    // Initialize Motor Pins as PWM
    uint motor_pins[] = {M1A, M1B, M2A, M2B};
    for (int i = 0; i < 4; i++) {
        setup_pwm_pin(motor_pins[i]);
        pwm_set_gpio_level(motor_pins[i], 0); 
    }

    // Main Control Loop
    while (true) {
        bool btn1_pressed = !gpio_get(BTN_1); 
        bool btn2_pressed = !gpio_get(BTN_2);

        if (btn1_pressed) {
            // Drive forward at 50% speed
            pwm_set_gpio_level(M1A, SPEED_HALF); 
            pwm_set_gpio_level(M1B, 0); 
            pwm_set_gpio_level(M2A, 0);
            pwm_set_gpio_level(M2B, SPEED_HALF); 
        } 
        else if (btn2_pressed) {
            // Drive backward at 25% speed
            pwm_set_gpio_level(M1A, 0);
            pwm_set_gpio_level(M1B, SPEED_SLOW); 
            pwm_set_gpio_level(M2A, SPEED_SLOW);
            pwm_set_gpio_level(M2B, 0); 
        } 
        else {
            // Stop (Coast) - 0% speed
            pwm_set_gpio_level(M1A, 0);
            pwm_set_gpio_level(M1B, 0); 
            pwm_set_gpio_level(M2A, 0);
            pwm_set_gpio_level(M2B, 0); 
        }

        // Print the real-time encoder count to the Serial Monitor
        printf("Encoder Ticks: %d\n", encoder_ticks);
        
        sleep_ms(100); // 100ms delay to make terminal readable
    }

    return 0;
}