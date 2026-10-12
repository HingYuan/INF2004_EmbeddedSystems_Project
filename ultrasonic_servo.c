#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#define SERVO_PIN 15
#define TRIG_PIN 1
#define ECHO_PIN 0

// 60-degree sweep parameters
#define POS_LEFT   2045 // ~120 degrees
#define POS_CENTER 1695 // 90 degrees (straight ahead)
#define POS_RIGHT  1345 // ~60 degrees

// Initialize 50Hz PWM for the servo
void setup_servo() {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f); // 1 tick = 1 microsecond
    pwm_init(slice_num, &config, true);
    
    pwm_set_wrap(slice_num, 20000); // 20ms period (50Hz)
}

// Move servo and wait for mechanics to catch up
void set_servo_position(uint pulse_width_us) {
    pwm_set_gpio_level(SERVO_PIN, pulse_width_us);
    sleep_ms(300); // CRITICAL: Wait 300ms for physical movement to finish
}

// Trigger HC-SR04 and calculate distance in cm
float get_distance_cm() {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint32_t timeout = time_us_32() + 26000; // 26ms timeout (~400cm max range)
    
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_32() > timeout) return -1.0; 
    }
    uint32_t start_time = time_us_32();

    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_32() > timeout) return -1.0; 
    }
    uint32_t end_time = time_us_32();

    uint32_t duration = end_time - start_time;
    return (duration * 0.0343) / 2.0;
}

int main() {
    stdio_init_all();
    setup_servo();

    // Initialize Ultrasonic pins
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    sleep_ms(2000); // Let power stabilize before starting loop

    float dist_left, dist_center, dist_right;

    while (true) {
        // 1. Look Left
        set_servo_position(POS_LEFT);
        dist_left = get_distance_cm();

        // 2. Look Center
        set_servo_position(POS_CENTER);
        dist_center = get_distance_cm();

        // 3. Look Right
        set_servo_position(POS_RIGHT);
        dist_right = get_distance_cm();

        // 4. Return to Center (optional, but good for resetting)
        set_servo_position(POS_CENTER);

        // Print the scan results in a clean format
        printf("--- Radar Scan Complete ---\n");
        printf("Left   (120 deg) : %.1f cm\n", dist_left);
        printf("Center ( 90 deg) : %.1f cm\n", dist_center);
        printf("Right  ( 60 deg) : %.1f cm\n\n", dist_right);
        
        sleep_ms(1000); // Wait a second before the next sweep
    }
    
    return 0;
}