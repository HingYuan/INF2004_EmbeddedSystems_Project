#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>
#include <tk/tkernel.h>

// DC Motor Pins (From Hardware Integration Table)
#define MOTOR_LEFT_A        8
#define MOTOR_LEFT_B        9
#define MOTOR_RIGHT_A      10
#define MOTOR_RIGHT_B      11

// Optical Wheel Encoder Pins
#define MOTOR_LEFT_ENC_A   16
#define MOTOR_LEFT_ENC_B   17
#define MOTOR_RIGHT_ENC_A   2
#define MOTOR_RIGHT_ENC_B   3

// Defensive Clamping Macro 
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

// This is the packet the Navigator sends to you
// Speed is a percentage: -100..100 (Positive = forward, Negative = backward)
// Tribunal 1: 32-bit (4-byte) bit-packed motor command
typedef union {
    uint32_t raw;
    struct {
        int8_t   target_speed_left;   // Bits 0..7  (-100 to +100%)
        int8_t   target_speed_right;  // Bits 8..15 (-100 to +100%)
        uint16_t turn_ticks : 10;     // Bits 16..25 (0 to 1023 ticks)
        uint16_t turn_dir   : 1;      // Bit 26 (0=Left, 1=Right)
        uint16_t reserved   : 5;      // Bits 27..31
    } __attribute__((packed));
    struct {
        int8_t   _spd_l;
        int8_t   _spd_r;
        uint8_t  turn_ticks_left;     // Preserves compatibility with existing code
        uint8_t  turn_ticks_right;
    } __attribute__((packed));
} MOTOR_COMMAND;

// Tribunal 1: 3-Bit State Packing (2^3 = 8 states)
typedef struct {
    uint8_t state : 3;      // 7 motor states fit into 3 bits (0..7)
    uint8_t reserved : 5;
} MOTOR_STATE_PACKED;

// Tribunal 1 Part C: Unified 16-Bit Fault Word Bitmask
#define FAULT_BIT_STALL     (1U << 0)  // Bit 0: Motor Stall
#define FAULT_BIT_ENCODER   (1U << 1)  // Bit 1: Encoder Failure
#define FAULT_BIT_ESTOP     (1U << 2)  // Bit 2: Emergency Stop

IMPORT volatile UH motor_fault_word;   // 16-bit packed fault register

// Set TRUE from any task to emergency stop, FALSE to release
IMPORT volatile BOOL motor_estop;
// Navigator requests confirmation when the motors have stopped
IMPORT volatile BOOL motor_stop_ack_requested;
// Diagnostics and fault flags (Manual Section 7)
IMPORT volatile UW buffer_drop_count;
IMPORT volatile BOOL encoder_degraded;

// Encoder tick counts, updated by motor_task
IMPORT volatile W enc_l_ticks;
IMPORT volatile W enc_r_ticks;

// Odometry constants
// Wheel diameter = 60mm, 20 slots/rev -> ~9.4 mm per tick
#define WHEEL_DIAMETER_MM    60
#define ENCODER_SLOTS        20
#define MM_PER_TICK_X10      85    // 8.5 mm per tick calibrated (5cm ruler test)
#define WHEELBASE_MM         100   // 100mm track width between wheels

// Helper functions to get physical distance and speed
IMPORT INT motor_get_left_dist_mm(void);
IMPORT INT motor_get_right_dist_mm(void);
IMPORT INT motor_get_avg_dist_mm(void);
IMPORT INT motor_get_avg_dist_cm(void);

// Make your task function visible to the main program
IMPORT void motor_task(INT stacd, void *exinf);

#endif