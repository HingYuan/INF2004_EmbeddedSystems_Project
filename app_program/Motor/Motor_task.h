#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <tk/typedef.h>
#include <tk/tkernel.h>
#include "../robot_tasks.h"

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

// 32-bit command: preserve independent 0..255 tick targets for both wheels.
typedef union {
    UW raw;
    struct {
        B target_speed_left;         // -100..100 needs 8 signed bits.
        B target_speed_right;
        UB turn_ticks_left;
        UB turn_ticks_right;
    } __attribute__((packed));
} MOTOR_COMMAND;

// 3-Bit State Packing (2^3 = 8 states)
typedef struct {
    UW state : 3;
} MOTOR_STATE_PACKED;

typedef enum { MS_IDLE, MS_ACCEL, MS_CRUISE, MS_TURN, MS_BRAKE, MS_FAULT, MS_ESTOP } MOTOR_STATE;
typedef struct __attribute__((packed)) {
    UW state : 3;
    UW move_done_pending : 1;
    UW right_backup_reported : 1;
    UW move_stable_periods : 2;
    UW stop_stable_periods : 2;
    UW stall_left : 7;
    UW stall_right : 7;
    UW print_timer : 5; // 0..25.
} MOTOR_RUNTIME_STATE;
typedef struct __attribute__((packed)) {
    UH filters : 8;    // Four channels, each with stable level and one pending sample.
    UH using_b : 1;
} ENCODER_ISR_STATE;

// Motor bits retain their numbers within the shared application fault word.
#define FAULT_BIT_STALL     (1U << 0)  // Bit 0: Motor Stall
#define FAULT_BIT_ENCODER   (1U << 1)  // Bit 1: Encoder Failure
#define FAULT_BIT_ESTOP     (1U << 2)  // Bit 2: Emergency Stop

#define motor_fault_word (robot_flags_get() & (FAULT_BIT_STALL | FAULT_BIT_ENCODER | FAULT_BIT_ESTOP))
#define motor_estop ((robot_flags_get() & ROBOT_ESTOP_REQUEST) != 0U)
#define motor_stop_ack_requested ((robot_flags_get() & ROBOT_STOP_ACK_REQUEST) != 0U)
IMPORT volatile UW buffer_drop_count;
#define encoder_degraded ((robot_flags_get() & ROBOT_ENCODER_DEGRADED) != 0U)

// Encoder tick counts, updated by motor_task
IMPORT volatile W enc_l_ticks;
IMPORT volatile W enc_r_ticks;

// Calibrated Odometry Constants (749 ticks = 247.5 mm on ruler -> 0.33 mm/tick)
#define WHEEL_DIAMETER_MM    60
#define ENCODER_SLOTS        20
#define MM_PER_TICK_X100     33    // 0.33 mm per tick calibrated from 24.75 cm ruler test!
#define MM_PER_TICK_X10      3     // Compatibility fallback
#define WHEELBASE_MM         100   // 100mm track width between wheels

// Odometry & Speed Estimation APIs (Buddy 2 Write-up Requirements)
IMPORT INT motor_get_left_dist_mm(void);
IMPORT INT motor_get_right_dist_mm(void);
IMPORT INT motor_get_avg_dist_mm(void);
IMPORT INT motor_get_avg_dist_cm(void);
IMPORT INT motor_get_speed_mm_s(void);

// Reusable Motion-Control APIs (Buddy 2 Write-up Requirements)
IMPORT ER moveForward(INT distance_mm);
IMPORT ER moveBackward(INT distance_mm);
IMPORT ER turnLeft(INT angle_deg);
IMPORT ER turnRight(INT angle_deg);
IMPORT ER stop(void);

IMPORT void motor_task(INT stacd, void *exinf);

#endif
