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
#define MOTOR_LEFT_ENC_A   26
#define MOTOR_LEFT_ENC_B   27
#define MOTOR_RIGHT_ENC_A   2
#define MOTOR_RIGHT_ENC_B   3

// This is the packet the Navigator sends to you
// Speed is a percentage: -100..100 (Positive = forward, Negative = backward)
typedef struct {
    int16_t target_speed_left;
    int16_t target_speed_right;
    int16_t turn_ticks;
} MotionCommand;

// Mailbox created in app_main.c: Navigator -> Motor Controller
IMPORT ID motion_mbf_id;

// Encoder tick counts, updated by motor_task
IMPORT volatile W enc_l_ticks;
IMPORT volatile W enc_r_ticks;

// Make your task function visible to the main program
IMPORT void motor_task(INT stacd, void *exinf);

#endif