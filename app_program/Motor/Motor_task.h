#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <tk/tkernel.h>

#define MOTOR_LEFT_A        8
#define MOTOR_LEFT_B        9
#define MOTOR_RIGHT_A      10
#define MOTOR_RIGHT_B      11
#define MOTOR_LEFT_ENC_A    2
#define MOTOR_LEFT_ENC_B    3
#define MOTOR_RIGHT_ENC_A  26
#define MOTOR_RIGHT_ENC_B  27

IMPORT void motor_task(INT stacd, void *exinf);

#endif