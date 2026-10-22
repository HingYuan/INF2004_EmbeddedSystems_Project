#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#define SERVO_PIN          15
#define SONAR_TRIG_PIN      1
#define SONAR_ECHO_PIN      0

#include <tk/tkernel.h>

IMPORT void servo_task(INT stacd, void *exinf);
IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif