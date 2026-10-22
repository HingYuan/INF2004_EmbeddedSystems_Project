#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <tk/tkernel.h>
#define SERVO_PIN          15
#define SONAR_TRIG_PIN     28
#define SONAR_ECHO_PIN      7

// Navigator sets this flag after bypass and line recovery are complete.
#define SONAR_RESUME_FLAG (1U << 0)

IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif

