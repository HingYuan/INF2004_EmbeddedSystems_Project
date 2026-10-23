
#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <tk/tkernel.h>

// Hardware connections
#define SERVO_PIN          15
#define SONAR_TRIG_PIN     28
#define SONAR_ECHO_PIN      7

// Navigator commands to the ultrasonic task
#define SONAR_RESUME_FLAG  (1U << 0)
#define SONAR_SCAN_REQUEST (1U << 1)

IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif
