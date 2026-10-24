#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <tk/tkernel.h>

// Check against actual Robo Pico wiring before flashing.
#define SERVO_PIN           15
#define SONAR_TRIG_PIN      28
#define SONAR_ECHO_PIN       7

// Shared ranger_flgid event bits
#define SONAR_RESUME_FLAG  (1U << 0)
#define SONAR_SCAN_REQUEST (1U << 1)

IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif
