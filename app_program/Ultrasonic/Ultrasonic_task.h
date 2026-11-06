#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <tk/tkernel.h>

// Hardware pins
#define SERVO_PIN          15
#define SONAR_TRIG_PIN      5
#define SONAR_ECHO_PIN      4

// Event flags shared with Navigator
#define SONAR_RESUME_FLAG       (1U << 0)
#define SONAR_SCAN_REQUEST      (1U << 1)

#define SONAR_PROBE_FRONT_FLAG  (1U << 2)
#define SONAR_PROBE_LEFT_FLAG   (1U << 3)
#define SONAR_PROBE_RIGHT_FLAG  (1U << 4)

IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif
