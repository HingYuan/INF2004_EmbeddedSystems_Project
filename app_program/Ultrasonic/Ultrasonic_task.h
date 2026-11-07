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
#define SONAR_TRACK_LEFT_FLAG   (1U << 5)
#define SONAR_TRACK_RIGHT_FLAG  (1U << 6)
#define SONAR_PAUSE_FLAG        (1U << 7)

typedef enum {
    SONAR_INVALID = 0,       // No measurement, or an invalid request.
    SONAR_OK,                // Measured distance is usable.
    SONAR_NO_ECHO,           // No pulse, or a completed pulse beyond range; clearance is unmeasured.
    SONAR_TOO_CLOSE,         // Pulse below the 2 cm measurement limit.
    SONAR_SENSOR_FAULT,      // Trigger write failed, or ECHO stayed high.
    SONAR_SERVO_FAULT,       // Servo command failed.
    SONAR_UNSTABLE           // Too few consistent measured samples.
} SONAR_STATUS;

// Front-centred sonar, 12 x 20 cm body; shared by profiling and bypass.
#define ROBOT_HALF_WIDTH_CM      6U  // Half the full body width: 12 cm / 2.
#define ROBOT_LENGTH_CM         20U  // Front-to-back body length; used to allow the rear to clear the corner (cm).
#define WALL_BODY_GAP_CM         5U  // Desired gap between the car's body and the obstacle (cm), not the diagonal echo.

IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif
