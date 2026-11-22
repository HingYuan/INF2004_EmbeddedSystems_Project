#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <tk/tkernel.h>
#include "../robot_tasks.h"

// Hardware pins
#define SERVO_PIN          15
#define SONAR_TRIG_PIN      5
#define SONAR_ECHO_PIN      4
#define OBSTACLE_TRIGGER_CM 20U

// Event flags shared with Navigator
#define SONAR_RESUME_FLAG       (1U << 0)
#define SONAR_SCAN_REQUEST      (1U << 1)

#define SONAR_PROBE_FRONT_FLAG  (1U << 2)
#define SONAR_PROBE_LEFT_FLAG   (1U << 3)
#define SONAR_PROBE_RIGHT_FLAG  (1U << 4)
#define SONAR_TRACK_LEFT_FLAG   (1U << 5)
#define SONAR_TRACK_RIGHT_FLAG  (1U << 6)
#define SONAR_PROFILE_RAYS       7U // -45, -30, -15, 0, +15, +30, +45 degrees.

typedef enum {
    SONAR_INVALID,
    SONAR_OK,
    SONAR_NO_ECHO,
    SONAR_TOO_CLOSE,
    SONAR_SENSOR_FAULT,
    SONAR_SERVO_FAULT,
    SONAR_UNSTABLE
} SONAR_STATUS;

#define ROBOT_HALF_WIDTH_CM       6U
#define ROBOT_LENGTH_CM          20U
#define WALL_BODY_GAP_CM          5U

// Latched separately so Navigator cannot release a hardware fault as a scan stop.
#define sonar_fault ((robot_flags_get() & (ROBOT_FAULT_SONAR | ROBOT_FAULT_SERVO)) != 0U)

typedef struct __attribute__((packed)) {
    UW confirmations : 2; // 0..2 before switching into assessment.
    UW tracking : 1;
    W tracking_angle : 7; // -45..45.
} SONAR_RUNTIME_STATE;

IMPORT void ultrasonic_task(INT stacd, void *exinf);

#endif
