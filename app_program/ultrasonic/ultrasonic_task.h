/**
 * @file ultrasonic_task.h
 * @brief HC-SR04 ultrasonic ranger and pan-servo scanning task interface.
 *
 * Reference: HC-SR04 Ultrasonic Ranging Module Datasheet (58 usec per cm).
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel task entry signature mandates INT stacd parameter.
 */

#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../robot_tasks.h"

/* Hardware pins and detection threshold. */
#define SONAR_SERVO_PIN                 (15u)
#define SONAR_TRIG_PIN                  (5u)
#define SONAR_ECHO_PIN                  (4u)
#define SONAR_OBSTACLE_TRIG_CM          (20u)

/* Event flags shared with Navigator. */
#define SONAR_RESUME_FLAG               (1u << 0u)
#define SONAR_SCAN_REQUEST              (1u << 1u)
#define SONAR_PROBE_FRONT_FLAG          (1u << 2u)
#define SONAR_PROBE_LEFT_FLAG           (1u << 3u)
#define SONAR_PROBE_RIGHT_FLAG          (1u << 4u)
#define SONAR_TRACK_LEFT_FLAG           (1u << 5u)
#define SONAR_TRACK_RIGHT_FLAG          (1u << 6u)

/* Seven rays: -45, -30, -15, 0, +15, +30, +45 degrees. */
#define SONAR_PROFILE_RAYS              (7u)

/* Vehicle body clearance geometry in centimetres. */
#define SONAR_ROBOT_HALF_WIDTH_CM       (6u)
#define SONAR_ROBOT_LENGTH_CM           (20u)
#define SONAR_WALL_BODY_GAP_CM          (5u)

typedef enum
{
    SONAR_INVALID      = 0,
    SONAR_OK           = 1,
    SONAR_NO_ECHO      = 2,
    SONAR_TOO_CLOSE    = 3,
    SONAR_SENSOR_FAULT = 4,
    SONAR_SERVO_FAULT  = 5,
    SONAR_UNSTABLE     = 6
} ultrasonic_task_status_t;

/**
 * @brief Check whether a latched sonar or servo hardware fault is active.
 * @return True if either sonar or servo fault flag is set.
 */
bool ultrasonic_task_has_fault(void);

/**
 * @brief Main ultrasonic obstacle detection, profiling, and tracking task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ultrasonic_task_run_task(INT stacd, void * p_exinf);

#endif /* ULTRASONIC_TASK_H */

/*** end of file ***/
