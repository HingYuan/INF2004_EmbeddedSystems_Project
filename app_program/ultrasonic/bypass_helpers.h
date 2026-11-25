/**
 * @file bypass_helpers.h
 * @brief Rectangular obstacle bypass and front-face width estimation helpers.
 */

#ifndef BYPASS_HELPERS_H
#define BYPASS_HELPERS_H

#include <stdbool.h>
#include <stdint.h>

#include "../navigator/navigator_task.h"

/*
 * Nominal 15-degree HC-SR04 cone: subtract 7.5 degrees at each detected edge.
 * WARNING: Starting estimate, not a measured calibration of this sensor.
 * Keep servo PWM and bypass clearance calculations independent of this value.
 */
#ifndef BYPASS_WIDTH_BEAM_HALF_DDEG
#define BYPASS_WIDTH_BEAM_HALF_DDEG     (75u)
#endif
#define BYPASS_WIDTH_DEPTH_TOL_CM       (2u)
#define BYPASS_WIDTH_REFINE_PASSES      (3u)
#define BYPASS_WIDTH_REPEAT_TOL_MM      (20u)
#define BYPASS_WIDTH_MAX_SPAN_MM        (20u)
#define BYPASS_WIDTH_SAMPLE_SPREAD_CM   (2u)

typedef enum
{
    WIDTH_RAY_UNCERTAIN = 0,
    WIDTH_RAY_FACE      = 1,
    WIDTH_RAY_CLEAR     = 2
} bypass_helpers_ray_class_t;

/**
 * @brief Classify one sonar ray relative to the obstacle front face plane.
 * @param[in] front_cm  Measured center standoff in cm.
 * @param[in] angle_deg Signed servo angle in degrees (-45..45).
 * @param[in] dist_cm   Measured ray distance in cm.
 * @param[in] status    Ultrasonic status code.
 * @return Classification of ray as face, clear, or uncertain.
 */
bypass_helpers_ray_class_t bypass_helpers_classify_ray(
    uint32_t front_cm,
    int32_t angle_deg,
    uint32_t dist_cm,
    ultrasonic_task_status_t status);

/**
 * @brief Convert a beam-compensated ray angle at front_cm to lateral mm offset.
 * @param[in] front_cm  Front standoff distance in cm.
 * @param[in] angle_deg Unsigned ray angle magnitude in degrees (0..45).
 * @return Lateral offset in millimetres.
 */
uint32_t bypass_helpers_width_offset_mm(uint32_t front_cm, uint32_t angle_deg);

/**
 * @brief Check whether three sorted sonar distance samples are consistent.
 * @param[in] low_cm    Smallest sample in cm.
 * @param[in] middle_cm Median sample in cm.
 * @param[in] high_cm   Largest sample in cm.
 * @return True if sorted and at least one adjacent gap is within tolerance.
 */
bool bypass_helpers_samples_valid(uint32_t low_cm,
                                  uint32_t middle_cm,
                                  uint32_t high_cm);

/**
 * @brief Combine one or two obstacle scans into a dashboard width estimate.
 * @param[in] p_first  First stationary obstacle profile.
 * @param[in] p_second Optional second profile taken after 2 cm reverse.
 * @return Packed obstacle width and status result.
 */
navigator_task_width_t bypass_helpers_estimate_width(
    const navigator_task_obs_profile_t * p_first,
    const navigator_task_obs_profile_t * p_second);

/**
 * @brief Print the obstacle width result to the serial console.
 * @param[in] width Packed obstacle width and status structure.
 */
void bypass_helpers_report_width(navigator_task_width_t width);

#ifndef NDEBUG
/**
 * @brief Run debug-only unit checks for obstacle width estimation.
 * @return True if all assertions pass, false otherwise.
 */
bool bypass_helpers_width_check(void);
#endif

/**
 * @brief Choose detour side and initial outward shift from two sonar profiles.
 * @param[in,out] p_bypass Bypass state context.
 * @param[in]     p_second Second obstacle profile after 2 cm reverse.
 */
void bypass_helpers_plan_width(navigator_task_bypass_ctx_t * p_bypass,
                               const navigator_task_obs_profile_t * p_second);

#ifndef NDEBUG
/**
 * @brief Run debug-only unit checks for bypass offset and pivot helpers.
 * @return True if all assertions pass, false otherwise.
 */
bool bypass_helpers_self_check(void);
#endif

/**
 * @brief Check whether a motor stall, encoder, or sonar fault is active.
 * @return True if hardware fault requires bypass abort.
 */
bool bypass_helpers_has_hw_fault(void);

/**
 * @brief Abort bypass detour, latch E-stop, and halt motors safely.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer zeroed on abort.
 * @param[in]     p_reason Diagnostic abort reason string.
 */
void bypass_helpers_abort(navigator_task_bypass_ctx_t * p_bypass,
                          motor_task_cmd_t * p_cmd,
                          const char * p_reason);

/**
 * @brief Queue one measured bypass movement and arm its fault deadline.
 * @param[in,out] p_bypass   Bypass state context.
 * @param[out]    p_cmd      Motor command tracking structure.
 * @param[in]     phase      Target bypass phase.
 * @param[in]     left_spd   Left motor power percentage (-100..100).
 * @param[in]     right_spd  Right motor power percentage (-100..100).
 * @param[in]     amount_val Distance (mm) or pivot angle (deg).
 */
void bypass_helpers_begin(navigator_task_bypass_ctx_t * p_bypass,
                          motor_task_cmd_t * p_cmd,
                          navigator_task_bypass_phase_t phase,
                          int32_t left_spd,
                          int32_t right_spd,
                          uint32_t amount_val);

/**
 * @brief Queue an in-place 90-degree pivot in the requested direction.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 * @param[in]     phase    Target bypass phase.
 * @param[in]     dir_sign Pivot direction (+1 left, -1 right).
 */
void bypass_helpers_pivot(navigator_task_bypass_ctx_t * p_bypass,
                          motor_task_cmd_t * p_cmd,
                          navigator_task_bypass_phase_t phase,
                          int32_t dir_sign);

/**
 * @brief Request side-facing ultrasonic tracking on the obstacle side.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer used if abort occurs.
 */
void bypass_helpers_track_side(navigator_task_bypass_ctx_t * p_bypass,
                               motor_task_cmd_t * p_cmd);

/**
 * @brief Request a stationary front sonar probe for clearance confirmation.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer zeroed while stationary.
 * @param[in]     phase    Probe phase (width check or return check).
 */
void bypass_helpers_probe(navigator_task_bypass_ctx_t * p_bypass,
                          motor_task_cmd_t * p_cmd,
                          navigator_task_bypass_phase_t phase);

/**
 * @brief Advance past obstacle depth or pivot inward to probe return path.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 */
void bypass_helpers_follow_next(navigator_task_bypass_ctx_t * p_bypass,
                                motor_task_cmd_t * p_cmd);

/**
 * @brief Validate settled odometry after a bounded translation completes.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command buffer used if abort occurs.
 * @return True if odometry is valid, false if bypass was aborted.
 */
bool bypass_helpers_complete_leg(navigator_task_bypass_ctx_t * p_bypass,
                                 motor_task_cmd_t * p_cmd);

/**
 * @brief Process a fresh stationary front sonar probe reply during bypass.
 * @param[in,out] p_bypass Bypass state context.
 * @param[out]    p_cmd    Motor command tracking structure.
 * @param[in]     p_probe  Latest sonar probe message payload.
 */
void bypass_helpers_probe_result(navigator_task_bypass_ctx_t * p_bypass,
                                 motor_task_cmd_t * p_cmd,
                                 const navigator_task_sonar_probe_t * p_probe);

#endif /* BYPASS_HELPERS_H */

/*** end of file ***/
