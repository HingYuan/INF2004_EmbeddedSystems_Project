/**
 * @file navigator_task.h
 * @brief Central Navigator FSM types, shared telemetry API, and bypass config.
 *
 * NOTE: See docs/fsm_diagrams.md for the Navigator and Bypass state machine
 * diagrams. All bit-fields use unsigned integer types (BARR-C Rule 5.3.a);
 * signed domains use explicit 2's-complement pack/sign-extend accessors.
 *
 * BARR-C Deviation (Rules 1.1.c, 5.2.a, 5.6.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Packed IPC and telemetry structs save ~7.8 KB SRAM across the
 *   64-slot Navigator message buffer; 1-bit flags use unsigned bit-fields per
 *   Rule 5.3.a; uT-Kernel task entry signature mandates INT stacd.
 */

#ifndef NAVIGATOR_TASK_H
#define NAVIGATOR_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../motor/motor_task.h"
#include "../robot_tasks.h"
#include "../ultrasonic/ultrasonic_task.h"

/* Navigator speed settings (% PWM). */
#define NAV_FORWARD_SPEED               (16)
#define NAV_FORWARD_SPEED_HUMP          (30)
#define NAV_STOP_SPEED                  (0)

/* Smooth differential steering speeds for line following. */
#define NAV_LEFT_TURN_L_BASE            (10)
#define NAV_LEFT_TURN_R_BASE            (30)
#define NAV_RIGHT_TURN_L_BASE           (30)
#define NAV_RIGHT_TURN_R_BASE           (10)

/* Ramped steering speeds when stuck correcting on a curve. */
#define NAV_LEFT_TURN_L_RAMP            (5)
#define NAV_LEFT_TURN_R_RAMP            (40)
#define NAV_RIGHT_TURN_L_RAMP           (40)
#define NAV_RIGHT_TURN_R_RAMP           (5)

/* In-place junction pivot speeds. */
#define NAV_LEFT_PIVOT_L                (23)
#define NAV_LEFT_PIVOT_R                (23)
#define NAV_RIGHT_PIVOT_L               (23)
#define NAV_RIGHT_PIVOT_R               (23)

#define NAV_TURNING_RAMPUP_LIMIT        (50u)
#define NAV_NUMBER_OF_COMMANDS          (5u)

/* Bit widths for signed domains stored in unsigned bit-fields. */
#define NAV_ANGLE_BIT_WIDTH             (7u)
#define NAV_TILT_BIT_WIDTH              (12u)
#define NAV_SIDE_BIT_WIDTH              (2u)

/* Bypass detour tuning and safety bounds (70 mm outward/depth clearances). */
#define BYPASS_API_POWER                (25)
#define BYPASS_LEFT_TURN_DEG            (90u)
#define BYPASS_RIGHT_TURN_DEG           (90u)
#define BYPASS_TURN_TIMEOUT_MSEC        (5000u)
#define BYPASS_MOVE_MARGIN_MSEC         (3000u)
#define BYPASS_MOVE_TMO_MSEC_CM         (500u)
#define BYPASS_SCAN_BACK_MM             (20u)
#define BYPASS_FORWARD_STEP_MM          (150u)
#define BYPASS_WIDTH_STEP_MM            (150u)
#define BYPASS_PIVOT_CLEAR_MM           (70u)
#define BYPASS_DEPTH_CLEAR_MM           (70u)
#define BYPASS_PROBE_MSEC               (2000u)
#define BYPASS_CLEAR_CONFIRM            (2u)
#define BYPASS_PROFILE_DEPTH_CM         (10u)
#define BYPASS_MAX_SIDE_MM              (1500u)
#define BYPASS_MAX_FORWARD_MM           (3000u)
#define BYPASS_PIVOT_FROM_FRONT_CM      (10u)
#define BYPASS_SWEPT_RADIUS_CM          (12u)
#define BYPASS_WIDTH_CLEAR_MM           (100u)
#define BYPASS_TRACK_STALE_MSEC         (1000u)
#define BYPASS_RESPONSE_MSEC            (1000u)
#define BYPASS_SCAN_MSEC                (15000u)
#define BYPASS_POLL_MSEC                (20u)

typedef enum
{
    IR_STATE_W_W  = 0,
    IR_STATE_W_B  = 1,
    IR_STATE_B_W  = 2,
    IR_STATE_B_B  = 3,
    IR_STATE_NULL = 4
} navigator_task_ir_state_t;

/* Width reporting is independent of conservative bypass clearance profile. */
typedef enum
{
    WIDTH_UNAVAILABLE  = 0,
    WIDTH_SCANNING     = 1,
    WIDTH_ESTIMATED    = 2,
    WIDTH_INCONSISTENT = 3
} navigator_task_width_status_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t left_min_mm      : 12;
    uint32_t left_max_mm      : 12;
    uint32_t right_min_mm     : 12;
    uint32_t right_max_mm     : 12;
    uint32_t b_is_left_valid  : 1;
    uint32_t b_is_right_valid : 1;
} navigator_task_width_scan_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t width_cm : 10;
    uint32_t status   : 2;
} navigator_task_width_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t                    closest_cm            : 9;
    uint32_t                    closest_ang_raw       : 7;
    uint32_t                    front_cm              : 9;
    uint32_t                    left_scan_cm          : 9;
    uint32_t                    right_scan_cm         : 9;
    uint32_t                    left_scan_status      : 3;
    uint32_t                    right_scan_status     : 3;
    uint32_t                    left_edge_deg_raw     : 7;
    uint32_t                    right_edge_deg_raw    : 7;
    uint32_t                    estimated_width_cm    : 10;
    uint32_t                    left_offset_cm        : 9;
    uint32_t                    right_offset_cm       : 9;
    uint32_t                    left_clearance_cm     : 9;
    uint32_t                    right_clearance_cm    : 9;
    uint32_t                    b_is_left_edge_valid  : 1;
    uint32_t                    b_is_right_edge_valid : 1;
    uint32_t                    b_is_width_valid      : 1;
    uint32_t                    b_is_left_off_valid   : 1;
    uint32_t                    b_is_right_off_valid  : 1;
    uint32_t                    b_is_left_clr_valid   : 1;
    uint32_t                    b_is_right_clr_valid  : 1;
    uint32_t                    b_is_profile_valid    : 1;
    navigator_task_width_scan_t measured_width;
    uint8_t                     rays[((SONAR_PROFILE_RAYS * 12u) + 7u) / 8u];
} navigator_task_obs_profile_t;

/* Retain existing telemetry phase IDs; new phases are appended. */
typedef enum
{
    BYPASS_IDLE            = 0,
    BYPASS_TURN_OUT        = 1,
    BYPASS_SHIFT_OUT       = 3,
    BYPASS_TURN_PARALLEL   = 4,
    BYPASS_ADVANCE         = 7,
    BYPASS_TURN_IN         = 10,
    BYPASS_FIND_LINE       = 12,
    BYPASS_LINE_FOUND      = 13,
    BYPASS_ABORT           = 17,
    BYPASS_WAIT_RELEASE    = 18,
    BYPASS_SCAN_BACK       = 19,
    BYPASS_SCAN_SECOND     = 20,
    BYPASS_SCAN_RETURN     = 21,
    BYPASS_WIDTH_CHECK     = 22,
    BYPASS_WIDTH_TURN_OUT  = 23,
    BYPASS_WIDTH_EXTEND    = 24,
    BYPASS_WIDTH_ALIGN     = 25,
    BYPASS_DEPTH_TURN_BACK = 26,
    BYPASS_RETURN_CHECK    = 27
} navigator_task_bypass_phase_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t                     phase                : 5;
    uint32_t                     side_raw             : 2;
    uint32_t                     phase_start_usec;
    uint32_t                     deadline_usec;
    uint32_t                     requested_usec;
    uint32_t                     last_track_usec;
    uint32_t                     last_sample_usec;
    navigator_task_obs_profile_t first_scan;
    uint32_t                     front_cm             : 9;
    uint32_t                     planned_shift_mm     : 13;
    uint32_t                     shift_mm             : 11;
    uint32_t                     forward_mm           : 12;
    uint32_t                     leg_mm               : 12;
    int32_t                      leg_start_mm;
    uint32_t                     probe_clear_cnt      : 2;
    uint32_t                     probe_attempts       : 2;
    uint32_t                     b_has_width_clr_pend : 1;
    uint32_t                     b_has_depth_clr_pend : 1;
    uint32_t                     b_is_tracking        : 1;
    uint32_t                     b_has_move_pending   : 1;
    uint32_t                     black_cnt            : 2;
    uint32_t                     b_owns_obs_stop      : 1;
} navigator_task_bypass_ctx_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t sampled_at_usec;
    uint16_t dist_cm       : 9;
    uint8_t  angle_deg_raw : 7;
    uint32_t b_is_valid    : 1;
    uint32_t status        : 3;
} navigator_task_sonar_probe_t;

typedef enum
{
    IMU_MSG_NONE              = 0,
    IMU_MSG_NO_HUMP           = 1,
    IMU_MSG_HUMP_DETECTED     = 2,
    IMU_MSG_LIFTED_OVERTURNED = 3
} navigator_task_imu_event_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t event          : 2;
    uint16_t pitch_ddeg_raw : 12;
    uint16_t roll_ddeg_raw  : 12;
    uint16_t peak_mg        : 12;
    uint16_t height_dmm     : 10;
} navigator_task_imu_msg_t;

typedef enum
{
    MODE_BARCODE_SCANNING        = 0,
    MODE_WIFI_SEND_COMMS         = 1,
    MODE_LINE_FOLLOWING          = 2,
    MODE_GOING_OVER_HUMP         = 3,
    MODE_DECISION_MAKING_ADVANCE = 4,
    MODE_DECISION_MAKING_TURN    = 5,
    MODE_WALL_HUGGING            = 6,
    MODE_REENTRY_ADVANCE         = 7,
    MODE_REENTRY_PIVOT           = 8,
    MODE_OBSTACLE_ASSESS         = 9,
    MODE_SAFE_STOP               = 10
} navigator_task_mode_t;

typedef enum
{
    DIR_FRONT   = 0,
    DIR_BACK    = 1,
    DIR_LEFT    = 2,
    DIR_RIGHT   = 3,
    DIR_STOP    = 4,
    DIR_INVALID = 5
} navigator_task_dir_t;

typedef enum
{
    MSG_WIFI_BARCODE_MAPPING = 0,
    MSG_IR_UPDATE            = 1,
    MSG_BARCODE_DECODED      = 2,
    MSG_OBSTACLE_DETECTED    = 3,
    MSG_MOTOR_STOPPED        = 4,
    MSG_OBSTACLE_PROFILED    = 5,
    MSG_TRACKING_OBSTACLE    = 6,
    MSG_SONAR_PROBED         = 7,
    MSG_MOTOR_MOVE_DONE      = 8,
    MSG_IMU_UPDATE           = 9,
    MSG_REMOTE_CMD           = 10,
    MSG_SONAR_DISTANCE       = 11,
    MSG_BYPASS_TICK          = 12
} navigator_task_msg_type_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint16_t dist_cm    : 9;
    uint32_t b_is_valid : 1;
    uint32_t status     : 3;
} navigator_task_sonar_dist_t;

typedef union ROBOT_TASKS_PACKED
{
    uint8_t                      ir_state;
    uint8_t                      decoded_char;
    uint16_t                     obs_dist_cm;
    navigator_task_obs_profile_t obs_profile;
    navigator_task_sonar_probe_t sonar_probe;
    navigator_task_sonar_dist_t  sonar_dist;
    navigator_task_imu_msg_t     imu_msg;
    motor_task_cmd_t             remote_cmd;
    uint16_t                     packed_mapping;
} navigator_task_payload_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t                 type : 4;
    navigator_task_payload_t data;
} navigator_task_msg_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t seq_num;
    int32_t  enc_left;
    int32_t  enc_right;
    int32_t  dist_cm;
    int8_t   speed_left;
    int8_t   speed_right;
    uint16_t obs_dist_cm        : 9;
    uint16_t closest_cm         : 9;
    uint16_t closest_ang_raw    : 7;
    uint16_t est_width_cm       : 10;
    uint16_t pitch_ddeg_raw     : 12;
    uint16_t roll_ddeg_raw      : 12;
    uint16_t heading_deg        : 9;
    uint16_t height_dmm         : 10;
    uint32_t movement_mode      : 4;
    uint32_t bypass_phase       : 5;
    uint32_t ir_state           : 3;
    uint32_t last_barcode       : 7;
    uint32_t inst_idx           : 3;
    uint32_t b_is_obs_dist_valid : 1;
    uint32_t obs_status         : 3;
    uint32_t imu_state          : 2;
    uint32_t imu_event          : 2;
    uint32_t obs_width_status   : 2;
} navigator_task_telemetry_t;

typedef struct
{
    uint8_t bits[((NAV_NUMBER_OF_COMMANDS * 3u) + 7u) / 8u];
} navigator_task_route_t;

/**
 * @brief Read one sonar ray distance from a packed obstacle profile.
 * @param[in] p_profile Obstacle profile structure.
 * @param[in] ray_idx   Ray index (0..6).
 * @return Distance in cm (0..511).
 */
uint32_t navigator_task_get_ray_cm(
    const navigator_task_obs_profile_t * p_profile,
    uint32_t ray_idx);

/**
 * @brief Read one sonar ray status code from a packed obstacle profile.
 * @param[in] p_profile Obstacle profile structure.
 * @param[in] ray_idx   Ray index (0..6).
 * @return Ultrasonic status enum value.
 */
ultrasonic_task_status_t navigator_task_get_ray_status(
    const navigator_task_obs_profile_t * p_profile,
    uint32_t ray_idx);

/**
 * @brief Store one sonar ray distance and status in a packed obstacle profile.
 * @param[in,out] p_profile Obstacle profile structure.
 * @param[in]     ray_idx   Ray index (0..6).
 * @param[in]     dist_cm   Measured distance in cm (0..511).
 * @param[in]     status    Ultrasonic status enum value.
 */
void navigator_task_set_ray(navigator_task_obs_profile_t * p_profile,
                            uint32_t ray_idx,
                            uint32_t dist_cm,
                            ultrasonic_task_status_t status);

/**
 * @brief Read a packed 3-bit route direction entry.
 * @param[in] p_route   Packed route array.
 * @param[in] route_idx Zero-based route step index.
 * @return Stored direction, or DIR_INVALID if index is out of range.
 */
navigator_task_dir_t navigator_task_get_route(
    const navigator_task_route_t * p_route,
    uint32_t route_idx);

/**
 * @brief Store a packed 3-bit route direction entry.
 * @param[in,out] p_route   Packed route array.
 * @param[in]     route_idx Zero-based route step index.
 * @param[in]     dir       Direction enum value to store.
 */
void navigator_task_set_route(navigator_task_route_t * p_route,
                              uint32_t route_idx,
                              navigator_task_dir_t dir);

/**
 * @brief Read signed bypass detour side (+1 left, -1 right, 0 unset).
 * @param[in] p_bypass Bypass context structure.
 * @return Signed side indicator (-1, 0, or +1).
 */
int32_t navigator_task_get_side(const navigator_task_bypass_ctx_t * p_bypass);

/**
 * @brief Set signed bypass detour side (+1 left, -1 right, 0 unset).
 * @param[in,out] p_bypass Bypass context structure.
 * @param[in]     side_val Signed side indicator (-1, 0, or +1).
 */
void navigator_task_set_side(navigator_task_bypass_ctx_t * p_bypass,
                             int32_t side_val);

/**
 * @brief Copy the latest telemetry snapshot under the shared data lock.
 * @param[out] p_snapshot Destination snapshot structure.
 */
void navigator_task_read_telem(navigator_task_telemetry_t * p_snapshot);

/**
 * @brief Read the current IMU event code from shared telemetry.
 * @return IMU event enum value (0..3).
 */
uint8_t navigator_task_get_imu_event(void);

/**
 * @brief Read the latest compass heading in degrees from shared telemetry.
 * @return Heading angle (0..359 degrees).
 */
int16_t navigator_task_get_heading(void);

/**
 * @brief Update live ultrasonic distance fields in shared telemetry.
 * @param[in] b_is_valid True if distance reading is valid.
 * @param[in] status     Ultrasonic status enum value.
 * @param[in] dist_cm    Distance in cm (zeroed when invalid).
 */
void navigator_task_set_sonar_telem(bool b_is_valid,
                                    uint8_t status,
                                    uint32_t dist_cm);

/**
 * @brief Update IMU live hump height in tenths of a millimetre.
 * @param[in] height_dmm Clamped height (0..680).
 */
void navigator_task_set_imu_height(int16_t height_dmm);

/**
 * @brief Update IMU event code in shared telemetry.
 * @param[in] imu_event IMU event enum value (0..3).
 */
void navigator_task_set_imu_event(uint8_t imu_event);

/**
 * @brief Update IMU FSM state code in shared telemetry.
 * @param[in] imu_state IMU state enum value (0..3).
 */
void navigator_task_set_imu_state(uint8_t imu_state);

/**
 * @brief Atomically update IMU pitch, roll, heading, and optional state.
 * @param[in] pitch_ddeg         Pitch in tenths of a degree (-1800..1800).
 * @param[in] roll_ddeg          Roll angle in tenths of a degree (-1800..1800).
 * @param[in] heading_deg        Compass heading in degrees (0..359).
 * @param[in] b_should_set_state True to also update imu_state.
 * @param[in] imu_state          IMU state enum when b_should_set_state is true.
 */
void navigator_task_set_imu_pose(int16_t pitch_ddeg,
                                 int16_t roll_ddeg,
                                 int16_t heading_deg,
                                 bool b_should_set_state,
                                 uint8_t imu_state);

/**
 * @brief Atomically latch IMU lifted/overturned telemetry fields.
 * @param[in] pitch_ddeg Pitch angle in tenths of a degree (-1800..1800).
 * @param[in] roll_ddeg  Roll angle in tenths of a degree (-1800..1800).
 * @param[in] imu_state  IMU state enum value.
 * @param[in] imu_event  IMU event enum value.
 */
void navigator_task_set_imu_stop(int16_t pitch_ddeg,
                                 int16_t roll_ddeg,
                                 uint8_t imu_state,
                                 uint8_t imu_event);

/**
 * @brief Central Navigator coordination task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void navigator_task_run_task(INT stacd, void * p_exinf);

#endif /* NAVIGATOR_TASK_H */

/*** end of file ***/
