#ifndef NAVIGATOR_TASK_H
#define NAVIGATOR_TASK_H

#include <tk/tkernel.h>
#include "../Motor/Motor_task.h"
#include "../Ultrasonic/Ultrasonic_task.h"

/* 
=========================================
|       NAVIGATOR SPEED SETTINGS        |
=========================================
*/

// Forward
// #define FORWARD_SPEED           30
// #define FORWARD_SPEED_BARCODE   30
#define FORWARD_SPEED           30
#define FORWARD_SPEED_BARCODE   30

// Stop / brake
#define STOP_SPEED              0

// turning (maintain forward speed)
// #define LEFT_TURN_L_BASE        15
// #define LEFT_TURN_R_BASE        45
// #define RIGHT_TURN_L_BASE       55
// #define RIGHT_TURN_R_BASE       0
#define LEFT_TURN_L_BASE        15
#define LEFT_TURN_R_BASE        45
#define RIGHT_TURN_L_BASE       45
#define RIGHT_TURN_R_BASE       0

#define LEFT_TURN_L_BARCODE     20
#define LEFT_TURN_R_BARCODE     40
#define RIGHT_TURN_L_BARCODE    40
#define RIGHT_TURN_R_BARCODE    20

// turning w/ max rampup
#define LEFT_TURN_L_RAMP        0
#define LEFT_TURN_R_RAMP        50
// #define RIGHT_TURN_L_RAMP       65
#define RIGHT_TURN_L_RAMP       50
// #define RIGHT_TURN_R_RAMP       0
#define RIGHT_TURN_R_RAMP       -30

// pivoting / turning on the spot
#define LEFT_PIVOT_L            28
#define LEFT_PIVOT_R            28
#define RIGHT_PIVOT_L           28
#define RIGHT_PIVOT_R           28

/* 
=========================================
|                   IR                  |
=========================================
*/

typedef enum {
    STATE_W_W,
    STATE_W_B,
    STATE_B_W,
    STATE_B_B,
    STATE_NULL,
} IR_STATE;

#define TURNING_RAMPUP_LIMIT 10

/* 
=========================================
|               Ultrasonic              |
=========================================
*/

typedef struct {
    UW closest_cm;
    W closest_angle_deg;       // Positive angle is left
    UW front_cm;              // Stationary centre reading used to check room for the diagonal entry.

    W left_edge_deg;
    W right_edge_deg;
    BOOL left_edge_valid;
    BOOL right_edge_valid;

    UW estimated_width_cm;     // Approximate visible lateral extent
    BOOL width_valid;

    UW left_offset_cm;         // Proposed lateral centre offset, NOT clearance
    UW right_offset_cm;
    BOOL left_offset_valid;
    BOOL right_offset_valid;

    UW left_clearance_cm;      // Unknown until the entire route is checked
    UW right_clearance_cm;
    BOOL left_clearance_valid;
    BOOL right_clearance_valid;

    BOOL profile_valid;        // Basic scan quality, not a safe route
} OBSTACLE_PROFILE;

/*
 * MOVEMENT_MODE selects the main job; BYPASS_PHASE selects the current step inside bypass.
 * Normal path: IDLE -> CHECK_OUT -> TURN_OUT -> SHIFT_OUT -> TURN_PARALLEL -> CHECK_SIDE
 *              -> ADVANCE -> CLEAR_CORNER -> TURN_IN -> FIND_LINE -> LINE_FOUND.
 * A close obstacle uses REVERSE -> RECOVER_OUT -> resume_phase.
 * During assessment, REVERSE -> RESCAN_WAIT -> IDLE makes room for another scan.
 * Keep enum order unchanged: telemetry uses these numeric IDs, including unused older phases.
 */
typedef enum {
    BYPASS_IDLE,              // Wait for the initial stop acknowledgement or the stationary scan result.
    BYPASS_TURN_OUT,          // Pivot 45 degrees toward the selected passing side.
    BYPASS_CHECK_OUT,         // Wait for the first reading after parking the servo toward the obstacle.
    BYPASS_SHIFT_OUT,         // Drive diagonally until the car can clear the obstacle's side.
    BYPASS_TURN_PARALLEL,     // Pivot 45 degrees back to the original forward heading.
    BYPASS_CHECK_SIDE,        // Drive straight until the sonar ray reaches the side, then acquire the wall.
    BYPASS_CHECK_FORWARD,     // Unused older forward-check phase; fixed-side tracking does not use it.
    BYPASS_ADVANCE,           // Follow the obstacle, steering to maintain the target side distance.
    BYPASS_CHECK_CLEAR,       // Unused older phase; ADVANCE now confirms consecutive far-edge readings.
    BYPASS_CLEAR_CORNER,      // Drive farther so the rear clears the box; seeing the wall again cancels this.
    BYPASS_TURN_IN,           // Pivot 45 degrees inward toward the black line.
    BYPASS_CHECK_RETURN,      // Unused older forward-check phase before returning to the line.
    BYPASS_FIND_LINE,         // Drive toward the line until two consecutive IR readings contain black.
    BYPASS_LINE_FOUND,        // Stop normally, restore forward sonar, and hand over to MODE_REENTRY_ADVANCE.
    BYPASS_TURN_ALIGN,        // Unused older phase; the teammate's MODE_REENTRY_PIVOT handles alignment.
    BYPASS_VERIFY_LINE,       // Unused older line-verification phase.
    BYPASS_COMPLETE,          // Unused older completion marker; the current handoff occurs at LINE_FOUND.
    BYPASS_ABORT,             // Hardware/communication fault stop, not a routine close-distance response.
    BYPASS_REVERSE,           // Reverse 5 cm to gain entry room, recover clearance, or reacquire the wall.
    BYPASS_RECOVER_OUT,       // Move forward while steering outward after a close-distance reverse.
    BYPASS_RESCAN_WAIT        // Briefly settle after stopping, then request another assessment scan.
} BYPASS_PHASE;

// shortcut: timed motion depends on battery and grip; recalibrate these values on the car.
#define BYPASS_SPEED                 40  // Base forward/reverse motor command power.
#define BYPASS_PIVOT_SPEED           40  // Motor command power during a stationary pivot.
#define BYPASS_MS_PER_CM             80U // Estimated travel time per cm at BYPASS_SPEED.
#define BYPASS_LEFT_45_MS           700U // Calibrate left and right turns separately.
#define BYPASS_RIGHT_45_MS          700U
#define BYPASS_PIVOT_FROM_FRONT_CM   10U // Front of car to the pivot point between the driven wheels.
#define BYPASS_SWEPT_RADIUS_CM       12U  // ceil(sqrt(10*10 + 6*6)) for the measured body.
#define BYPASS_REVERSE_CM             5U // Distance of each short recovery reverse.
#define BYPASS_WALL_TARGET_CM        16U // Diagonal sonar distance at 45 degrees, about a 5 cm body gap.
#define BYPASS_WALL_DEADBAND_CM       2  // No steering correction within +/-2 cm of the target.
#define BYPASS_STEER_LIMIT           10  // Maximum power adjustment to each wheel around BYPASS_SPEED.
#define BYPASS_CLOSE_CM             12U // Readings at/below this steer away; persistent readings reverse.
#define BYPASS_WALL_SEARCH_CM        40U // Largest echo accepted when initially acquiring the side wall.
#define BYPASS_EDGE_JUMP_CM          10U // A rise beyond this from the last wall reading suggests its end.
#define BYPASS_CONFIRM_COUNT         3U // Consecutive readings needed for close/far confirmation.
#define BYPASS_TRACK_STALE_MS      300U // Missing tracking messages cause a fault; no-echo messages still count.
#define BYPASS_RESPONSE_MS        1000U // Wait limit for the stop acknowledgement or tracking startup.
#define BYPASS_SCAN_MS           15000U // Wait limit for a complete stationary scan.
#define BYPASS_STOP_SETTLE_MS      150U // Pause after a reverse before rescanning.
#define BYPASS_POLL_MS              20U // Wake Navigator for phase deadlines even without new sensor messages.

// Navigator owns this context. All *_us values are microseconds; timestamps come from time_us().
typedef struct {
    BYPASS_PHASE phase;           // What the car is doing now.
    BYPASS_PHASE resume_phase;    // Step to continue after reversing/recovering.
    INT side;                     // +1: pass left (sonar right); -1: pass right (sonar left); kept for reentry.
    UW phase_started_us;          // Start time; ignore readings taken before this phase.
    UW deadline_us;               // Time to finish this phase; zero means wait for sensor events instead.
    UW requested_us;              // Start of the current side-tracking request; reject older replies.
    UW last_tracking_us;          // Last fresh tracking message, including ordinary no-echo results.
    UW last_sample_us;            // Last accepted sample timestamp; prevents counting one sample twice.
    UW shift_us;                  // Calculated duration of the 45-degree diagonal drive.
    UW acquire_us;                // Straight approach time before trusting an echo as the side wall.
    UW remaining_us;              // Time left in the interrupted movement, saved before reversing.
    UW wall_cm;                   // Last accepted obstacle distance, used to detect a jump at the far edge.
    UW lost_count;                // Consecutive missing/far echoes during acquisition or edge confirmation.
    UW close_count;               // Consecutive readings too close to the body.
    UW black_count;               // Consecutive IR readings containing black during FIND_LINE only.
    BOOL wall_seen;               // Must see the obstacle before interpreting echo loss as its far edge.
    BOOL owns_obstacle_stop;      // Allows releasing the initial scan stop, never another task's fault stop.
    BOOL recovery_outward;        // TRUE: steer outward after reversing; FALSE: assessment/acquisition retry.
} BYPASS_CONTEXT;

typedef struct {
    UW distance_cm;               // Measured echo distance; use only when valid is TRUE.
    W angle_deg;                 // Positive points left, negative points right.
    BOOL valid;                  // TRUE only for SONAR_OK; no echo is not a measured clear distance.
    SONAR_STATUS status;         // Distinguishes an echo, no echo, too close, and hardware failure.
    UW sampled_at_us;            // time_us() when this measurement finished.
} SONAR_PROBE_RESULT;

/* 
=========================================
|                  IMU                  |
=========================================
*/

typedef enum {
    IMU_MSG_NONE,
    IMU_MSG_NO_HUMP,
    IMU_MSG_HUMP_DETECTED,
    IMU_MSG_TILT_WARNING,
    IMU_MSG_SHOCK_DETECTED,
} IMU_EVENT_TYPE;

typedef struct {
    IMU_EVENT_TYPE event;
    float pitch_deg;
    float roll_deg;
    float peak_g;
    float estimated_height_mm;
} IMU_TELEMETRY_MSG;

/* 
=========================================
|               Navigator               |
=========================================
*/

#define NUMBER_OF_COMMANDS 1

typedef enum {
    MODE_BARCODE_SCANNING,          // Slow speed barcode scanning
    MODE_WIFI_SEND_COMMS,           // Wifi sending command mappings over to navigator
    MODE_LINE_FOLLOWING,            // IR is aware of line and navigator is following it
    MODE_GOING_OVER_HUMP,           // Traversing over hump
    MODE_DECISION_MAKING_ADVANCE,   // Car advances a bit after an intersection to center wheels on intersection
    MODE_DECISION_MAKING_TURN,      // Car stops at intersection, need to refer to next instruction, rotate
    MODE_WALL_HUGGING,              // Run BYPASS_PHASE steps: diagonal entry, wall following, rear clearance and line search.
    MODE_REENTRY_ADVANCE,           // Teammate's code: advance onto the line after the bypass handoff.
    MODE_REENTRY_PIVOT,             // Teammate's code: pivot to align with the line.
    MODE_OBSTACLE_ASSESS,           // Stop, scan, choose a side, and reverse/rescan if diagonal entry needs more room.
    MODE_SAFE_STOP                 // Hold stopped after a latched hardware or communication fault.
} MOVEMENT_MODE;

typedef enum {
    DIR_FRONT,
    DIR_BACK,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_STOP,
    DIR_INVALID,
} DIRECTION;

typedef enum {
    MSG_WIFI_BARCODE_MAPPING,   // Wifi sends navigator a command mapping (example 'A' = turn left)
    MSG_IR_UPDATE,              // Line following sensor update, car veering off
    MSG_BARCODE_DECODED,        // Barcode scanning sensor decoded letter
    MSG_OBSTACLE_DETECTED,      // Ultrasonic sensor detect obstacle
    MSG_MOTOR_STOPPED,          
    MSG_OBSTACLE_PROFILED,      // Left/center/right scan completed
    MSG_TRACKING_OBSTACLE,
    MSG_SONAR_PROBED,
    MSG_MOTOR_MOVE_DONE,
    MSG_IMU_UPDATE,
    MSG_REMOTE_CMD,             // Remote motor command from MQTT
    MSG_SONAR_DISTANCE,
    MSG_BYPASS_TICK             // Navigator-local timeout; never sent by a sensor.
} MESSAGE_TYPE;

typedef struct {
    UW distance_cm;
    BOOL valid;
} SONAR_DISTANCE_MSG;

typedef struct {
    MESSAGE_TYPE type;
    union {
        IR_STATE ir_state;
        B decoded_char;
        UW obstacle_distance_cm;
        OBSTACLE_PROFILE obstacle_profile;
        SONAR_PROBE_RESULT sonar_probe;
        SONAR_DISTANCE_MSG sonar_distance;
        IMU_TELEMETRY_MSG imu_telemetry_msg;
        MOTOR_COMMAND remote_cmd;
        UH packed_command_mapping;
    } data;                     // Union data type for variable message data, update as project expands.
} NAVIGATOR_MESSAGE;

/* 
=========================================
|                Network                |
=========================================
*/

typedef struct {
    // Sequence / Tick counter
    uint32_t seq;

    // High Priority: Mode & Navigation
    uint8_t  movement_mode;     // MOVEMENT_MODE enum
    uint8_t  bypass_phase;      // BYPASS_PHASE enum
    uint8_t  ir_state;          // IR_STATE enum
    char     last_barcode;      // Decoded barcode char (or 0)
    uint8_t  instruction_idx;   // Current instruction index in mission route

    // Motion & Motors
    int16_t  speed_left;        // Target speed %
    int16_t  speed_right;
    int32_t  enc_l;             // Left encoder ticks
    int32_t  enc_r;             // Right encoder ticks

    // Sensor Specific: Ultrasonic
    uint32_t obstacle_distance_cm; // Direct front obstacle distance
    uint8_t  obstacle_distance_valid; // 1 = valid reading, 0 = no valid echo
    uint32_t closest_cm;        // Closest obstacle distance from scan
    int32_t  closest_angle_deg; // Closest obstacle angle
    uint32_t estimated_width_cm;// Obstacle estimated width

    // Sensor Specific: IMU
    float    pitch_deg;         // Vehicle pitch
    float    roll_deg;          // Vehicle roll
    uint8_t  imu_state;             // IMU_FSM_STATE enum
    float    estimated_height_mm;   // Peak/current hump height in mm
    uint8_t  imu_event;         // IMU_EVENT_TYPE enum
} TELEMETRY_SNAPSHOT;

IMPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot;

IMPORT void navigator_task(INT stacd, void *exinf);

#endif
