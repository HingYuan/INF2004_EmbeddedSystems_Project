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
    UW front_cm;              // Front standoff used to calculate travel past the box.
    UW left_scan_cm;          // Measured +45-degree ray; meaningful only with SONAR_OK.
    UW right_scan_cm;         // Measured -45-degree ray; meaningful only with SONAR_OK.
    SONAR_STATUS left_scan_status;
    SONAR_STATUS right_scan_status;

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
 * Simple rectangular bypass:
 * IDLE -> TURN_OUT (90 degrees) -> SHIFT_OUT (sideways) -> TURN_PARALLEL (90 degrees back)
 *      -> ADVANCE (past the whole box) -> TURN_IN (90 degrees toward line) -> FIND_LINE -> LINE_FOUND.
 * Keep the original numeric IDs for telemetry. The old diagonal/reverse/acquisition phases are unused.
 */
typedef enum {
    BYPASS_IDLE = 0,          // Wait for the initial stop acknowledgement and left/right scan.
    BYPASS_TURN_OUT = 1,      // Turn 90 degrees toward the chosen side; do not wait for a tracking reply.
    BYPASS_SHIFT_OUT = 3,     // Drive sideways far enough for the measured box width and turning body.
    BYPASS_TURN_PARALLEL = 4, // Turn 90 degrees back to face along the original track.
    BYPASS_ADVANCE = 7,       // Drive straight past the box; only steer outward if the side gets too close.
    BYPASS_TURN_IN = 10,      // Turn 90 degrees toward the black line after the rear has cleared.
    BYPASS_FIND_LINE = 12,    // Drive toward the line until two consecutive IR readings contain black.
    BYPASS_LINE_FOUND = 13,   // Stop and hand over to the teammate's MODE_REENTRY_ADVANCE.
    BYPASS_ABORT = 17         // Hardware or communication fault; never silently advance to the next move.
} BYPASS_PHASE;

#define BYPASS_SPEED                 30  // Keep the current forward power setting.
#define BYPASS_PIVOT_SPEED           35  // Keep the current pivot power setting.
// shortcut: these are timed moves; verify travel time at power 30 and both 90-degree turns on the car.
#define BYPASS_MS_PER_CM            110U // Starting estimate: old 80 ms/cm at power 40, scaled for power 30.
#define BYPASS_LEFT_90_MS           800U // User's starting value; calibrate left and right independently.
#define BYPASS_RIGHT_90_MS          800U
// shortcut: this route assumes the measured box size below; update it before using a larger obstacle.
#define BYPASS_OBSTACLE_WIDTH_CM     15U // Measured box width across the track.
#define BYPASS_OBSTACLE_LENGTH_CM    10U // Measured box depth along the track.
#define BYPASS_PIVOT_FROM_FRONT_CM   10U // Front sensor/body edge to the pivot between the driven wheels.
#define BYPASS_SWEPT_RADIUS_CM       12U // Covers the 12 x 20 cm body while it turns.
#define BYPASS_SIDE_CM (BYPASS_OBSTACLE_WIDTH_CM + BYPASS_SWEPT_RADIUS_CM + WALL_BODY_GAP_CM)
// Use the full box width above, so an obstacle off-centre on the line still has clearance.
#define BYPASS_CLOSE_CM             12U // Too close to the parked +/-45 degree sonar: steer outward.
#define BYPASS_STEER_LIMIT           10  // Outward correction around BYPASS_SPEED; no inward wall search.
#define BYPASS_TRACK_STALE_MS     1000U // Missing messages are a fault; a no-echo message is still a reply.
#define BYPASS_RESPONSE_MS        1000U // Wait limit for the initial stop acknowledgement.
#define BYPASS_SCAN_MS           15000U // Wait limit for the stationary scan.
#define BYPASS_POLL_MS              20U // Check movement deadlines even if no sensor message arrives.

// Navigator owns these values. All *_us fields use microseconds.
typedef struct {
    BYPASS_PHASE phase;           // Current step of the rectangular detour.
    INT side;                     // +1: pass left, sonar right; -1: pass right, sonar left. Never zero once moving.
    UW phase_started_us;          // Start of the current movement.
    UW deadline_us;               // End of a timed movement; zero means wait for line detection.
    UW requested_us;              // Time side tracking was requested; older replies are discarded.
    UW last_tracking_us;          // Last fresh reply, including no echo.
    UW last_sample_us;            // Reject duplicate samples.
    UW shift_us;                  // Duration of the sideways drive.
    UW forward_us;                // Front standoff + pivot offset + box depth + turning radius + body gap.
    UW black_count;               // Consecutive IR readings containing black during FIND_LINE.
    BOOL owns_obstacle_stop;      // Release the scan stop only; never release a later hardware fault.
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
    MODE_WALL_HUGGING,              // Run the rectangular bypass and return to the line.
    MODE_REENTRY_ADVANCE,           // Teammate's code: advance onto the line after the bypass handoff.
    MODE_REENTRY_PIVOT,             // Teammate's code: pivot to align with the line.
    MODE_OBSTACLE_ASSESS,           // Stop at 30 cm, scan left/right, choose a side, then start the 90-degree turn.
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
