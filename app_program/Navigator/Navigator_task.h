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
#define FORWARD_SPEED           50
#define FORWARD_SPEED_BARCODE   30

// Stop / brake
#define STOP_SPEED              0

// turning (maintain forward speed)
#define LEFT_TURN_L_BASE        20
#define LEFT_TURN_R_BASE        40
#define RIGHT_TURN_L_BASE       40 
#define RIGHT_TURN_R_BASE       20

// turning w/ max rampup
#define LEFT_TURN_L_RAMP        25
#define LEFT_TURN_R_RAMP        50
#define RIGHT_TURN_L_RAMP       50
#define RIGHT_TURN_R_RAMP       25

// pivoting / turning on the spot
#define LEFT_PIVOT_L            35
#define LEFT_PIVOT_R            35
#define RIGHT_PIVOT_L           35
#define RIGHT_PIVOT_R           35

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

#define TURNING_RAMPUP_LIMIT 20

/* 
=========================================
|               Ultrasonic              |
=========================================
*/

typedef struct {
    UW closest_cm;
    W closest_angle_deg;       // Positive angle is left

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

// Starting estimates for bench testing: tune at speed 30 / pivot speed 35.
// shortcut: timed travel varies with battery and grip, recalibrate before course runs.
#define BYPASS_MS_PER_CM            80U  // Time per cm at speed 30; increase if travel is too short, decrease if too far.
#define BYPASS_LEFT_90_MS          700U  // Left 90-degree pivot time; increase if under-turning, decrease if over-turning.
#define BYPASS_RIGHT_90_MS         700U  // Right 90-degree pivot time; tune separately from the left turn.
#define BYPASS_STOP_DISTANCE_CM      3U  // Must cover measured forward travel after requesting a stop (cm).
#define BYPASS_PIVOT_FROM_FRONT_CM  10U  // Front of car to midpoint between driven wheels (cm); measure on your car.
#define BYPASS_PEEK_DEG              5U  // Small stationary pivot to look around a corner; allowed range is 1..5 degrees.
#define BYPASS_SEARCH_STEP_CM        5U  // Maximum forward distance per timed corner-search step (cm).
#define BYPASS_FRONT_INTERVAL_MS   500U  // Maximum driving time between stopped front-distance checks (ms).
#define BYPASS_RESPONSE_MS        1000U  // Maximum wait for a motor-stop acknowledgement or a probe/tracking reply (ms).
#define BYPASS_TRACK_STALE_MS      300U  // Stop if the latest wall-tracking reading is this old (ms).
#define BYPASS_SCAN_MS           10000U  // Maximum wait for the initial obstacle profile (ms).
#define BYPASS_TOTAL_MS          60000U  // Whole bypass timeout, including initial scanning and scan stops (ms).
#define BYPASS_RETURN_MS         12000U  // Line-search timeout after turning inward, including scan stops (ms).
#define BYPASS_SCAN_ATTEMPTS         3U  // Probe attempts per check before corner search or a safe stop.
#define BYPASS_WALL_TARGET_CM (((ROBOT_HALF_WIDTH_CM + WALL_BODY_GAP_CM) * 1414U + 999U) / 1000U) // Calculated 45-degree sonar target (~16 cm); adjust WALL_BODY_GAP_CM instead.
#define BYPASS_WALL_DEADBAND_CM      2   // Ignore distance errors within +/-2 cm of the diagonal sonar target.
#define BYPASS_STEER_LIMIT          10   // Cap each wheel's speed adjustment at +/-10 around forward speed 30.
#define BYPASS_EDGE_JUMP_CM         10U  // A reading at least this far above the target suggests a possible wall end (cm).

// Bypass movement phases
typedef enum {
    BYPASS_IDLE,
    BYPASS_TURN_OUT,
    BYPASS_CHECK_OUT,
    BYPASS_SHIFT_OUT,
    BYPASS_TURN_PARALLEL,
    BYPASS_CHECK_SIDE,
    BYPASS_CHECK_FORWARD,
    BYPASS_ADVANCE,
    BYPASS_CHECK_CLEAR,
    BYPASS_CLEAR_CORNER,
    BYPASS_TURN_IN,
    BYPASS_CHECK_RETURN,
    BYPASS_FIND_LINE,
    BYPASS_LINE_FOUND,
    BYPASS_COMPLETE = 16,      // Preserve existing telemetry IDs.
    BYPASS_ABORT,
    BYPASS_PEEK_OUT,
    BYPASS_CHECK_PEEK,
    BYPASS_PEEK_BACK
} BYPASS_PHASE;

typedef enum {
    BYPASS_WAIT_NONE, BYPASS_WAIT_STOP, BYPASS_WAIT_SCAN,
    BYPASS_WAIT_PROBE, BYPASS_WAIT_TRACK, BYPASS_WAIT_MOVE
} BYPASS_WAIT;

typedef struct {
    BYPASS_PHASE phase;
    BYPASS_WAIT wait;
    MOTOR_COMMAND command;
    INT side;                    // +1: pass left, obstacle on right; -1: mirrored
    UW started_us;
    UW deadline_us;
    UW requested_us;
    UW last_tracking_us;
    UW return_started_us;
    UW last_ir_us;
    UW shift_remaining_ms;
    UW corner_remaining_ms;
    UW segment_ms;
    UW front_budget_ms;
    UW wall_cm;
    UW attempts;
    UW black_count;
    UINT probe_flag;
    W probe_angle;
    BOOL wall_seen;
    BOOL corner_active;
} BYPASS_CONTEXT;

typedef struct {
    UW distance_cm;
    W angle_deg;
    BOOL valid;
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
    float estimated_height_cm;
} IMU_TELEMETRY_MSG;

/* 
=========================================
|               Navigator               |
=========================================
*/

#define NUMBER_OF_COMMANDS 4

typedef enum {
    MODE_BARCODE_SCANNING,          // Slow speed barcode scanning
    MODE_WIFI_SEND_COMMS,           // Wifi sending command mappings over to navigator
    MODE_LINE_FOLLOWING,            // IR is aware of line and navigator is following it
    MODE_GOING_OVER_HUMP,           // Traversing over hump
    MODE_DECISION_MAKING_ADVANCE,   // Car advances a bit after an intersection to center wheels on intersection
    MODE_DECISION_MAKING_TURN,      // Car stops at intersection, need to refer to next instruction, rotate
    MODE_WALL_HUGGING,              // Ultrasonic sensor is aware of obstacle and navigtor is navigating around it
    MODE_REENTRY_ADVANCE,           // Navigator moves car forward onto black line after MODE_REENTRY_DETECT to position itself for pivot
    MODE_REENTRY_PIVOT,             // Navigator spins the car in place to align with line
    MODE_OBSTACLE_ASSESS,  
    MODE_SAFE_STOP              
} MOVEMENT_MODE;

typedef enum {
    DIR_FRONT,
    DIR_BACK,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_INVALID,
} DIRECTION;

typedef enum {
    MSG_WIFI_BARCODE_MAPPING,   // Wifi sends navigator a command mapping (example 'A' = turn left)
    MSG_IR_UPDATE,              // Line following sensor update, car veering off
    MSG_BARCODE_DECODED,        // Barcode scanning sensor decoded letter
    MSG_OBSTACLE_DETECTED,      // Ultrasonic sensor detect obstacle
    MSG_MOTOR_STOPPED,          
    MSG_OBSTACLE_PROFILED,      // Left/center/right scan completed
    MSG_TRACKING_OBSTACLE,     // data.sonar_probe at the parked +/-45 degree angle
    MSG_SONAR_PROBED,
    MSG_MOTOR_MOVE_DONE,
    MSG_IMU_UPDATE,
    MSG_REMOTE_CMD,             // Remote motor command from MQTT
    MSG_SONAR_DISTANCE
} MESSAGE_TYPE;

typedef struct {
    UW distance_cm;
    BOOL valid;
} SONAR_DISTANCE_MSG;

typedef struct {
    MESSAGE_TYPE type;
    UW sampled_at_us;          // time_us(): populated by IR and sonar producers
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
    uint8_t  imu_event;         // IMU_EVENT_TYPE enum
} TELEMETRY_SNAPSHOT;

IMPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot;

IMPORT void navigator_task(INT stacd, void *exinf);

#endif
