#ifndef NAVIGATOR_TASK_H
#define NAVIGATOR_TASK_H

#include <tk/tkernel.h>
#include "../Motor/Motor_task.h"

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
    BYPASS_TURN_ALIGN,
    BYPASS_VERIFY_LINE,
    BYPASS_COMPLETE,
    BYPASS_ABORT
} BYPASS_PHASE;

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
    float estimated_height_mm;
} IMU_TELEMETRY_MSG;

/* 
=========================================
|               Navigator               |
=========================================
*/

#define NUMBER_OF_COMMANDS 2

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
    MSG_SONAR_DISTANCE
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
    int32_t  dist_cm;           // Average distance travelled in cm

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