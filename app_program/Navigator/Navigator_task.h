#ifndef NAVIGATOR_TASK_H
#define NAVIGATOR_TASK_H

#include <tk/tkernel.h>

typedef enum {
    MODE_LINE_FOLLOWING,        // IR is aware of line and navigator is following it
    MODE_DECISION_MAKING,       // Car stops at intersection, need to refer to next instruction
    MODE_WALL_HUGGING,          // Ultrasonic sensor is aware of obstacle and navigtor is navigating around it
    MODE_REENTRY_ADVANCE,       // Navigator moves car forward onto black line after MODE_REENTRY_DETECT to position itself for pivot
    MODE_REENTRY_PIVOT,         // Navigator spins the car in place to align with line
    MODE_OBSTACLE_ASSESS,  
    MODE_SAFE_STOP              
} MOVEMENT_MODE;

typedef enum {
    STATE_W_W,
    STATE_W_B,
    STATE_B_W,
    STATE_B_B,
    STATE_NULL,
} IR_STATE;

typedef enum {
    DIR_FRONT,
    DIR_BACK,
    DIR_LEFT,
    DIR_RIGHT,
} DIRECTION;

typedef enum {
    MSG_IR_UPDATE,              // Line following sensor update, car veering off
    MSG_BARCODE_DECODED,        // Barcode scanning sensor decoded letter
    MSG_OBSTACLE_DETECTED,      // Ultrasonic sensor detect obstacle
    MSG_MOTOR_STOPPED,          
    MSG_OBSTACLE_PROFILED,      // Left/center/right scan completed
    MSG_TRACKING_OBSTACLE,
} MESSAGE_TYPE;

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

typedef enum {                  // Movement phases within wall hugging
    BYPASS_TURN_OUT,                
    BYPASS_FOLLOW_SIDE,
    BYPASS_VERIFY_EDGE,
    BYPASS_CLEAR_CORNER
} BYPASS_PHASE;

typedef struct {
    MESSAGE_TYPE type;
    union {
        IR_STATE ir_state;
        B decoded_char;
        UW obstacle_distance_cm;
        OBSTACLE_PROFILE obstacle_profile;
    } data;                     // Union data type for variable message data, update as project expands.
} NAVIGATOR_MESSAGE;

IMPORT void navigator_task(INT stacd, void *exinf);

#endif