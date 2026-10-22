#ifndef NAVIGATOR_TASK_H
#define NAVIGATOR_TASK_H

#include <tk/tkernel.h>

typedef enum {
    MODE_LINE_FOLLOWING,        // IR is aware of line and navigator is following it
    MODE_DECISION_MAKING,       // Car stops at intersection, need to refer to next instruction
    MODE_WALL_HUGGING,          // Ultrasonic sensor is aware of obstacle and navigtor is navigating around it
    MODE_REENTRY_ADVANCE,       // Navigator moves car forward onto black line after MODE_REENTRY_DETECT to position itself for pivot
    MODE_REENTRY_PIVOT,         // Navigator spins the car in place to align with line
} MOVEMENT_MODE;

typedef enum {
    STATE_W_W,
    STATE_W_B,
    STATE_B_W,
    STATE_B_B,
    STATE_NULL,
} IR_STATE;

typedef enum {
    DIR_LEFT,
    DIR_RIGHT,
} DIRECTION;

typedef enum {
    MSG_IR_UPDATE,              // Line following sensor update, car veering off
    MSG_BARCODE_DECODED,        // Barcode scanning sensor decoded letter
    MSG_OBSTACLE_DETECTED,      // Ultrasonic sensor detect obstacle
    MSG_TRACKING_OBSTACLE,      // Reserved for future avoidance updates
    MSG_OBSTACLE_PROFILED,      // Left/center/right scan completed
} MESSAGE_TYPE;

typedef struct {
    UW left_cm;
    UW center_cm;
    UW right_cm;
    BOOL left_valid;
    BOOL center_valid;
    BOOL right_valid;
} OBSTACLE_PROFILE;

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