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
#define FORWARD_SPEED           18
// #define FORWARD_SPEED_BARCODE   30
#define FORWARD_SPEED_HUMP      30

// Stop / brake
#define STOP_SPEED              0

// Smooth differential steering for line following (no sudden jerks)
#define LEFT_TURN_L_BASE        0
#define LEFT_TURN_R_BASE        20
#define RIGHT_TURN_L_BASE       20
#define RIGHT_TURN_R_BASE       0

// #define LEFT_TURN_L_BARCODE     18
// #define LEFT_TURN_R_BARCODE     28
// #define RIGHT_TURN_L_BARCODE    28
// #define RIGHT_TURN_R_BARCODE    18

// Turning w/ max rampup (keep inner wheel moving at 15% instead of locking to 0!)
#define LEFT_TURN_L_RAMP        0
#define LEFT_TURN_R_RAMP        40
#define RIGHT_TURN_L_RAMP       40
#define RIGHT_TURN_R_RAMP       0

// pivoting / turning on the spot
#define LEFT_PIVOT_L            25
#define LEFT_PIVOT_R            25
#define RIGHT_PIVOT_L           25
#define RIGHT_PIVOT_R           25

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

#define TURNING_RAMPUP_LIMIT 50

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
    UW ray_cm[SONAR_PROFILE_RAYS]; // Preserve missing echoes separately from invalid measurements.
    SONAR_STATUS ray_status[SONAR_PROFILE_RAYS];

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
 * Scan -> reverse 2 cm -> scan -> restore 2 cm -> choose one side.
 * TURN_OUT -> SHIFT_OUT -> TURN_PARALLEL -> WIDTH_CHECK; extend sideways if blocked.
 * Once clear, move outward another 6 cm once, turn parallel, then ADVANCE without another width scan.
 * ADVANCE to the front face, then check the return path in forward steps.
 * If the return path is blocked, turn parallel and step forward 15 cm.
 * Once clear, turn parallel, add 5 cm once, then turn inward and search for the line without another depth check.
 * Existing phase numbers are retained for telemetry; new phases are appended.
 */
typedef enum {
    BYPASS_IDLE = 0,          // Wait for the initial stop acknowledgement and first scan.
    BYPASS_TURN_OUT = 1,      // Turn 90 degrees toward the chosen side; do not wait for a tracking reply.
    BYPASS_SHIFT_OUT = 3,     // Drive a bounded edge offset, or start a short outward search.
    BYPASS_TURN_PARALLEL = 4, // Turn 90 degrees back to face along the original track.
    BYPASS_ADVANCE = 7,       // Approach/search the depth, or perform the final 5 cm forward clearance move.
    BYPASS_TURN_IN = 10,      // Face the line to check depth; after the extra 5 cm, start the return directly.
    BYPASS_FIND_LINE = 12,    // Drive toward the line until two consecutive IR readings contain black.
    BYPASS_LINE_FOUND = 13,   // Stop and hand over to the teammate's MODE_REENTRY_ADVANCE.
    BYPASS_ABORT = 17,        // Hardware or communication fault; never silently advance to the next move.
    BYPASS_WAIT_RELEASE = 18, // Wait until Motor releases the initial stop before the 2 cm scan reverse.
    BYPASS_SCAN_BACK = 19,    // The only reverse: 2 cm to obtain a second viewing position.
    BYPASS_SCAN_SECOND = 20,  // Stationary second profile; never reverse again for missing echoes.
    BYPASS_SCAN_RETURN = 21,  // Restore the original position before choosing and entering the detour.
    BYPASS_WIDTH_CHECK = 22,  // Two clear forward probes start the one-time 6 cm outward clearance move.
    BYPASS_WIDTH_TURN_OUT = 23,
    BYPASS_WIDTH_EXTEND = 24, // Search outward 15 cm if blocked; add 6 cm once after the front is clear.
    BYPASS_WIDTH_ALIGN = 25,  // Recheck after a search step; start ADVANCE after the extra 6 cm.
    BYPASS_DEPTH_TURN_BACK = 26, // Face parallel for a 15 cm search step or the final 5 cm clearance move.
    BYPASS_RETURN_CHECK = 27  // Two clear front probes while facing the line start the one-time 5 cm depth allowance.
} BYPASS_PHASE;

#define BYPASS_API_POWER             25 // Telemetry only: the existing motor functions choose their own power.
#define BYPASS_LEFT_TURN_DEG         90U // Argument to turnLeft(); tune here if the physical turn differs.
#define BYPASS_RIGHT_TURN_DEG        90U // Argument to turnRight(); calibrate each direction independently.
#define BYPASS_TURN_TIMEOUT_MS     5000U // Fault limits only; encoders finish each movement.
#define BYPASS_MOVE_MARGIN_MS      3000U
#define BYPASS_MOVE_TIMEOUT_MS_CM   500U
#define BYPASS_SCAN_BACK_MM          20U
#define BYPASS_FORWARD_STEP_MM       150U
#define BYPASS_WIDTH_STEP_MM        150U
#define BYPASS_PIVOT_CLEAR_MM         70U // One extra outward move after the forward width check is clear.
#define BYPASS_DEPTH_CLEAR_MM         50U // One extra forward move parallel to the box after the return check is clear.
#define BYPASS_PROBE_MS            2000U
#define BYPASS_CLEAR_CONFIRM          2U // Each stationary probe already takes three sonar pings.
#define BYPASS_PROFILE_DEPTH_CM      10U // Reject background echoes beyond the front face.
// Exploration limits are fault bounds, not assumed obstacle dimensions.
#define BYPASS_MAX_SIDE_MM         1500U
#define BYPASS_MAX_FORWARD_MM      3000U
#define BYPASS_PIVOT_FROM_FRONT_CM   10U // Front sensor/body edge to the pivot between the driven wheels.
#define BYPASS_SWEPT_RADIUS_CM       12U // Covers the 12 x 20 cm body while it turns.
#define BYPASS_WIDTH_CLEAR_MM       100U // Original allowance added to a scan-measured edge before the width check.
#define BYPASS_TRACK_STALE_MS     1000U // Missing messages are a fault; a no-echo message is still a reply.
#define BYPASS_RESPONSE_MS        1000U // Wait limit for the initial stop acknowledgement.
#define BYPASS_SCAN_MS           15000U // Wait limit for the stationary scan.
#define BYPASS_POLL_MS              20U // Check movement deadlines even if no sensor message arrives.

// Navigator owns these values. All *_us fields use microseconds.
typedef struct {
    BYPASS_PHASE phase;           // Current step of the rectangular detour.
    INT side;                     // +1: pass left, sonar right; -1: pass right, sonar left. Never zero once moving.
    UW phase_started_us;          // Start of the current movement.
    UW deadline_us;               // Fault deadline, never evidence that movement completed.
    UW requested_us;              // Time side tracking was requested; older replies are discarded.
    UW last_tracking_us;          // Last fresh reply, including no echo.
    UW last_sample_us;            // Reject duplicate samples.
    OBSTACLE_PROFILE first_scan;  // Compare the second scan without pretending missing ranges are distances.
    UW front_cm;                  // Measured standoff at the restored starting position.
    UW planned_shift_mm;          // Confirmed edge + original width allowance, or the first outward search step.
    UW shift_mm;                  // Completed lateral travel, including outward extensions.
    UW forward_mm;                // Completed travel parallel to the original track.
    UW leg_mm;                    // Distance of the one outstanding translation.
    INT leg_start_mm;             // Motor odometry baseline; include coast after the commanded distance.
    UW probe_clear_count;
    UW probe_attempts;
    BOOL width_clearance_pending; // Extra 6 cm is in progress; the following WIDTH_ALIGN skips another width check.
    BOOL depth_clearance_pending; // Extra 5 cm is in progress; the following TURN_IN starts FIND_LINE without rechecking.
    BOOL tracking_active;         // Watchdog applies only while side tracking was requested.
    BOOL move_pending;            // One API call at a time: Motor's completion message has no ID.
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
    IMU_MSG_LIFTED_OVERTURNED,
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

// #define NUMBER_OF_COMMANDS 2
#define NUMBER_OF_COMMANDS 5

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
    MODE_OBSTACLE_ASSESS,           // Stop at OBSTACLE_TRIGGER_CM, scan, choose a side, then turn.
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
    MSG_MOTOR_MOVE_DONE,        // Existing Motor notification; no payload or command ID.
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
    int32_t  dist_cm;           // Average distance travelled in cm

    // Sensor Specific: Ultrasonic
    uint32_t obstacle_distance_cm; // Direct front obstacle distance
    uint8_t  obstacle_distance_valid; // 1 = valid reading, 0 = no valid echo
    uint8_t  obstacle_status;       // SONAR_STATUS enum
    uint32_t closest_cm;        // Closest obstacle distance from scan
    int32_t  closest_angle_deg; // Closest obstacle angle
    uint32_t estimated_width_cm;// Obstacle estimated width

    // Sensor Specific: IMU
    float    pitch_deg;         // Vehicle pitch
    float    roll_deg;          // Vehicle roll
    int16_t  heading_deg;       // Magnetometer compass heading (0-359 deg)
    uint8_t  imu_state;             // IMU_FSM_STATE enum
    float    estimated_height_mm;   // Peak/current hump height in mm
    uint8_t  imu_event;         // IMU_EVENT_TYPE enum
} TELEMETRY_SNAPSHOT;

IMPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot;

IMPORT void navigator_task(INT stacd, void *exinf);

#endif
