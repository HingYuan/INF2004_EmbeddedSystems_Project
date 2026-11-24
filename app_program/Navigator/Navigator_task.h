#ifndef NAVIGATOR_TASK_H
#define NAVIGATOR_TASK_H

#include <tk/tkernel.h>
#include "../robot_tasks.h"
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
#define FORWARD_SPEED           16
// #define FORWARD_SPEED_BARCODE   30
#define FORWARD_SPEED_HUMP      30

// Stop / brake
#define STOP_SPEED              0

// Smooth differential steering for line following (no sudden jerks)
#define LEFT_TURN_L_BASE        10
#define LEFT_TURN_R_BASE        30
#define RIGHT_TURN_L_BASE       30
#define RIGHT_TURN_R_BASE       10

// #define LEFT_TURN_L_BARCODE     18
// #define LEFT_TURN_R_BARCODE     28
// #define RIGHT_TURN_L_BARCODE    28
// #define RIGHT_TURN_R_BARCODE    18

// Turning w/ max rampup (keep inner wheel moving at 15% instead of locking to 0!)
#define LEFT_TURN_L_RAMP        5
#define LEFT_TURN_R_RAMP        40
#define RIGHT_TURN_L_RAMP       40
#define RIGHT_TURN_R_RAMP       5

// pivoting / turning on the spot
#define LEFT_PIVOT_L            23
#define LEFT_PIVOT_R            23
#define RIGHT_PIVOT_L           23
#define RIGHT_PIVOT_R           23

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

typedef struct __attribute__((packed)) {
    UW closest_cm : 9;           // 0..400 cm, zero means no measured echo.
    W closest_angle_deg : 7;     // -45..45 degrees.
    UW front_cm : 9;
    UW left_scan_cm : 9;
    UW right_scan_cm : 9;
    UW left_scan_status : 3;
    UW right_scan_status : 3;
    W left_edge_deg : 7;
    W right_edge_deg : 7;
    UW estimated_width_cm : 10;  // Up to 800 cm between two measured points.
    UW left_offset_cm : 9;       // Up to 411 cm including body/gap allowance.
    UW right_offset_cm : 9;
    UW left_clearance_cm : 9;
    UW right_clearance_cm : 9;
    UW left_edge_valid : 1;
    UW right_edge_valid : 1;
    UW width_valid : 1;
    UW left_offset_valid : 1;
    UW right_offset_valid : 1;
    UW left_clearance_valid : 1;
    UW right_clearance_valid : 1;
    UW profile_valid : 1;
    UB rays[(SONAR_PROFILE_RAYS * 12U + 7U) / 8U]; // Each ray: 9 distance + 3 status bits.
} OBSTACLE_PROFILE;

static inline UW sonar_ray_cm(const OBSTACLE_PROFILE *p, UINT ray)
{
    return robot_bits_get(p->rays, ray * 12U, 9U);
}
static inline SONAR_STATUS sonar_ray_status(const OBSTACLE_PROFILE *p, UINT ray)
{
    return (SONAR_STATUS)robot_bits_get(p->rays, ray * 12U + 9U, 3U);
}
static inline void sonar_ray_set(OBSTACLE_PROFILE *p, UINT ray, UW cm, SONAR_STATUS status)
{
    robot_bits_set(p->rays, ray * 12U, 12U, (cm & 511U) | ((UW)status << 9));
}

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
#define BYPASS_DEPTH_CLEAR_MM         70U // One extra forward move parallel to the box after the return check is clear.
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
typedef struct __attribute__((packed)) {
    UW phase : 5;           // Current step of the rectangular detour.
    W side : 2;                     // +1: pass left, sonar right; -1: pass right, sonar left. Never zero once moving.
    UW phase_started_us;          // Start of the current movement.
    UW deadline_us;               // Fault deadline, never evidence that movement completed.
    UW requested_us;              // Time side tracking was requested; older replies are discarded.
    UW last_tracking_us;          // Last fresh reply, including no echo.
    UW last_sample_us;            // Reject duplicate samples.
    OBSTACLE_PROFILE first_scan;  // Compare the second scan without pretending missing ranges are distances.
    UW front_cm : 9;                  // Measured standoff at the restored starting position.
    UW planned_shift_mm : 13;          // Confirmed edge + original width allowance, or the first outward search step.
    UW shift_mm : 11;                  // Completed lateral travel, including outward extensions.
    UW forward_mm : 12;                // Completed travel parallel to the original track.
    UW leg_mm : 12;                    // Distance of the one outstanding translation.
    INT leg_start_mm;             // Motor odometry baseline; include coast after the commanded distance.
    UW probe_clear_count : 2;
    UW probe_attempts : 2;
    UW width_clearance_pending : 1; // Extra 6 cm is in progress; the following WIDTH_ALIGN skips another width check.
    UW depth_clearance_pending : 1; // Extra 5 cm is in progress; the following TURN_IN starts FIND_LINE without rechecking.
    UW tracking_active : 1;         // Watchdog applies only while side tracking was requested.
    UW move_pending : 1;            // One API call at a time: Motor's completion message has no ID.
    UW black_count : 2;               // Consecutive IR readings containing black during FIND_LINE.
    UW owns_obstacle_stop : 1;      // Release the scan stop only; never release a later hardware fault.
} BYPASS_CONTEXT;

typedef struct __attribute__((packed)) {
    UW sampled_at_us;           // Full wrapping microsecond timestamp is required for freshness.
    UH distance_cm : 9;         // 0..400 cm.
    B angle_deg : 7;            // -45..45 degrees.
    UW valid : 1;
    UW status : 3;
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

typedef struct __attribute__((packed)) {
    UW event : 2;
    H pitch_ddeg : 12;          // -1800..1800, tenths of a degree; both tilt axes retained.
    H roll_ddeg : 12;
    UH peak_mg : 12;            // 0..4000 mg.
    UH height_dmm : 10;         // 0..680, tenths of a millimetre (68 mm chassis model).
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

typedef struct __attribute__((packed)) {
    UH distance_cm : 9;
    UW valid : 1;
    UW status : 3;
} SONAR_DISTANCE_MSG;

typedef struct __attribute__((packed)) {
    UW type : 4;                // MESSAGE_TYPE 0..12.
    union __attribute__((packed)) {
        struct { UW ir_state : 3; };
        struct { UW decoded_char : 7; }; // ASCII, includes unknown '?'.
        struct { UH obstacle_distance_cm : 9; };
        OBSTACLE_PROFILE obstacle_profile;
        SONAR_PROBE_RESULT sonar_probe;
        SONAR_DISTANCE_MSG sonar_distance;
        IMU_TELEMETRY_MSG imu_telemetry_msg;
        MOTOR_COMMAND remote_cmd;
        struct { UH packed_command_mapping : 10; }; // ASCII 7 + direction 3.
    } data;
} NAVIGATOR_MESSAGE;

/* 
=========================================
|                Network                |
=========================================
*/

typedef struct __attribute__((packed)) {
    UW seq;                    // Wrapping sequence counter, full 32-bit domain.
    W enc_l, enc_r;            // Signed lifetime tick counters; no artificial route limit.
    W dist_cm;                 // Existing signed odometry domain.
    B speed_left, speed_right; // -100..100%, 8 bits each.
    UH obstacle_distance_cm : 9;
    UH closest_cm : 9;
    H closest_angle_deg : 7;
    UH estimated_width_cm : 10;
    H pitch_ddeg : 12;
    H roll_ddeg : 12;
    UH heading_deg : 9;        // 0..359.
    UH height_dmm : 10;
    UW movement_mode : 4;      // 0..10.
    UW bypass_phase : 5;       // Preserve phase numbers 0..27.
    UW ir_state : 3;           // 0..4, includes STATE_NULL.
    UW last_barcode : 7;       // ASCII or zero.
    UW instruction_idx : 3;    // 0..5, includes end of route.
    UW obstacle_distance_valid : 1;
    UW obstacle_status : 3;
    UW imu_state : 2;
    UW imu_event : 2;
} TELEMETRY_SNAPSHOT;

/* Writers hold robot_data_lock only around field stores. Readers copy under it. */
IMPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot;
IMPORT void telemetry_read(TELEMETRY_SNAPSHOT *snapshot);
IMPORT UB telemetry_imu_event(void);
IMPORT H telemetry_heading(void);

typedef struct __attribute__((packed)) {
    UW m_mode : 4;
    UW prev_ir_state : 3;
    UW reentry : 1;
    UW reentry_dir : 3;
    UW instruction_idx : 3;
    UW instruction_idx_barcode : 3;
    UW mapping_increment : 3;
    W num_rotations : 3;         // -1..2 while completing a turn.
    UW hump_pop : 1;
    UW spam_mitigation : 4;      // 0..11, resets after exceeding 10.
    UW scan_requested : 1;
    UW live_sonar_distance_valid : 1;
    UW live_sonar_status : 3;
    UW turning_rampup_count : 6; // 0..51 (51 means ramp already sent).
    UW last_decoded_char : 7;
    UW last_obstacle_dist_cm : 9;
    UW last_closest_cm : 9;
    W last_closest_angle_deg : 7;
    UW last_estimated_width_cm : 10;
    UW live_sonar_distance_cm : 9;
} NAVIGATOR_STATE;

/* Five route directions occupy 15 bits. Out-of-range reads mean no instruction. */
typedef struct { UB bits[(NUMBER_OF_COMMANDS * 3U + 7U) / 8U]; } PACKED_ROUTE;
static inline DIRECTION route_get(const PACKED_ROUTE *route, UINT index)
{
    return index < NUMBER_OF_COMMANDS ? (DIRECTION)robot_bits_get(route->bits, index * 3U, 3U) : DIR_INVALID;
}
static inline void route_set(PACKED_ROUTE *route, UINT index, DIRECTION direction)
{
    if (index < NUMBER_OF_COMMANDS && direction <= DIR_INVALID)
        robot_bits_set(route->bits, index * 3U, 3U, direction);
}

_Static_assert(sizeof(OBSTACLE_PROFILE) == 26, "obstacle profile packing");
_Static_assert(sizeof(NAVIGATOR_MESSAGE) == 27, "navigator envelope packing");
_Static_assert(sizeof(TELEMETRY_SNAPSHOT) == 32, "telemetry packing");
_Static_assert(BYPASS_MAX_SIDE_MM <= 2047U && BYPASS_MAX_FORWARD_MM <= 4095U, "bypass travel bit widths");
_Static_assert(NUMBER_OF_COMMANDS <= 7 && TURNING_RAMPUP_LIMIT < 63, "navigator counter bit widths");
_Static_assert(sizeof(MOTOR_COMMAND) == 4, "motor command ABI");
_Static_assert(sizeof(SONAR_DISTANCE_MSG) == 2, "sonar distance packing");
_Static_assert(sizeof(SONAR_PROBE_RESULT) == 7, "sonar probe packing");
_Static_assert(sizeof(IMU_TELEMETRY_MSG) == 6, "IMU message packing");

IMPORT void navigator_task(INT stacd, void *exinf);

#endif
