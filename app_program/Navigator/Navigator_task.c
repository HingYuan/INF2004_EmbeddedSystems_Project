#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"
#include "../Motor/Motor_task.h"
#include "../Ultrasonic/Ultrasonic_task.h"
#include "../IMU/IMU_task.h"
#include "../Network/mqtt_client.h"

// Shared telemetry snapshot instance owned by Navigator
EXPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot = {0};

// Private bypass helpers are implemented after navigator_task().
LOCAL INT bypass_choose_side(const OBSTACLE_PROFILE *profile);
LOCAL MOTOR_COMMAND bypass_pivot_command(INT direction);
LOCAL MOTOR_COMMAND bypass_forward_command(INT side, BOOL close);
#ifndef NDEBUG
LOCAL BOOL bypass_self_check(void);
#endif
LOCAL BOOL bypass_hardware_fault(void);
LOCAL void bypass_abort(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, const char *reason);
LOCAL void bypass_command(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, INT left, INT right);
LOCAL void bypass_begin(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT left, INT right, UW duration_us);
LOCAL void bypass_pivot(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT direction);

void clear_navigator_mbf() {
    NAVIGATOR_MESSAGE discard_msg;
    while (tk_rcv_mbf(navigator_mbfid, &discard_msg, TMO_POL) >= 0) { }
}

/*
Note for Bryan, rmb to remove mqtt_get_movement_for_barcode function from network code
In case you need it
curr_speed_left = mot_cmd.target_speed_left;
curr_speed_right = mot_cmd.target_speed_right;
}
*/

/*
CHANGE m_mode
CHANGE from instructions_fake to instructions
*/

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    // General
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;
    MOTOR_COMMAND mot_cmd = {0};
    // MOVEMENT_MODE m_mode = MODE_WIFI_SEND_COMMS;
    // MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;
    MOVEMENT_MODE m_mode = MODE_BARCODE_SCANNING;

    // Sensor specific telemetry cache
    char last_decoded_char = 0;
    UW last_obstacle_dist_cm = 0;
    UW last_closest_cm = 0;
    W last_closest_angle_deg = 0;
    UW last_estimated_width_cm = 0;
    float last_pitch_deg = 0.0f;
    float last_roll_deg = 0.0f;
    UB last_imu_event = 0;

    // Line following
    IR_STATE prev_ir_state = STATE_NULL;
    UH turning_rampup_count = 0;

    // Barcode decoding
    B instruction_idx = 0;                                                          // Iterate through instructions for each intersection
    B barcode_characters[NUMBER_OF_COMMANDS] = {};                                  // Character key, sent from wifi
    DIRECTION barcode_instructions[NUMBER_OF_COMMANDS] = {};                        // Instruction mapping, sent from wifi
    B mapping_increment = 0;                                                        // incrementing index for syncing barcode and instruction arrays
    DIRECTION instructions[NUMBER_OF_COMMANDS] = {};                                // Instructions scanned from barcodes
    DIRECTION instructions_fake[NUMBER_OF_COMMANDS] = { DIR_FRONT };
    for (int i=0; i<NUMBER_OF_COMMANDS; i++) instructions[i] = DIR_INVALID;         // Preset all instructions as DIR_INVALID

    // Decision making
    B num_rotations = 0;

    // Hump
    BOOL hump_pop = 0;
    UH spam_mitigation = 0;
    UH spam_mitigation_threshold = 10;

    // Obstacle
    BOOL scan_requested = FALSE;
    BYPASS_CONTEXT bypass = {0};  // bypass.side remains available to the reentry cases.

    // Unused
    INT left_debounce_ticks = 0;
    INT right_debounce_ticks = 0;

    // Motion tracking for telemetry
    INT curr_speed_left = 0;
    INT curr_speed_right = 0;
    UW live_sonar_distance_cm = 0;
    BOOL live_sonar_distance_valid = FALSE;
    W turn_start_ticks = 0;

    // 2s grace period to place car on track, then flush any stale startup sensor messages
    tk_dly_tsk(2000);
    clear_navigator_mbf();
#ifndef NDEBUG
    if (!bypass_self_check()) {
        motor_estop = TRUE;
        tm_printf((UB *)"[bypass] Self-check failed\n");
        return;
    }
#endif

    while (1) {
        // During bypass, wake every 20 ms so timed moves finish even if no IR message arrives.
        BOOL bypass_active = m_mode == MODE_OBSTACLE_ASSESS || m_mode == MODE_WALL_HUGGING;
        message_size = tk_rcv_mbf(navigator_mbfid, (void *)&n_msg,
                                  bypass_active ? (TMO)BYPASS_POLL_MS : TMO_FEVR);
        if (bypass_active && message_size == E_TMOUT) {
            n_msg = (NAVIGATOR_MESSAGE){.type = MSG_BYPASS_TICK};
            message_size = sizeof(n_msg);
        }
        if (bypass_active && (bypass_hardware_fault() ||
            (motor_estop && !bypass.owns_obstacle_stop) ||
            (message_size < E_OK && message_size != E_TMOUT))) {
            bypass_abort(&bypass, &mot_cmd, "hardware or Navigator receive fault");
            m_mode = MODE_SAFE_STOP;
            bypass_active = FALSE;
        }

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            if (n_msg.type == MSG_REMOTE_CMD && !bypass_active) {

                if(!motor_estop && m_mode != MODE_WALL_HUGGING && m_mode != MODE_SAFE_STOP) {
                    mot_cmd = n_msg.data.remote_cmd;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                }
            } else if (!bypass_active && n_msg.type == MSG_IMU_UPDATE &&
                      (n_msg.data.imu_telemetry_msg.event == IMU_MSG_SHOCK_DETECTED ||
                       n_msg.data.imu_telemetry_msg.event == IMU_MSG_TILT_WARNING)) {
                // motor_estop = TRUE;
                // m_mode = MODE_SAFE_STOP;
                // mot_cmd.target_speed_left = STOP_SPEED;
                // mot_cmd.target_speed_right = STOP_SPEED;
                // tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_POL);
                // tm_printf((UB *)"[nav] EMERGENCY STOP triggered by IMU event %d\n", n_msg.data.imu_telemetry_msg.event);
            } else if (!bypass_active && n_msg.type == MSG_SONAR_DISTANCE) {
                live_sonar_distance_valid = n_msg.data.sonar_distance.valid;
                
                if (live_sonar_distance_valid) {
                    live_sonar_distance_cm = n_msg.data.sonar_distance.distance_cm;
                }
            } else {
                switch (m_mode) {
                    case MODE_WIFI_SEND_COMMS:
                        instruction_idx = 0;

                        // Stop car
                        mot_cmd.target_speed_left =      STOP_SPEED;
                        mot_cmd.target_speed_right =     STOP_SPEED;
                        mot_cmd.turn_ticks_left =        0;
                        mot_cmd.turn_ticks_right =       0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                        switch (n_msg.type) {
                            case MSG_WIFI_BARCODE_MAPPING: {    // Note for bryan & future self: packed data must look like 0bxxxx xxxx yyyy yyyy, where x is direction and y is character.
                                DIRECTION direction = n_msg.data.packed_command_mapping >> 8;      // Cast int to enum
                                B code39_char = n_msg.data.packed_command_mapping & 0x00ff;        // bitwise & mask to get least-significant 8 bits

                                // Capitalize lowercase characters
                                if (code39_char >= 'a' && code39_char <= 'z') code39_char = code39_char - 32;
                                
                                //Sends temp log for debugging purposes, can be removed later ~ Bryan
                                tm_printf((UB *)"[nav] mapping received: %c -> %d\n",code39_char,direction);

                                barcode_characters[mapping_increment] = code39_char;
                                barcode_instructions[mapping_increment] = direction;
                                mapping_increment++;

                                if (mapping_increment >= NUMBER_OF_COMMANDS) {
                                    m_mode = MODE_BARCODE_SCANNING;
                                }

                                break;
                            }
                            default:
                                
                                break;
                        }

                        break;
                    case MODE_BARCODE_SCANNING:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:
                                switch (n_msg.data.ir_state) {
                                    case STATE_W_B:
                                        mot_cmd.target_speed_left =     RIGHT_TURN_L_BARCODE;
                                        mot_cmd.target_speed_right =   -RIGHT_TURN_R_BARCODE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    case STATE_B_W:
                                        mot_cmd.target_speed_left =     LEFT_TURN_L_BARCODE;
                                        mot_cmd.target_speed_right =   -LEFT_TURN_R_BARCODE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    default:
                                        mot_cmd.target_speed_left =     FORWARD_SPEED_BARCODE;
                                        mot_cmd.target_speed_right =   -FORWARD_SPEED_BARCODE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                }

                                break;
                            case MSG_BARCODE_DECODED:
                                last_decoded_char = (char)n_msg.data.decoded_char;

                                // Map detected character to instruction, then add to instruction array
                                for (B i=0; i<NUMBER_OF_COMMANDS; i++) {
                                    if (last_decoded_char == barcode_characters[i]) {
                                        instructions[instruction_idx] = barcode_instructions[i];
                                        instruction_idx++;
                                    }

                                    // If max instructions recorded
                                    if (instruction_idx >= NUMBER_OF_COMMANDS) {
                                        m_mode = MODE_LINE_FOLLOWING;
                                        instruction_idx = 0;

                                        mot_cmd.target_speed_left =     STOP_SPEED;
                                        mot_cmd.target_speed_right =   -STOP_SPEED;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        tk_dly_tsk(1000);
                                    }
                                }

                                break;
                        }

                        break;
                    case MODE_LINE_FOLLOWING:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE: {
                                IR_STATE curr_state = n_msg.data.ir_state;

                                    // tm_printf("Current state: %d VS previous state: %d\n", curr_state, prev_ir_state);
      
                                    if (curr_state != prev_ir_state) {

                                        if (curr_state == STATE_W_W) {
                                            turning_rampup_count = 0;

                                            mot_cmd.target_speed_left =     FORWARD_SPEED;
                                            mot_cmd.target_speed_right =   -FORWARD_SPEED;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        } else if (curr_state == STATE_W_B) {
                                            // Car veering left, adjust right
                                            mot_cmd.target_speed_left =     RIGHT_TURN_L_BASE;
                                            mot_cmd.target_speed_right =   -RIGHT_TURN_R_BASE;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        } else if (curr_state == STATE_B_W) {
                                            // Car veering right, adjust left
                                            mot_cmd.target_speed_left =     LEFT_TURN_L_BASE;
                                            mot_cmd.target_speed_right =   -LEFT_TURN_R_BASE;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        } else if (curr_state == STATE_B_B) {
                                            turning_rampup_count = 0;

                                            // Car at instersection
                                            mot_cmd.target_speed_left =     STOP_SPEED;
                                            mot_cmd.target_speed_right =   -STOP_SPEED;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            m_mode = MODE_DECISION_MAKING_ADVANCE;
                                        } else {
                                            // Errorneous state

                                        }

                                        // Update previous IR state
                                        prev_ir_state = curr_state;
                                    } else {
                                        // Ramp up turning if stuck on turning
                                        if (curr_state == STATE_B_W || curr_state == STATE_W_B) {
                                            if (turning_rampup_count < TURNING_RAMPUP_LIMIT) {
                                                turning_rampup_count++;
                                            }

                                            if (turning_rampup_count == TURNING_RAMPUP_LIMIT) {
                                                turning_rampup_count++;     

                                                mot_cmd.turn_ticks_left =   0;
                                                mot_cmd.turn_ticks_right =  0;
                                                
                                                if (curr_state == STATE_B_W) {
                                                    mot_cmd.target_speed_left =     LEFT_TURN_L_RAMP;
                                                    mot_cmd.target_speed_right =   -LEFT_TURN_R_RAMP;
                                                } else if (curr_state == STATE_W_B) {
                                                    mot_cmd.target_speed_left =     RIGHT_TURN_L_RAMP;
                                                    mot_cmd.target_speed_right =   -RIGHT_TURN_R_RAMP;
                                                }

                                                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                            }                                    
                                        }
                                    }

                                break;
                            }
                            case MSG_OBSTACLE_DETECTED:

                                last_obstacle_dist_cm = n_msg.data.obstacle_distance_cm;
                                // Each obstacle starts with a fresh side choice and confirmation counters.
                                bypass = (BYPASS_CONTEXT){.phase = BYPASS_IDLE,
                                    .owns_obstacle_stop = TRUE,
                                    .deadline_us = time_us() + BYPASS_RESPONSE_MS * 1000U};
                                scan_requested = FALSE;
                                mot_cmd = (MOTOR_COMMAND){0};
                                live_sonar_distance_valid = FALSE;
                                motor_stop_ack_requested = TRUE;
                                m_mode = MODE_OBSTACLE_ASSESS;

                                tm_printf((UB *)"[nav] Obstacle detected\n");

                                break;
                            case MSG_IMU_UPDATE:
                                if (n_msg.data.imu_telemetry_msg.event == IMU_MSG_HUMP_DETECTED) {
                                    m_mode = MODE_GOING_OVER_HUMP;
                                }

                                break;
                            default:

                                break;
                        }

                        break;

                    case MODE_GOING_OVER_HUMP: {
                        if (n_msg.type == MSG_IMU_UPDATE &&
                            n_msg.data.imu_telemetry_msg.event == IMU_MSG_NO_HUMP) {
                            prev_ir_state = STATE_NULL;
                            m_mode = MODE_LINE_FOLLOWING;
                        }
                        break;
                    }

                    case MODE_DECISION_MAKING_ADVANCE:
                        tk_dly_tsk(250);

                        mot_cmd.target_speed_left =     FORWARD_SPEED;
                        mot_cmd.target_speed_right =   -FORWARD_SPEED;
                        mot_cmd.turn_ticks_left =       0;
                        mot_cmd.turn_ticks_right =      0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                        tk_dly_tsk(600);

                        mot_cmd.target_speed_left =     0;
                        mot_cmd.target_speed_right =   -0;
                        mot_cmd.turn_ticks_left =       0;
                        mot_cmd.turn_ticks_right =      0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                        
                        m_mode = MODE_DECISION_MAKING_TURN;
                        clear_navigator_mbf();

                        tk_dly_tsk(30);     // wait for next IR update

                        break;

                    // Junction Navigation
                    case MODE_DECISION_MAKING_TURN:

                        switch (n_msg.type) {
                            case MSG_IR_UPDATE: {

                                IR_STATE curr_ir_state = n_msg.data.ir_state;
                                DIRECTION curr_decision = instructions[instruction_idx];
                                // DIRECTION curr_decision = instructions_fake[instruction_idx];

                                if (prev_ir_state == STATE_B_B) {       // should still be B_B after advance
                                    switch (curr_decision) {
                                        case DIR_BACK:
                                            num_rotations = 2;

                                            mot_cmd.target_speed_left =    -LEFT_PIVOT_L;
                                            mot_cmd.target_speed_right =   -RIGHT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_LEFT:
                                            num_rotations = 1;

                                            mot_cmd.target_speed_left =    -LEFT_PIVOT_L;
                                            mot_cmd.target_speed_right =   -LEFT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_RIGHT:
                                            num_rotations = 1;

                                            mot_cmd.target_speed_left =     RIGHT_PIVOT_L;
                                            mot_cmd.target_speed_right =    RIGHT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_STOP:
                                            num_rotations = 0;
                                           
                                        // DIR_FRONT
                                        default:
                                            num_rotations = 0;

                                            mot_cmd.target_speed_left =     FORWARD_SPEED;
                                            mot_cmd.target_speed_right =   -FORWARD_SPEED;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            prev_ir_state = STATE_NULL;

                                            m_mode = MODE_LINE_FOLLOWING;

                                            // Linger on last instruction on overflow
                                            if (++instruction_idx >= NUMBER_OF_COMMANDS) { instruction_idx = 0; }

                                            break;
                                    }

                                } else {
                                    if (curr_ir_state == STATE_B_W && prev_ir_state == STATE_W_W && (curr_decision == DIR_LEFT || curr_decision == DIR_BACK)) {
                                        num_rotations--;
                                    } else if (curr_ir_state == STATE_W_B && prev_ir_state == STATE_W_W && curr_decision == DIR_RIGHT) {
                                        num_rotations--;
                                    }
                                    
                                    if (num_rotations <= 0) {

                                        tk_dly_tsk(50);

                                        mot_cmd.target_speed_left =     STOP_SPEED;
                                        mot_cmd.target_speed_right =    STOP_SPEED;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        tk_dly_tsk(250);

                                        // Linger on last instruction on overflow
                                        if (++instruction_idx >= NUMBER_OF_COMMANDS) { instruction_idx = 0; }

                                        // Reset variables
                                        num_rotations = 0;
                                        prev_ir_state = STATE_NULL;
                                        m_mode = MODE_LINE_FOLLOWING;
                                    }
                                }

                                if (prev_ir_state != curr_ir_state) { prev_ir_state = curr_ir_state; }

                                break;
                            }
                            default:
                                
                                break;
                        }

                        break;
                    // Scan once; after choosing a side, keep the sonar facing the obstacle.
                    case MODE_OBSTACLE_ASSESS: {
                        switch (n_msg.type) {
                            case MSG_MOTOR_STOPPED:
                                // Scan only after the wheels have stopped.
                                if (!scan_requested && bypass.phase == BYPASS_IDLE) {
                                    if (tk_set_flg(ranger_flgid, SONAR_SCAN_REQUEST) < E_OK) {
                                        bypass_abort(&bypass, &mot_cmd, "scan request");
                                        break;
                                    }
                                    scan_requested = TRUE;
                                    bypass.deadline_us = time_us() + BYPASS_SCAN_MS * 1000U;
                                }
                                break;

                            case MSG_OBSTACLE_PROFILED: {
                                if (!scan_requested) break;
                                scan_requested = FALSE;
                                const OBSTACLE_PROFILE *p = &n_msg.data.obstacle_profile;
                                last_closest_cm = p->closest_cm;
                                last_closest_angle_deg = p->closest_angle_deg;
                                last_estimated_width_cm = p->width_valid ? p->estimated_width_cm : 0U;

                                // Compare the outer rays directly; missing profile edges cannot leave side at zero.
                                bypass.side = bypass_choose_side(p);
                                UW front_cm = OBSTACLE_TRIGGER_CM;
                                if (last_obstacle_dist_cm > front_cm && last_obstacle_dist_cm <= 400U)
                                    front_cm = last_obstacle_dist_cm;
                                if (p->front_cm > front_cm && p->front_cm <= 400U)
                                    front_cm = p->front_cm;
                                UW forward_cm = front_cm + BYPASS_PIVOT_FROM_FRONT_CM +
                                    BYPASS_OBSTACLE_LENGTH_CM + BYPASS_SWEPT_RADIUS_CM + WALL_BODY_GAP_CM;
                                bypass.shift_us = BYPASS_SIDE_CM * BYPASS_MS_PER_CM * 1000U;
                                bypass.forward_us = forward_cm * BYPASS_MS_PER_CM * 1000U;
                                bypass.requested_us = time_us();
                                bypass.last_tracking_us = bypass.last_sample_us = bypass.requested_us;

                                tm_printf((UB *)"[bypass] Pass %s: scan L=%u/status%d R=%u/status%d; sideways=%u cm forward=%u cm\n",
                                          bypass.side > 0 ? "left" : "right",
                                          p->left_scan_cm, p->left_scan_status, p->right_scan_cm, p->right_scan_status,
                                          BYPASS_SIDE_CM, forward_cm);
                                if (p->left_scan_status == SONAR_INVALID && p->right_scan_status == SONAR_INVALID)
                                    tm_printf((UB *)"[bypass] No side measurements; using left fallback\n");

                                // Park the sonar toward the box, then start moving without waiting for an echo.
                                if (tk_set_flg(ranger_flgid, bypass.side > 0 ?
                                               SONAR_TRACK_RIGHT_FLAG : SONAR_TRACK_LEFT_FLAG) < E_OK) {
                                    bypass_abort(&bypass, &mot_cmd, "side tracking request");
                                    break;
                                }
                                bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_OUT, bypass.side);
                                m_mode = MODE_WALL_HUGGING;
                                break;
                            }
                            case MSG_IMU_UPDATE:
                                if (n_msg.data.imu_telemetry_msg.event == IMU_MSG_SHOCK_DETECTED)
                                    bypass_abort(&bypass, &mot_cmd, "IMU shock");
                                break;
                            default:
                                break;
                        }

                        if (m_mode == MODE_OBSTACLE_ASSESS && bypass.phase != BYPASS_ABORT &&
                            bypass.deadline_us && (W)(time_us() - bypass.deadline_us) >= 0)
                            bypass_abort(&bypass, &mot_cmd, "stop acknowledgement or scan timeout");
                        if (bypass.phase == BYPASS_ABORT) m_mode = MODE_SAFE_STOP;
                        break;
                    }

                    // Keep the existing mode name; its steps now make a simple rectangular detour.
                    case MODE_WALL_HUGGING: {
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:
                                // Ignore the original line until the whole box is behind the car.
                                if (bypass.phase == BYPASS_FIND_LINE) {
                                    IR_STATE state = n_msg.data.ir_state;
                                    BOOL black = state == STATE_B_W || state == STATE_W_B || state == STATE_B_B;
                                    bypass.black_count = black ? bypass.black_count + 1U : 0U;
                                    if (bypass.black_count >= 2U) {
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_LINE_FOUND, 0, 0, 0);
                                        if (bypass.phase != BYPASS_ABORT &&
                                            tk_set_flg(ranger_flgid, SONAR_RESUME_FLAG) >= E_OK) {
                                            scan_requested = FALSE;
                                            prev_ir_state = STATE_NULL;
                                            turning_rampup_count = 0;
                                            m_mode = MODE_REENTRY_ADVANCE;
                                            tm_printf((UB *)"[bypass] Line found; reentry side=%d\n", bypass.side);
                                        } else if (bypass.phase != BYPASS_ABORT) {
                                            bypass_abort(&bypass, &mot_cmd, "forward sonar resume");
                                        }
                                    }
                                }
                                break;

                            case MSG_OBSTACLE_DETECTED:
                                // Discard any queued front detection while sonar is parked.
                                break;

                            case MSG_TRACKING_OBSTACLE: {
                                const SONAR_PROBE_RESULT *probe = &n_msg.data.sonar_probe;
                                UW now = time_us();
                                if (probe->angle_deg != -45 * bypass.side ||
                                    (W)(probe->sampled_at_us - bypass.requested_us) < 0 ||
                                    (W)(probe->sampled_at_us - bypass.last_sample_us) <= 0 ||
                                    (UW)(now - probe->sampled_at_us) >= BYPASS_TRACK_STALE_MS * 1000U)
                                    break;
                                if (probe->status == SONAR_SENSOR_FAULT || probe->status == SONAR_SERVO_FAULT) {
                                    bypass_abort(&bypass, &mot_cmd, "sonar hardware");
                                    break;
                                }
                                if (probe->status != SONAR_OK && probe->status != SONAR_NO_ECHO &&
                                    probe->status != SONAR_TOO_CLOSE && probe->status != SONAR_UNSTABLE)
                                    break;
                                if (probe->status == SONAR_OK &&
                                    (!probe->valid || probe->distance_cm < 2U || probe->distance_cm > 400U))
                                    break;
                                bypass.last_sample_us = bypass.last_tracking_us = probe->sampled_at_us;
                                if ((W)(probe->sampled_at_us - bypass.phase_started_us) < 0) break;

                                // A tracking reply must never overwrite either 90-degree pivot.
                                if (bypass.phase == BYPASS_ADVANCE) {
                                    BOOL close = probe->status == SONAR_TOO_CLOSE ||
                                                 (probe->status == SONAR_OK && probe->distance_cm <= BYPASS_CLOSE_CM);
                                    MOTOR_COMMAND forward = bypass_forward_command(bypass.side, close);
                                    bypass_command(&bypass, &mot_cmd, forward.target_speed_left, forward.target_speed_right);
                                }
                                break;
                            }
                            case MSG_IMU_UPDATE:
                                if (n_msg.data.imu_telemetry_msg.event == IMU_MSG_SHOCK_DETECTED)
                                    bypass_abort(&bypass, &mot_cmd, "IMU shock");
                                break;
                            default:
                                break;
                        }

                        if (m_mode != MODE_WALL_HUGGING) break;
                        UW now = time_us();
                        if (bypass.phase != BYPASS_ABORT &&
                            (UW)(now - bypass.last_tracking_us) >= BYPASS_TRACK_STALE_MS * 1000U)
                            bypass_abort(&bypass, &mot_cmd, "side tracking messages stopped");

                        if (bypass.phase != BYPASS_ABORT && bypass.deadline_us &&
                            (W)(now - bypass.deadline_us) >= 0) {
                            switch (bypass.phase) {
                                case BYPASS_TURN_OUT:
                                    bypass_begin(&bypass, &mot_cmd, BYPASS_SHIFT_OUT,
                                                 BYPASS_SPEED, -BYPASS_SPEED, bypass.shift_us);
                                    break;
                                case BYPASS_SHIFT_OUT:
                                    bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_PARALLEL, -bypass.side);
                                    break;
                                case BYPASS_TURN_PARALLEL:
                                    bypass_begin(&bypass, &mot_cmd, BYPASS_ADVANCE,
                                                 BYPASS_SPEED, -BYPASS_SPEED, bypass.forward_us);
                                    break;
                                case BYPASS_ADVANCE:
                                    bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_IN, -bypass.side);
                                    break;
                                case BYPASS_TURN_IN:
                                    bypass.black_count = 0;
                                    bypass_begin(&bypass, &mot_cmd, BYPASS_FIND_LINE, BYPASS_SPEED, -BYPASS_SPEED, 0);
                                    break;
                                default:
                                    break;
                            }
                        }
                        if (bypass.phase == BYPASS_ABORT) m_mode = MODE_SAFE_STOP;
                        break;
                    }

                    case MODE_REENTRY_ADVANCE:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:

                                break;
                            default:

                                break;
                        }

                        break;

                    case MODE_REENTRY_PIVOT:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:

                                break;
                            default:

                                break;
                        }

                        break;

                    default:

                        break;
                }

                
            }

            // Update shared telemetry snapshot
            // telemetry_snapshot.speed_left = curr_speed_left;
            // telemetry_snapshot.speed_right = curr_speed_right;
            telemetry_snapshot.speed_left = mot_cmd.target_speed_left;
            telemetry_snapshot.speed_right = mot_cmd.target_speed_right;

            telemetry_snapshot.ir_state = (uint8_t)prev_ir_state;
            telemetry_snapshot.movement_mode = (uint8_t)m_mode;
            telemetry_snapshot.bypass_phase = (uint8_t)bypass.phase;
            telemetry_snapshot.last_barcode = last_decoded_char;
            telemetry_snapshot.instruction_idx = (uint8_t)instruction_idx;
            telemetry_snapshot.enc_l = (int32_t)enc_l_ticks;
            telemetry_snapshot.enc_r = (int32_t)enc_r_ticks;
            telemetry_snapshot.dist_cm = (int32_t)motor_get_avg_dist_cm();
            telemetry_snapshot.obstacle_distance_cm = last_obstacle_dist_cm;
            telemetry_snapshot.closest_cm = last_closest_cm;
            telemetry_snapshot.closest_angle_deg = (int32_t)last_closest_angle_deg;
            telemetry_snapshot.estimated_width_cm = last_estimated_width_cm;
            telemetry_snapshot.seq++;
            telemetry_snapshot.obstacle_distance_valid = live_sonar_distance_valid;
            if (live_sonar_distance_valid) {
                telemetry_snapshot.obstacle_distance_cm = live_sonar_distance_cm;
            } else {
                telemetry_snapshot.obstacle_distance_cm = last_obstacle_dist_cm;
            }

        // Unexpected behaviour.
        } else {

            // Fatal RTOS error occurred (e.g., buffer was deleted or ID is invalid)

        }
    }
}

/* Bypass helpers: side selection and motor commands used by the cases above. */
_Static_assert(BYPASS_SPEED > BYPASS_STEER_LIMIT &&
               BYPASS_SPEED + BYPASS_STEER_LIMIT <= 100, "invalid bypass power");
_Static_assert(BYPASS_PIVOT_FROM_FRONT_CM <= ROBOT_LENGTH_CM, "invalid pivot position");
_Static_assert(BYPASS_SWEPT_RADIUS_CM * BYPASS_SWEPT_RADIUS_CM >=
               BYPASS_PIVOT_FROM_FRONT_CM * BYPASS_PIVOT_FROM_FRONT_CM + ROBOT_HALF_WIDTH_CM * ROBOT_HALF_WIDTH_CM &&
               BYPASS_SWEPT_RADIUS_CM * BYPASS_SWEPT_RADIUS_CM >=
               (ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM) * (ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM) +
               ROBOT_HALF_WIDTH_CM * ROBOT_HALF_WIDTH_CM, "swept radius must cover the whole body");


// No echo is open space, not a failed scan. Other unavailable readings rank below usable rays.
LOCAL UW bypass_side_room(SONAR_STATUS status, UW cm)
{
    if (status == SONAR_NO_ECHO) return 401U;
    if (status == SONAR_OK && cm >= 2U && cm <= 400U) return cm;
    return 0U;
}

LOCAL INT bypass_choose_side(const OBSTACLE_PROFILE *profile)
{
    UW left = bypass_side_room(profile->left_scan_status, profile->left_scan_cm);
    UW right = bypass_side_room(profile->right_scan_status, profile->right_scan_cm);
    // Left wins ties, including an unmeasured edge on both sides; never return zero.
    return left >= right ? 1 : -1;
}

// Wiring uses opposite signs for forward, but the SAME signs for an in-place pivot.
LOCAL MOTOR_COMMAND bypass_pivot_command(INT direction)
{
    MOTOR_COMMAND command = {0};
    if (direction == 1 || direction == -1) {
        command.target_speed_left = -direction * BYPASS_PIVOT_SPEED;
        command.target_speed_right = -direction * BYPASS_PIVOT_SPEED;
    }
    return command;
}

// Keep the rectangular path straight unless the parked sonar reports the box too close.
LOCAL MOTOR_COMMAND bypass_forward_command(INT side, BOOL close)
{
    INT correction = close ? side * BYPASS_STEER_LIMIT : 0;
    MOTOR_COMMAND command = {0};
    command.target_speed_left = BYPASS_SPEED - correction;
    command.target_speed_right = -(BYPASS_SPEED + correction);
    return command;
}

#ifndef NDEBUG
#define BYPASS_CHECK(condition) do { if (!(condition)) return FALSE; } while (0)
// Debug builds check side choice and mirrored motor signs without moving the car.
LOCAL BOOL bypass_self_check(void)
{
    OBSTACLE_PROFILE profile = {0};
    BYPASS_CHECK(bypass_choose_side(&profile) == 1);
    profile.left_scan_status = profile.right_scan_status = SONAR_OK;
    profile.left_scan_cm = 20U;
    profile.right_scan_cm = 40U;
    BYPASS_CHECK(bypass_choose_side(&profile) == -1);
    profile.left_scan_cm = 40U;
    BYPASS_CHECK(bypass_choose_side(&profile) == 1);
    profile.right_scan_status = SONAR_NO_ECHO;
    BYPASS_CHECK(bypass_choose_side(&profile) == -1);
    profile.left_scan_status = SONAR_NO_ECHO;
    BYPASS_CHECK(bypass_choose_side(&profile) == 1);
    for (INT side = -1; side <= 1; side += 2) {
        MOTOR_COMMAND pivot = bypass_pivot_command(side);
        MOTOR_COMMAND straight = bypass_forward_command(side, FALSE);
        MOTOR_COMMAND outward = bypass_forward_command(side, TRUE);
        MOTOR_COMMAND mirror = bypass_forward_command(-side, TRUE);
        BYPASS_CHECK(pivot.target_speed_left == -side * BYPASS_PIVOT_SPEED);
        BYPASS_CHECK(pivot.target_speed_right == pivot.target_speed_left);
        BYPASS_CHECK(pivot.turn_ticks_left == 0 && pivot.turn_ticks_right == 0);
        BYPASS_CHECK(straight.target_speed_left == BYPASS_SPEED && straight.target_speed_right == -BYPASS_SPEED);
        BYPASS_CHECK(side * (outward.target_speed_left - BYPASS_SPEED) < 0);
        BYPASS_CHECK(outward.target_speed_left == -mirror.target_speed_right);
        BYPASS_CHECK(outward.target_speed_left >= BYPASS_SPEED - BYPASS_STEER_LIMIT &&
                     outward.target_speed_left <= BYPASS_SPEED + BYPASS_STEER_LIMIT);
        BYPASS_CHECK(outward.turn_ticks_left == 0 && outward.turn_ticks_right == 0);
    }
    return TRUE;
}
#undef BYPASS_CHECK
#endif

// These faults belong to the motor, IMU or sonar tasks; bypass must not release them.
LOCAL BOOL bypass_hardware_fault(void)
{
    return sonar_fault || telemetry_snapshot.imu_event == IMU_MSG_SHOCK_DETECTED ||
           (motor_fault_word & (FAULT_BIT_STALL | FAULT_BIT_ENCODER)) != 0U;
}

// Close echoes steer outward; only hardware or message-delivery failures abort the detour.
LOCAL void bypass_abort(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, const char *reason)
{
    motor_estop = TRUE;
    motor_stop_ack_requested = FALSE;
    bypass->owns_obstacle_stop = FALSE;
    bypass->phase = BYPASS_ABORT;
    *command = (MOTOR_COMMAND){0};
    (void)tk_snd_mbf(motor_mbfid, command, sizeof(*command), TMO_POL);
    tm_printf((UB *)"[bypass] Fault: %s (motor=%u sonar=%d IMU=%u)\n",
              reason, motor_fault_word, sonar_fault, telemetry_snapshot.imu_event);
}

// Send a fully initialized command; zero tick targets leave timing to Navigator.
LOCAL void bypass_command(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, INT left, INT right)
{
    if (bypass_hardware_fault() || (motor_estop && !bypass->owns_obstacle_stop)) {
        bypass_abort(bypass, command, "motor, IMU or sonar fault");
        return;
    }
    if ((left != 0 || right != 0) && bypass->owns_obstacle_stop) {
        // Release only the obstacle-detection stop, never a later fault stop.
        bypass->owns_obstacle_stop = FALSE;
        motor_stop_ack_requested = FALSE;
        motor_estop = FALSE;
    }
    *command = (MOTOR_COMMAND){0};
    command->target_speed_left = left;
    command->target_speed_right = right;
    if (tk_snd_mbf(motor_mbfid, command, sizeof(*command), BYPASS_POLL_MS) < E_OK)
        bypass_abort(bypass, command, "motor command delivery");
}

// Name each step in serial logs so it can be matched to the physical movement.
LOCAL const char *bypass_phase_name(BYPASS_PHASE phase)
{
    switch (phase) {
        case BYPASS_TURN_OUT:      return "TURN_OUT_90";
        case BYPASS_SHIFT_OUT:     return "SIDE_STEP";
        case BYPASS_TURN_PARALLEL: return "TURN_BACK_90";
        case BYPASS_ADVANCE:       return "PASS_BOX";
        case BYPASS_TURN_IN:       return "TURN_TO_LINE_90";
        case BYPASS_FIND_LINE:     return "FIND_LINE";
        case BYPASS_LINE_FOUND:    return "LINE_FOUND";
        case BYPASS_ABORT:         return "FAULT";
        default:                  return "IDLE";
    }
}

// Use deadlines instead of sleeping so sensor messages keep being consumed.
LOCAL void bypass_begin(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT left, INT right, UW duration_us)
{
    bypass->phase = phase;
    bypass->phase_started_us = time_us();
    bypass->deadline_us = duration_us ? bypass->phase_started_us + duration_us : 0U;
    bypass_command(bypass, command, left, right);
    tm_printf((UB *)"[bypass] %s side=%d L=%d R=%d duration=%u ms\n",
              bypass_phase_name(bypass->phase), bypass->side,
              command->target_speed_left, command->target_speed_right, duration_us / 1000U);
}

LOCAL void bypass_pivot(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT direction)
{
    if (direction != 1 && direction != -1) {
        bypass_abort(bypass, command, "invalid pivot direction");
        return;
    }
    UW ms = direction > 0 ? BYPASS_LEFT_90_MS : BYPASS_RIGHT_90_MS;
    MOTOR_COMMAND pivot = bypass_pivot_command(direction);
    bypass_begin(bypass, command, phase, pivot.target_speed_left, pivot.target_speed_right, ms * 1000U);
}
