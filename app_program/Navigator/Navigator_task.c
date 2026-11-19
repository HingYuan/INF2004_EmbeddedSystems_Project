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
LOCAL void bypass_plan_width(BYPASS_CONTEXT *bypass, const OBSTACLE_PROFILE *second);
LOCAL MOTOR_COMMAND bypass_pivot_command(INT direction);
#ifndef NDEBUG
LOCAL BOOL bypass_self_check(void);
#endif
LOCAL BOOL bypass_hardware_fault(void);
LOCAL void bypass_abort(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, const char *reason);
LOCAL void bypass_begin(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT left, INT right, UW amount);
LOCAL void bypass_pivot(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT direction);
LOCAL void bypass_track_side(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command);
LOCAL void bypass_probe(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, BYPASS_PHASE phase);
LOCAL void bypass_follow_next(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command);
LOCAL BOOL bypass_complete_leg(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command);
LOCAL void bypass_probe_result(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                               const SONAR_PROBE_RESULT *probe);

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
    MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;
    // MOVEMENT_MODE m_mode = MODE_BARCODE_SCANNING;

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
    B instruction_idx = 0;                                                          // Track instructions for navigating intersections
    B instruction_idx_barcode = 0;                                                  // Same as above, but for adding new instructions from decoded barcodes along the track
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
        // During bypass, check fault deadlines even if sensor messages stop arriving.
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
            } else if (n_msg.type == MSG_IMU_UPDATE &&
                       n_msg.data.imu_telemetry_msg.event == IMU_MSG_LIFTED_OVERTURNED) {
                motor_estop = TRUE;
                m_mode = MODE_SAFE_STOP;
                mot_cmd.target_speed_left = STOP_SPEED;
                mot_cmd.target_speed_right = STOP_SPEED;
                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_POL);
                tm_printf((UB *)"[nav] STOP: robot lifted/overturned\n");
            } else if (n_msg.type == MSG_SONAR_DISTANCE) {
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
                                    m_mode = MODE_LINE_FOLLOWING;
                                }

                                break;
                            }
                            default:
                                
                                break;
                        }

                        break;
                    // case MODE_BARCODE_SCANNING:
                    //     switch (n_msg.type) {
                    //         case MSG_IR_UPDATE:
                    //             switch (n_msg.data.ir_state) {
                    //                 case STATE_W_B:
                    //                     mot_cmd.target_speed_left =     RIGHT_TURN_L_BARCODE;
                    //                     mot_cmd.target_speed_right =   -RIGHT_TURN_R_BARCODE;
                    //                     mot_cmd.turn_ticks_left =       0;
                    //                     mot_cmd.turn_ticks_right =      0;

                    //                     tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    //                     break;
                    //                 case STATE_B_W:
                    //                     mot_cmd.target_speed_left =     LEFT_TURN_L_BARCODE;
                    //                     mot_cmd.target_speed_right =   -LEFT_TURN_R_BARCODE;
                    //                     mot_cmd.turn_ticks_left =       0;
                    //                     mot_cmd.turn_ticks_right =      0;

                    //                     tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    //                     break;
                    //                 default:
                    //                     mot_cmd.target_speed_left =     FORWARD_SPEED_BARCODE;
                    //                     mot_cmd.target_speed_right =   -FORWARD_SPEED_BARCODE;
                    //                     mot_cmd.turn_ticks_left =       0;
                    //                     mot_cmd.turn_ticks_right =      0;

                    //                     tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    //                     break;
                    //             }

                    //             break;
                    //         case MSG_BARCODE_DECODED:
                    //             last_decoded_char = (char)n_msg.data.decoded_char;

                    //             // Map detected character to instruction, then add to instruction array
                    //             for (B i=0; i<NUMBER_OF_COMMANDS; i++) {
                    //                 if (last_decoded_char == barcode_characters[i]) {
                    //                     instructions[instruction_idx] = barcode_instructions[i];
                    //                     instruction_idx++;
                    //                 }

                    //                 // If max instructions recorded
                    //                 if (instruction_idx >= NUMBER_OF_COMMANDS) {
                    //                     m_mode = MODE_LINE_FOLLOWING;
                    //                     instruction_idx = 0;

                    //                     mot_cmd.target_speed_left =     STOP_SPEED;
                    //                     mot_cmd.target_speed_right =   -STOP_SPEED;
                    //                     mot_cmd.turn_ticks_left =       0;
                    //                     mot_cmd.turn_ticks_right =      0;

                    //                     tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    //                     tk_dly_tsk(1000);
                    //                 }
                    //             }

                    //             break;
                    //     }

                    //     break;
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
                            case MSG_BARCODE_DECODED:
                                last_decoded_char = (char)n_msg.data.decoded_char;

                                // Map detected character to instruction, then add to instruction array
                                for (B i=0; i<NUMBER_OF_COMMANDS; i++) {
                                    if (last_decoded_char == barcode_characters[i]) {
                                        instructions[instruction_idx_barcode] = barcode_instructions[i];
                                        instruction_idx_barcode++;
                                    }
                                }

                                break;
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
                                    mot_cmd.target_speed_left =     FORWARD_SPEED_HUMP;
                                    mot_cmd.target_speed_right =   -FORWARD_SPEED_HUMP;
                                    mot_cmd.turn_ticks_left =       0;
                                    mot_cmd.turn_ticks_right =      0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    
                                    m_mode = MODE_GOING_OVER_HUMP;
                                }

                                break;
                            default:

                                break;
                        }

                        break;

                    case MODE_GOING_OVER_HUMP: {
                        if (n_msg.type == MSG_IMU_UPDATE) {
                            VB left_speed =     FORWARD_SPEED_HUMP;
                            VB right_speed =   -FORWARD_SPEED_HUMP;

                            VB pitch = n_msg.data.imu_telemetry_msg.pitch_deg;
                            VB roll = n_msg.data.imu_telemetry_msg.roll_deg;

                            switch (n_msg.data.imu_telemetry_msg.event) {
                                case IMU_MSG_HUMP_DETECTED:
                                    if (pitch < 0) {
                                        mot_cmd.target_speed_left =  (left_speed + pitch) > 10 ? left_speed + pitch : 10;
                                        mot_cmd.target_speed_right = right_speed;
                                    } else {
                                        mot_cmd.target_speed_left =  left_speed;
                                        mot_cmd.target_speed_right = (right_speed + pitch) < -10 ? right_speed + pitch : -10;
                                    }

                                    if (roll < -10 && hump_pop != 1) {
                                        hump_pop = 1;
                                    } else if (roll > 10 && hump_pop != 0) {
                                        hump_pop = 0;
                                    }
                                    
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    spam_mitigation++;
                                    if (spam_mitigation > spam_mitigation_threshold) {
                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        spam_mitigation = 0;
                                    }

                                    break;
                                case IMU_MSG_NO_HUMP:
                                    // tm_printf("NO HUMP RECEIVED\n");
                                    hump_pop = 0;
                                    prev_ir_state = STATE_NULL;
                                    m_mode = MODE_LINE_FOLLOWING;
                                    mot_cmd.target_speed_left =     FORWARD_SPEED;
                                    mot_cmd.target_speed_right =   -FORWARD_SPEED;
                                    mot_cmd.turn_ticks_left =       0;
                                    mot_cmd.turn_ticks_right =      0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    break;
                                default:
                                    break;
                            }
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

                                // If no new instruction, linger on latest one
                                if (instructions[instruction_idx_barcode] == DIR_INVALID && instruction_idx > 0) {
                                    instruction_idx--;
                                } else if (instructions[instruction_idx] == DIR_INVALID && instruction_idx <= 0) {
                                    instruction_idx = 0;
                                }

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
                                            if (++instruction_idx >= NUMBER_OF_COMMANDS) { instruction_idx--; }

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
                                        if (++instruction_idx >= NUMBER_OF_COMMANDS) { instruction_idx--; }

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
                    // Exactly two profiles, separated by a 2 cm reverse; restore that distance before entry.
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

                                switch (bypass.phase) {
                                    case BYPASS_IDLE:
                                        bypass.first_scan = *p;
                                        bypass.front_cm = last_obstacle_dist_cm;
                                        if (p->front_cm >= 2U &&
                                            p->front_cm <= OBSTACLE_TRIGGER_CM + BYPASS_PROFILE_DEPTH_CM)
                                            bypass.front_cm = p->front_cm;
                                        if (bypass.front_cm + BYPASS_PIVOT_FROM_FRONT_CM <
                                            BYPASS_SWEPT_RADIUS_CM + WALL_BODY_GAP_CM) {
                                            bypass_abort(&bypass, &mot_cmd, "insufficient room for initial pivot");
                                            break;
                                        }
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_WAIT_RELEASE, 0, 0, 0);
                                        break;
                                    case BYPASS_SCAN_SECOND:
                                        bypass_plan_width(&bypass, p);
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_SCAN_RETURN,
                                                     BYPASS_API_POWER, -BYPASS_API_POWER, BYPASS_SCAN_BACK_MM);
                                        break;
                                    default:
                                        break;
                                }
                                break;
                            }
                            case MSG_MOTOR_MOVE_DONE:
                                if (!bypass.move_pending) break;
                                bypass.move_pending = FALSE;
                                switch (bypass.phase) {
                                    case BYPASS_SCAN_BACK:
                                        bypass.phase = BYPASS_SCAN_SECOND;
                                        bypass.deadline_us = time_us() + BYPASS_SCAN_MS * 1000U;
                                        scan_requested = TRUE;
                                        if (tk_set_flg(ranger_flgid, SONAR_SCAN_REQUEST) < E_OK)
                                            bypass_abort(&bypass, &mot_cmd, "second scan request");
                                        break;
                                    case BYPASS_SCAN_RETURN:
                                        bypass_track_side(&bypass, &mot_cmd);
                                        bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_OUT, bypass.side);
                                        m_mode = MODE_WALL_HUGGING;
                                        break;
                                    default:
                                        break;
                                }
                                break;
                            case MSG_IMU_UPDATE:
                                if (n_msg.data.imu_telemetry_msg.event == IMU_MSG_SHOCK_DETECTED)
                                    bypass_abort(&bypass, &mot_cmd, "IMU shock");
                                break;
                            default:
                                break;
                        }

                        switch (bypass.phase) {
                            case BYPASS_WAIT_RELEASE:
                                if (!(motor_fault_word & FAULT_BIT_ESTOP))
                                    bypass_begin(&bypass, &mot_cmd, BYPASS_SCAN_BACK,
                                                 -BYPASS_API_POWER, BYPASS_API_POWER, BYPASS_SCAN_BACK_MM);
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
                            case MSG_MOTOR_MOVE_DONE:
                                // The unchanged Motor API has no IDs; serialize every call until completion.
                                if (!bypass.move_pending) break;
                                bypass.move_pending = FALSE;
                                if (!bypass_complete_leg(&bypass, &mot_cmd)) break;
                                switch (bypass.phase) {
                                    case BYPASS_TURN_OUT:
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_SHIFT_OUT,
                                                     BYPASS_API_POWER, -BYPASS_API_POWER, bypass.planned_shift_mm);
                                        break;
                                    case BYPASS_SHIFT_OUT:
                                        bypass.shift_mm += bypass.leg_mm;
                                        bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_PARALLEL, -bypass.side);
                                        break;
                                    case BYPASS_TURN_PARALLEL:
                                    case BYPASS_WIDTH_ALIGN:
                                        bypass_probe(&bypass, &mot_cmd, BYPASS_WIDTH_CHECK);
                                        break;
                                    case BYPASS_WIDTH_TURN_OUT:
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_WIDTH_EXTEND,
                                                     BYPASS_API_POWER, -BYPASS_API_POWER, bypass.extend_mm);
                                        break;
                                    case BYPASS_WIDTH_EXTEND:
                                        bypass.shift_mm += bypass.leg_mm;
                                        bypass_pivot(&bypass, &mot_cmd, BYPASS_WIDTH_ALIGN, -bypass.side);
                                        break;
                                    case BYPASS_ADVANCE:
                                        bypass.forward_mm += bypass.leg_mm;
                                        bypass_follow_next(&bypass, &mot_cmd);
                                        break;
                                    case BYPASS_TURN_IN:
                                        bypass_probe(&bypass, &mot_cmd, BYPASS_RETURN_CHECK);
                                        break;
                                    case BYPASS_DEPTH_TURN_BACK:
                                        bypass_track_side(&bypass, &mot_cmd);
                                        bypass_begin(&bypass, &mot_cmd,
                                                     bypass.clear_after_turn ? BYPASS_REAR_CLEAR : BYPASS_ADVANCE,
                                                     BYPASS_API_POWER, -BYPASS_API_POWER,
                                                     bypass.clear_after_turn ? BYPASS_TURN_CLEAR_MM : BYPASS_FORWARD_STEP_MM);
                                        break;
                                    case BYPASS_REAR_CLEAR:
                                        bypass.forward_mm += bypass.leg_mm;
                                        bypass.rear_clear_done = TRUE;
                                        bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_IN, -bypass.side);
                                        break;
                                    case BYPASS_FIND_LINE:
                                        bypass_abort(&bypass, &mot_cmd, "line not found within search distance");
                                        break;
                                    default:
                                        break;
                                }
                                break;

                            case MSG_SONAR_PROBED:
                                bypass_probe_result(&bypass, &mot_cmd, &n_msg.data.sonar_probe);
                                break;

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
                                if (!bypass.tracking_active || probe->angle_deg != -45 * bypass.side ||
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
                        if (bypass.phase != BYPASS_ABORT && bypass.tracking_active &&
                            (UW)(now - bypass.last_tracking_us) >= BYPASS_TRACK_STALE_MS * 1000U)
                            bypass_abort(&bypass, &mot_cmd, "side tracking messages stopped");

                        if (bypass.phase != BYPASS_ABORT && bypass.deadline_us &&
                            (W)(now - bypass.deadline_us) >= 0)
                            bypass_abort(&bypass, &mot_cmd, "motor movement timeout");
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
_Static_assert(BYPASS_LEFT_TURN_DEG > 0 && BYPASS_RIGHT_TURN_DEG > 0, "invalid bypass turn angle");
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

LOCAL UW bypass_side_offset(const OBSTACLE_PROFILE *first, const OBSTACLE_PROFILE *second,
                            INT side, UW front_cm, BOOL *known)
{
    static const UW sine[4] = {0, 259, 500, 707};
    static const UW cosine[4] = {1000, 966, 866, 707};
    static const UW tangent[4] = {0, 268, 577, 1000};
    const OBSTACLE_PROFILE *scans[2] = {first, second};
    UW reference[2] = {first->front_cm, second->front_cm};
    for (INT scan = 0; scan < 2; scan++)
        if (reference[scan] < 2U || reference[scan] > front_cm + BYPASS_PROFILE_DEPTH_CM +
            (scan ? BYPASS_SCAN_BACK_MM / 10U : 0U))
            reference[scan] = front_cm + (scan ? BYPASS_SCAN_BACK_MM / 10U : 0U);
    UW depth = reference[0] > reference[1] ? reference[0] : reference[1];
    UW observed_x = 0;
    INT occupied = 0;
    *known = FALSE;

    // Inspect all outer rays: an isolated missing inner ray must not hide a farther occupied ray.
    for (INT ray = 1; ray <= 3; ray++) {
        INT index = 3 + side * ray;
        for (INT scan = 0; scan < 2; scan++) {
            UW cm = scans[scan]->ray_cm[index];
            SONAR_STATUS status = scans[scan]->ray_status[index];
            if (status == SONAR_TOO_CLOSE || (status == SONAR_OK && cm >= 2U && cm <= 400U &&
                cm * cosine[ray] / 1000U <= reference[scan] + BYPASS_PROFILE_DEPTH_CM)) {
                occupied = ray;
                UW x = status == SONAR_TOO_CLOSE ? depth * tangent[ray] / 1000U :
                       (cm * sine[ray] + 999U) / 1000U;
                if (x > observed_x) observed_x = x;
            }
        }
    }

    // The first ray beyond every hit must be open in BOTH scans before it bounds the edge.
    UW edge_cm = 0;
    for (INT ray = occupied + 1; ray <= 3; ray++) {
        INT index = 3 + side * ray;
        BOOL open = TRUE;
        for (INT scan = 0; scan < 2; scan++) {
            SONAR_STATUS status = scans[scan]->ray_status[index];
            UW cm = scans[scan]->ray_cm[index];
            open = open && (status == SONAR_NO_ECHO || (status == SONAR_OK && cm >= 2U && cm <= 400U &&
                   cm * cosine[ray] / 1000U > reference[scan] + BYPASS_PROFILE_DEPTH_CM));
        }
        if (open) {
            edge_cm = (depth * tangent[ray] + 999U) / 1000U;
            *known = TRUE;
            break;
        }
    }

    // An unbounded scan gives no width: find the edge with short stationary forward checks.
    if (!*known) return 0;

    BOOL fine = side > 0 ? first->left_offset_valid && second->left_offset_valid :
                          first->right_offset_valid && second->right_offset_valid;
    W a = side > 0 ? first->left_edge_deg : -first->right_edge_deg;
    W b = side > 0 ? second->left_edge_deg : -second->right_edge_deg;
    if (*known && fine && a >= occupied * 15 && b >= occupied * 15) {
        UW x = side > 0 ? first->left_offset_cm : first->right_offset_cm;
        UW y = side > 0 ? second->left_offset_cm : second->right_offset_cm;
        UW fine_cm = x > y ? x : y;
        if (fine_cm >= ROBOT_HALF_WIDTH_CM + WALL_BODY_GAP_CM) {
            fine_cm -= ROBOT_HALF_WIDTH_CM + WALL_BODY_GAP_CM;
            if (fine_cm < edge_cm) edge_cm = fine_cm;
        }
    }
    if (observed_x > edge_cm) edge_cm = observed_x;
    return edge_cm * 10U + BYPASS_TURN_CLEAR_MM;
}

LOCAL void bypass_plan_width(BYPASS_CONTEXT *bypass, const OBSTACLE_PROFILE *second)
{
    if (second->front_cm >= BYPASS_SCAN_BACK_MM / 10U + 2U &&
        second->front_cm <= bypass->front_cm + BYPASS_PROFILE_DEPTH_CM + BYPASS_SCAN_BACK_MM / 10U) {
        UW original_front = second->front_cm - BYPASS_SCAN_BACK_MM / 10U;
        if (original_front > bypass->front_cm) bypass->front_cm = original_front;
    }
    BOOL left_known, right_known;
    UW left = bypass_side_offset(&bypass->first_scan, second, 1, bypass->front_cm, &left_known);
    UW right = bypass_side_offset(&bypass->first_scan, second, -1, bypass->front_cm, &right_known);
    UW left_room = bypass_side_room(second->left_scan_status, second->left_scan_cm);
    UW right_room = bypass_side_room(second->right_scan_status, second->right_scan_cm);
    if (left_known != right_known)
        bypass->side = left_known ? 1 : -1;
    else
        bypass->side = left < right || (left == right && left_room >= right_room) ? 1 : -1;
    bypass->width_needs_margin = !(bypass->side > 0 ? left_known : right_known);
    bypass->planned_shift_mm = bypass->width_needs_margin ? BYPASS_WIDTH_STEP_MM :
                               (bypass->side > 0 ? left : right);
    tm_printf((UB *)"[bypass] Dynamic offsets L=%u R=%u mm (0=unknown); pass %s, edge=%s, front=%u cm, first step=%u mm\n",
              left, right, bypass->side > 0 ? "left" : "right",
              bypass->width_needs_margin ? "unmeasured: verify outward" : "bounded by both scans",
              bypass->front_cm, bypass->planned_shift_mm);
}

// Wiring uses opposite signs for forward, but the SAME signs for an in-place pivot.
LOCAL MOTOR_COMMAND bypass_pivot_command(INT direction)
{
    MOTOR_COMMAND command = {0};
    if (direction == 1 || direction == -1) {
        command.target_speed_left = -direction * BYPASS_API_POWER;
        command.target_speed_right = -direction * BYPASS_API_POWER;
    }
    return command;
}

#ifndef NDEBUG
#define BYPASS_CHECK(condition) do { if (!(condition)) return FALSE; } while (0)
// Debug builds check side choice and mirrored motor signs without moving the car.
LOCAL BOOL bypass_self_check(void)
{
    OBSTACLE_PROFILE first = {.front_cm = 30U}, second = {.front_cm = 32U};
    BOOL known;
    BYPASS_CHECK(bypass_side_offset(&first, &second, 1, 30U, &known) == 0U && !known);
    first.ray_status[4] = second.ray_status[4] = SONAR_NO_ECHO;
    BYPASS_CHECK(bypass_side_offset(&first, &second, 1, 30U, &known) == 260U && known);
    first.ray_status[6] = second.ray_status[6] = SONAR_OK;
    first.ray_cm[6] = 43U; second.ray_cm[6] = 45U;
    BYPASS_CHECK(bypass_side_offset(&first, &second, 1, 30U, &known) == 0U && !known);
    BYPASS_CHECK(bypass_pivot_command(0).raw == 0U);
    for (INT side = -1; side <= 1; side += 2) {
        MOTOR_COMMAND pivot = bypass_pivot_command(side);
        BYPASS_CHECK(pivot.target_speed_left == -side * BYPASS_API_POWER);
        BYPASS_CHECK(pivot.target_speed_right == pivot.target_speed_left);
        BYPASS_CHECK(pivot.turn_ticks_left == 0 && pivot.turn_ticks_right == 0);
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

// Ordinary no-echo replies do not abort; hardware, delivery and movement faults do.
LOCAL void bypass_abort(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, const char *reason)
{
    motor_estop = TRUE;
    motor_stop_ack_requested = FALSE;
    bypass->owns_obstacle_stop = FALSE;
    bypass->phase = BYPASS_ABORT;
    bypass->move_pending = FALSE;
    bypass->deadline_us = 0;
    *command = (MOTOR_COMMAND){0};
    (void)stop();
    tm_printf((UB *)"[bypass] Fault: %s (motor=%u sonar=%d IMU=%u)\n",
              reason, motor_fault_word, sonar_fault, telemetry_snapshot.imu_event);
}

// Name each step in serial logs so it can be matched to the physical movement.
LOCAL const char *bypass_phase_name(BYPASS_PHASE phase)
{
    switch (phase) {
        case BYPASS_WAIT_RELEASE:  return "WAIT_MOTOR_RELEASE";
        case BYPASS_SCAN_BACK:     return "SCAN_BACK_2CM";
        case BYPASS_SCAN_RETURN:   return "RESTORE_2CM";
        case BYPASS_WIDTH_TURN_OUT:return "WIDTH_TURN_OUT";
        case BYPASS_WIDTH_EXTEND:  return "WIDTH_EXTEND";
        case BYPASS_WIDTH_ALIGN:   return "WIDTH_ALIGN";
        case BYPASS_DEPTH_TURN_BACK:return "FACE_PARALLEL";
        case BYPASS_REAR_CLEAR:    return "REAR_CLEARANCE";
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

// Queue one measured movement. Deadlines only detect faults; MOVE_DONE advances the switch.
LOCAL void bypass_begin(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT left, INT right, UW amount)
{
    if (bypass_hardware_fault() || (motor_estop && !bypass->owns_obstacle_stop)) {
        bypass_abort(bypass, command, "motor, IMU or sonar fault");
        return;
    }
    bypass->phase = phase;
    bypass->phase_started_us = time_us();
    bypass->move_pending = FALSE;
    bypass->deadline_us = 0;
    bypass->leg_mm = 0;
    bypass->leg_start_mm = motor_get_avg_dist_mm();
    *command = (MOTOR_COMMAND){0};
    command->target_speed_left = left;
    command->target_speed_right = right;
    ER result;
    UW timeout_ms = 0;
    const char *unit = "mm";
    switch (phase) {
        case BYPASS_WAIT_RELEASE:
            if (!bypass->owns_obstacle_stop) {
                bypass_abort(bypass, command, "scan stop not owned");
                return;
            }
            bypass->owns_obstacle_stop = FALSE;
            motor_stop_ack_requested = FALSE;
            motor_estop = FALSE;
            result = E_OK;
            timeout_ms = BYPASS_RESPONSE_MS;
            break;
        case BYPASS_TURN_OUT:
        case BYPASS_TURN_PARALLEL:
        case BYPASS_TURN_IN:
        case BYPASS_WIDTH_TURN_OUT:
        case BYPASS_WIDTH_ALIGN:
        case BYPASS_DEPTH_TURN_BACK:
            result = left < 0 ? turnLeft((INT)amount) : turnRight((INT)amount);
            timeout_ms = BYPASS_TURN_TIMEOUT_MS;
            unit = "deg";
            break;
        case BYPASS_SHIFT_OUT:
        case BYPASS_ADVANCE:
        case BYPASS_FIND_LINE:
        case BYPASS_WIDTH_EXTEND:
        case BYPASS_REAR_CLEAR:
        case BYPASS_SCAN_RETURN:
            if (!amount ||
                ((phase == BYPASS_SHIFT_OUT || phase == BYPASS_WIDTH_EXTEND) &&
                 amount + bypass->shift_mm > BYPASS_MAX_SIDE_MM) ||
                ((phase == BYPASS_ADVANCE || phase == BYPASS_REAR_CLEAR) &&
                 amount + bypass->forward_mm > BYPASS_MAX_FORWARD_MM)) {
                bypass_abort(bypass, command, "exploration distance limit");
                return;
            }
            bypass->leg_mm = amount;
            result = moveForward((INT)amount);
            timeout_ms = BYPASS_MOVE_MARGIN_MS + ((amount + 9U) / 10U) * BYPASS_MOVE_TIMEOUT_MS_CM;
            break;
        case BYPASS_SCAN_BACK:
            bypass->leg_mm = BYPASS_SCAN_BACK_MM;
            result = moveBackward(BYPASS_SCAN_BACK_MM);
            timeout_ms = BYPASS_MOVE_MARGIN_MS + BYPASS_SCAN_BACK_MM / 10U * BYPASS_MOVE_TIMEOUT_MS_CM;
            break;
        case BYPASS_LINE_FOUND:
            result = stop();
            break;
        default:
            bypass_abort(bypass, command, "invalid movement phase");
            return;
    }
    if (result < E_OK) {
        bypass_abort(bypass, command, "motor command delivery");
        return;
    }
    bypass->move_pending = phase != BYPASS_LINE_FOUND && phase != BYPASS_WAIT_RELEASE;
    if (timeout_ms) bypass->deadline_us = bypass->phase_started_us + timeout_ms * 1000U;
    tm_printf((UB *)"[bypass] %s side=%d target=%u %s L=%d R=%d\n",
              bypass_phase_name(phase), bypass->side, amount, unit, left, right);
}

LOCAL void bypass_pivot(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT direction)
{
    if (direction != 1 && direction != -1) {
        bypass_abort(bypass, command, "invalid pivot direction");
        return;
    }
    MOTOR_COMMAND pivot = bypass_pivot_command(direction);
    UW degrees = direction > 0 ? BYPASS_LEFT_TURN_DEG : BYPASS_RIGHT_TURN_DEG;
    bypass_begin(bypass, command, phase, pivot.target_speed_left, pivot.target_speed_right, degrees);
}

LOCAL void bypass_track_side(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command)
{
    if (bypass->phase == BYPASS_ABORT) return;
    bypass->requested_us = time_us();
    bypass->last_tracking_us = bypass->last_sample_us = bypass->requested_us;
    bypass->tracking_active = TRUE;
    if (tk_set_flg(ranger_flgid, bypass->side > 0 ? SONAR_TRACK_RIGHT_FLAG : SONAR_TRACK_LEFT_FLAG) < E_OK)
        bypass_abort(bypass, command, "side tracking request");
}

LOCAL void bypass_probe(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, BYPASS_PHASE phase)
{
    if (bypass->phase == BYPASS_ABORT) return;
    if (bypass->phase != phase) bypass->probe_clear_count = bypass->probe_attempts = 0;
    bypass->phase = phase;
    bypass->tracking_active = FALSE;
    bypass->requested_us = bypass->last_sample_us = time_us();
    bypass->deadline_us = bypass->requested_us + BYPASS_PROBE_MS * 1000U;
    *command = (MOTOR_COMMAND){0}; // Motor already reported completion; do not overwrite its next target.
    if (tk_set_flg(ranger_flgid, SONAR_PROBE_FRONT_FLAG) < E_OK)
        bypass_abort(bypass, command, "stationary probe request");
}

// MOVE_DONE arrives after the wheels settle, so include braking/coast in the route position.
LOCAL BOOL bypass_complete_leg(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command)
{
    if (!bypass->leg_mm) return TRUE;
    INT moved = motor_get_avg_dist_mm() - bypass->leg_start_mm;
    if (moved <= 0) {
        bypass_abort(bypass, command, "no forward odometry at movement completion");
        return FALSE;
    }
    tm_printf((UB *)"[bypass] %s travelled=%d mm (requested=%u mm)\n",
              bypass_phase_name(bypass->phase), moved, bypass->leg_mm);
    bypass->leg_mm = (UW)moved;
    return TRUE;
}

LOCAL void bypass_follow_next(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command)
{
    if (bypass->phase == BYPASS_ABORT) return;
    UW minimum = (bypass->front_cm + BYPASS_PIVOT_FROM_FRONT_CM) * 10U;
    if (bypass->forward_mm >= minimum) {
        // Check the actual cross-section; a diagonal echo is not a measurement of box depth.
        bypass_pivot(bypass, command, BYPASS_TURN_IN, -bypass->side);
    } else {
        bypass_begin(bypass, command, BYPASS_ADVANCE,
                     BYPASS_API_POWER, -BYPASS_API_POWER, minimum - bypass->forward_mm);
    }
}

LOCAL void bypass_probe_result(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                               const SONAR_PROBE_RESULT *probe)
{
    if (bypass->phase != BYPASS_WIDTH_CHECK && bypass->phase != BYPASS_RETURN_CHECK) return;
    if (probe->angle_deg != 0 || (W)(probe->sampled_at_us - bypass->requested_us) < 0 ||
        (W)(probe->sampled_at_us - bypass->last_sample_us) <= 0 ||
        (UW)(time_us() - probe->sampled_at_us) >= BYPASS_PROBE_MS * 1000U) return;
    bypass->last_sample_us = probe->sampled_at_us;
    if (probe->status == SONAR_SENSOR_FAULT || probe->status == SONAR_SERVO_FAULT) {
        bypass_abort(bypass, command, "front probe hardware");
        return;
    }
    UW required_mm = bypass->phase == BYPASS_WIDTH_CHECK ?
                     (bypass->front_cm + ROBOT_LENGTH_CM) * 10U :
                     bypass->shift_mm + (ROBOT_LENGTH_CM + WALL_BODY_GAP_CM) * 10U;
    BOOL valid = probe->status == SONAR_OK && probe->valid &&
                 probe->distance_cm >= 2U && probe->distance_cm <= 400U;
    BOOL clear = probe->status == SONAR_NO_ECHO || (valid && probe->distance_cm * 10U > required_mm);
    BOOL blocked = probe->status == SONAR_TOO_CLOSE || (valid && !clear);
    bypass->probe_attempts++;
    bypass->probe_clear_count = clear ? bypass->probe_clear_count + 1U : 0U;
    if ((clear && bypass->probe_clear_count < BYPASS_CLEAR_CONFIRM) ||
        (!clear && !blocked && bypass->probe_attempts < BYPASS_CLEAR_CONFIRM)) {
        bypass_probe(bypass, command, bypass->phase);
        return;
    }
    tm_printf((UB *)"[bypass] %s front %s: %u cm/status%d, sideways=%u along=%u mm\n",
              bypass->phase == BYPASS_WIDTH_CHECK ? "Width" : "Return", clear ? "clear" : "blocked/uncertain",
              probe->distance_cm, probe->status, bypass->shift_mm, bypass->forward_mm);
    switch (bypass->phase) {
        case BYPASS_WIDTH_CHECK:
            if (clear && !bypass->width_needs_margin) {
                bypass_track_side(bypass, command);
                bypass_follow_next(bypass, command);
            } else {
                bypass->extend_mm = clear ? BYPASS_TURN_CLEAR_MM : BYPASS_WIDTH_STEP_MM;
                bypass->width_needs_margin = !clear;
                bypass_track_side(bypass, command);
                bypass_pivot(bypass, command, BYPASS_WIDTH_TURN_OUT, bypass->side);
            }
            break;
        case BYPASS_RETURN_CHECK:
            if (clear && bypass->rear_clear_done) {
                bypass->black_count = 0;
                bypass_begin(bypass, command, BYPASS_FIND_LINE, BYPASS_API_POWER, -BYPASS_API_POWER,
                             bypass->shift_mm + ROBOT_LENGTH_CM * 10U);
            } else {
                bypass->clear_after_turn = clear;
                bypass->rear_clear_done = FALSE;
                bypass_track_side(bypass, command);
                bypass_pivot(bypass, command, BYPASS_DEPTH_TURN_BACK, bypass->side);
            }
            break;
        default:
            break;
    }
}
