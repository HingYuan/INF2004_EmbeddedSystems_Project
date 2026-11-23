#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"
#include "../Motor/Motor_task.h"
#include "../Ultrasonic/Ultrasonic_task.h"
#include "../Ultrasonic/Bypass_helpers.h"
#include "../IMU/IMU_task.h"
#include "../Network/mqtt_client.h"

// Shared telemetry snapshot instance owned by Navigator
EXPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot = {0};
EXPORT void telemetry_read(TELEMETRY_SNAPSHOT *snapshot)
{
    UINT saved = robot_data_lock();
    *snapshot = telemetry_snapshot;
    robot_data_unlock(saved);
}
EXPORT UB telemetry_imu_event(void)
{
    UINT saved = robot_data_lock();
    UB event = telemetry_snapshot.imu_event;
    robot_data_unlock(saved);
    return event;
}
EXPORT H telemetry_heading(void)
{
    UINT saved = robot_data_lock();
    H heading = telemetry_snapshot.heading_deg;
    robot_data_unlock(saved);
    return heading;
}


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
    NAVIGATOR_STATE nav = {.m_mode = MODE_LINE_FOLLOWING, .prev_ir_state = STATE_NULL,
                           .reentry_dir = DIR_FRONT, .live_sonar_status = SONAR_INVALID};
    // MOVEMENT_MODE m_mode = MODE_BARCODE_SCANNING;

    // Barcode decoding
    B barcode_characters[NUMBER_OF_COMMANDS] = {};                                  // Character key, sent from wifi
    PACKED_ROUTE barcode_instructions = {0};
    PACKED_ROUTE instructions = {0};

    barcode_characters[0] = 'A';
    route_set(&barcode_instructions, 0, DIR_LEFT);

    for (int i=0; i<NUMBER_OF_COMMANDS; i++) route_set(&instructions, i, DIR_INVALID);         // Preset all instructions as DIR_INVALID

    // Hump
    const UH spam_mitigation_threshold = 10;

    // Obstacle
    BYPASS_CONTEXT bypass = {0};  // bypass.side remains available to the reentry cases.

    // 2s grace period to place car on track, then flush any stale startup sensor messages
    tk_dly_tsk(2000);
    clear_navigator_mbf();
#ifndef NDEBUG
    if (!bypass_self_check() || !mqtt_codec_self_check()) {
        robot_flag_assign(ROBOT_ESTOP_REQUEST, TRUE);
        tm_printf((UB *)"[bypass] Self-check failed\n");
        return;
    }
#endif

    while (1) {
        // During bypass, check fault deadlines even if sensor messages stop arriving.
        BOOL bypass_active = nav.m_mode == MODE_OBSTACLE_ASSESS || nav.m_mode == MODE_WALL_HUGGING;
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
            nav.m_mode = MODE_SAFE_STOP;
            bypass_active = FALSE;
        }

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            if (n_msg.type == MSG_REMOTE_CMD && !bypass_active) {

                if(!motor_estop && nav.m_mode != MODE_WALL_HUGGING && nav.m_mode != MODE_SAFE_STOP) {
                    mot_cmd = n_msg.data.remote_cmd;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                }
            } else if (!bypass_active && n_msg.type == MSG_IMU_UPDATE &&
                       n_msg.data.imu_telemetry_msg.event == IMU_MSG_LIFTED_OVERTURNED) {
                robot_flag_assign(ROBOT_ESTOP_REQUEST, TRUE);
                nav.m_mode = MODE_SAFE_STOP;
                mot_cmd.target_speed_left = STOP_SPEED;
                mot_cmd.target_speed_right = STOP_SPEED;
                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_POL);
                tm_printf((UB *)"[nav] STOP: robot lifted/overturned\n");
            } else if (!bypass_active && n_msg.type == MSG_SONAR_DISTANCE) {
                nav.live_sonar_distance_valid = n_msg.data.sonar_distance.valid;
                nav.live_sonar_status = n_msg.data.sonar_distance.status;
                
                if (nav.live_sonar_distance_valid) {
                    nav.live_sonar_distance_cm = n_msg.data.sonar_distance.distance_cm;
                }
            } else {
                switch (nav.m_mode) {
                    case MODE_WIFI_SEND_COMMS:
                        nav.instruction_idx = 0;

                        // Stop car
                        mot_cmd.target_speed_left =      STOP_SPEED;
                        mot_cmd.target_speed_right =     STOP_SPEED;
                        mot_cmd.turn_ticks_left =        0;
                        mot_cmd.turn_ticks_right =       0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                        switch (n_msg.type) {
                            case MSG_WIFI_BARCODE_MAPPING: {    // Packed mapping: direction in bits 7..9, ASCII in bits 0..6.
                                DIRECTION direction = n_msg.data.packed_command_mapping >> 7;      // Cast int to enum
                                B code39_char = n_msg.data.packed_command_mapping & 0x007f;        // bitwise & mask to get least-significant 7 bits

                                // Capitalize lowercase characters
                                if (code39_char >= 'a' && code39_char <= 'z') code39_char = code39_char - 32;
                                
                                //Sends temp log for debugging purposes, can be removed later ~ Bryan
                                tm_printf((UB *)"[nav] mapping received: %c -> %d\n",code39_char,direction);

                                if (nav.mapping_increment >= NUMBER_OF_COMMANDS || direction > DIR_STOP) break;
                                barcode_characters[nav.mapping_increment] = code39_char;
                                route_set(&barcode_instructions, nav.mapping_increment, direction);
                                nav.mapping_increment++;

                                if (nav.mapping_increment >= NUMBER_OF_COMMANDS) {
                                    nav.m_mode = MODE_LINE_FOLLOWING;
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
                    //                 if (last_decoded_char == barcode_characters[i] && instruction_idx_barcode < NUMBER_OF_COMMANDS) {
                    //                     route_set(&instructions, instruction_idx, route_get(&barcode_instructions, i));
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

                                    if (curr_state != nav.prev_ir_state || nav.reentry) {

                                        if (curr_state == STATE_W_W) {
                                            nav.turning_rampup_count = 0;

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
                                            nav.turning_rampup_count = 0;

                                            // Car at instersection
                                            mot_cmd.target_speed_left =     STOP_SPEED;
                                            mot_cmd.target_speed_right =   -STOP_SPEED;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            nav.m_mode = MODE_DECISION_MAKING_ADVANCE;
                                        } else {
                                            // Errorneous state

                                        }

                                        // Update previous IR state
                                        nav.prev_ir_state = curr_state;
                                    } else {
                                        // Ramp up turning if stuck on turning
                                        if (curr_state == STATE_B_W || curr_state == STATE_W_B) {
                                            if (nav.turning_rampup_count < TURNING_RAMPUP_LIMIT) {
                                                nav.turning_rampup_count++;
                                            }

                                            if (nav.turning_rampup_count == TURNING_RAMPUP_LIMIT) {
                                                nav.turning_rampup_count++;

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
                                nav.last_decoded_char = (char)n_msg.data.decoded_char;

                                tm_printf("HELLO %c\n", (char)n_msg.data.decoded_char);

                                // Map detected character to instruction, then add to instruction array
                                for (B i=0; i<NUMBER_OF_COMMANDS; i++) {
                                    if (nav.last_decoded_char == barcode_characters[i]) {
                                        route_set(&instructions, nav.instruction_idx_barcode, route_get(&barcode_instructions, i));
                                        nav.instruction_idx_barcode++;
                                    }
                                }

                                break;
                            case MSG_OBSTACLE_DETECTED:

                                nav.last_obstacle_dist_cm = n_msg.data.obstacle_distance_cm;
                                // Each obstacle starts with a fresh side choice and confirmation counters.
                                bypass = (BYPASS_CONTEXT){.phase = BYPASS_IDLE,
                                    .owns_obstacle_stop = TRUE,
                                    .deadline_us = time_us() + BYPASS_RESPONSE_MS * 1000U};
                                nav.scan_requested = FALSE;
                                mot_cmd = (MOTOR_COMMAND){0};
                                nav.live_sonar_distance_valid = FALSE;
                                nav.live_sonar_distance_cm = 0U;
                                nav.live_sonar_status = SONAR_INVALID;
                                robot_flag_assign(ROBOT_STOP_ACK_REQUEST, TRUE);
                                nav.m_mode = MODE_OBSTACLE_ASSESS;
                                nav.reentry = 1;

                                tm_printf((UB *)"[nav] Obstacle detected\n");

                                break;
                            case MSG_IMU_UPDATE:
                                if (n_msg.data.imu_telemetry_msg.event == IMU_MSG_HUMP_DETECTED) {
                                    mot_cmd.target_speed_left =     FORWARD_SPEED_HUMP;
                                    mot_cmd.target_speed_right =   -FORWARD_SPEED_HUMP;
                                    mot_cmd.turn_ticks_left =       0;
                                    mot_cmd.turn_ticks_right =      0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    
                                    nav.m_mode = MODE_GOING_OVER_HUMP;
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

                            VB pitch = (n_msg.data.imu_telemetry_msg.pitch_ddeg / 10);
                            VB roll = (n_msg.data.imu_telemetry_msg.roll_ddeg / 10);

                            switch (n_msg.data.imu_telemetry_msg.event) {
                                case IMU_MSG_HUMP_DETECTED:
                                    if (pitch < 0) {
                                        mot_cmd.target_speed_left =  (left_speed + pitch) > 10 ? left_speed + pitch : 10;
                                        mot_cmd.target_speed_right = right_speed;
                                    } else {
                                        mot_cmd.target_speed_left =  left_speed;
                                        mot_cmd.target_speed_right = (right_speed + pitch) < -10 ? right_speed + pitch : -10;
                                    }

                                    if (roll < -10 && nav.hump_pop != 1) {
                                        nav.hump_pop = 1;
                                    } else if (roll > 10 && nav.hump_pop != 0) {
                                        nav.hump_pop = 0;
                                    }
                                    
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    nav.spam_mitigation++;
                                    if (nav.spam_mitigation > spam_mitigation_threshold) {
                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        nav.spam_mitigation = 0;
                                    }

                                    break;
                                case IMU_MSG_NO_HUMP:
                                    // tm_printf("NO HUMP RECEIVED\n");
                                    nav.hump_pop = 0;
                                    nav.prev_ir_state = STATE_NULL;
                                    nav.m_mode = MODE_LINE_FOLLOWING;
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
                        
                        nav.m_mode = MODE_DECISION_MAKING_TURN;
                        clear_navigator_mbf();

                        tk_dly_tsk(30);     // wait for next IR update

                        break;

                    // Junction Navigation
                    case MODE_DECISION_MAKING_TURN:

                        switch (n_msg.type) {
                            case MSG_IR_UPDATE: {

                                IR_STATE curr_ir_state = n_msg.data.ir_state;

                                // If no new instruction, linger on latest one
                                if (route_get(&instructions, nav.instruction_idx_barcode) == DIR_INVALID && nav.instruction_idx > 0) {
                                    nav.instruction_idx--;
                                } else if (route_get(&instructions, nav.instruction_idx) == DIR_INVALID && nav.instruction_idx <= 0) {
                                    nav.instruction_idx = 0;
                                }

                                DIRECTION curr_decision = route_get(&instructions, nav.instruction_idx);
                                // DIRECTION curr_decision = instructions_fake[instruction_idx];
                                if (nav.reentry) {
                                    curr_decision = nav.reentry_dir;
                                    nav.reentry = 0;
                                }

                                if (nav.prev_ir_state == STATE_B_B) {       // should still be B_B after advance
                                    switch (curr_decision) {
                                        case DIR_BACK:
                                            nav.num_rotations = 2;

                                            mot_cmd.target_speed_left =    -LEFT_PIVOT_L;
                                            mot_cmd.target_speed_right =   -RIGHT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_LEFT:
                                            nav.num_rotations = 1;

                                            mot_cmd.target_speed_left =    -LEFT_PIVOT_L;
                                            mot_cmd.target_speed_right =   -LEFT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_RIGHT:
                                            nav.num_rotations = 1;

                                            mot_cmd.target_speed_left =     RIGHT_PIVOT_L;
                                            mot_cmd.target_speed_right =    RIGHT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_STOP:
                                            nav.num_rotations = 0;
                                           
                                        // DIR_FRONT
                                        default:
                                            nav.num_rotations = 0;

                                            mot_cmd.target_speed_left =     FORWARD_SPEED;
                                            mot_cmd.target_speed_right =   -FORWARD_SPEED;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            nav.prev_ir_state = STATE_NULL;

                                            nav.m_mode = MODE_LINE_FOLLOWING;

                                            // Linger on last instruction on overflow
                                            if (++nav.instruction_idx >= NUMBER_OF_COMMANDS) { nav.instruction_idx--; }

                                            break;
                                    }

                                } else {
                                    if (curr_ir_state == STATE_B_W && nav.prev_ir_state == STATE_W_W && (curr_decision == DIR_LEFT || curr_decision == DIR_BACK)) {
                                        nav.num_rotations--;
                                    } else if (curr_ir_state == STATE_W_B && nav.prev_ir_state == STATE_W_W && curr_decision == DIR_RIGHT) {
                                        nav.num_rotations--;
                                    }
                                    
                                    if (nav.num_rotations <= 0) {

                                        tk_dly_tsk(50);

                                        mot_cmd.target_speed_left =     STOP_SPEED;
                                        mot_cmd.target_speed_right =    STOP_SPEED;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        tk_dly_tsk(250);

                                        // Linger on last instruction on overflow
                                        if (++nav.instruction_idx >= NUMBER_OF_COMMANDS) { nav.instruction_idx--; }

                                        // Reset variables
                                        nav.num_rotations = 0;
                                        nav.prev_ir_state = STATE_NULL;
                                        nav.m_mode = MODE_LINE_FOLLOWING;
                                    }
                                }

                                if (nav.prev_ir_state != curr_ir_state) { nav.prev_ir_state = curr_ir_state; }

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
                                if (!nav.scan_requested && bypass.phase == BYPASS_IDLE) {
                                    if (tk_set_flg(ranger_flgid, SONAR_SCAN_REQUEST) < E_OK) {
                                        bypass_abort(&bypass, &mot_cmd, "scan request");
                                        break;
                                    }
                                    nav.scan_requested = TRUE;
                                    bypass.deadline_us = time_us() + BYPASS_SCAN_MS * 1000U;
                                }
                                break;

                            case MSG_OBSTACLE_PROFILED: {
                                if (!nav.scan_requested) break;
                                nav.scan_requested = FALSE;
                                const OBSTACLE_PROFILE *p = &n_msg.data.obstacle_profile;
                                nav.last_closest_cm = p->closest_cm;
                                nav.last_closest_angle_deg = p->closest_angle_deg;
                                nav.last_estimated_width_cm = p->width_valid ? p->estimated_width_cm : 0U;
                                nav.live_sonar_status = sonar_ray_status(p, 3);
                                nav.live_sonar_distance_valid =
                                    nav.live_sonar_status == SONAR_OK &&
                                    p->front_cm >= 2U &&
                                    p->front_cm <= 400U;
                                nav.live_sonar_distance_cm =
                                    nav.live_sonar_distance_valid ? p->front_cm : 0U;

                                switch (bypass.phase) {
                                    case BYPASS_IDLE:
                                        bypass.first_scan = *p;
                                        bypass.front_cm = nav.last_obstacle_dist_cm;
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
                                        nav.scan_requested = TRUE;
                                        if (tk_set_flg(ranger_flgid, SONAR_SCAN_REQUEST) < E_OK)
                                            bypass_abort(&bypass, &mot_cmd, "second scan request");
                                        break;
                                    case BYPASS_SCAN_RETURN:
                                        bypass_track_side(&bypass, &mot_cmd);
                                        bypass_pivot(&bypass, &mot_cmd, BYPASS_TURN_OUT, bypass.side);
                                        nav.m_mode = MODE_WALL_HUGGING;
                                        break;
                                    default:
                                        break;
                                }
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
                        if (nav.m_mode == MODE_OBSTACLE_ASSESS && bypass.phase != BYPASS_ABORT &&
                            bypass.deadline_us && (W)(time_us() - bypass.deadline_us) >= 0)
                            bypass_abort(&bypass, &mot_cmd, "stop acknowledgement or scan timeout");
                        if (bypass.phase == BYPASS_ABORT) nav.m_mode = MODE_SAFE_STOP;
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
                                        bypass_probe(&bypass, &mot_cmd, BYPASS_WIDTH_CHECK);
                                        break;
                                    case BYPASS_WIDTH_ALIGN:
                                        if (bypass.width_clearance_pending) {
                                            bypass.width_clearance_pending = FALSE;
                                            bypass_follow_next(&bypass, &mot_cmd);
                                        } else {
                                            bypass_probe(&bypass, &mot_cmd, BYPASS_WIDTH_CHECK);
                                        }
                                        break;
                                    case BYPASS_WIDTH_TURN_OUT:
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_WIDTH_EXTEND,
                                                     BYPASS_API_POWER, -BYPASS_API_POWER,
                                                     bypass.width_clearance_pending ? BYPASS_PIVOT_CLEAR_MM : BYPASS_WIDTH_STEP_MM);
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
                                        if (bypass.depth_clearance_pending) {
                                            bypass.depth_clearance_pending = FALSE;
                                            bypass.black_count = 0;
                                            bypass_begin(&bypass, &mot_cmd, BYPASS_FIND_LINE,
                                                         BYPASS_API_POWER, -BYPASS_API_POWER,
                                                         bypass.shift_mm + ROBOT_LENGTH_CM * 10U);
                                        } else {
                                            bypass_probe(&bypass, &mot_cmd, BYPASS_RETURN_CHECK);
                                        }
                                        break;
                                    case BYPASS_DEPTH_TURN_BACK:
                                        bypass_track_side(&bypass, &mot_cmd);
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_ADVANCE,
                                                     BYPASS_API_POWER, -BYPASS_API_POWER,
                                                     bypass.depth_clearance_pending ? BYPASS_DEPTH_CLEAR_MM : BYPASS_FORWARD_STEP_MM);
                                        break;
                                    case BYPASS_FIND_LINE:
                                        bypass_abort(&bypass, &mot_cmd, "line not found within search distance");
                                        break;
                                    default:
                                        break;
                                }
                                break;

                            case MSG_SONAR_PROBED:
                                nav.live_sonar_status = n_msg.data.sonar_probe.status;
                                nav.live_sonar_distance_valid =
                                    n_msg.data.sonar_probe.valid &&
                                    n_msg.data.sonar_probe.status == SONAR_OK &&
                                    n_msg.data.sonar_probe.distance_cm >= 2U &&
                                    n_msg.data.sonar_probe.distance_cm <= 400U;
                                nav.live_sonar_distance_cm =
                                    nav.live_sonar_distance_valid
                                        ? n_msg.data.sonar_probe.distance_cm
                                        : 0U;
                                bypass_probe_result(&bypass, &mot_cmd, &n_msg.data.sonar_probe);
                                break;

                            case MSG_IR_UPDATE:
                                // Ignore the original line until the whole box is behind the car.
                                if (bypass.phase == BYPASS_FIND_LINE) {
                                    IR_STATE state = n_msg.data.ir_state;
                                    BOOL black = state == STATE_B_W || state == STATE_W_B || state == STATE_B_B;
                                    bypass.black_count = black ? (bypass.black_count < 2U ? bypass.black_count + 1U : 2U) : 0U;
                                    if (bypass.black_count >= 2U) {
                                        bypass_begin(&bypass, &mot_cmd, BYPASS_LINE_FOUND, 0, 0, 0);
                                        if (bypass.phase != BYPASS_ABORT &&
                                            tk_set_flg(ranger_flgid, SONAR_RESUME_FLAG) >= E_OK) {
                                            nav.scan_requested = FALSE;
                                            nav.prev_ir_state = STATE_NULL;
                                            nav.turning_rampup_count = 0;
                                            nav.m_mode = MODE_LINE_FOLLOWING;
                                            
                                            if (bypass.side == 1) {
                                                nav.reentry_dir = DIR_LEFT;
                                            } else if (bypass.side == 0) {
                                                nav.reentry_dir = DIR_RIGHT;
                                            }

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
                            default:
                                break;
                        }

                        if (nav.m_mode != MODE_WALL_HUGGING) break;
                        UW now = time_us();
                        if (bypass.phase != BYPASS_ABORT && bypass.tracking_active &&
                            (UW)(now - bypass.last_tracking_us) >= BYPASS_TRACK_STALE_MS * 1000U)
                            bypass_abort(&bypass, &mot_cmd, "side tracking messages stopped");

                        if (bypass.phase != BYPASS_ABORT && bypass.deadline_us &&
                            (W)(now - bypass.deadline_us) >= 0)
                            bypass_abort(&bypass, &mot_cmd, "motor movement timeout");
                        if (bypass.phase == BYPASS_ABORT) nav.m_mode = MODE_SAFE_STOP;
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
            UINT telemetry_saved = robot_data_lock();
            telemetry_snapshot.speed_left = mot_cmd.target_speed_left;
            telemetry_snapshot.speed_right = mot_cmd.target_speed_right;

            telemetry_snapshot.ir_state = (UB)nav.prev_ir_state;
            telemetry_snapshot.movement_mode = (UB)nav.m_mode;
            telemetry_snapshot.bypass_phase = (UB)bypass.phase;
            telemetry_snapshot.last_barcode = nav.last_decoded_char;
            telemetry_snapshot.instruction_idx = (UB)nav.instruction_idx;
            telemetry_snapshot.enc_l = (W)enc_l_ticks;
            telemetry_snapshot.enc_r = (W)enc_r_ticks;
            telemetry_snapshot.dist_cm = (W)motor_get_avg_dist_cm();
            telemetry_snapshot.obstacle_distance_cm = nav.last_obstacle_dist_cm;
            telemetry_snapshot.closest_cm = nav.last_closest_cm;
            telemetry_snapshot.closest_angle_deg = (W)nav.last_closest_angle_deg;
            telemetry_snapshot.estimated_width_cm = nav.last_estimated_width_cm;
            telemetry_snapshot.seq++;
            telemetry_snapshot.obstacle_distance_valid = nav.live_sonar_distance_valid;
            telemetry_snapshot.obstacle_status = (UB)nav.live_sonar_status;
            if (nav.live_sonar_distance_valid) {
                telemetry_snapshot.obstacle_distance_cm = nav.live_sonar_distance_cm;
            } else if (bypass_active) {
                telemetry_snapshot.obstacle_distance_cm = 0U;
            } else {
                telemetry_snapshot.obstacle_distance_cm = nav.last_obstacle_dist_cm;
            }

            robot_data_unlock(telemetry_saved);

        // Unexpected behaviour.
        } else {

            // Fatal RTOS error occurred (e.g., buffer was deleted or ID is invalid)

        }
    }
}
