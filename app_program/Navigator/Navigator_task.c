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

void clear_navigator_mbf() {
    NAVIGATOR_MESSAGE discard_msg;
    while (tk_rcv_mbf(navigator_mbfid, &discard_msg, TMO_POL) >= 0) { }
}

static BOOL execute_mapped_barcode(B barcode, MOTOR_COMMAND *command)
{
    if(!mqtt_get_movement_for_barcode((char)barcode, command)) {
        tm_printf((UB *)"[nav] unmapped barcode: %c\n", barcode);
        return FALSE;
    }
    if(tk_snd_mbf(motor_mbfid, (void *)command, sizeof(MOTOR_COMMAND),
                  TMO_FEVR) < E_OK) {
        tm_printf((UB *)"[nav] failed to send mapped barcode\n");
        return FALSE;
    }
    return TRUE;
}

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    // General
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;
    MOTOR_COMMAND mot_cmd;
    // MOVEMENT_MODE m_mode = MODE_BARCODE_SCANNING;
    // MOVEMENT_MODE prev_m_mode = MODE_BARCODE_SCANNING;
    MOVEMENT_MODE m_mode = MODE_WIFI_SEND_COMMS;
    MOVEMENT_MODE prev_m_mode = MODE_LINE_FOLLOWING;

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
    UB instruction_idx = 0;
    B barcode_characters[NUMBER_OF_COMMANDS] = {};
    DIRECTION instructions[NUMBER_OF_COMMANDS] = {};
    for (int i=0; i<NUMBER_OF_COMMANDS; i++) instructions[i] = DIR_INVALID;
    INT instructions_count = 0;
    B barcode_count = 0;

    // Decision making
    B num_rotations = 0;

    // Hump
    BOOL hump_pop = 0;
    UH spam_mitigation = 0;
    UH spam_mitigation_threshold = 10;

    // Obstacle
    BOOL scan_requested = FALSE;
    BYPASS_PHASE bypass_phase = BYPASS_TURN_OUT;

    // Unused
    INT left_debounce_ticks = 0;
    INT right_debounce_ticks = 0;

    // Motion tracking for telemetry
    INT curr_speed_left = 0;
    INT curr_speed_right = 0;
    UW live_sonar_distance_cm = 0;
    BOOL live_sonar_distance_valid = FALSE;

    while (1) {
        message_size = tk_rcv_mbf(navigator_mbfid, (void *)&n_msg, TMO_FEVR); // Set as timeout forever (TMO_FEVR) as IR is continuously sending updates, so no timeout needed.

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            if (n_msg.type == MSG_REMOTE_CMD) {
                if(!motor_estop && m_mode != MODE_WALL_HUGGING &&
                   m_mode != MODE_SAFE_STOP) {
                    mot_cmd = n_msg.data.remote_cmd;
                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                    curr_speed_left = mot_cmd.target_speed_left;
                    curr_speed_right = mot_cmd.target_speed_right;
                }
            } else if (n_msg.type == MSG_SONAR_DISTANCE) {
                live_sonar_distance_valid = n_msg.data.sonar_distance.valid;
                if (live_sonar_distance_valid) {
                    live_sonar_distance_cm = n_msg.data.sonar_distance.distance_cm;
                }
            } else {
            switch (m_mode) {
                case MODE_BARCODE_SCANNING:
                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:
                            switch (n_msg.data.ir_state) {
                                case STATE_W_B:
                                    mot_cmd.target_speed_left =     40;
                                    mot_cmd.target_speed_right =   -28;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    break;
                                case STATE_B_W:
                                    mot_cmd.target_speed_left =     28;
                                    mot_cmd.target_speed_right =   -40;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    break;
                                default:
                                    mot_cmd.target_speed_left =     30;
                                    mot_cmd.target_speed_right =   -30;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    break;
                            }

                            break;
                        case MSG_BARCODE_DECODED:
                            last_decoded_char = (char)n_msg.data.decoded_char;
                            if(execute_mapped_barcode(n_msg.data.decoded_char, &mot_cmd)) {
                                curr_speed_left = mot_cmd.target_speed_left;
                                curr_speed_right = mot_cmd.target_speed_right;
                                if(mot_cmd.target_speed_left == 0 &&
                                   mot_cmd.target_speed_right == 0) {
                                    m_mode = MODE_SAFE_STOP;
                                }
                            }

                            break;
                    }

                    break;
                case MODE_WIFI_SEND_COMMS:
                    instruction_idx = 0;

                    // Stop car after scanning barcodes
                    mot_cmd.target_speed_left =      0;
                    mot_cmd.target_speed_right =     0;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    switch (n_msg.type) {
                        case MSG_WIFI_BARCODE_MAPPING:      // Note for bryan & future self: packed data must look like 0bxxxx xxxx yyyy yyyy, where x is direction and y is character.
                            DIRECTION direction = n_msg.data.packed_command_mapping >> 8;      // Cast int to enum
                            B code39_char = n_msg.data.packed_command_mapping & 0x00ff;         // bitwise & mask to get least-significant 8 bits
                                //Sends temp log for debugging purposes, can be removed later ~ Bryan
                                tm_printf((UB *)"[nav] mapping received: %c -> %d\n",code39_char,direction);

                            for (int i=0; i<NUMBER_OF_COMMANDS; i++) {
                                if (barcode_characters[i] == code39_char) {
                                    instructions[i] = direction;
                                    break;
                                }
                                if (barcode_characters[i] == 0) {
                                    barcode_characters[i] = code39_char;
                                    instructions[i] = direction;
                                    if (instructions_count < NUMBER_OF_COMMANDS) {
                                        instructions_count++;
                                    }
                                    break;
                                }
                            }

                            // Stay in current mode if not all commands filled out (MQTT has not transmitted all the command mappings yet)
                            m_mode = MODE_LINE_FOLLOWING;
                            for (int i=0; i<NUMBER_OF_COMMANDS; i++) {
                                if (barcode_characters[i] == 0 ||
                                    instructions[i] == DIR_INVALID) {
                                    m_mode = MODE_WIFI_SEND_COMMS;
                                }
                            }

                            break;
                        default:
                            
                            break;
                    }

                    break;
                case MODE_LINE_FOLLOWING:
                    switch (n_msg.type) {
                        case MSG_BARCODE_DECODED:
                            last_decoded_char = (char)n_msg.data.decoded_char;
                            if(execute_mapped_barcode(n_msg.data.decoded_char, &mot_cmd)) {
                                curr_speed_left = mot_cmd.target_speed_left;
                                curr_speed_right = mot_cmd.target_speed_right;
                                if(mot_cmd.target_speed_left == 0 &&
                                   mot_cmd.target_speed_right == 0) {
                                    m_mode = MODE_SAFE_STOP;
                                }
                            }

                            break;
                        case MSG_IR_UPDATE:
                            IR_STATE curr_state = n_msg.data.ir_state;

                                if (curr_state != prev_ir_state) {

                                if (curr_state == STATE_W_W) {
                                    turning_rampup_count = 0;

                                    mot_cmd.target_speed_left =     25;
                                    mot_cmd.target_speed_right =   -25;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                } else if (curr_state == STATE_W_B) {
                                    // Car veering left, adjust right
                                    mot_cmd.target_speed_left =     35;
                                    mot_cmd.target_speed_right =   -20;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                } else if (curr_state == STATE_B_W) {
                                    // Car veering right, adjust left
                                    mot_cmd.target_speed_left =     20;
                                    mot_cmd.target_speed_right =   -35;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                } else if (curr_state == STATE_B_B) {
                                    turning_rampup_count = 0;

                                    // Car at instersection
                                    mot_cmd.target_speed_left =      0;
                                    mot_cmd.target_speed_right =    -0;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    curr_speed_left = mot_cmd.target_speed_left;
                                    curr_speed_right = mot_cmd.target_speed_right;
                                    
                                    m_mode = MODE_DECISION_MAKING_ADVANCE;
                                } else {
                                    // Errorneous state

                                    // Update previous IR state
                                    prev_ir_state = curr_state;
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

                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;
                                        
                                        if (curr_state == STATE_B_W) {
                                            mot_cmd.target_speed_left =     20;
                                            mot_cmd.target_speed_right =   -45;
                                        } else if (curr_state == STATE_W_B) {
                                            mot_cmd.target_speed_left =     45;
                                            mot_cmd.target_speed_right =   -20;
                                        }

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    }                                    
                                }
                            }

                            break;
                        case MSG_OBSTACLE_DETECTED:

                            last_obstacle_dist_cm = n_msg.data.obstacle_distance_cm;
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

                case MODE_GOING_OVER_HUMP:
                    VB left_speed = 40;
                    VB right_speed = -40;

                    VB pitch = n_msg.data.imu_telemetry_msg.pitch_deg;
                    VB roll = n_msg.data.imu_telemetry_msg.roll_deg;

                    switch (n_msg.data.imu_telemetry_msg.event) {
                        case IMU_MSG_HUMP_DETECTED:
                            if (pitch < 0) {
                                mot_cmd.target_speed_left =  (left_speed + pitch) > 10 ? left_speed + pitch : 10;
                                mot_cmd.target_speed_right = -40;
                            } else {
                                mot_cmd.target_speed_left =  40;
                                mot_cmd.target_speed_right = (right_speed + pitch) < -10 ? right_speed + pitch : -10;
                            }

                            if (roll < -10 && hump_pop != 1) {
                                hump_pop = 1;
                            } else if (roll > 10 && hump_pop != 0) {
                                hump_pop = 0;
                            }

                            tm_printf("left: %d, right: %d\n", mot_cmd.target_speed_left, mot_cmd.target_speed_right);
                            
                            mot_cmd.turn_ticks_left =        0;
                            mot_cmd.turn_ticks_right =       0;

                            spam_mitigation++;
                            if (spam_mitigation > spam_mitigation_threshold) {
                                tm_printf("SENT\n\n");
                                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                spam_mitigation = 0;
                            }

                            break;
                        case IMU_MSG_NO_HUMP:
                            tm_printf("NO HUMP RECEIVED\n");
                            if (hump_pop == 0) {
                                tm_printf("RECOVERED\n");
                                m_mode = MODE_LINE_FOLLOWING;
                            } else {
                                mot_cmd.turn_ticks_left =        0;
                                mot_cmd.turn_ticks_right =       0;

                                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                            }

                            break;
                    }

                    break;

                case MODE_DECISION_MAKING_ADVANCE:
                    tk_dly_tsk(250);

                    mot_cmd.target_speed_left =     35;
                    mot_cmd.target_speed_right =   -35;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                    curr_speed_left = mot_cmd.target_speed_left;
                    curr_speed_right = mot_cmd.target_speed_right;

                    tk_dly_tsk(700);

                    mot_cmd.target_speed_left =      0;
                    mot_cmd.target_speed_right =    -0;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                    curr_speed_left = mot_cmd.target_speed_left;
                    curr_speed_right = mot_cmd.target_speed_right;

                    tk_dly_tsk(250);

                    m_mode = MODE_DECISION_MAKING_TURN;
                    clear_navigator_mbf();

                    tk_dly_tsk(30);     // wait for next IR update

                    break;

                // Junction Navigation
                case MODE_DECISION_MAKING_TURN:

                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:

                            IR_STATE curr_ir_state = n_msg.data.ir_state;
                            // DIRECTION curr_decision = instructions[instruction_idx];
                            DIRECTION curr_decision = instructions[instruction_idx];

                            if (prev_ir_state == STATE_B_B) {       // should still be B_B after advance
                                switch (curr_decision) {
                                    case DIR_FRONT:
                                        m_mode = MODE_LINE_FOLLOWING;

                                        break;
                                    case DIR_BACK:
                                        num_rotations = 2;

                                        // mot_cmd.target_speed_left =    -50;
                                        // mot_cmd.target_speed_right =   -50;
                                        // mot_cmd.turn_ticks_left =        0;
                                        // mot_cmd.turn_ticks_right =       0;

                                        // tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        // tk_dly_tsk(800);

                                        mot_cmd.target_speed_left =    -28;
                                        mot_cmd.target_speed_right =   -28;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        curr_speed_left = mot_cmd.target_speed_left;
                                        curr_speed_right = mot_cmd.target_speed_right;

                                        break;
                                    case DIR_LEFT:
                                        num_rotations = 1;

                                        // mot_cmd.target_speed_left =    -50;
                                        // mot_cmd.target_speed_right =   -50;
                                        // mot_cmd.turn_ticks_left =        0;
                                        // mot_cmd.turn_ticks_right =       0;

                                        // tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        // tk_dly_tsk(400);

                                        mot_cmd.target_speed_left =    -28;
                                        mot_cmd.target_speed_right =   -28;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        curr_speed_left = mot_cmd.target_speed_left;
                                        curr_speed_right = mot_cmd.target_speed_right;

                                        break;
                                    case DIR_RIGHT:
                                        num_rotations = 1;

                                        // mot_cmd.target_speed_left =     50;
                                        // mot_cmd.target_speed_right =    50;
                                        // mot_cmd.turn_ticks_left =        0;
                                        // mot_cmd.turn_ticks_right =       0;

                                        // tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        // tk_dly_tsk(400);

                                        mot_cmd.target_speed_left =     28;     // right motor weak
                                        mot_cmd.target_speed_right =    28;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    default:
                                        m_mode = MODE_LINE_FOLLOWING;

                                        break;
                                }

                            } else {
                                if (curr_ir_state == STATE_B_W && prev_ir_state == STATE_W_W && (curr_decision == DIR_LEFT || curr_decision == DIR_BACK)) {
                                    num_rotations--;
                                } else if (curr_ir_state == STATE_W_B && prev_ir_state == STATE_W_W && curr_decision == DIR_RIGHT) {
                                    num_rotations--;
                                }
                                
                                if (num_rotations <= 0) {

                                    mot_cmd.target_speed_left =     0;
                                    mot_cmd.target_speed_right =    0;
                                    mot_cmd.turn_ticks_left =       0;
                                    mot_cmd.turn_ticks_right =      0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    curr_speed_left = mot_cmd.target_speed_left;
                                    curr_speed_right = mot_cmd.target_speed_right;

                                    tk_dly_tsk(250);

                                    if (++instruction_idx > instructions_count) { instruction_idx--; }      // Linger on last instruction on overflow

                                    // Reset variables
                                    num_rotations = 0;
                                    prev_ir_state = STATE_NULL;
                                    m_mode = MODE_LINE_FOLLOWING;
                                }
                            }

                            if (prev_ir_state != curr_ir_state) { prev_ir_state = curr_ir_state; }

                            break;
                        default:
                            
                            break;
                    }

                    break;
                case MODE_OBSTACLE_ASSESS:
                    switch (n_msg.type) {

                        case MSG_MOTOR_STOPPED:
                            if (!scan_requested) {
                                ER er = tk_set_flg(
                                    ranger_flgid,
                                    SONAR_SCAN_REQUEST
                                );

                                if (er < E_OK) {
                                    motor_estop = TRUE;
                                    m_mode = MODE_SAFE_STOP;
                                    break;
                                }

                                scan_requested = TRUE;
                            }
                            break;

                        case MSG_OBSTACLE_PROFILED: {
                            const OBSTACLE_PROFILE *p =
                                &n_msg.data.obstacle_profile;

                            if (!scan_requested || !p->profile_valid) {
                                tm_printf((UB *)"[nav] Scan incomplete/invalid\n");
                                motor_estop = TRUE;
                                m_mode = MODE_SAFE_STOP;
                                break;
                            }

                            tm_printf((UB *)"[nav] Closest: %u cm at %d deg\n",
                                      p->closest_cm, p->closest_angle_deg);

                            last_closest_cm = p->closest_cm;
                            last_closest_angle_deg = p->closest_angle_deg;
                            if (p->width_valid) last_estimated_width_cm = p->estimated_width_cm;

                            if (p->right_edge_valid)
                                tm_printf((UB *)"[nav] Right edge: %d deg\n",
                                          p->right_edge_deg);
                            else
                                tm_printf((UB *)"[nav] Right edge: unknown\n");

                            if (p->left_edge_valid)
                                tm_printf((UB *)"[nav] Left edge: %d deg\n",
                                          p->left_edge_deg);
                            else
                                tm_printf((UB *)"[nav] Left edge: unknown\n");

                            if (p->width_valid)
                                tm_printf((UB *)"[nav] Approx width: %u cm\n",
                                          p->estimated_width_cm);
                            else
                                tm_printf((UB *)"[nav] Approx width: unknown\n");

                            // Candidate offsets include robot half-width and gap.
                            if (p->left_offset_valid)
                                tm_printf((UB *)"[nav] Left centre offset: %u cm\n",
                                          p->left_offset_cm);
                            if (p->right_offset_valid)
                                tm_printf((UB *)"[nav] Right centre offset: %u cm\n",
                                          p->right_offset_cm);

                            // A candidate offset does not prove a safe corridor.
                            motor_estop = TRUE;
                            m_mode = MODE_SAFE_STOP;
                            tm_printf((UB *)"[nav] Bypass disabled: clearance not verified\n");
                            break;
                        }

                        default:
                            break;
                    }

                    break;

                case MODE_WALL_HUGGING:
                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:

                            break;
                        case MSG_OBSTACLE_DETECTED:

                            motor_estop = TRUE;
                            m_mode = MODE_SAFE_STOP;

                            break;
                        case MSG_TRACKING_OBSTACLE:

                            break;
                        default:

                            break;
                    }

                    break;

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

            // tm_printf("%d ", m_mode);

            // Update shared telemetry snapshot
            telemetry_snapshot.speed_left = curr_speed_left;
            telemetry_snapshot.speed_right = curr_speed_right;
            telemetry_snapshot.ir_state = (uint8_t)prev_ir_state;
            telemetry_snapshot.movement_mode = (uint8_t)m_mode;
            telemetry_snapshot.bypass_phase = (uint8_t)bypass_phase;
            telemetry_snapshot.last_barcode = last_decoded_char;
            telemetry_snapshot.instruction_idx = (uint8_t)instruction_idx;
            telemetry_snapshot.enc_l = (int32_t)enc_l_ticks;
            telemetry_snapshot.enc_r = (int32_t)enc_r_ticks;
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
