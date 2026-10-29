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

void clear_navigator_mbf() {
    NAVIGATOR_MESSAGE discard_msg;
    while (tk_rcv_mbf(navigator_mbfid, &discard_msg, TMO_POL) >= 0) { }
}

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    // General
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;
    MOTOR_COMMAND mot_cmd;
    // MOVEMENT_MODE m_mode = MODE_BARCODE_SCANNING;
    // MOVEMENT_MODE prev_m_mode = MODE_BARCODE_SCANNING;
    MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;
    MOVEMENT_MODE prev_m_mode = MODE_LINE_FOLLOWING;

    // Line following
    IR_STATE prev_ir_state = STATE_NULL;

    // Barcode decoding
    UB instruction_idx = 0;
    B barcode_characters[NUMBER_OF_COMMANDS] = {};
    DIRECTION instructions[NUMBER_OF_COMMANDS] = {};
    for (int i=0; i<NUMBER_OF_COMMANDS; i++) instructions[i] = DIR_INVALID;
    // REMOVE BELOW 
    DIRECTION instructions_fake[NUMBER_OF_COMMANDS] = { DIR_RIGHT, DIR_LEFT, DIR_BACK, DIR_FRONT };       // Initialize instructions (fake for now)
    // REMOVE ABOVE
    INT instructions_count = sizeof(instructions_fake) / sizeof(DIRECTION);
    B barcode_count = 0;

    // Decision making
    B num_rotations = 0;

    // Obstacle
    BOOL scan_requested = FALSE;
    BYPASS_PHASE bypass_phase = BYPASS_TURN_OUT;

    // Unused
    INT left_debounce_ticks = 0;
    INT right_debounce_ticks = 0;

    while (1) {
        message_size = tk_rcv_mbf(navigator_mbfid, (void *)&n_msg, TMO_FEVR); // Set as timeout forever (TMO_FEVR) as IR is continuously sending updates, so no timeout needed.

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            switch (m_mode) {
                case MODE_BARCODE_SCANNING:
                    mot_cmd.target_speed_left =     28;
                    mot_cmd.target_speed_right =   -28;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    switch (n_msg.type) {
                        case MSG_BARCODE_DECODED:

                            barcode_characters[instruction_idx] = n_msg.data.decoded_char;
                            instruction_idx++;

                            barcode_count++;

                            // Next mode after reading all 4 barcodes
                            if (barcode_count >= NUMBER_OF_COMMANDS) m_mode = MODE_WIFI_SEND_COMMS;

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

                            for (int i=0; i<NUMBER_OF_COMMANDS; i++) {
                                if (barcode_characters[i] == code39_char) {
                                    instructions[i] = direction;
                                }
                            }

                            // Stay in current mode if not all commands filled out (MQTT has not transmitted all the command mappings yet)
                            m_mode = MODE_LINE_FOLLOWING;
                            for (int i=0; i<NUMBER_OF_COMMANDS; i++) {
                                if (instructions[i] == DIR_INVALID) m_mode = MODE_WIFI_SEND_COMMS;
                            }

                            break;
                        default:
                            
                            break;
                    }

                    break;
                case MODE_LINE_FOLLOWING:
                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:
                            IR_STATE curr_state = n_msg.data.ir_state;

                            if (curr_state != prev_ir_state) {

                                if (curr_state == STATE_W_W) {
                                    mot_cmd.target_speed_left =     40;
                                    mot_cmd.target_speed_right =   -40;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                } else if (curr_state == STATE_W_B) {
                                    // Car veering left, adjust right
                                    mot_cmd.target_speed_left =     50;
                                    mot_cmd.target_speed_right =   -35;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                } else if (curr_state == STATE_B_W) {
                                    // Car veering right, adjust left
                                    mot_cmd.target_speed_left =     35;
                                    mot_cmd.target_speed_right =   -50;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                } else if (curr_state == STATE_B_B) {
                                    // Car at instersection
                                    mot_cmd.target_speed_left =      0;
                                    mot_cmd.target_speed_right =    -0;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    
                                    m_mode = MODE_DECISION_MAKING_ADVANCE;
                                    // m_mode = MODE_DECISION_MAKING_TURN;
                                } else {
                                    // Errorneous state

                                }

                                // Update previous IR state
                                prev_ir_state = curr_state;
                            }

                            break;
                        case MSG_OBSTACLE_DETECTED:
                            // Ultrasonic has already requested a stop
                            motor_estop = TRUE;
                            m_mode = MODE_OBSTACLE_ASSESS;
                            scan_requested = FALSE;
                            prev_ir_state = STATE_NULL;

                            // Ask Motor to confirm it has stopped
                            motor_stop_ack_requested = TRUE;

                            tm_printf((UB *)"[nav] Obstacle detected\n");
                            break;
                        case MSG_IMU_UPDATE:
                            tm_printf("[IMU] PITCH: %d, ROLL: %d\n", n_msg.data.imu_telemetry_msg.pitch_deg, n_msg.data.imu_telemetry_msg.roll_deg);

                        default:
                            // Ignore other states

                            break;
                    }

                    break;

                case MODE_DECISION_MAKING_ADVANCE:
                    tk_dly_tsk(500);

                    mot_cmd.target_speed_left =     45;
                    mot_cmd.target_speed_right =   -45;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    tk_dly_tsk(500);

                    mot_cmd.target_speed_left =      0;
                    mot_cmd.target_speed_right =    -0;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    tk_dly_tsk(500);

                    m_mode = MODE_DECISION_MAKING_TURN;
                    clear_navigator_mbf();

                    tk_dly_tsk(11);     // wait for next IR update

                    break;

                // Junction Navigation
                case MODE_DECISION_MAKING_TURN:

                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:

                            IR_STATE curr_ir_state = n_msg.data.ir_state;
                            // DIRECTION curr_decision = instructions[instruction_idx];
                            DIRECTION curr_decision = instructions_fake[instruction_idx];

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

                                        break;
                                    case DIR_LEFT:
                                        num_rotations = 1;

                                        // mot_cmd.target_speed_left =    -50;
                                        // mot_cmd.target_speed_right =   -50;
                                        // mot_cmd.turn_ticks_left =        0;
                                        // mot_cmd.turn_ticks_right =       0;

                                        // tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        // tk_dly_tsk(400);

                                        mot_cmd.target_speed_left =    -27;
                                        mot_cmd.target_speed_right =   -27;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    case DIR_RIGHT:
                                        num_rotations = 1;

                                        // mot_cmd.target_speed_left =     50;
                                        // mot_cmd.target_speed_right =    50;
                                        // mot_cmd.turn_ticks_left =        0;
                                        // mot_cmd.turn_ticks_right =       0;

                                        // tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        // tk_dly_tsk(400);

                                        mot_cmd.target_speed_left =     27;     // right motor weak
                                        mot_cmd.target_speed_right =    27;
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
                            IR_STATE ir_state = n_msg.data.ir_state;

                            if (ir_state == STATE_B_W || ir_state == STATE_W_B || ir_state == STATE_B_B) m_mode = MODE_REENTRY_ADVANCE;

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
                case MODE_SAFE_STOP:
                    motor_estop = TRUE;
                    break;
                default:

                    break;
            }

        // Unexpected behaviour.
        } else {

            // Fatal RTOS error occurred (e.g., buffer was deleted or ID is invalid)

        }


        // Add code to send m_mode to wifi driver
        if (m_mode != prev_m_mode) {
            // Send telemetry

        }   
        prev_m_mode = m_mode;

        tk_dly_tsk(10);
    }
}

