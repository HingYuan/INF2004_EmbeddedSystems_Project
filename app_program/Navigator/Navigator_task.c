#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"
#include "../Motor/Motor_task.h"
#include "../Ultrasonic/Ultrasonic_task.h"

void clear_navigator_mbf() {
    NAVIGATOR_MESSAGE discard_msg;
    while (tk_rcv_mbf(navigator_mbfid, &discard_msg, TMO_POL) > 0) { }
}

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    // General
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;
    MOTOR_COMMAND mot_cmd;
    MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;
    MOVEMENT_MODE prev_m_mode = MODE_LINE_FOLLOWING;

    // Line following
    IR_STATE prev_ir_state = STATE_NULL;

    // Barcode decoding
    C_RING_BUFFER barcode_characters;                                           // initialize circular ring buffer to store decoded characters
    crb_init(&barcode_characters);
    INT instruction_idx = 0;
    DIRECTION instructions_fake[50] = { DIR_LEFT, DIR_RIGHT, DIR_BACK, DIR_FRONT };       // Initialize instructions (fake for now)
    INT instructions_count = sizeof(instructions_fake) / sizeof(DIRECTION);

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
                case MODE_LINE_FOLLOWING:
                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:
                            IR_STATE curr_state = n_msg.data.ir_state;

                            if (curr_state != prev_ir_state) {

                                if (curr_state == STATE_W_W) {
                                    mot_cmd.target_speed_left =     30;
                                    mot_cmd.target_speed_right =   -30;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    tm_printf("W_W\n");

                                } else if (curr_state == STATE_W_B) {
                                    // Car veering left, adjust right
                                    mot_cmd.target_speed_left =     55;
                                    mot_cmd.target_speed_right =   -35;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    tm_printf("W_B\n");

                                } else if (curr_state == STATE_B_W) {
                                    // Car veering right, adjust left
                                    mot_cmd.target_speed_left =     35;
                                    mot_cmd.target_speed_right =   -55;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    tm_printf("B_W\n");

                                } else if (curr_state == STATE_B_B) {
                                    // Car at instersection
                                    mot_cmd.target_speed_left =      0;
                                    mot_cmd.target_speed_right =    -0;
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    
                                    m_mode = MODE_DECISION_MAKING_ADVANCE;
                                    // m_mode = MODE_DECISION_MAKING_TURN;

                                    tm_printf("B_B\n");

                                } else {
                                    // Errorneous state

                                }

                                // Update previous IR state
                                prev_ir_state = curr_state;
                            }

                            break;
                        case MSG_BARCODE_DECODED:

                            crb_push(&barcode_characters, n_msg.data.decoded_char);

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

                        default:
                            // Ignore other states

                            break;
                    }

                    break;

                case MODE_DECISION_MAKING_ADVANCE:
                    tk_dly_tsk(500);

                    mot_cmd.target_speed_left =     50;
                    mot_cmd.target_speed_right =   -50;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    tk_dly_tsk(500);

                    mot_cmd.target_speed_left =      0;
                    mot_cmd.target_speed_right =    -0;
                    mot_cmd.turn_ticks_left =        0;
                    mot_cmd.turn_ticks_right =       0;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                    tk_dly_tsk(1000);

                    m_mode = MODE_DECISION_MAKING_TURN;

                    tk_dly_tsk(1);

                    break;

                // Junction Navigation
                case MODE_DECISION_MAKING_TURN:

                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:

                            IR_STATE curr_ir_state = n_msg.data.ir_state;

                            if (prev_ir_state == STATE_B_B) {       // should still be B_B after advance
                                DIRECTION curr_decision = instructions_fake[instruction_idx];

                                switch (curr_decision) {
                                    case DIR_FRONT:
                                        m_mode = MODE_LINE_FOLLOWING;

                                        break;
                                    case DIR_BACK:
                                        if (curr_ir_state == STATE_B_W) {
                                            num_rotations = 3;
                                        } else {
                                            num_rotations = 2;
                                        }

                                        mot_cmd.target_speed_left =    -35;
                                        mot_cmd.target_speed_right =   -35;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    case DIR_LEFT:
                                        if (curr_ir_state == STATE_B_W) {
                                            num_rotations = 2;
                                        } else {
                                            num_rotations = 1;
                                        }

                                        mot_cmd.target_speed_left =    -30;
                                        mot_cmd.target_speed_right =   -30;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    case DIR_RIGHT:
                                        if (curr_ir_state == STATE_B_W) {
                                            num_rotations = 2;
                                        } else {
                                            num_rotations = 2;
                                        }

                                        mot_cmd.target_speed_left =     30;
                                        mot_cmd.target_speed_right =    30;
                                        mot_cmd.turn_ticks_left =        0;
                                        mot_cmd.turn_ticks_right =       0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    default:
                                        m_mode = MODE_LINE_FOLLOWING;

                                        break;
                                }

                            } else if (curr_ir_state == STATE_B_W) {
                                num_rotations--;
                                
                                if (num_rotations <= 0) {
                                    mot_cmd.target_speed_left =     0;
                                    mot_cmd.target_speed_right =    0;
                                    mot_cmd.turn_ticks_left =       0;
                                    mot_cmd.turn_ticks_right =      0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    tk_dly_tsk(250);

                                    if (++instruction_idx > instructions_count) { instructions_idx--; }

                                    // Reset variables
                                    num_rotations = 0;
                                    prev_ir_state = STATE_NULL;
                                    m_mode = MODE_LINE_FOLLOWING;
                                }
                            }

                            prev_ir_state = curr_ir_state;

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

    }
}

