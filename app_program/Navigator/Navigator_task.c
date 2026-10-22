#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"
#include "../Motor/Motor_task.h"

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;

    MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;

    // initialize circular ring buffer to store decoded characters
    C_RING_BUFFER barcode_characters;
    crb_init(&barcode_characters);

    INT left_debounce_ticks = 0;
    INT right_debounce_ticks = 0;

    IR_STATE prev_ir_state = STATE_NULL;

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
                                    MOTOR_COMMAND mot_cmd = { 20, -20, 0 };
                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    tm_printf("W_W\n");

                                } else if (curr_state == STATE_W_B) {
                                    // Car veering left, adjust right
                                    MOTOR_COMMAND mot_cmd = { 25, -15, 0 };
                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                    tm_printf("W_B\n");

                                } else if (curr_state == STATE_B_W) {
                                    // Car veering right, adjust left
                                    MOTOR_COMMAND mot_cmd = { 15, -25, 0 };
                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                } else if (curr_state == STATE_B_B) {
                                    // Car at instersection
                                    MOTOR_COMMAND mot_cmd = { 0, -0, 0 };
                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    
                                    m_mode = MODE_DECISION_MAKING;

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
                            motor_estop = TRUE;             // set to stop for now 
                            m_mode = MODE_WALL_HUGGING;
                            prev_ir_state = STATE_NULL;

                            tm_printf((UB *)"[nav] Obstacle at %u cm; stopped\n", n_msg.data.obstacle_distance_cm);

                            break;
                        default:
                            // Ignore other states

                            break;
                    }

                    break;
                case MODE_DECISION_MAKING:
                    

                    break;
                case MODE_WALL_HUGGING:
                    switch (n_msg.type) {
                        case MSG_IR_UPDATE:

                            break;
                        case MSG_OBSTACLE_DETECTED:


                            break;
                        case MSG_TRACKING_OBSTACLE:


                            break;
                        case MSG_OBSTACLE_PROFILED: {   // require brackets for variable 
                            // Put the profiling here for now
                            const OBSTACLE_PROFILE *p = &n_msg.data.obstacle_profile;

                            tm_printf((UB *)"[nav] Scan: L=%u(%d) C=%u(%d) R=%u(%d)\n",
                                    p->left_cm, p->left_valid,
                                    p->center_cm, p->center_valid,
                                    p->right_cm, p->right_valid);

                            // No bypass command yet: keep the car stopped.
                            break;
                        }
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

        // Unexpected behaviour.
        } else {

            // Fatal RTOS error occurred (e.g., buffer was deleted or ID is invalid)

        }

        tk_dly_tsk(10);
    }
}

