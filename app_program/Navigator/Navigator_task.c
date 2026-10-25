#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"
#include "../Motor/Motor_task.h"
#include "../Ultrasonic/Ultrasonic_task.h"

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;

    MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;
    BOOL scan_requested = FALSE;

    // initialize circular ring buffer to store decoded characters
    C_RING_BUFFER barcode_characters;
    crb_init(&barcode_characters);

    INT left_debounce_ticks = 0;
    INT right_debounce_ticks = 0;

    MOTOR_COMMAND mot_cmd = { 0, 0, 0 };

    INT instruction_idx = 0;
    DIRECTION instructions_fake[10] = { DIR_LEFT, DIR_RIGHT, DIR_FRONT };

    IR_STATE prev_ir_state = STATE_NULL;
    INT curr_speed_left = 0;
    INT curr_speed_right = 0;

    while (1) {
        message_size = tk_rcv_mbf(navigator_mbfid, (void *)&n_msg, TMO_FEVR); // Set as timeout forever (TMO_FEVR) as IR is continuously sending updates, so no timeout needed.

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            if (n_msg.type == MSG_REMOTE_CMD) {
                MOTOR_COMMAND mot_cmd = n_msg.data.remote_cmd;
                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                curr_speed_left = mot_cmd.target_speed_left;
                curr_speed_right = mot_cmd.target_speed_right;
                tm_printf("[navigator] Remote CMD: L=%d R=%d Turn=%d\n",
                          mot_cmd.target_speed_left, mot_cmd.target_speed_right, mot_cmd.turn_ticks);
            } else {
                switch (m_mode) {
                    case MODE_LINE_FOLLOWING:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:
                                IR_STATE curr_state = n_msg.data.ir_state;

                                if (curr_state != prev_ir_state) {

                                if (curr_state == STATE_W_W) {
                                    mot_cmd.target_speed_left =     30;
                                    mot_cmd.target_speed_right =   -30;
                                    mot_cmd.turn_ticks =             0;

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

                            m_mode = MODE_WALL_HUGGING;

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

            // Update shared telemetry snapshot
            telemetry_snapshot.speed_left = curr_speed_left;
            telemetry_snapshot.speed_right = curr_speed_right;
            telemetry_snapshot.ir_state = (uint8_t)prev_ir_state;
            telemetry_snapshot.movement_mode = (uint8_t)m_mode;
            telemetry_snapshot.enc_l = (int32_t)enc_l_ticks;
            telemetry_snapshot.enc_r = (int32_t)enc_r_ticks;
            telemetry_snapshot.seq++;

        // Unexpected behaviour.
        } else {

            // Fatal RTOS error occurred (e.g., buffer was deleted or ID is invalid)

        }

        tk_dly_tsk(10);
    }
}

