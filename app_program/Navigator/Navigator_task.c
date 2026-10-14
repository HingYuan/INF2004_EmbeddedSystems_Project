#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;

    MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;

    // initialize circular ring buffer to store decoded characters
    C_RING_BUFFER barcode_characters;
    crb_init(&barcode_characters);

    while (1) {
        message_size = tk_rcv_mbf(navigator_mbfid, (void *)&n_msg, 50); // sleep for 50 ms if no message, send E_TMOUT once 50 ms passes with no new message

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            switch (n_msg.type) {
                case MSG_IR_UPDATE:
                    switch (m_mode) {
                        // Handle line following.
                        case MODE_LINE_FOLLOWING:
                            IR_STATE new_state = n_msg.data.ir_state;
                            
                            if (new_state == STATE_W_W) {
                                // Do nothing. Car is following line properly.
                            } else if (new_state == STATE_W_B) {
                                // Car veering left, tell motor to turn right

                            } else if (new_state == STATE_B_W) {
                                // Car veering right, tell motor to turn left

                            } else if (new_state == STATE_B_B) {
                                // Intersection detected, follow next instruction from instruction hashmap

                            }

                            break;
                        // Line detected during wall hugging, stop car, switch to reentry: advance.
                        case MODE_WALL_HUGGING:
                            

                            break;
                        // Move car forward by car length minus 30 ticks from debouncing, then rotate car and switch to reentry: pivot
                        case MODE_REENTRY_ADVANCE:
                        

                            break;
                        // Line detected during rotation, stop car and switch back to line following.
                        case MODE_REENTRY_PIVOT:


                            break;
                    }

                    break;
                case MSG_BARCODE_DECODED:
                    // push decoded character into circular ring buffer
                    crb_push(&barcode_characters, n_msg.data.decoded_char);

                    // match with network's CRB(to do) to match each decoded letter to the commands, store into instruction hashmap

                    break;
                case MSG_OBSTACLE_DETECTED:

                    m_mode = MODE_WALL_HUGGING;

                    // perform necessary actions
                    // - if obstacle in front, close in to it
                    // - turn left/right and update DIRECTION struct
                    // - navigate around obstacle

                    
                    break;
            }
        // No message from mbf for 50ms
        } else if (message_size == E_TMOUT) {
            switch (m_mode) {
                // Move forward
                case MODE_LINE_FOLLOWING:


                    break;
                // Stop car
                case MODE_WALL_HUGGING:
                    

                    break;
                // Do nothing, car must keep moving forward from above reentry-advance code
                case MODE_REENTRY_ADVANCE:


                    break;
                // Do nothing, car must keep rotating from above reentry-pivot code
                case MODE_REENTRY_PIVOT:


                    break;
            }
        } else {
            
            // Fatal RTOS error occurred (e.g., buffer was deleted or ID is invalid)

        }
    }
}

