#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "IR_task.h"
#include "../Navigator/Navigator_task.h"
#include "../robot_tasks.h"

typedef struct {
    UH pattern;
    char character;
} CODE_39_ENTRY;

static const CODE_39_ENTRY code39_dict[] = {
// Letters
    {0b100001001, 'A'},
    {0b001001001, 'B'},
    {0b101001000, 'C'},
    {0b000011001, 'D'},
    {0b100011000, 'E'},
    {0b001011000, 'F'},
    {0b000001101, 'G'},
    {0b100001100, 'H'},
    {0b001001100, 'I'},
    {0b000011100, 'J'},
    {0b100000011, 'K'},
    {0b001000011, 'L'},
    {0b101000010, 'M'},
    {0b000010011, 'N'},
    {0b100010010, 'O'},
    {0b001010010, 'P'},
    {0b000000111, 'Q'},
    {0b100000110, 'R'},
    {0b001000110, 'S'},
    {0b000010110, 'T'},
    {0b110000001, 'U'},
    {0b011000001, 'V'},
    {0b111000000, 'W'},
    {0b010010001, 'X'},
    {0b110010000, 'Y'},
    {0b011010000, 'Z'},
// Numbers
    {0b000110100, '0'},
    {0b100100001, '1'},
    {0b001100001, '2'},
    {0b101100000, '3'},
    {0b000110001, '4'},
    {0b100110000, '5'},
    {0b001110000, '6'},
    {0b000100101, '7'},
    {0b100100100, '8'},
    {0b001100100, '9'},
// Symbols
    {0b010000101, '-'},
    {0b110000100, '.'},
    {0b011000100, ' '},
    {0b010101000, '$'},
    {0b010100010, '/'},
    {0b010001010, '+'},
    {0b000101010, '%'},
// Asterisk
    {0b010010100, '*'}
};

#define CODE39_DICT_SIZE (sizeof(code39_dict) / sizeof(code39_dict[0]))
  
#define BARS_PER_CHAR      10
#define BAR_THRESHOLD    2500

EXPORT void ir_sensor_task(INT pin_left, INT pin_right, INT side, void *exinf)
{
	INT left_prev_state;
    INT left_curr_state;

	INT right_prev_state;
    INT right_curr_state;

    INT left_state_change;
    INT right_state_change;

    // Set up left IR
	(void)exinf;
	if(gpio_set_pin((UINT)pin_left, GPIO_MODE_IN) < E_OK) {
		tm_printf((UB *)"[ir] GP%d input setup failed\n", pin_left);
		return;
	}

    // Set up right IR
    (void)exinf;
	if(gpio_set_pin((UINT)pin_right, GPIO_MODE_IN) < E_OK) {
		tm_printf((UB *)"[ir] GP%d input setup failed\n", pin_right);
		return;
	}

    left_prev_state = (INT)gpio_get_val((UINT)pin_left);
    right_prev_state = (INT)gpio_get_val((UINT)pin_right);
    
	while(1) {
		left_curr_state = (INT)gpio_get_val((UINT)pin_left);
        right_curr_state = (INT)gpio_get_val((UINT)pin_right);

        left_state_change = left_curr_state != left_prev_state;
        right_state_change = right_curr_state != right_prev_state;
        
        if (left_state_change || right_state_change) {
			// Construct and send message to navigator
			NAVIGATOR_MESSAGE nm;
			nm.type = MSG_IR_UPDATE;

			// If state change is for both sensors
            if (left_state_change == right_state_change && right_state_change == 1) {			// change to Black-Black
				nm.data.ir_state = STATE_B_B;
            } else if (left_state_change == right_state_change && right_state_change == 0) {	// change to White-White
				nm.data.ir_state = STATE_W_W;
            }

			// If state change for only one sensor
            if (left_state_change > right_state_change) {										// change to Black-White
				nm.data.ir_state = STATE_B_W;
            } else if (left_state_change < right_state_change) {								// change to White-Black
				nm.data.ir_state = STATE_W_B;
            }

			tk_snd_mbf(navigator_mbfid, (void *)&nm, sizeof(NAVIGATOR_MESSAGE), TMO_FEVR);
        }

		tk_dly_tsk(10);
	}
}

EXPORT void ir_barcode_task(INT pin, void *exinf)
{
	INT previous_state;
	INT current_state;
    UW current_time;

	(void)exinf;
	if(gpio_set_pin((UINT)pin, GPIO_MODE_IN) < E_OK) {
		tm_printf((UB *)"[ir] GP%d input setup failed\n", pin);
		return;
	}

	previous_state = (INT)gpio_get_val((UINT)pin);
    UW previous_time = time_us();

	tm_printf((UB *)"[ir] GP%d initial %s\n", pin, previous_state ? "HIGH" : "LOW");

	while(1) {
		current_state = (INT)gpio_get_val((UINT)pin);
        current_time = time_us();

        // State change
		if(current_state != previous_state) {
            // Record state
            BAR_RECORD br = {
                previous_state,
                current_time - previous_time,
            };
            
            // Push to message buffer
            tk_snd_mbf(barcode_mbfid, (void *)&br, sizeof(BAR_RECORD), TMO_FEVR);

            tm_printf((UB *)"[ir] GP%d %s %u\n", pin, current_state ? "HIGH" : "LOW", current_time-previous_time);

            // reset state
			previous_state = current_state;
            previous_time = current_time;
		}
		tk_dly_tsk(5);
	}
}

EXPORT void ir_barcode_decoder_task(INT stacd, void *exinf) 
{
    BAR_RECORD br;
    BAR_RECORD bar_arr[BARS_PER_CHAR];
	INT msg_size;
    INT bar_count = 0;

	(void)stacd;
	(void)exinf;
	tm_printf((UB *)"[decoder] Task started, waiting for bars...\n");

	while(1) {
		// Wait asleep until a message arrives in the buffer
		msg_size = tk_rcv_mbf(barcode_mbfid, (void *)&br, TMO_FEVR);
		
		if (msg_size == sizeof(BAR_RECORD)) {

            // Add to array
			bar_arr[bar_count] = br;
            bar_count++;
                      
            // Decode only if array exceeds 10 bars
            if (bar_count >= BARS_PER_CHAR) {
                UH current_pattern = 0;

                // Skip first white bar,
                // Create bit-shifted binary value based on detected Code39 character.
                for (INT i=1; i<BARS_PER_CHAR; i++) {
                    // Detect wide bars
                    if (bar_arr[i].duration > BAR_THRESHOLD) {
                        current_pattern |= (1 << (BARS_PER_CHAR-1-i)); // -1 for leading whitespace, -i for digit offset (which includes the -1 for bitshift traversal as loop starts from 1, not 0)
                    }
                    // if thin bar, do nothing
                }

                // default unknown characters to '?'
                char decoded_char = '?';

                // Search code39_dict for pattern
                for (INT i = 0; i < CODE39_DICT_SIZE; i++) {
                    if (code39_dict[i].pattern == current_pattern) {
                        decoded_char = code39_dict[i].character;
                        break;
                    }
                }

                tm_printf((UB *)"[decoder] Matched pattern 0x%03X to character: %c\n", current_pattern, decoded_char);

                // Construct and send message to navigator
                NAVIGATOR_MESSAGE nm = {
                    .type = MSG_BARCODE_DECODED,
                    .data.decoded_char = decoded_char,
                };

                tk_snd_mbf(navigator_mbfid, (void *)&nm, sizeof(NAVIGATOR_MESSAGE), TMO_FEVR);

                // Decrease bar_count to remove the decoded character's bars
                bar_count -= BARS_PER_CHAR;
            }
		}
	}
}