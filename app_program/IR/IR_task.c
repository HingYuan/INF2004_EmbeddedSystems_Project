#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "IR_task.h"
#include "../Navigator/Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/code39.h"

// typedef struct {
//     UH pattern;
//     B character;
// } CODE_39_ENTRY;

// static const CODE_39_ENTRY code39_dict[] = {
// // Letters
//     {0b100001001, 'A'},
//     {0b001001001, 'B'},
//     {0b101001000, 'C'},
//     {0b000011001, 'D'},
//     {0b100011000, 'E'},
//     {0b001011000, 'F'},
//     {0b000001101, 'G'},
//     {0b100001100, 'H'},
//     {0b001001100, 'I'},
//     {0b000011100, 'J'},
//     {0b100000011, 'K'},
//     {0b001000011, 'L'},
//     {0b101000010, 'M'},
//     {0b000010011, 'N'},
//     {0b100010010, 'O'},
//     {0b001010010, 'P'},
//     {0b000000111, 'Q'},
//     {0b100000110, 'R'},
//     {0b001000110, 'S'},
//     {0b000010110, 'T'},
//     {0b110000001, 'U'},
//     {0b011000001, 'V'},
//     {0b111000000, 'W'},
//     {0b010010001, 'X'},
//     {0b110010000, 'Y'},
//     {0b011010000, 'Z'},
// // Numbers
//     {0b000110100, '0'},
//     {0b100100001, '1'},
//     {0b001100001, '2'},
//     {0b101100000, '3'},
//     {0b000110001, '4'},
//     {0b100110000, '5'},
//     {0b001110000, '6'},
//     {0b000100101, '7'},
//     {0b100100100, '8'},
//     {0b001100100, '9'},
// // Symbols
//     {0b010000101, '-'},
//     {0b110000100, '.'},
//     {0b011000100, ' '},
//     {0b010101000, '$'},
//     {0b010100010, '/'},
//     {0b010001010, '+'},
//     {0b000101010, '%'},
// // Asterisk
//     {0b010010100, '*'}
// };

#define CODE39_DICT_SIZE (sizeof(code39_dict) / sizeof(code39_dict[0]))
  
#define BARS_PER_CHAR      10
#define BAR_THRESHOLD    2000
#define DEBOUNCE_TICKS      5

EXPORT void ir_sensor_zeroing(INT stacd, void *exinf) 
{
    // unpack packed pin numbers
    INT pin_left = (stacd >> 16) & 0xFF;
    INT pin_right = (stacd >> 8) & 0xFF;

    INT left_state;
    INT right_state;

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

    while(1) {
		INT left_state = (INT)gpio_get_val((UINT)pin_left);
        INT right_state = (INT)gpio_get_val((UINT)pin_right);

        tm_printf("[IR left] %d\t", left_state);
        tm_printf("[IR right] %d\n", right_state);

        tk_dly_tsk(1000);
    }
}

EXPORT void ir_sensor_task(INT stacd, void *exinf)
{
    // unpack packed pin numbers
    INT pin_left = (stacd >> 16) & 0xFF;
    INT pin_right = (stacd >> 8) & 0xFF;

    INT left_state;
    INT right_state;

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

    while(1) {
		INT left_state = (INT)gpio_get_val((UINT)pin_left);
        INT right_state = (INT)gpio_get_val((UINT)pin_right);

        NAVIGATOR_MESSAGE nm;
        nm.type = MSG_IR_UPDATE;
        if (left_state == 1 && right_state == 1) nm.data.ir_state = STATE_B_B;          // Black-black
        else if (left_state == 0 && right_state == 0) nm.data.ir_state = STATE_W_W;     // White-white
        else if (left_state == 1 && right_state == 0) nm.data.ir_state = STATE_B_W;     // Black-white
        else nm.data.ir_state = STATE_W_B;                                              // White-black
        
        tk_snd_mbf(navigator_mbfid, (void *)&nm, sizeof(NAVIGATOR_MESSAGE), TMO_FEVR);

        tk_dly_tsk(10);
    }
}

EXPORT void ir_barcode_task(INT stacd, void *exinf)
{
	INT previous_state;
	INT current_state;
    UW current_time;

	(void)exinf;
	if(gpio_set_pin((UINT)stacd, GPIO_MODE_IN) < E_OK) {
		tm_printf((UB *)"[ir] GP%d input setup failed\n", stacd);
		return;
	}

	previous_state = (INT)gpio_get_val((UINT)stacd);
    UW previous_time = time_us();

    tm_printf((UB *)"[ir] Task started, waiting for bars...\n");
	tm_printf((UB *)"[ir] GP%d initial %s\n", stacd, previous_state ? "HIGH" : "LOW");

	while(1) {
		current_state = (INT)gpio_get_val((UINT)stacd);
        current_time = time_us();

        // State change
		if(current_state != previous_state) {
            // Record state
            BAR_RECORD br = {
                previous_state,
                (current_time - previous_time) / 100,
            };
            tm_printf("%d", current_time - previous_time);
            
            // Push to message buffer
            tk_snd_mbf(barcode_mbfid, (void *)&br, sizeof(BAR_RECORD), TMO_FEVR);

            tm_printf((UB *)"[ir] GP%d %s %u %u\n", stacd, current_state ? "HIGH" : "LOW", current_time-previous_time, (current_time-previous_time)>=BAR_THRESHOLD ? 1 : 0);

            // reset state
			previous_state = current_state;
            previous_time = current_time;
		}
		tk_dly_tsk(10);
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

                UW sum = 0;
                UW average = 0;

                // Find average duration of all bars
                for (INT i=1; i<BARS_PER_CHAR; i++) {
                    sum += bar_arr[i].duration;
                }
                average = sum / (BARS_PER_CHAR - 1); // find average duration of the 9 bars

                // Create bit-shifted binary value based on detected Code39 character. Use average as threshold between thin and thick
                for (INT i=1; i<BARS_PER_CHAR; i++) {
                    // Detect wide bars
                    if (bar_arr[i].duration > average) {
                        current_pattern |= (1 << (BARS_PER_CHAR-1-i)); // -1 for leading whitespace, -i for digit offset (which includes the -1 for bitshift traversal as loop starts from 1, not 0)
                    }
                    // if thin bar, do nothing
                }

                // default unknown characters to '?'
                B decoded_char = '?';

                // Search code39_dict for pattern
                for (INT i = 0; i < CODE39_DICT_SIZE; i++) {
                    if (code39_dict[i].pattern == current_pattern) {
                        decoded_char = code39_dict[i].character;
                        break;
                    }
                }

                tm_printf((UB *)"[decoder] Matched pattern 0x%03X to character: %c\n", current_pattern, decoded_char);

                if (decoded_char != '*') {  //ignore asterisk
                    // Construct and send message to navigator
                    NAVIGATOR_MESSAGE nm = {
                        .type = MSG_BARCODE_DECODED,
                        .data.decoded_char = decoded_char,
                    };

                    tk_snd_mbf(navigator_mbfid, (void *)&nm, sizeof(NAVIGATOR_MESSAGE), TMO_FEVR);
                }

                // Decrease bar_count to remove the decoded character's bars
                bar_count -= BARS_PER_CHAR;
            }
		}

        tk_dly_tsk(10);
	}
}