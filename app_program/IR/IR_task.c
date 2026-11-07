#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "IR_task.h"
#include "../Navigator/Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/code39.h"
  
#define BARS_PER_CHAR      10   // Includes whitespace before the first bar (1 + 9 Code39 bars)

EXPORT void ir_sensor_zeroing(INT stacd, void *exinf)
{
    // unpack packed pin numbers
    INT pin_left = (stacd >> 16) & 0xFF;
    INT pin_right = (stacd >> 8) & 0xFF;
    INT pin_barcode = 7;

    INT left_state;
    INT right_state;
    INT barcode_state;

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

    ID adc_dd = tk_opn_dev((const UB *)"adca", TD_READ);
    if (adc_dd < E_OK) {
        tm_printf((UB *)"[ir] opening ADC failed: %d\n", adc_dd);
        return;
    }

    while(1) {
		INT left_state = (INT)gpio_get_val((UINT)pin_left);
        INT right_state = (INT)gpio_get_val((UINT)pin_right);

        UW barcode_raw;
        SZ actual;
        ER err;

        err = tk_srea_dev(adc_dd, 0, &barcode_raw, 1, &actual);
        if (err != E_OK) {
            tm_printf((UB *)"[ir] ADC read failed: %d\n", err);
            return;
        }

        INT barcode_mv = (INT)((barcode_raw * 3300U) / 4095U);

        tm_printf("[IR left] %d\t", left_state);
        tm_printf("[IR right] %d\n", right_state);
        tm_printf("[IR barcode] %d\n", barcode_mv);

        tk_dly_tsk(20);
    }
}

EXPORT void ir_sensor_task(INT stacd, void *exinf)
{
    // unpack packed pin numbers
    UB pin_left = (stacd >> 8) & 0xFF;
    UB pin_right = stacd & 0xFF;

    ID adc_dd = tk_opn_dev((const UB *)"adca", TD_READ);
    if (adc_dd < E_OK) {
        tm_printf((UB *)"[ir] opening ADC failed: %d\n", adc_dd);
        return;
    }

    while(1) {
        UW raw_left, raw_right;
        SZ actual;
        ER err;

        err = tk_srea_dev(adc_dd, 1, &raw_right, 1, &actual);
        if (err != E_OK) {
            tm_printf((UB *)"[ir] GP26 ADC read failed: %d\n", err);
            return;
        }

        err = tk_srea_dev(adc_dd, 2, &raw_left, 1, &actual);
        if (err != E_OK) {
            tm_printf((UB *)"[ir] GP27 ADC read failed: %d\n", err);
            return;
        }

        UH right_state = (UH)((raw_right * 3300U) / 4095U);
        UH left_state = (UH)((raw_left * 3300U) / 4095U);

        NAVIGATOR_MESSAGE nm;
        nm.type = MSG_IR_UPDATE;
        if (left_state >= ADC_IR_THRESHOLD && right_state >= ADC_IR_THRESHOLD) nm.data.ir_state = STATE_B_B;        // Black-black
        else if (left_state < ADC_IR_THRESHOLD && right_state < ADC_IR_THRESHOLD) nm.data.ir_state = STATE_W_W;     // White-white
        else if (left_state >= ADC_IR_THRESHOLD && right_state < ADC_IR_THRESHOLD) nm.data.ir_state = STATE_B_W;    // Black-white
        else nm.data.ir_state = STATE_W_B;                                                                          // White-black

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
            
            // Push to message buffer
            tk_snd_mbf(barcode_mbfid, (void *)&br, sizeof(BAR_RECORD), TMO_FEVR);

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

                if (decoded_char != '*' && decoded_char != '?') {  //ignore asterisk
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