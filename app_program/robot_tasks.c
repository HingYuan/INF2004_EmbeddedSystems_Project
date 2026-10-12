#include <stdint.h>

#include <sys/machine.h>
#include <sys/sysdef.h>
#include <tk/tkernel.h>
#include <tk/device.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "robot_tasks.h"

/* 
==========================
|   ULTRASONIC SENSOR    |
==========================
*/

#define SERVO_PIN          15
#define SONAR_TRIG_PIN      1
#define SONAR_ECHO_PIN      0

#define SERVO_PWM_WRAP   19999U
#define SONAR_TIMEOUT_US 26000U
#define SERVO_POSITION_READY   (1U << 0)
#define SONAR_MEASUREMENT_DONE (1U << 1)

/* 
==========================
|       IR SENSOR        |
==========================
*/

typedef struct {
    INT state;
    UW duration;
} BAR_RECORD;

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

#define IR_LEFT_PIN         5
#define IR_CENTER_PIN      17
#define IR_RIGHT_PIN       26
#define BARS_PER_CHAR      10
#define BAR_THRESHOLD    2500

/* 
==========================
|     MOTOR ENCODER      |
==========================
*/

#define MOTOR_LEFT_A        8
#define MOTOR_LEFT_B        9
#define MOTOR_RIGHT_A      10
#define MOTOR_RIGHT_B      11
#define MOTOR_LEFT_ENC_A    2
#define MOTOR_LEFT_ENC_B    3
#define MOTOR_RIGHT_ENC_A  26
#define MOTOR_RIGHT_ENC_B  27

#define MOTOR_PWM_WRAP    6249U
#define MOTOR_TEST_PERCENT  15U

/* 
==========================
|       NAVIGATOR        |
==========================
*/

typedef enum {
    MODE_LINE_FOLLOWING,    // IR is aware of line and navigator is following it
    MODE_WALL_HUGGING,      // Ultrasonic sensor is aware of obstacle and navigtor is navigating around it
    MODE_REENTRY_DETECT,    // IR detects black line during MODE_WALL_HUGGING
    MODE_REENTRY_ADVANCE,   // Navigator moves car forward onto black line after MODE_REENTRY_DETECT to position itself for pivot
    MODE_REENTRY_PIVOT,     // Navigator spins the car in place to align with line
} MOVEMENT_MODE;

typedef enum {
    STATE_W_W,
    STATE_W_B,
    STATE_B_W,
    STATE_B_B,
} IR_STATE;

typedef enum {
    DIR_LEFT,
    DIR_RIGHT,
} DIRECTION;

typedef enum {
    MSG_IR_UPDATE,
    MSG_OBSTACLE_DETECTED,
    MSG_BARCODE_DECODED,
} MESSAGE_TYPE;

typedef struct {
    MESSAGE_TYPE type;
    union {
        IR_STATE ir_state;
        VB decoded_char;
    } data;                     // Union data type for variable message data, update as project expands.
} NAVIGATOR_MESSAGE;

/* 
==========================
|     T KERNEL MISC      |
==========================    
*/

#define PERIPHERAL_TASK_STACK    4096

typedef void (*PERIPHERAL_TASK)(INT stacd, void *exinf);

typedef struct {
	PERIPHERAL_TASK task;
    INT priority;
	INT stacd;
	UW assprc;
	const char *name;
} TASK_START;

// Message buffers
LOCAL ID barcode_mbfid;
LOCAL ID navigator_mbfid;

// Flags
LOCAL ID ranger_flgid;

/* 
==========================
|          CODE          |
==========================    
*/

LOCAL UW time_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL void delay_us(UW delay)
{
	UW start = time_us();

	while((UW)(time_us() - start) < delay) {
	}
}

LOCAL BOOL read_distance_tenths_cm(UW *distance)
{
	UW start;
	UW echo_start;
	UW pulse_width;

	gpio_set_val(SONAR_TRIG_PIN, 0);
	delay_us(2);
	gpio_set_val(SONAR_TRIG_PIN, 1);
	delay_us(10);
	gpio_set_val(SONAR_TRIG_PIN, 0);

	start = time_us();
	while(gpio_get_val(SONAR_ECHO_PIN) == 0) {
		if((UW)(time_us() - start) >= SONAR_TIMEOUT_US) return FALSE;
	}
	echo_start = time_us();

	while(gpio_get_val(SONAR_ECHO_PIN) != 0) {
		if((UW)(time_us() - echo_start) >= SONAR_TIMEOUT_US) return FALSE;
	}
	pulse_width = time_us() - echo_start;
	*distance = (pulse_width * 343U + 1000U) / 2000U;
	return TRUE;
}

LOCAL void servo_task(INT stacd, void *exinf)
{
	static const UINT positions[] = { 2045U, 1695U, 1345U };
	UINT pattern;
	ER er;
	INT i;

	(void)stacd;
	(void)exinf;
	if(pwm_set_divider(SERVO_PIN, 125, 0) < E_OK
	   || pwm_set_wrap(SERVO_PIN, SERVO_PWM_WRAP) < E_OK
	   || pwm_set_pin(SERVO_PIN) < E_OK
	   || pwm_set_cc(SERVO_PIN, positions[1]) < E_OK
	   || pwm_set_enabled(SERVO_PIN, TRUE) < E_OK) {
		tm_printf((UB *)"[servo] PWM initialization failed\n");
		return;
	}
	tm_printf((UB *)"[servo] PWM output on GP%d\n", SERVO_PIN);

	while(1) {
		for(i = 0; i < 3; i++) {
			if(pwm_set_cc(SERVO_PIN, positions[i]) < E_OK) return;
			tk_dly_tsk(300);
			er = tk_set_flg(ranger_flgid, SERVO_POSITION_READY);
			if(er < E_OK) return;
			er = tk_wai_flg(ranger_flgid, SONAR_MEASUREMENT_DONE,
					TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
			if(er < E_OK) return;
		}
		if(pwm_set_cc(SERVO_PIN, positions[1]) < E_OK) return;
		tk_dly_tsk(300);
		tk_dly_tsk(500);
	}
}

LOCAL void ultrasonic_task(INT stacd, void *exinf)
{
	UW distance[3];
	BOOL valid[3];
	UW scan = 0;
	UINT pattern;
	UINT position = 0;
	ER er;

	(void)stacd;
	(void)exinf;
	if(gpio_set_pin(SONAR_TRIG_PIN, GPIO_MODE_OUT) < E_OK
	   || gpio_set_pin(SONAR_ECHO_PIN, GPIO_MODE_IN) < E_OK) {
		tm_printf((UB *)"[sonar] GPIO initialization failed\n");
		return;
	}
	gpio_set_val(SONAR_TRIG_PIN, 0);
	tm_printf((UB *)"[sonar] trigger GP%d, echo GP%d\n",
		  SONAR_TRIG_PIN, SONAR_ECHO_PIN);

	while(1) {
		er = tk_wai_flg(ranger_flgid, SERVO_POSITION_READY,
				TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
		if(er < E_OK) return;
		valid[position] = read_distance_tenths_cm(&distance[position]);
		er = tk_set_flg(ranger_flgid, SONAR_MEASUREMENT_DONE);
		if(er < E_OK) return;
		position++;
		if(position < 3) continue;

		scan++;
		tm_printf((UB *)"[sonar] scan %u left=", scan);
		if(valid[0]) {
			tm_printf((UB *)"%u.%u cm", distance[0] / 10U, distance[0] % 10U);
		} else {
			tm_printf((UB *)"timeout");
		}
		tm_printf((UB *)" center=");
		if(valid[1]) {
			tm_printf((UB *)"%u.%u cm", distance[1] / 10U, distance[1] % 10U);
		} else {
			tm_printf((UB *)"timeout");
		}
		tm_printf((UB *)" right=");
		if(valid[2]) {
			tm_printf((UB *)"%u.%u cm", distance[2] / 10U, distance[2] % 10U);
		} else {
			tm_printf((UB *)"timeout");
		}
		tm_printf((UB *)"\n");
		position = 0;
	}
}

LOCAL void ir_sensor_task(INT pin_left, INT pin_right, INT side, void *exinf)
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

LOCAL void ir_barcode_task(INT pin, void *exinf)
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

LOCAL void ir_barcode_decoder_task(INT stacd, void *exinf) 
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

LOCAL INT quadrature_delta(UINT previous, UINT current)
{
	static const int8_t delta[16] = {
		 0, -1,  1,  0,
		 1,  0,  0, -1,
		-1,  0,  0,  1,
		 0,  1, -1,  0
	};

	return delta[(previous << 2) | current];
}

LOCAL UINT read_encoder_state(UINT pin_a, UINT pin_b)
{
	return ((gpio_get_val(pin_a) != 0) ? 2U : 0U)
		| ((gpio_get_val(pin_b) != 0) ? 1U : 0U);
}

LOCAL void motor_task(INT stacd, void *exinf)
{    
	static const UINT output_pins[] = {
		MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_RIGHT_A, MOTOR_RIGHT_B
	};
	static const UINT encoder_pins[] = {
		MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B,
		MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B
	};
	const UW duty = (MOTOR_PWM_WRAP + 1U) * MOTOR_TEST_PERCENT / 100U;
	UW last_report = time_us();
	UINT left_state;
	UINT right_state;
	int32_t left_ticks = 0;
	int32_t right_ticks = 0;
	INT i;

	(void)stacd;
	(void)exinf;
	for(i = 0; i < 4; i++) {
		if(pwm_set_divider(output_pins[i], 1, 0) < E_OK
		   || pwm_set_wrap(output_pins[i], MOTOR_PWM_WRAP) < E_OK
		   || pwm_set_pin(output_pins[i]) < E_OK
		   || pwm_set_cc(output_pins[i], 0) < E_OK
		   || pwm_set_enabled(output_pins[i], TRUE) < E_OK) {
			tm_printf((UB *)"[motors] PWM setup failed on GP%d\n", output_pins[i]);
			return;
		}
	}
	for(i = 0; i < 4; i++) {
		if(gpio_set_pin(encoder_pins[i], GPIO_MODE_IN) < E_OK) {
			tm_printf((UB *)"[motors] encoder GPIO setup failed on GP%d\n",
				  encoder_pins[i]);
			return;
		}
	}

	left_state = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
	right_state = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);
	if(pwm_set_cc(MOTOR_LEFT_A, duty) < E_OK
	   || pwm_set_cc(MOTOR_LEFT_B, 0) < E_OK
	   || pwm_set_cc(MOTOR_RIGHT_A, 0) < E_OK
	   || pwm_set_cc(MOTOR_RIGHT_B, duty) < E_OK) {
		tm_printf((UB *)"[motors] output setup failed\n");
		return;
	}
	tm_printf((UB *)"[motors] forward test at %u%%; encoder inputs GP%d/%d, GP%d/%d\n",
		  MOTOR_TEST_PERCENT, MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B,
		  MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);

	while(1) {
		UINT next_left = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
		UINT next_right = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);
		UW now = time_us();

		left_ticks += quadrature_delta(left_state, next_left);
		right_ticks += quadrature_delta(right_state, next_right);
		left_state = next_left;
		right_state = next_right;

        UW elapsed = now - last_report;
		if(elapsed >= 10000) {
			last_report = now;
			// tm_printf((UB *)"[motors] dt=%u left=%d right=%d ticks\n",
			// 	  elapsed, left_ticks, right_ticks);
		}
		tk_dly_tsk(1);
	}
}
/*
LOCAL ER read_accel(ID iic, int16_t *x, int16_t *y, int16_t *z)
{
	UB data[6];
	INT i;

	for(i = 0; i < 6; i++) {
		ER er = i2c_read_reg(iic, 0x19, (UW)(0xA8 + i), &data[i]);
		if(er < E_OK) return er;
	}
	*x = (int16_t)(((UINT)data[1] << 8) | data[0]) >> 4;
	*y = (int16_t)(((UINT)data[3] << 8) | data[2]) >> 4;
	*z = (int16_t)(((UINT)data[5] << 8) | data[4]) >> 4;
	return E_OK;
}

LOCAL ER read_magnetometer(ID iic, int16_t *x, int16_t *y, int16_t *z)
{
	UB data[6];
	INT i;
	static const UB registers[6] = { 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

	for(i = 0; i < 6; i++) {
		ER er = i2c_read_reg(iic, 0x1E, registers[i], &data[i]);
		if(er < E_OK) return er;
	}
	*x = (int16_t)(((UINT)data[0] << 8) | data[1]);
	*z = (int16_t)(((UINT)data[2] << 8) | data[3]);
	*y = (int16_t)(((UINT)data[4] << 8) | data[5]);
	return E_OK;
}

LOCAL void imu_task(INT stacd, void *exinf)
{
	ID iic;
	ER er;
	int16_t ax, ay, az;
	int16_t mx, my, mz;

	(void)stacd;
	(void)exinf;
	iic = tk_opn_dev((const UB *)"iica", TD_UPDATE);
	if(iic <= E_OK) {
		tm_printf((UB *)"[imu] cannot open iica, er=%d\n", iic);
		return;
	}

	er = i2c_write_reg(iic, 0x19, 0x20, 0x57);
	if(er >= E_OK) er = i2c_write_reg(iic, 0x19, 0x23, 0x08);
	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x00, 0x14);
	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x01, 0x20);
	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x02, 0x00);
	if(er < E_OK) {
		tm_printf((UB *)"[imu] sensor setup failed, er=%d\n", er);
		(void)tk_cls_dev(iic, 0);
		return;
	}
	tm_printf((UB *)"[imu] LSM303DLHC at accel=0x19 mag=0x1e; raw units\n");

	while(1) {
		er = read_accel(iic, &ax, &ay, &az);
		if(er >= E_OK) er = read_magnetometer(iic, &mx, &my, &mz);
		if(er >= E_OK) {
			tm_printf((UB *)"[imu] accel=%d,%d,%d mag=%d,%d,%d\n",
				  ax, ay, az, mx, my, mz);
		} else {
			tm_printf((UB *)"[imu] I2C read failed, er=%d\n", er);
		}
		tk_dly_tsk(250);
	}
}
*/

LOCAL ER start_task(PERIPHERAL_TASK entry, INT priority, INT stacd, UW assprc, const char *name)
{
	T_CTSK ctsk = {0};
	ID tid;
	ER er;

	ctsk.tskatr = TA_HLNG | TA_RNG3;
	#if TK_SUPPORT_SMP
	if(assprc != 0) {
		ctsk.tskatr |= TA_ASSPRC;
		ctsk.assprc = assprc;
	}
	#else
	(void)assprc;
	#endif
	ctsk.itskpri = priority;
	ctsk.stksz = PERIPHERAL_TASK_STACK;
	ctsk.task = entry;
	tid = tk_cre_tsk(&ctsk);
	if(tid <= E_OK) {
		tm_printf((UB *)"[init] create %s failed, er=%d\n", (UB *)name, tid);
		return (ER)tid;
	}

	er = tk_sta_tsk(tid, stacd);
	if(er < E_OK) {
		tm_printf((UB *)"[init] start %s failed, er=%d\n", (UB *)name, er);
		(void)tk_del_tsk(tid);
	}
	return er;
}

EXPORT ER robot_tasks_start(void)
{
    tk_dly_tsk(1000);

	static const TASK_START tasks[] = {
		// { servo_task, 0, "servo" },
		// { ultrasonic_task, 0, "ultrasonic" },
		// { ir_sensor_task, IR_LEFT_PIN, "IR-left" },
		// { ir_sensor_task, IR_CENTER_PIN, "IR-center" },
		{ ir_barcode_task, 5, IR_LEFT_PIN, TP_PRC1, "IR-right" },
		{ ir_barcode_decoder_task, 5, 0, TP_PRC1, "IR-barcode-decoder" },
		{ motor_task, 9, 0, TP_PRC2, "motors" },
		// { imu_task, 0, "IMU" }
	};
    
    // Create flags
	T_CFLG cflg = {0};
	cflg.flgatr = TA_TFIFO;

	ranger_flgid = tk_cre_flg(&cflg);
	if(ranger_flgid <= E_OK) {
		tm_printf((UB *)"[init] create ranger event flag failed, er=%d\n",
			  ranger_flgid);
		return (ER)ranger_flgid;
	}

    // Create message buffers
    T_CMBF cmbf = {0};
	cmbf.mbfatr = TA_TFIFO; // First-In, First-Out queue
	cmbf.maxmsz = sizeof(BAR_RECORD); // Max size of one message
	cmbf.bufsz = sizeof(BAR_RECORD) * 18; // Capacity for 20 queued bars

	barcode_mbfid = tk_cre_mbf(&cmbf);
	if(barcode_mbfid <= E_OK) {
		tm_printf((UB *)"[init] create barcode mbf failed, er=%d\n", barcode_mbfid);
		return (ER)barcode_mbfid;
	}

    cmbf.maxmsz = sizeof(NAVIGATOR_MESSAGE);
	cmbf.bufsz = sizeof(NAVIGATOR_MESSAGE) * 20;

	navigator_mbfid = tk_cre_mbf(&cmbf);
	if(navigator_mbfid <= E_OK) {
		tm_printf((UB *)"[init] create navigator mbf failed, er=%d\n", navigator_mbfid);
		return (ER)navigator_mbfid;
	}

    UINT i;
	ER er;
	for(i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
		er = start_task(tasks[i].task, tasks[i].priority, tasks[i].stacd, tasks[i].assprc, tasks[i].name);
		if(er < E_OK) return er;
	}
	return E_OK;
}