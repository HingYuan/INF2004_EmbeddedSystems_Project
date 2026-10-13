#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Motor_task.h"

// 125 MHz / (6249 + 1) = 20 kHz PWM
#define MOTOR_PWM_WRAP  6249U

EXPORT volatile W enc_l_ticks = 0; // Left wheel encoder tick count
EXPORT volatile W enc_r_ticks = 0; // Right wheel encoder tick count

// The BSP has no GPIO interrupt API, so encoders are polled every 1 ms
// and decoded as quadrature (state = A<<1 | B).
LOCAL INT quadrature_delta(UINT previous, UINT current)
{
	static const B delta[16] = {
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

// Drive one motor. speed: -100..100 percent.
LOCAL void set_motor(UINT pin_a, UINT pin_b, INT speed)
{
	UW duty;

	if(speed > 100) speed = 100;
	if(speed < -100) speed = -100;
	duty = (MOTOR_PWM_WRAP + 1U) * (UW)(speed < 0 ? -speed : speed) / 100U;

	if(speed >= 0) {
		pwm_set_cc(pin_a, duty);
		pwm_set_cc(pin_b, 0);
	} else {
		pwm_set_cc(pin_a, 0);
		pwm_set_cc(pin_b, duty); // Flip direction
	}
}

EXPORT void motor_task(INT stacd, void *exinf)
{
	static const UINT output_pins[] = {
		MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_RIGHT_A, MOTOR_RIGHT_B
	};
	static const UINT encoder_pins[] = {
		MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B,
		MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B
	};
	MotionCommand cmd = {0, 0};
	UINT left_state, right_state;
	INT i;

	(void)stacd;
	(void)exinf;

	// Set up Motor Pins (start with motors safely off)
	for(i = 0; i < 4; i++) {
		if(pwm_set_divider(output_pins[i], 1, 0) < E_OK
		   || pwm_set_wrap(output_pins[i], MOTOR_PWM_WRAP) < E_OK
		   || pwm_set_pin(output_pins[i]) < E_OK
		   || pwm_set_cc(output_pins[i], 0) < E_OK
		   || pwm_set_enabled(output_pins[i], TRUE) < E_OK) {
			tm_printf((UB *)"[motors] PWM setup failed on GP%d\n", output_pins[i]);
			tk_ext_tsk();
		}
	}

	// Set up Encoder Pins (inputs)
	for(i = 0; i < 4; i++) {
		if(gpio_set_pin(encoder_pins[i], GPIO_MODE_IN) < E_OK) {
			tm_printf((UB *)"[motors] encoder GPIO setup failed on GP%d\n",
				  encoder_pins[i]);
			tk_ext_tsk();
		}
	}

	left_state = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
	right_state = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);

	while(1) {
		UINT next_left = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
		UINT next_right = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);

		enc_l_ticks += quadrature_delta(left_state, next_left);
		enc_r_ticks += quadrature_delta(right_state, next_right);
		left_state = next_left;
		right_state = next_right;

		// TMO_POL: grab a new command if one is waiting, never block
		if(tk_rcv_mbf(motion_mbf_id, &cmd, TMO_POL) == (INT)sizeof(cmd)) {
			set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, cmd.target_speed_left);
			set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, cmd.target_speed_right);
		}

		tk_dly_tsk(1);
	}
}