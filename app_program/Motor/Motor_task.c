#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Motor_task.h"
#include "../robot_tasks.h"

#define MOTOR_PWM_WRAP    6249U
#define MOTOR_TEST_PERCENT  15U

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

EXPORT void motor_task(INT stacd, void *exinf)
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
	W left_ticks = 0;
	W right_ticks = 0;
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