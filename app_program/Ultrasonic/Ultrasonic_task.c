#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Ultrasonic_task.h"
#include "../robot_tasks.h"

#define SERVO_PWM_WRAP   19999U
#define SONAR_TIMEOUT_US 26000U
#define SERVO_POSITION_READY   (1U << 0)
#define SONAR_MEASUREMENT_DONE (1U << 1)

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

EXPORT void servo_task(INT stacd, void *exinf)
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

EXPORT void ultrasonic_task(INT stacd, void *exinf)
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