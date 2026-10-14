#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Motor_task.h"

// 125 MHz / (6249 + 1) = 20 kHz PWM
#define MOTOR_PWM_WRAP  6249U

#define CTRL_PERIOD_MS          20   /* state machine / PI period */
#define RAMP_STEP_PCT           5    /* ACCEL: +5% per period */
#define BRAKE_STEP_PCT          10   /* BRAKE: -10% per period */
#define TICKS_PER_PERIOD_AT_100 10   /* TUNE: encoder ticks per 20 ms at 100% */
#define KP                      3    /* TUNE */
#define KI                      1    /* TUNE */
#define INTEG_MAX               30
#define STALL_MIN_PCT           20   /* only check stall above this power */
#define STALL_PERIODS           25   /* 25 x 20 ms = 0.5 s with no ticks = stall */

typedef enum { MS_IDLE, MS_ACCEL, MS_CRUISE, MS_TURN, MS_BRAKE, MS_FAULT, MS_ESTOP } MOTOR_STATE;

#define SET_STATE(s) do { state = (s); tm_printf((UB *)"[motors] state=%d\n", (INT)(s)); } while(0)

EXPORT volatile W enc_l_ticks = 0; // Left wheel encoder tick count
EXPORT volatile W enc_r_ticks = 0; // Right wheel encoder tick count
EXPORT volatile BOOL motor_estop = FALSE;

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

LOCAL INT iabs(INT v) { return v < 0 ? -v : v; }

LOCAL INT clamp(INT v, INT lo, INT hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

/* Move cur toward target by at most step */
LOCAL INT ramp(INT cur, INT target, INT step)
{
	if(cur < target) return (target - cur < step) ? target : cur + step;
	if(cur > target) return (cur - target < step) ? target : cur - step;
	return cur;
}

/* PI speed loop: target is feed-forward, encoder error corrects it */
LOCAL INT pi_speed(INT target_pct, W measured, INT *integ)
{
	INT want = target_pct * TICKS_PER_PERIOD_AT_100 / 100;
	INT err = want - (INT)measured;
	INT out;

	*integ = clamp(*integ + err, -INTEG_MAX, INTEG_MAX);
	out = target_pct + KP * err + KI * (*integ);
	return clamp(out, -100, 100);
}

/* TRUE when a wheel is powered but its encoder has not moved for STALL_PERIODS */
LOCAL BOOL stalled(INT out_l, INT out_r, W dl, W dr, INT *cnt)
{
	BOOL l = (iabs(out_l) >= STALL_MIN_PCT && dl == 0);
	BOOL r = (iabs(out_r) >= STALL_MIN_PCT && dr == 0);

	if(l || r) (*cnt)++; else *cnt = 0;
	return (*cnt >= STALL_PERIODS);
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

    MOTOR_STATE state = MS_IDLE;
	INT tgt_l = 0, tgt_r = 0;       /* target % from Navigator */
	INT out_l = 0, out_r = 0;       /* % actually sent to PWM */
	INT integ_l = 0, integ_r = 0;
	W prev_l = 0, prev_r = 0;
	W turn_start = 0;
	INT turn_goal = 0;
	INT stall_cnt = 0;
	INT ms = 0;

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
		/* ---- every 1 ms: encoders + e-stop ---- */
		UINT next_left = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
		UINT next_right = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);

		enc_l_ticks += quadrature_delta(left_state, next_left);
		enc_r_ticks += quadrature_delta(right_state, next_right);
		left_state = next_left;
		right_state = next_right;

		if(motor_estop) {
			if(state != MS_ESTOP) {
				tgt_l = tgt_r = out_l = out_r = 0;
				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
				SET_STATE(MS_ESTOP);
			}
		} else if(state == MS_ESTOP) {
			SET_STATE(MS_IDLE);
		}

		/* ---- every 20 ms: state machine ---- */
		if(++ms >= CTRL_PERIOD_MS) {
			BOOL new_cmd = FALSE;
			W dl, dr;

			ms = 0;
			dl = enc_l_ticks - prev_l;
			dr = enc_r_ticks - prev_r;
			prev_l = enc_l_ticks;
			prev_r = enc_r_ticks;

			/* always drain the buffer, even in E-STOP (command is then ignored) */
			if(tk_rcv_mbf(motion_mbf_id, &cmd, TMO_POL) == (INT)sizeof(cmd)) new_cmd = TRUE;

			switch(state) {
			case MS_IDLE:
				if(new_cmd && (cmd.target_speed_left != 0 || cmd.target_speed_right != 0)) {
					tgt_l = cmd.target_speed_left;
					tgt_r = cmd.target_speed_right;
					turn_goal = cmd.turn_ticks;
					integ_l = integ_r = 0;
					stall_cnt = 0;
					SET_STATE(MS_ACCEL);
				}
				break;

			case MS_ACCEL:
				if(new_cmd) {
					tgt_l = cmd.target_speed_left;
					tgt_r = cmd.target_speed_right;
					turn_goal = cmd.turn_ticks;
					if(tgt_l == 0 && tgt_r == 0) { SET_STATE(MS_BRAKE); break; }
				}
				out_l = ramp(out_l, tgt_l, RAMP_STEP_PCT);
				out_r = ramp(out_r, tgt_r, RAMP_STEP_PCT);
				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
				if(out_l == tgt_l && out_r == tgt_r) {
					if(turn_goal != 0) { turn_start = enc_l_ticks; SET_STATE(MS_TURN); }
					else SET_STATE(MS_CRUISE);
				}
				break;

			case MS_CRUISE:
				if(new_cmd) {
					tgt_l = cmd.target_speed_left;
					tgt_r = cmd.target_speed_right;
					if(tgt_l == 0 && tgt_r == 0) { SET_STATE(MS_BRAKE); break; }
					integ_l = integ_r = 0;
					if(cmd.turn_ticks != 0) {
						turn_goal = cmd.turn_ticks;
						turn_start = enc_l_ticks;
						SET_STATE(MS_TURN);
						break;
					}
					/* else: steering adjustment, new targets, stay in CRUISE */
				}
				out_l = pi_speed(tgt_l, dl, &integ_l);
				out_r = pi_speed(tgt_r, dr, &integ_r);
				if(stalled(out_l, out_r, dl, dr, &stall_cnt)) {
					out_l = out_r = 0;
					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
					SET_STATE(MS_FAULT);
					break;
				}
				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
				break;

			case MS_TURN:
				out_l = pi_speed(tgt_l, dl, &integ_l);
				out_r = pi_speed(tgt_r, dr, &integ_r);
				if(stalled(out_l, out_r, dl, dr, &stall_cnt)) {
					out_l = out_r = 0;
					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
					SET_STATE(MS_FAULT);
					break;
				}
				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
				if(iabs((INT)(enc_l_ticks - turn_start)) >= turn_goal) {
					turn_goal = 0;
					SET_STATE(MS_BRAKE);
				}
				break;

			case MS_BRAKE:
				out_l = ramp(out_l, 0, BRAKE_STEP_PCT);
				out_r = ramp(out_r, 0, BRAKE_STEP_PCT);
				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
				if(out_l == 0 && out_r == 0) {
					tgt_l = tgt_r = 0;
					SET_STATE(MS_IDLE);
				}
				break;

			case MS_FAULT:
				/* fault clears when Navigator sends a stop command {0,0,0} */
				if(new_cmd && cmd.target_speed_left == 0 && cmd.target_speed_right == 0) {
					tgt_l = tgt_r = 0;
					SET_STATE(MS_IDLE);
				}
				break;

			case MS_ESTOP:
			default:
				break;
			}
		}

		tk_dly_tsk(1);
	}
}