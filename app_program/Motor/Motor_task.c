#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Motor_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"

// 125 MHz / (6249 + 1) = 20 kHz PWM
// #define MOTOR_PWM_WRAP  6249U
#define MOTOR_PWM_WRAP  3124U		// halved the PWM wrap. Below I also maintained the PWM duty cycle. Testing stuff

#define CTRL_PERIOD_MS          8   /* state machine period */
#define BRAKE_STEP_PCT          200  /* BRAKE: immediate stop */
#define STALL_MIN_PCT           10   /* only check stall above this power */
#define STALL_PERIODS           125  /* 125 x 8 ms = 1.0 s with no ticks = stall */

typedef enum { MS_IDLE, MS_CRUISE, MS_TURN, MS_BRAKE, MS_FAULT, MS_ESTOP } MOTOR_STATE;

#define SET_STATE(s) do { state = (s); tm_printf((UB *)"[motors] state=%d\n", (INT)(s)); } while(0)

EXPORT volatile W enc_l_ticks = 0; // Left wheel encoder tick count
EXPORT volatile W enc_r_ticks = 0; // Right wheel encoder tick count
EXPORT volatile BOOL motor_estop = FALSE;
EXPORT volatile BOOL motor_stop_ack_requested = FALSE;
EXPORT volatile UW buffer_drop_count = 0;
EXPORT volatile BOOL encoder_degraded = FALSE;
EXPORT volatile UH motor_fault_word = 0; // Tribunal 1 Fault Word

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
	// duty = (MOTOR_PWM_WRAP + 1U) * (UW)(speed < 0 ? -speed : speed) / 100U;
	duty = (MOTOR_PWM_WRAP + 1U) * (UW)(speed < 0 ? -speed : speed) / 200U;

	if(speed >= 0) {
		pwm_set_cc(pin_a, duty);
		pwm_set_cc(pin_b, 0);
	} else {
		pwm_set_cc(pin_a, 0);
		pwm_set_cc(pin_b, duty); // Flip direction
	}
}

/* Odometry helper implementations */
EXPORT INT motor_get_left_dist_mm(void)
{
	return (INT)((enc_l_ticks * MM_PER_TICK_X10) / 10);
}

EXPORT INT motor_get_right_dist_mm(void)
{
	return (INT)((enc_r_ticks * MM_PER_TICK_X10) / 10);
}

EXPORT INT motor_get_avg_dist_mm(void)
{
	return (motor_get_left_dist_mm() + motor_get_right_dist_mm()) / 2;
}

LOCAL INT iabs(INT v) { return v < 0 ? -v : v; }

/* Move cur toward target by at most step */
LOCAL INT ramp(INT cur, INT target, INT step)
{
	if(cur < target) return (target - cur < step) ? target : cur + step;
	if(cur > target) return (cur - target < step) ? target : cur - step;
	return cur;
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
	MOTOR_COMMAND cmd = {0};
	UINT left_state, right_state;
	INT i;

	MOTOR_STATE state = MS_IDLE;
	INT tgt_l = 0, tgt_r = 0;       /* target % from Navigator */
	INT out_l = 0, out_r = 0;       /* % actually sent to PWM */
	W prev_l = 0, prev_r = 0;

	W turn_start_l = 0;
	W turn_start_r = 0;
	INT turn_goal_l = 0;
	INT turn_goal_r = 0;
	
	INT stall_cnt = 0;
	INT ms = 0;
	INT stop_stable_periods = 0;

	// Track completion of an encoder-limited movement
	BOOL move_done_pending = FALSE;
	INT move_stable_periods = 0;

	INT print_timer = 0;
	W last_print_l = 0, last_print_r = 0;

	(void)stacd;
	(void)exinf;

	tm_printf((UB *)"[motor] Task started\n");

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

	// Set up Encoder Pins (inputs with pull-up & Schmitt trigger)
	for(i = 0; i < 4; i++) {
		if(gpio_set_pin(encoder_pins[i], GPIO_MODE_IN) < E_OK) {
			tm_printf((UB *)"[motors] encoder GPIO setup failed on GP%d\n",
				  encoder_pins[i]);
			tk_ext_tsk();
		}
		// Enable pull-up and Schmitt trigger (vital for ADC pins GP26/GP27)
		set_w(GPIO(encoder_pins[i]), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
	}

	left_state = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
	right_state = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);

	while(1) {
		/* ---- every 1 ms: encoders + e-stop ---- */
		// record current encoder state
		UINT next_left = read_encoder_state(MOTOR_LEFT_ENC_A, MOTOR_LEFT_ENC_B);
		UINT next_right = read_encoder_state(MOTOR_RIGHT_ENC_A, MOTOR_RIGHT_ENC_B);

		// Count ticks: detects rising edge on either Channel A (bit 1) or Channel B (bit 0)
		// Works reliably even if optical sensor wire is on pin A or pin B
		UINT diff_l = left_state ^ next_left;
		if (diff_l & next_left) {
			enc_l_ticks += (out_l > 0) ? 1 : (out_l < 0 ? -1 : 1);
		}

		UINT diff_r = right_state ^ next_right;
		if (diff_r & next_right) {
			// Right motor mirrored: negative PWM is forward (+1), positive is backward (-1)
			enc_r_ticks += (out_r < 0) ? 1 : (out_r > 0 ? -1 : 1);
		}

		left_state = next_left;
		right_state = next_right;

		if (motor_estop) {
			if (state != MS_ESTOP) {
				tgt_l = tgt_r = out_l = out_r = 0;
				turn_goal_l = turn_goal_r = 0;

				// Cancel any unfinished movement
				move_done_pending = FALSE;
				move_stable_periods = 0;
				stop_stable_periods = 0;

				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);

				motor_fault_word |= FAULT_BIT_ESTOP;   // Set Bit 2 (E-Stop active)
				SET_STATE(MS_ESTOP);
			}
		} else if (state == MS_ESTOP) {
			motor_fault_word &= ~FAULT_BIT_ESTOP;  // Clear Bit 2 (E-Stop released)
			stop_stable_periods = 0;
			SET_STATE(MS_IDLE);
		}

		/* ---- every CTRL_PERIOD_MS: state machine ---- */
		if(++ms >= CTRL_PERIOD_MS) {
			BOOL new_cmd = FALSE;
			W dl, dr;

			ms = 0;
			dl = enc_l_ticks - prev_l;
			dr = enc_r_ticks - prev_r;
			prev_l = enc_l_ticks;
			prev_r = enc_r_ticks;

			/* always drain the buffer, even in E-STOP (command is then ignored) */
			while(tk_rcv_mbf(motor_mbfid, &cmd, TMO_POL) == (INT)sizeof(cmd)){
				if(new_cmd) buffer_drop_count++;
				new_cmd = TRUE;
			}

			// Periodic tick logging when wheels rotate or power applied (every ~250 ms)
			if (++print_timer >= (250 / CTRL_PERIOD_MS)) {
				print_timer = 0;
				if (enc_l_ticks != last_print_l || enc_r_ticks != last_print_r || out_l != 0 || out_r != 0) {
					last_print_l = enc_l_ticks;
					last_print_r = enc_r_ticks;
					tm_printf((UB *)"[enc] ticks L=%d R=%d | dist L=%d R=%d mm\n",
						(INT)enc_l_ticks, (INT)enc_r_ticks,
						motor_get_left_dist_mm(), motor_get_right_dist_mm());
				}
			}

			switch(state) {

			case MS_IDLE:
				// A bounded movement has reached both encoder targets
				if (move_done_pending) {
					// Wait for stable encoder readings after braking
					if (dl == 0 && dr == 0) {
						if (move_stable_periods < 3)
							move_stable_periods++;
					} else {
						move_stable_periods = 0;
					}

					// Notify Navigator once the wheels are stable
					if (move_stable_periods >= 3) {
						NAVIGATOR_MESSAGE msg = {0};
						msg.type = MSG_MOTOR_MOVE_DONE;

						ER er = tk_snd_mbf(
							navigator_mbfid,
							&msg,
							sizeof(msg),
							TMO_POL
						);

						if (er >= E_OK) {
							move_done_pending = FALSE;
							move_stable_periods = 0;

							tm_printf((UB *)"[motors] Move completed\n");
						}
					}

					break;
				}

				// Existing handling of new motor commands
				if (new_cmd &&
					(cmd.target_speed_left != 0 ||
					cmd.target_speed_right != 0)) {

					tgt_l = cmd.target_speed_left;
					tgt_r = cmd.target_speed_right;

					turn_goal_l = cmd.turn_ticks_left;
					turn_goal_r = cmd.turn_ticks_right;
					turn_start_l = enc_l_ticks;
					turn_start_r = enc_r_ticks;

					stall_cnt = 0;
					out_l = tgt_l;
					out_r = tgt_r;

					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);

					if (turn_goal_l != 0 || turn_goal_r != 0) {
						SET_STATE(MS_TURN);
					} else {
						SET_STATE(MS_CRUISE);
					}
				}
				break;

			case MS_CRUISE:
				if(new_cmd) {
					tgt_l = cmd.target_speed_left;
					tgt_r = cmd.target_speed_right;

					if(tgt_l == 0 && tgt_r == 0) { SET_STATE(MS_BRAKE); break; }

					if(cmd.turn_ticks_left != 0 || cmd.turn_ticks_right != 0) {
						turn_goal_l = cmd.turn_ticks_left;
						turn_goal_r = cmd.turn_ticks_right;

						turn_start_l = enc_l_ticks;
						turn_start_r = enc_r_ticks;

						out_l = tgt_l;
						out_r = tgt_r;
						set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
						set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
						
						SET_STATE(MS_TURN);
						break;
					}
					/* else: steering adjustment, new targets, stay in CRUISE */
				}

				out_l = tgt_l;
				out_r = tgt_r;

				// Driving straight forward means opposite signs (e.g. +50 left, -50 right)
				if(tgt_l == -tgt_r && tgt_l != 0) {
					// Fail-Safe: Check for broken encoder sensor (Section 7)
					if((dl == 0 && iabs((INT)dr) > 1) ||
					   (dr == 0 && iabs((INT)dl) > 1)) {
						encoder_degraded = TRUE;
						motor_fault_word |= FAULT_BIT_ENCODER; // Set Bit 1 (Sensor failed)
					}
				}

				if(stalled(out_l, out_r, dl, dr, &stall_cnt)) {
					out_l = out_r = 0;

					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);

					motor_fault_word |= FAULT_BIT_STALL;  // Set Bit 0 (Stalled)
					SET_STATE(MS_FAULT);
					
					break;
				}

				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);

				break;

			case MS_TURN: {
				if(new_cmd){
					// If Navigator says stop, abort turn immediately
					if(cmd.target_speed_left == 0 && cmd.target_speed_right == 0) {
						tgt_l = tgt_r = 0;
						turn_goal_l = turn_goal_r = 0;
						move_done_pending = FALSE;
						SET_STATE(MS_BRAKE);
						break;
					}
					// Only reset turn_start if command parameters actually changed
					if(cmd.target_speed_left != tgt_l || cmd.target_speed_right != tgt_r ||
					   cmd.turn_ticks_left != turn_goal_l || cmd.turn_ticks_right != turn_goal_r) {
						tgt_l = cmd.target_speed_left;
						tgt_r = cmd.target_speed_right;
						turn_goal_l = cmd.turn_ticks_left;
						turn_goal_r = cmd.turn_ticks_right;
						turn_start_l = enc_l_ticks;
						turn_start_r = enc_r_ticks;
					}
				}
				// Check each wheel's encoder target
				BOOL left_done =
					(turn_goal_l == 0) ||
					(iabs((INT)(enc_l_ticks - turn_start_l)) >= turn_goal_l);

				BOOL right_done =
					(turn_goal_r == 0) ||
					(iabs((INT)(enc_r_ticks - turn_start_r)) >= turn_goal_r);

				// Symmetrical in-place turn: stop both when turn angle reached (prevents one wheel running away)
				BOOL turn_finished = (turn_goal_l > 0 && turn_goal_r > 0) ?
					(left_done || right_done) : (left_done && right_done);

				if (turn_finished) {
					out_l = 0;
					out_r = 0;
					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);

					turn_goal_l = 0;
					turn_goal_r = 0;

					// Report completion after braking
					move_done_pending = TRUE;
					move_stable_periods = 0;

					SET_STATE(MS_BRAKE);
					break;
				}

				// Stop each wheel individually at its target
				out_l = left_done ? 0 : tgt_l;
				out_r = right_done ? 0 : tgt_r;

				// Keep existing stall protection
				if (stalled(out_l, out_r, dl, dr, &stall_cnt)) {
					out_l = out_r = 0;

					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);

					move_done_pending = FALSE;
					motor_fault_word |= FAULT_BIT_STALL;  // Set Bit 0 (Stalled)
					SET_STATE(MS_FAULT);
					break;
				}

				set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
				set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);

				break;
			}

			case MS_BRAKE:
				if (new_cmd && (cmd.target_speed_left != 0 || cmd.target_speed_right != 0)) {
					tgt_l = cmd.target_speed_left;
					tgt_r = cmd.target_speed_right;
					turn_goal_l = cmd.turn_ticks_left;
					turn_goal_r = cmd.turn_ticks_right;
					turn_start_l = enc_l_ticks;
					turn_start_r = enc_r_ticks;
					
					stall_cnt = 0;
					move_done_pending = FALSE; // Cancel any prior turn completion
					out_l = tgt_l;
					out_r = tgt_r;

					set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
					set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
					
					if (turn_goal_l != 0 || turn_goal_r != 0) {
						SET_STATE(MS_TURN);
					} else {
						SET_STATE(MS_CRUISE);
					}
					break;
				}

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
					motor_fault_word &= ~FAULT_BIT_STALL; // Clear Bit 0 (Stall cleared)
					SET_STATE(MS_IDLE);
				}

				break;

			case MS_ESTOP:
				// Confirm that wheel encoders have stopped changing
				if (dl == 0 && dr == 0) {
					if (stop_stable_periods < 3)
						stop_stable_periods++;
				} else {
					stop_stable_periods = 0;
				}

				if (motor_stop_ack_requested && stop_stable_periods >= 3) {
					NAVIGATOR_MESSAGE msg = {0};
					msg.type = MSG_MOTOR_STOPPED;

					// Non-blocking send keeps motor control responsive
					ER er = tk_snd_mbf(navigator_mbfid, &msg,
									sizeof(msg), TMO_POL);

					if (er >= E_OK) {
						motor_stop_ack_requested = FALSE;
						tm_printf((UB *)"[motors] Stop confirmed\n");
					}
				}

				break;
			default:
				break;
			}
		}

		tk_dly_tsk(1);
	}
}
