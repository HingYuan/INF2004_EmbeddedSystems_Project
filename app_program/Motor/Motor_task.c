#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Motor_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"

// 125 MHz / (6249 + 1) = 20 kHz PWM
// #define MOTOR_PWM_WRAP  6249U
#define MOTOR_PWM_WRAP  3124U		// halved the PWM wrap. Below I also maintained the PWM duty cycle. Testing stuff

#define CTRL_PERIOD_MS          1   /* 1 x 20 ms (tk_dly_tsk(2)) = 20 ms (50 Hz) state machine period */
#define BRAKE_STEP_PCT          200  /* BRAKE: immediate stop */
#define STALL_MIN_PCT           10   /* only check stall above this power */
#define STALL_PERIODS           125   /* 50 x 20 ms = 1.0 s with no ticks = stall */
#define KP_TRIM                 0    /* Closed-loop straight-line wheel trim gain (% PWM per tick diff) */
#define MAX_TRIM_PCT            10   /* Maximum closed-loop trim adjustment (% PWM) */

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

// Track current motor PWM outputs for encoder direction sign inside ISR
LOCAL volatile INT isr_out_l = 0;
LOCAL volatile INT isr_out_r = 0;
LOCAL volatile W isr_ticks_la = 0, isr_ticks_lb = 0;
LOCAL volatile W isr_ticks_ra = 0, isr_ticks_rb = 0;

// 0.5 ms (500 us) RP2040 Hardware Timer Alarm 0 ISR for jitter-free optical encoder sampling
LOCAL void encoder_alarm_isr(UINT intno)
{
	out_w(TIMER_INTR, 1U);
	out_w(TIMER_ALARM0, in_w(TIMER_TIMERAWL) + 500U);
	ClearInt(intno);

	UW v = in_w(GPIO_IN);
	UINT raw_la = (v >> MOTOR_LEFT_ENC_A) & 1U;
	UINT raw_lb = (v >> MOTOR_LEFT_ENC_B) & 1U;
	UINT raw_ra = (v >> MOTOR_RIGHT_ENC_A) & 1U;
	UINT raw_rb = (v >> MOTOR_RIGHT_ENC_B) & 1U;

	INT dir_l = (isr_out_l > 0) ? 1 : (isr_out_l < 0 ? -1 : 1);
	INT dir_r = (isr_out_r < 0) ? 1 : (isr_out_r > 0 ? -1 : 1);

	static UINT deb_la = 1, deb_lb = 1, deb_ra = 1, deb_rb = 1;
	static UB cnt_la = 0, cnt_lb = 0, cnt_ra = 0, cnt_rb = 0;
	static UB cd_la = 0, cd_lb = 0, cd_ra = 0, cd_rb = 0;
	static W raw_cnt_la = 0, raw_cnt_lb = 0, raw_cnt_ra = 0, raw_cnt_rb = 0;

	if (cd_la > 0) cd_la--;
	if (cd_lb > 0) cd_lb--;
	if (cd_ra > 0) cd_ra--;
	if (cd_rb > 0) cd_rb--;

	// 2 samples (1.0 ms) fast debounce + 90 samples (45 ms) cooldown captures both slot edges (~40 edges/rev)
	// Dividing by 2 converts the 2 edges per slot into 1 tick per slot (~20 ticks/rev)
	if (raw_la != deb_la) {
		if (++cnt_la >= 2) {
			deb_la = raw_la; cnt_la = 0;
			if (deb_la && cd_la == 0) { raw_cnt_la += dir_l; cd_la = 90; }
		}
	} else { cnt_la = 0; }

	if (raw_lb != deb_lb) {
		if (++cnt_lb >= 2) {
			deb_lb = raw_lb; cnt_lb = 0;
			if (deb_lb && cd_lb == 0) { raw_cnt_lb += dir_l; cd_lb = 90; }
		}
	} else { cnt_lb = 0; }

	if (raw_ra != deb_ra) {
		if (++cnt_ra >= 2) {
			deb_ra = raw_ra; cnt_ra = 0;
			if (deb_ra && cd_ra == 0) { raw_cnt_ra += dir_r; cd_ra = 90; }
		}
	} else { cnt_ra = 0; }

	if (raw_rb != deb_rb) {
		if (++cnt_rb >= 2) {
			deb_rb = raw_rb; cnt_rb = 0;
			if (deb_rb && cd_rb == 0) { raw_cnt_rb += dir_r; cd_rb = 90; }
		}
	} else { cnt_rb = 0; }

	isr_ticks_la = raw_cnt_la / 2;
	isr_ticks_lb = raw_cnt_lb / 2;
	isr_ticks_ra = raw_cnt_ra / 2;
	isr_ticks_rb = raw_cnt_rb / 2;

	enc_l_ticks = isr_ticks_la;
	enc_r_ticks = isr_ticks_rb;
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
LOCAL BOOL stalled(INT out_l, INT out_r, W dl, W dr, INT *cnt_l, INT *cnt_r)
{
	if (iabs(out_l) >= STALL_MIN_PCT) {
		if (dl == 0) (*cnt_l)++; else *cnt_l = 0;
	} else {
		*cnt_l = 0;
	}

	if (iabs(out_r) >= STALL_MIN_PCT) {
		if (dr == 0) (*cnt_r)++; else *cnt_r = 0;
	} else {
		*cnt_r = 0;
	}

	return (*cnt_l >= STALL_PERIODS || *cnt_r >= STALL_PERIODS);
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
	W cruise_start_l = 0;
	W cruise_start_r = 0;
	
	INT stall_cnt_l = 0, stall_cnt_r = 0;
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

	// Start 2000 Hz (500 us) RP2040 Hardware Timer Alarm 0 (IRQ 0)
	{
		T_DINT dint = {0};
		dint.intatr = TA_HLNG;
		dint.inthdr = (FP)encoder_alarm_isr;
		if (tk_def_int(0, &dint) == E_OK) {
			out_w(TIMER_INTR, 1U);
			set_w(TIMER_INTE, 1U);
			out_w(TIMER_ALARM0, in_w(TIMER_TIMERAWL) + 500U);
			ClearInt(0);
			EnableInt(0, 2);
		}
	}

	while(1) {
		/* ---- encoders sampled at 2000 Hz in hardware ISR encoder_alarm_isr ---- */
		(void)left_state;
		(void)right_state;
		isr_out_l = out_l;
		isr_out_r = out_r;

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

			/* Always drain the buffer; commands remain ignored while E-STOP is held. */
			while(tk_rcv_mbf(motor_mbfid, &cmd, TMO_POL) == (INT)sizeof(cmd)){
				if(new_cmd) buffer_drop_count++;
				new_cmd = TRUE;
			}

			// Check release after receiving: Navigator may release E-STOP and enqueue a pivot together.
			if (state == MS_ESTOP && !motor_estop) {
				motor_fault_word &= ~FAULT_BIT_ESTOP;
				stop_stable_periods = 0;
				SET_STATE(MS_IDLE);
			}

			// Periodic tick logging when wheels rotate or power applied
			static W last_la = 0, last_lb = 0, last_ra = 0, last_rb = 0;
			if (++print_timer >= 15) {
				print_timer = 0;
				if (isr_ticks_la != last_la || isr_ticks_lb != last_lb || isr_ticks_ra != last_ra || isr_ticks_rb != last_rb || out_l != 0 || out_r != 0) {
					last_la = isr_ticks_la; last_lb = isr_ticks_lb;
					last_ra = isr_ticks_ra; last_rb = isr_ticks_rb;
					last_print_l = enc_l_ticks;
					last_print_r = enc_r_ticks;
					tm_printf((UB *)"[enc] ticks L=%d R=%d (LA=%d LB=%d | RA=%d RB=%d) | dist L=%d R=%d mm\n",
						(INT)enc_l_ticks, (INT)enc_r_ticks,
						(INT)isr_ticks_la, (INT)isr_ticks_lb, (INT)isr_ticks_ra, (INT)isr_ticks_rb,
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
					cruise_start_l = enc_l_ticks;
					cruise_start_r = enc_r_ticks;

					stall_cnt_l = 0; stall_cnt_r = 0;
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
					if(cmd.target_speed_left != tgt_l || cmd.target_speed_right != tgt_r) {
						cruise_start_l = enc_l_ticks;
						cruise_start_r = enc_r_ticks;
					}
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
					// Closed-loop encoder trim so both wheels track the exact same distance
					INT moved_l = iabs((INT)(enc_l_ticks - cruise_start_l));
					INT moved_r = iabs((INT)(enc_r_ticks - cruise_start_r));
					INT trim = (moved_l - moved_r) * KP_TRIM;
					if (trim > MAX_TRIM_PCT) trim = MAX_TRIM_PCT;
					if (trim < -MAX_TRIM_PCT) trim = -MAX_TRIM_PCT;

					if (tgt_l > 0) {
						out_l = tgt_l - trim;
						out_r = tgt_r - trim; // tgt_r is negative, so -trim increases magnitude when trim > 0
					} else {
						out_l = tgt_l + trim;
						out_r = tgt_r + trim;
					}

					// Fail-Safe: Check for broken encoder sensor (Section 7)
					if((stall_cnt_l >= 35 && stall_cnt_r == 0) ||
					   (stall_cnt_r >= 35 && stall_cnt_l == 0)) {
						encoder_degraded = TRUE;
						motor_fault_word |= FAULT_BIT_ENCODER; // Set Bit 1 (Sensor failed)
					}
				}

				if(stalled(out_l, out_r, dl, dr, &stall_cnt_l, &stall_cnt_r)) {
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

				// Stop each wheel individually when it reaches its target; complete turn when both finish
				BOOL turn_finished = (left_done && right_done);

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
				if (stalled(out_l, out_r, dl, dr, &stall_cnt_l, &stall_cnt_r)) {
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
					cruise_start_l = enc_l_ticks;
					cruise_start_r = enc_r_ticks;
					
					stall_cnt_l = 0; stall_cnt_r = 0;
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

		tk_dly_tsk(2);
	}
}
