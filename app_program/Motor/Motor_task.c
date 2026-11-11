#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include "Motor_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"

// ============================================================================
// SET TO 1 FOR STANDALONE CONSISTENCY TEST (Drives 4s -> Pauses 4s -> Repeats)
// SET TO 0 FOR NORMAL FULL-ROBOT OPERATION WITH NAVIGATOR
// ============================================================================
#define STAGE2_STANDALONE_PAD_TEST  0

// 125 MHz / (6249 + 1) = 20 kHz hardware PWM
#define MOTOR_PWM_WRAP          6249U

// Timing & Soft-Start Ramp parameters (tk_dly_tsk(2) = 20 ms = 50 Hz)
#define LOOP_DELAY_MS           2
#define MIN_START_PCT           18   /* Start both motors right at friction threshold together */
#define RAMP_STEP_PCT           2    /* Gentle ramp (2% per 20ms) prevents launch twist! */
#define BRAKE_STEP_PCT          200  /* Immediate active braking */
#define MIN_MOTOR_PCT           16   /* Minimum active cruise PWM % */

// True Heading-Lock PID Parameters (15 ticks = 1 cm)
// Never erases cruise_start_l/r during a straight run so total heading angle is preserved!
#define TICKS_PER_PCT_KP        2    /* Position P: 1% PWM per 2 ticks (~1.3 mm) heading error */
#define KD_VEL_DIFF             3    /* Velocity D: 3% PWM per (dl - dr) tick/20ms speed diff */
#define KI_TRIM_NUM             1    /* Integral I: +1 unit per tick of error every 20 ms */
#define KI_TRIM_DEN             50   /* 50 units = 1% permanent motor balance offset */
#define MAX_INTEG_UNITS         300  /* Max +-6% learned permanent motor offset */
#define MAX_TRIM_PCT            8    /* Max +-8% total steering correction */

// Fault detection thresholds (at 20 ms period)
#define STALL_MIN_PCT           20   /* Only check stall when PWM >= 20% */
#define STALL_PERIODS           75   /* 75 x 20 ms = 1.5 s with 0 ticks on BOTH wheels = stall */

typedef enum {
    MS_IDLE,
    MS_ACCEL,
    MS_CRUISE,
    MS_TURN,
    MS_BRAKE,
    MS_FAULT,
    MS_ESTOP
} MOTOR_STATE;

#define SET_STATE(s) do { state = (s); tm_printf((UB *)"[motors] state=%d\n", (INT)(s)); } while(0)

EXPORT volatile W enc_l_ticks = 0;
EXPORT volatile W enc_r_ticks = 0;
EXPORT volatile BOOL motor_estop = FALSE;
EXPORT volatile BOOL motor_stop_ack_requested = FALSE;
EXPORT volatile UW buffer_drop_count = 0;
EXPORT volatile BOOL encoder_degraded = FALSE;
EXPORT volatile UH motor_fault_word = 0;

// Track current motor PWM outputs for encoder direction sign inside ISR
LOCAL volatile INT isr_out_l = 0;
LOCAL volatile INT isr_out_r = 0;
LOCAL volatile W isr_ticks_la = 0, isr_ticks_lb = 0;
LOCAL volatile W isr_ticks_ra = 0, isr_ticks_rb = 0;

LOCAL INT iabs(INT v)
{
    return v < 0 ? -v : v;
}

// Proven Stage 1 Encoder Filter: 2-sample (1.0 ms) rising-edge counter with NO cooldown ceiling.
LOCAL void count_clean_rising_edge(
    UINT raw_bit, INT dir, UINT *state, UB *cnt, volatile W *ticks)
{
    if (*state == 0U) {
        if (raw_bit != 0U) {
            if (++(*cnt) >= 2U) {
                *state = 1U;
                *cnt = 0U;
                *ticks += dir;
            }
        } else {
            *cnt = 0U;
        }
    } else {
        if (raw_bit == 0U) {
            if (++(*cnt) >= 2U) {
                *state = 0U;
                *cnt = 0U;
            }
        } else {
            *cnt = 0U;
        }
    }
}

// 0.5 ms (500 us) RP2040 Hardware Timer Alarm 0 ISR
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

    // Left forward is positive PWM; Right forward is negative PWM
    INT dir_l = (isr_out_l > 0) ? 1 : (isr_out_l < 0 ? -1 : 1);
    INT dir_r = (isr_out_r < 0) ? 1 : (isr_out_r > 0 ? -1 : 1);

    static UINT st_la = 0, st_lb = 0, st_ra = 0, st_rb = 0;
    static UB   cnt_la = 0, cnt_lb = 0, cnt_ra = 0, cnt_rb = 0;

    count_clean_rising_edge(raw_la, dir_l, &st_la, &cnt_la, &isr_ticks_la);
    count_clean_rising_edge(raw_lb, dir_l, &st_lb, &cnt_lb, &isr_ticks_lb);
    count_clean_rising_edge(raw_ra, dir_r, &st_ra, &cnt_ra, &isr_ticks_ra);
    count_clean_rising_edge(raw_rb, dir_r, &st_rb, &cnt_rb, &isr_ticks_rb);

    // Use Channel A (GP16 and GP2), both verified identical in Stage 1
    enc_l_ticks = isr_ticks_la;
    enc_r_ticks = isr_ticks_ra;
}

// Drive one motor channel (-100..100%)
LOCAL void set_motor(UINT pin_a, UINT pin_b, INT speed)
{
    INT s = CLAMP(speed, -100, 100);
    UW duty = (MOTOR_PWM_WRAP + 1U) * (UW)iabs(s) / 100U;

    if (s > 0) {
        pwm_set_cc(pin_a, duty);
        pwm_set_cc(pin_b, 0);
    } else if (s < 0) {
        pwm_set_cc(pin_a, 0);
        pwm_set_cc(pin_b, duty);
    } else {
        pwm_set_cc(pin_a, 0);
        pwm_set_cc(pin_b, 0);
    }
}

/* Odometry helper functions using calibrated 0.68 mm/tick (MM_PER_TICK_X100 = 68) */
EXPORT INT motor_get_left_dist_mm(void)
{
    return (INT)((enc_l_ticks * MM_PER_TICK_X100) / 100);
}

EXPORT INT motor_get_right_dist_mm(void)
{
    return (INT)((enc_r_ticks * MM_PER_TICK_X100) / 100);
}

EXPORT INT motor_get_avg_dist_mm(void)
{
    return (motor_get_left_dist_mm() + motor_get_right_dist_mm()) / 2;
}

EXPORT INT motor_get_avg_dist_cm(void)
{
    INT mm = motor_get_avg_dist_mm();
    return (mm >= 0) ? (mm + 5) / 10 : (mm - 5) / 10;
}

LOCAL INT ramp(INT cur, INT target, INT step)
{
    if (cur < target) return (target - cur < step) ? target : cur + step;
    if (cur > target) return (cur - target < step) ? target : cur - step;
    return cur;
}

// Compute synchronized straight-line wheel PWM (used in BOTH MS_ACCEL and MS_CRUISE!)
// Never shifts or erases cruise_start_l/r so launch slip is 100% corrected.
LOCAL void apply_straight_heading_lock(
    INT base_mag_l, INT base_mag_r,
    W cruise_start_l, W cruise_start_r,
    W dl, W dr, INT *integ_units,
    INT *out_mag_l, INT *out_mag_r)
{
    INT moved_l = iabs((INT)(enc_l_ticks - cruise_start_l));
    INT moved_r = iabs((INT)(enc_r_ticks - cruise_start_r));
    INT diff = moved_l - moved_r;               // Total heading error in ticks since launch
    INT vel_diff = iabs((INT)dl) - iabs((INT)dr); // Instantaneous speed error in this 20 ms

    // Slowly learn permanent motor asymmetry (only when error is small so launch doesn't wind up)
    if (iabs(diff) <= 15) {
        *integ_units = CLAMP(*integ_units + (diff * KI_TRIM_NUM), -MAX_INTEG_UNITS, MAX_INTEG_UNITS);
    }

    INT p_term = diff / TICKS_PER_PCT_KP;
    INT d_term = vel_diff * KD_VEL_DIFF;
    INT i_term = (*integ_units) / KI_TRIM_DEN;

    INT trim = CLAMP(p_term + d_term + i_term, -MAX_TRIM_PCT, MAX_TRIM_PCT);

    *out_mag_l = CLAMP(base_mag_l - trim, MIN_MOTOR_PCT, 100);
    *out_mag_r = CLAMP(base_mag_r + trim, MIN_MOTOR_PCT, 100);
}

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

    return (*cnt_l >= STALL_PERIODS && *cnt_r >= STALL_PERIODS);
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
    INT i;
    MOTOR_STATE state = MS_IDLE;
    INT tgt_l = 0, tgt_r = 0;
    INT ramp_mag_l = 0, ramp_mag_r = 0;
    INT out_l = 0, out_r = 0;
    W prev_l = 0, prev_r = 0;
    W turn_start_l = 0, turn_start_r = 0;
    INT turn_goal_l = 0, turn_goal_r = 0;
    W cruise_start_l = 0, cruise_start_r = 0;
    INT integ_units = 90; // Pre-seeded +1.8% right-wheel balance so cold power-on matches PWM L=24 R=-26
    INT stall_cnt_l = 0, stall_cnt_r = 0;
    INT stop_stable_periods = 0;
    BOOL move_done_pending = FALSE;
    INT move_stable_periods = 0;
    INT print_timer = 0;
#if STAGE2_STANDALONE_PAD_TEST
    INT test_cycles = 0;
    INT run_number = 0;
#endif

    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"[motor] Task started\n");

    for (i = 0; i < 4; i++) {
        if (pwm_set_divider(output_pins[i], 1, 0) < E_OK
            || pwm_set_wrap(output_pins[i], MOTOR_PWM_WRAP) < E_OK
            || pwm_set_pin(output_pins[i]) < E_OK
            || pwm_set_cc(output_pins[i], 0) < E_OK
            || pwm_set_enabled(output_pins[i], TRUE) < E_OK) {
            tm_printf((UB *)"[motors] PWM setup failed on GP%d\n", output_pins[i]);
            tk_ext_tsk();
        }
    }

    for (i = 0; i < 4; i++) {
        if (gpio_set_pin(encoder_pins[i], GPIO_MODE_IN) < E_OK) {
            tm_printf((UB *)"[motors] encoder GPIO setup failed on GP%d\n", encoder_pins[i]);
            tk_ext_tsk();
        }
        set_w(GPIO(encoder_pins[i]), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
    }

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

    while (1) {
        BOOL new_cmd = FALSE;
        W dl, dr;

        isr_out_l = out_l;
        isr_out_r = out_r;

#if STAGE2_STANDALONE_PAD_TEST
        while (tk_rcv_mbf(motor_mbfid, &cmd, TMO_POL) == (INT)sizeof(cmd)) {
        }

        motor_estop = FALSE;
        test_cycles++;

        // Repeating consistency test:
        // Wait 4s (200 cycles) -> Drive straight 3.5s (175 cycles) -> Stop 4s -> Repeat!
        INT phase = test_cycles % 375;
        if (phase == 200) {
            run_number++;
            cmd.target_speed_left = 25;
            cmd.target_speed_right = -25;
            cmd.turn_ticks_left = 0;
            cmd.turn_ticks_right = 0;
            new_cmd = TRUE;
            tm_printf((UB *)"\n=== [TEST RUN #%d] Driving straight at 25%% for 3.5s ===\n", run_number);
        } else if (phase == 0 && run_number > 0) {
            cmd.target_speed_left = 0;
            cmd.target_speed_right = 0;
            cmd.turn_ticks_left = 0;
            cmd.turn_ticks_right = 0;
            new_cmd = TRUE;
            tm_printf((UB *)"=== [TEST RUN #%d DONE] Stopped for 4s (reposition robot on pad!) ===\n", run_number);
        }
#else
        if (motor_estop) {
            if (state != MS_ESTOP) {
                tgt_l = tgt_r = out_l = out_r = 0;
                turn_goal_l = turn_goal_r = 0;
                move_done_pending = FALSE;
                move_stable_periods = 0;
                stop_stable_periods = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                motor_fault_word |= FAULT_BIT_ESTOP;
                SET_STATE(MS_ESTOP);
            }
        } else if (state == MS_ESTOP) {
            motor_fault_word &= ~FAULT_BIT_ESTOP;
            stop_stable_periods = 0;
            SET_STATE(MS_IDLE);
        }

        while (tk_rcv_mbf(motor_mbfid, &cmd, TMO_POL) == (INT)sizeof(cmd)) {
            if (new_cmd) buffer_drop_count++;
            new_cmd = TRUE;
        }
#endif

        dl = enc_l_ticks - prev_l;
        dr = enc_r_ticks - prev_r;
        prev_l = enc_l_ticks;
        prev_r = enc_r_ticks;

        if (++print_timer >= 25) {
            print_timer = 0;
            if (dl != 0 || dr != 0 || out_l != 0 || out_r != 0) {
                INT run_l = iabs((INT)(enc_l_ticks - cruise_start_l));
                INT run_r = iabs((INT)(enc_r_ticks - cruise_start_r));
                tm_printf((UB *)"[dist] total=%d cm | run_ticks L=%d R=%d (err=%d) | PWM L=%d R=%d\n",
                    motor_get_avg_dist_cm(), run_l, run_r, run_l - run_r, out_l, out_r);
            }
        }

        switch (state) {
        case MS_IDLE:
            if (move_done_pending) {
                if (dl == 0 && dr == 0) {
                    if (move_stable_periods < 3) move_stable_periods++;
                } else {
                    move_stable_periods = 0;
                }

                if (move_stable_periods >= 3) {
                    NAVIGATOR_MESSAGE msg = {0};
                    msg.type = MSG_MOTOR_MOVE_DONE;
                    if (tk_snd_mbf(navigator_mbfid, &msg, sizeof(msg), TMO_POL) >= E_OK) {
                        move_done_pending = FALSE;
                        move_stable_periods = 0;
                        tm_printf((UB *)"[motors] Move completed\n");
                    }
                }
            }

            if (new_cmd && (cmd.target_speed_left != 0 || cmd.target_speed_right != 0)) {
                tgt_l = CLAMP(cmd.target_speed_left, -100, 100);
                tgt_r = CLAMP(cmd.target_speed_right, -100, 100);
                turn_goal_l = cmd.turn_ticks_left;
                turn_goal_r = cmd.turn_ticks_right;
                turn_start_l = enc_l_ticks;
                turn_start_r = enc_r_ticks;
                // Lock cruise_start_l/r BEFORE the wheels move so launch slip is tracked!
                cruise_start_l = enc_l_ticks;
                cruise_start_r = enc_r_ticks;
                stall_cnt_l = stall_cnt_r = 0;

                if (turn_goal_l != 0 || turn_goal_r != 0) {
                    out_l = tgt_l;
                    out_r = tgt_r;
                    set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
                    set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
                    SET_STATE(MS_TURN);
                } else {
                    ramp_mag_l = (iabs(tgt_l) > MIN_START_PCT) ? MIN_START_PCT : iabs(tgt_l);
                    ramp_mag_r = (iabs(tgt_r) > MIN_START_PCT) ? MIN_START_PCT : iabs(tgt_r);
                    SET_STATE(MS_ACCEL);
                }
            }
            break;

        case MS_ACCEL: {
            if (new_cmd) {
                INT new_l = CLAMP(cmd.target_speed_left, -100, 100);
                INT new_r = CLAMP(cmd.target_speed_right, -100, 100);
                turn_goal_l = cmd.turn_ticks_left;
                turn_goal_r = cmd.turn_ticks_right;

                if (new_l == 0 && new_r == 0) {
                    SET_STATE(MS_BRAKE);
                    break;
                }

                if (turn_goal_l != 0 || turn_goal_r != 0) {
                    tgt_l = new_l;
                    tgt_r = new_r;
                    turn_start_l = enc_l_ticks;
                    turn_start_r = enc_r_ticks;
                    out_l = tgt_l;
                    out_r = tgt_r;
                    set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
                    set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
                    SET_STATE(MS_TURN);
                    break;
                }

                if (new_l != tgt_l || new_r != tgt_r) {
                    cruise_start_l = enc_l_ticks;
                    cruise_start_r = enc_r_ticks;
                }
                tgt_l = new_l;
                tgt_r = new_r;
            }

            ramp_mag_l = ramp(ramp_mag_l, iabs(tgt_l), RAMP_STEP_PCT);
            ramp_mag_r = ramp(ramp_mag_r, iabs(tgt_r), RAMP_STEP_PCT);

            INT mag_l = ramp_mag_l;
            INT mag_r = ramp_mag_r;

            // Active heading lock even during acceleration so launch never twists the nose!
            if (tgt_l == -tgt_r && tgt_l != 0) {
                apply_straight_heading_lock(
                    ramp_mag_l, ramp_mag_r,
                    cruise_start_l, cruise_start_r,
                    dl, dr, &integ_units,
                    &mag_l, &mag_r);
            }

            out_l = (tgt_l > 0) ? mag_l : ((tgt_l < 0) ? -mag_l : 0);
            out_r = (tgt_r > 0) ? mag_r : ((tgt_r < 0) ? -mag_r : 0);

            set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
            set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);

            if (ramp_mag_l == iabs(tgt_l) && ramp_mag_r == iabs(tgt_r)) {
                // DO NOT overwrite cruise_start_l/r here—keep the launch reference!
                SET_STATE(MS_CRUISE);
            }
            break;
        }

        case MS_CRUISE: {
            if (new_cmd) {
                INT new_l = CLAMP(cmd.target_speed_left, -100, 100);
                INT new_r = CLAMP(cmd.target_speed_right, -100, 100);

                if (new_l == 0 && new_r == 0) {
                    SET_STATE(MS_BRAKE);
                    break;
                }

                if (cmd.turn_ticks_left != 0 || cmd.turn_ticks_right != 0) {
                    tgt_l = new_l;
                    tgt_r = new_r;
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

                if (new_l != tgt_l || new_r != tgt_r) {
                    cruise_start_l = enc_l_ticks;
                    cruise_start_r = enc_r_ticks;
                }
                tgt_l = new_l;
                tgt_r = new_r;
            }

            INT mag_l = iabs(tgt_l);
            INT mag_r = iabs(tgt_r);

            if (tgt_l == -tgt_r && tgt_l != 0) {
                apply_straight_heading_lock(
                    mag_l, mag_r,
                    cruise_start_l, cruise_start_r,
                    dl, dr, &integ_units,
                    &mag_l, &mag_r);
            } else {
                // During gentle line-following turns, keep the learned base motor balance
                INT base_offset = integ_units / KI_TRIM_DEN;
                if (mag_l > 0) mag_l = CLAMP(mag_l - base_offset, MIN_MOTOR_PCT, 100);
                if (mag_r > 0) mag_r = CLAMP(mag_r + base_offset, MIN_MOTOR_PCT, 100);
            }

            out_l = (tgt_l > 0) ? mag_l : ((tgt_l < 0) ? -mag_l : 0);
            out_r = (tgt_r > 0) ? mag_r : ((tgt_r < 0) ? -mag_r : 0);

            if (stalled(out_l, out_r, dl, dr, &stall_cnt_l, &stall_cnt_r)) {
                out_l = out_r = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                motor_fault_word |= FAULT_BIT_STALL;
                SET_STATE(MS_FAULT);
                break;
            }

            set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
            set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
            break;
        }

        case MS_TURN: {
            if (new_cmd) {
                if (cmd.target_speed_left == 0 && cmd.target_speed_right == 0) {
                    tgt_l = tgt_r = 0;
                    turn_goal_l = turn_goal_r = 0;
                    move_done_pending = FALSE;
                    SET_STATE(MS_BRAKE);
                    break;
                }

                if (cmd.target_speed_left != tgt_l || cmd.target_speed_right != tgt_r ||
                    cmd.turn_ticks_left != turn_goal_l || cmd.turn_ticks_right != turn_goal_r) {
                    tgt_l = CLAMP(cmd.target_speed_left, -100, 100);
                    tgt_r = CLAMP(cmd.target_speed_right, -100, 100);
                    turn_goal_l = cmd.turn_ticks_left;
                    turn_goal_r = cmd.turn_ticks_right;
                    turn_start_l = enc_l_ticks;
                    turn_start_r = enc_r_ticks;
                }
            }

            BOOL left_done  = (turn_goal_l == 0) || (iabs((INT)(enc_l_ticks - turn_start_l)) >= turn_goal_l);
            BOOL right_done = (turn_goal_r == 0) || (iabs((INT)(enc_r_ticks - turn_start_r)) >= turn_goal_r);

            if (left_done && right_done) {
                out_l = 0;
                out_r = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                turn_goal_l = 0;
                turn_goal_r = 0;
                move_done_pending = TRUE;
                move_stable_periods = 0;
                SET_STATE(MS_BRAKE);
                break;
            }

            out_l = left_done  ? 0 : tgt_l;
            out_r = right_done ? 0 : tgt_r;

            if (stalled(out_l, out_r, dl, dr, &stall_cnt_l, &stall_cnt_r)) {
                out_l = out_r = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                move_done_pending = FALSE;
                motor_fault_word |= FAULT_BIT_STALL;
                SET_STATE(MS_FAULT);
                break;
            }

            set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
            set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
            break;
        }

        case MS_BRAKE:
            if (new_cmd && (cmd.target_speed_left != 0 || cmd.target_speed_right != 0)) {
                tgt_l = CLAMP(cmd.target_speed_left, -100, 100);
                tgt_r = CLAMP(cmd.target_speed_right, -100, 100);
                turn_goal_l = cmd.turn_ticks_left;
                turn_goal_r = cmd.turn_ticks_right;
                turn_start_l = enc_l_ticks;
                turn_start_r = enc_r_ticks;
                cruise_start_l = enc_l_ticks;
                cruise_start_r = enc_r_ticks;
                stall_cnt_l = stall_cnt_r = 0;
                move_done_pending = FALSE;

                if (turn_goal_l != 0 || turn_goal_r != 0) {
                    out_l = tgt_l;
                    out_r = tgt_r;
                    set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
                    set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);
                    SET_STATE(MS_TURN);
                } else {
                    ramp_mag_l = (iabs(tgt_l) > MIN_START_PCT) ? MIN_START_PCT : iabs(tgt_l);
                    ramp_mag_r = (iabs(tgt_r) > MIN_START_PCT) ? MIN_START_PCT : iabs(tgt_r);
                    SET_STATE(MS_ACCEL);
                }
                break;
            }

            out_l = ramp(out_l, 0, BRAKE_STEP_PCT);
            out_r = ramp(out_r, 0, BRAKE_STEP_PCT);
            set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, out_l);
            set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, out_r);

            if (out_l == 0 && out_r == 0) {
                tgt_l = tgt_r = 0;
                SET_STATE(MS_IDLE);
            }
            break;

        case MS_FAULT:
            if (new_cmd && cmd.target_speed_left == 0 && cmd.target_speed_right == 0) {
                tgt_l = tgt_r = 0;
                motor_fault_word &= ~FAULT_BIT_STALL;
                SET_STATE(MS_IDLE);
            }
            break;

        case MS_ESTOP:
            if (dl == 0 && dr == 0) {
                if (stop_stable_periods < 3) stop_stable_periods++;
            } else {
                stop_stable_periods = 0;
            }

            if (motor_stop_ack_requested && stop_stable_periods >= 3) {
                NAVIGATOR_MESSAGE msg = {0};
                msg.type = MSG_MOTOR_STOPPED;
                if (tk_snd_mbf(navigator_mbfid, &msg, sizeof(msg), TMO_POL) >= E_OK) {
                    motor_stop_ack_requested = FALSE;
                    tm_printf((UB *)"[motors] Stop confirmed\n");
                }
            }
            break;

        default:
            break;
        }

        tk_dly_tsk(LOOP_DELAY_MS);
    }
}
