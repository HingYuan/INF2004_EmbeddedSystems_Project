#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include "Motor_task.h"
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"

// ============================================================================
// SET TO 1 TO VERIFY 50.0 CM (500 MM) RULER DISTANCE TEST
// SET TO 0 FOR NORMAL FULL-ROBOT OPERATION WITH NAVIGATOR
// ============================================================================
#define STAGE2_STANDALONE_PAD_TEST  0
#define TEST_TARGET_DIST_MM         500  /* 500 mm = 50.0 cm target distance */

// 125 MHz / (6249 + 1) = 20 kHz hardware PWM
#define MOTOR_PWM_WRAP          6249U

// Timing & Soft-Start Ramp parameters (tk_dly_tsk(2) = 20 ms = 50 Hz)
#define LOOP_DELAY_MS           2
#define MIN_START_PCT           18   /* Start both motors right at friction threshold together */
#define RAMP_STEP_PCT           2    /* Gentle ramp (2% per 20ms) prevents launch twist! */
#define BRAKE_STEP_PCT          200  /* Immediate active braking */
#define MIN_MOTOR_PCT           16   /* Minimum active cruise PWM % */

// True Heading-Lock PID Parameters (30.3 ticks = 1 cm)
#define TICKS_PER_PCT_KP        2    /* Position P: 1% PWM per 2 ticks (~0.66 mm) heading error */
#define KD_VEL_DIFF             3    /* Velocity D: 3% PWM per (dl - dr) tick/20ms speed diff */
#define KI_TRIM_NUM             1    /* Integral I: +1 unit per tick of error every 20 ms */
#define KI_TRIM_DEN             50   /* 50 units = 1% permanent motor balance offset */
#define MAX_INTEG_UNITS         300  /* Max +-6% learned permanent motor offset */
#define MAX_TRIM_PCT            8    /* Max +-8% total steering correction */

// Physical Tire-Diameter Trim (in parts per 1000):
// +15 means Right target ticks is scaled down by 1.5% relative to Left to cancel any left drift
// (Increase to +25 if it still drifts slightly left; set to 0 if dead straight)
#define WHEEL_LEFT_DRIFT_PERMILLE  15

// Fault detection thresholds (at 20 ms period)
#define STALL_MIN_PCT           20   /* Only check stall when PWM >= 20% */
#define STALL_PERIODS           75   /* 75 x 20 ms = 1.5 s with 0 ticks on BOTH wheels = stall */
#define ENC_FAIL_TICK_DELTA     90   /* >90 ticks (~3 cm) on one wheel while other is 0 = fault */
#define ENC_BACKUP_MIN_TICKS     8   /* Filtered B pulses without A progress before switching. */
_Static_assert(STALL_PERIODS <= 127, "stall counter bit width");
#define ENC_CRAWL_PCT           20   /* 20% open-loop crawl fallback on encoder failure */

#define SET_STATE(s) do { motor.state = (s); tm_printf((UB *)"[motors] state=%d\n", (INT)(s)); } while(0)

EXPORT volatile W enc_l_ticks = 0;
EXPORT volatile W enc_r_ticks = 0;
EXPORT volatile UW buffer_drop_count = 0;

// Live speed estimation (mm/s), updated every 20 ms
LOCAL volatile INT current_speed_mm_s = 0;

// Optional distance/tick target for moveForward/moveBackward(>255 ticks)
LOCAL volatile INT bounded_target_ticks_l = 0;
LOCAL volatile INT bounded_target_ticks_r = 0;

// Track current motor PWM outputs for encoder direction sign inside ISR
LOCAL volatile B isr_out_l = 0;
LOCAL volatile B isr_out_r = 0;
LOCAL volatile W isr_ticks_la = 0, isr_ticks_lb = 0;
LOCAL volatile W isr_ticks_ra = 0, isr_ticks_rb = 0;
LOCAL volatile ENCODER_ISR_STATE encoder_state = {0};
#define right_encoder_using_b encoder_state.using_b
LOCAL volatile W right_encoder_last_a = 0;
LOCAL volatile W right_encoder_b_offset = 0;

LOCAL INT iabs(INT v)
{
    return v < 0 ? -v : v;
}

LOCAL W right_encoder_ticks(W a, W b)
{
    if (!right_encoder_using_b) {
        if (a != right_encoder_last_a) {
            right_encoder_last_a = a;
            right_encoder_b_offset = a - b;
        } else if (iabs((INT)(b + right_encoder_b_offset - a)) >= ENC_BACKUP_MIN_TICKS) {
            // Keep B until restart; intermittent A recovery must not jump the distance count.
            right_encoder_using_b = TRUE;
        }
    }
    // The offset preserves previous A travel and recovers pulses missed since A stopped.
    return right_encoder_using_b ? b + right_encoder_b_offset : a;
}

#ifndef NDEBUG
LOCAL BOOL right_encoder_self_check(void)
{
    BOOL ok = right_encoder_ticks(12, 13) == 12 && !right_encoder_using_b &&
              right_encoder_ticks(12, 20) == 12 && !right_encoder_using_b &&
              right_encoder_ticks(12, 21) == 20 && right_encoder_using_b &&
              right_encoder_ticks(12, 1) == 0 &&
              right_encoder_ticks(12, -59) == -60 &&
              right_encoder_ticks(-60, -59) == -60;
    // Run before enabling the encoder interrupt, then restore the initial counter state.
    right_encoder_last_a = right_encoder_b_offset = 0;
    right_encoder_using_b = FALSE;
    return ok && right_encoder_ticks(0, 0) == 0 && !right_encoder_using_b;
}
#endif

// Proven Stage 1 Encoder Filter: 2-sample (1.0 ms) rising-edge counter with NO cooldown ceiling.
LOCAL void count_clean_rising_edge(UINT raw, INT dir, UB *filters, UINT shift, volatile W *ticks)
{
    UINT stable = (*filters >> shift) & 1U;
    UINT pending = (*filters >> (shift + 1U)) & 1U;
    if (raw == stable) {
        pending = 0;
    } else if (pending) {
        stable = raw;
        pending = 0;
        if (stable) *ticks += dir;
    } else {
        pending = 1;
    }
    *filters = (UB)((*filters & ~(3U << shift)) | ((stable | (pending << 1U)) << shift));
}

// 0.25 ms (250 us = 4000 Hz) RP2040 Hardware Timer Alarm 0 ISR
// 2 samples at 250 us = 0.5 ms filter -> never drops narrow pulses when wheels speed up!
LOCAL void encoder_alarm_isr(UINT intno)
{
    out_w(TIMER_INTR, 1U);
    out_w(TIMER_ALARM0, in_w(TIMER_TIMERAWL) + 250U);
    ClearInt(intno);

    UW v = in_w(GPIO_IN);
    UINT raw_la = (v >> MOTOR_LEFT_ENC_A) & 1U;
    UINT raw_lb = (v >> MOTOR_LEFT_ENC_B) & 1U;
    UINT raw_ra = (v >> MOTOR_RIGHT_ENC_A) & 1U;
    UINT raw_rb = (v >> MOTOR_RIGHT_ENC_B) & 1U;

    // Left forward is positive PWM; Right forward is negative PWM
    INT dir_l = (isr_out_l > 0) ? 1 : (isr_out_l < 0 ? -1 : 1);
    INT dir_r = (isr_out_r < 0) ? 1 : (isr_out_r > 0 ? -1 : 1);

    UB filters = encoder_state.filters;

    count_clean_rising_edge(raw_la, dir_l, &filters, 0U, &isr_ticks_la);
    count_clean_rising_edge(raw_lb, dir_l, &filters, 2U, &isr_ticks_lb);
    count_clean_rising_edge(raw_ra, dir_r, &filters, 4U, &isr_ticks_ra);
    count_clean_rising_edge(raw_rb, dir_r, &filters, 6U, &isr_ticks_rb);
    encoder_state.filters = filters;

    // Left B has extra bounce; right B provides backup when right A stops counting.
    enc_l_ticks = isr_ticks_la;
    enc_r_ticks = right_encoder_ticks(isr_ticks_ra, isr_ticks_rb);
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

/* Odometry & Speed Estimation APIs (Calibrated 0.33 mm/tick -> MM_PER_TICK_X100 = 33) */
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

EXPORT INT motor_get_speed_mm_s(void)
{
    return current_speed_mm_s;
}

/* Reusable Motion-Control APIs (Required by Buddy 2 Write-up Page 6) */
EXPORT ER moveForward(INT distance_mm)
{
    MOTOR_COMMAND cmd = {0};
    INT ticks = (distance_mm > 0) ? (distance_mm * 100) / MM_PER_TICK_X100 : 0;
    bounded_target_ticks_l = ticks;
    bounded_target_ticks_r = ticks;
    cmd.target_speed_left  = 25;
    cmd.target_speed_right = -25;
    cmd.turn_ticks_left    = (ticks > 255) ? 255 : (UB)ticks;
    cmd.turn_ticks_right   = (ticks > 255) ? 255 : (UB)ticks;
    return tk_snd_mbf(motor_mbfid, &cmd, sizeof(cmd), TMO_POL);
}

EXPORT ER moveBackward(INT distance_mm)
{
    MOTOR_COMMAND cmd = {0};
    INT ticks = (distance_mm > 0) ? (distance_mm * 100) / MM_PER_TICK_X100 : 0;
    bounded_target_ticks_l = ticks;
    bounded_target_ticks_r = ticks;
    cmd.target_speed_left  = -25;
    cmd.target_speed_right = 25;
    cmd.turn_ticks_left    = (ticks > 255) ? 255 : (UB)ticks;
    cmd.turn_ticks_right   = (ticks > 255) ? 255 : (UB)ticks;
    return tk_snd_mbf(motor_mbfid, &cmd, sizeof(cmd), TMO_POL);
}

EXPORT ER turnLeft(INT angle_deg)
{
    MOTOR_COMMAND cmd = {0};
    // Arc length per wheel for in-place pivot: (pi * WHEELBASE_MM * angle_deg) / 360
    INT arc_mmx100 = (314 * WHEELBASE_MM * iabs(angle_deg)) / 360;
    INT ticks = arc_mmx100 / MM_PER_TICK_X100;
    bounded_target_ticks_l = ticks;
    bounded_target_ticks_r = ticks;
    cmd.target_speed_left  = -25;
    cmd.target_speed_right = -25;
    cmd.turn_ticks_left    = (ticks > 255) ? 255 : (UB)ticks;
    cmd.turn_ticks_right   = (ticks > 255) ? 255 : (UB)ticks;
    return tk_snd_mbf(motor_mbfid, &cmd, sizeof(cmd), TMO_POL);
}

EXPORT ER turnRight(INT angle_deg)
{
    MOTOR_COMMAND cmd = {0};
    INT arc_mmx100 = (314 * WHEELBASE_MM * iabs(angle_deg)) / 360;
    INT ticks = arc_mmx100 / MM_PER_TICK_X100;
    bounded_target_ticks_l = ticks;
    bounded_target_ticks_r = ticks;
    cmd.target_speed_left  = 25;
    cmd.target_speed_right = 25;
    cmd.turn_ticks_left    = (ticks > 255) ? 255 : (UB)ticks;
    cmd.turn_ticks_right   = (ticks > 255) ? 255 : (UB)ticks;
    return tk_snd_mbf(motor_mbfid, &cmd, sizeof(cmd), TMO_POL);
}

EXPORT ER stop(void)
{
    MOTOR_COMMAND cmd = {0};
    bounded_target_ticks_l = 0;
    bounded_target_ticks_r = 0;
    return tk_snd_mbf(motor_mbfid, &cmd, sizeof(cmd), TMO_POL);
}

LOCAL INT ramp(INT cur, INT target, INT step)
{
    if (cur < target) return (target - cur < step) ? target : cur + step;
    if (cur > target) return (cur - target < step) ? target : cur - step;
    return cur;
}

// Compute synchronized straight-line wheel PWM (used in MS_ACCEL, MS_CRUISE, and straight MS_TURN)
LOCAL void apply_straight_heading_lock(
    INT base_mag_l, INT base_mag_r,
    W cruise_start_l, W cruise_start_r,
    W dl, W dr, INT *integ_units,
    INT *out_mag_l, INT *out_mag_r)
{
    INT moved_l = iabs((INT)(enc_l_ticks - cruise_start_l));
    // Scale moved_r by (1000 + WHEEL_LEFT_DRIFT_PERMILLE)/1000 so Left wheel turns slightly faster
    INT raw_moved_r = iabs((INT)(enc_r_ticks - cruise_start_r));
    INT moved_r = (raw_moved_r * (1000 + WHEEL_LEFT_DRIFT_PERMILLE)) / 1000;
    INT diff = moved_l - moved_r;
    INT vel_diff = iabs((INT)dl) - iabs((INT)dr);

    // Fail-Safe: Encoder failure detection (>90 ticks on one wheel while other is 0)
    if (iabs(diff) > ENC_FAIL_TICK_DELTA && (moved_l == 0 || moved_r == 0)) {
        robot_flag_assign(ROBOT_ENCODER_DEGRADED, TRUE);
        robot_flags_update(FAULT_BIT_ENCODER, 0U);
    }

    if (encoder_degraded) {
        *out_mag_l = (base_mag_l > ENC_CRAWL_PCT) ? ENC_CRAWL_PCT : base_mag_l;
        *out_mag_r = (base_mag_r > ENC_CRAWL_PCT) ? ENC_CRAWL_PCT : base_mag_r;
        return;
    }

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

LOCAL BOOL stalled(INT out_l, INT out_r, W dl, W dr, MOTOR_RUNTIME_STATE *motor)
{
    if (iabs(out_l) >= STALL_MIN_PCT) {
        if (dl == 0) { if (motor->stall_left < STALL_PERIODS) motor->stall_left++; }
        else motor->stall_left = 0;
    } else {
        motor->stall_left = 0;
    }
    if (iabs(out_r) >= STALL_MIN_PCT) {
        if (dr == 0) { if (motor->stall_right < STALL_PERIODS) motor->stall_right++; }
        else motor->stall_right = 0;
    } else {
        motor->stall_right = 0;
    }
    return (motor->stall_left >= STALL_PERIODS && motor->stall_right >= STALL_PERIODS);
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
    MOTOR_RUNTIME_STATE motor = {.state = MS_IDLE};
    INT tgt_l = 0, tgt_r = 0;
    INT ramp_mag_l = 0, ramp_mag_r = 0;
    INT out_l = 0, out_r = 0;
    W prev_l = 0, prev_r = 0;
    W turn_start_l = 0, turn_start_r = 0;
    INT turn_goal_l = 0, turn_goal_r = 0;
    W cruise_start_l = 0, cruise_start_r = 0;
    INT integ_units = 0; // Start neutral (0) so right wheel doesn't push the car left


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

#ifndef NDEBUG
    if (!right_encoder_self_check()) {
        robot_flags_update(FAULT_BIT_ENCODER, 0U);
        tm_printf((UB *)"[motors] Encoder self-check failed\n");
        tk_ext_tsk();
    }
#endif
    T_DINT dint = {0};
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)encoder_alarm_isr;
    if (tk_def_int(0, &dint) == E_OK) {
        out_w(TIMER_INTR, 1U);
        set_w(TIMER_INTE, 1U);
        out_w(TIMER_ALARM0, in_w(TIMER_TIMERAWL) + 250U);
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

        robot_flag_assign(ROBOT_ESTOP_REQUEST, FALSE);
        test_cycles++;

        // Wait 3.0 seconds (150 x 20ms) after power-on, then start driving straight at 25%
        if (test_cycles == 150) {
            UINT intsts;
            DI(intsts);
            enc_l_ticks = 0;
            enc_r_ticks = 0;
            isr_ticks_la = isr_ticks_lb = isr_ticks_ra = isr_ticks_rb = 0;
            right_encoder_last_a = right_encoder_b_offset = 0;
            right_encoder_using_b = motor.right_backup_reported = FALSE;
            prev_l = prev_r = 0;
            EI(intsts);
            cmd.target_speed_left = 25;
            cmd.target_speed_right = -25;
            cmd.turn_ticks_left = 0;
            cmd.turn_ticks_right = 0;
            new_cmd = TRUE;
            tm_printf((UB *)"\n=== [DISTANCE TEST] Driving to %d mm (50.0 cm)... ===\n",
                      TEST_TARGET_DIST_MM);
        }

        // Stop as soon as average distance reaches 500 mm (50.0 cm)
        if (test_cycles > 150 && run_number == 0 &&
            motor_get_avg_dist_mm() >= TEST_TARGET_DIST_MM) {
            run_number = 1;
            cmd.target_speed_left = 0;
            cmd.target_speed_right = 0;
            cmd.turn_ticks_left = 0;
            cmd.turn_ticks_right = 0;
            new_cmd = TRUE;
        }

        // Print final settled distance after braking
        if (run_number == 1 && motor.state == MS_IDLE && dl == 0 && dr == 0) {
            run_number = 2;
            tm_printf((UB *)"\n=== [DISTANCE TEST COMPLETE] ===\n");
            tm_printf((UB *)"Reported: %d cm (L=%d mm, R=%d mm, Avg=%d mm)\n",
                      motor_get_avg_dist_cm(),
                      motor_get_left_dist_mm(),
                      motor_get_right_dist_mm(),
                      motor_get_avg_dist_mm());
            tm_printf((UB *)"Final Ticks: L=%d, R=%d (Avg=%d ticks)\n",
                      (INT)enc_l_ticks, (INT)enc_r_ticks,
                      (INT)((enc_l_ticks + enc_r_ticks) / 2));
            tm_printf((UB *)"Measure physical distance with a ruler now!\n");
        }
#else
        if (motor_estop) {
            if (motor.state != MS_ESTOP) {
                tgt_l = tgt_r = out_l = out_r = 0;
                turn_goal_l = turn_goal_r = 0;
                bounded_target_ticks_l = bounded_target_ticks_r = 0;
                motor.move_done_pending = FALSE;
                motor.move_stable_periods = 0;
                motor.stop_stable_periods = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                robot_flags_update(FAULT_BIT_ESTOP, 0U);
                SET_STATE(MS_ESTOP);
            }
        } else if (motor.state == MS_ESTOP) {
            robot_flags_update(0U, FAULT_BIT_ESTOP);
            motor.stop_stable_periods = 0;
            SET_STATE(MS_IDLE);
        }

        while (tk_rcv_mbf(motor_mbfid, &cmd, TMO_POL) == (INT)sizeof(cmd)) {
            if (new_cmd) buffer_drop_count++;
            new_cmd = TRUE;
        }
#endif

        if (right_encoder_using_b && !motor.right_backup_reported) {
            motor.right_backup_reported = TRUE;
            tm_printf((UB *)"[motors] Right encoder A missing; using B (A=%d B=%d)\n",
                      (INT)isr_ticks_ra, (INT)isr_ticks_rb);
        }
        dl = enc_l_ticks - prev_l;
        dr = enc_r_ticks - prev_r;
        prev_l = enc_l_ticks;
        prev_r = enc_r_ticks;

        // Live speed estimation in mm/s (50 Hz loop -> ticks_per_20ms * 50 * 0.33 mm/tick)
        INT avg_ticks_20ms = (iabs((INT)dl) + iabs((INT)dr)) / 2;
        current_speed_mm_s = (avg_ticks_20ms * 50 * MM_PER_TICK_X100) / 100;

        if (++motor.print_timer >= 25) {
            motor.print_timer = 0;
            if (dl != 0 || dr != 0 || out_l != 0 || out_r != 0) {
                INT run_l = iabs((INT)(enc_l_ticks - cruise_start_l));
                INT run_r = iabs((INT)(enc_r_ticks - cruise_start_r));
                tm_printf((UB *)"[dist] %d cm | L=%d(A:%d,B:%d) R=%d(A:%d,B:%d) err=%d | PWM L=%d R=%d\n",
                    motor_get_avg_dist_cm(),
                    run_l, (INT)isr_ticks_la, (INT)isr_ticks_lb,
                    run_r, (INT)isr_ticks_ra, (INT)isr_ticks_rb,
                    run_l - run_r, out_l, out_r);
            }
        }

        switch (motor.state) {
        case MS_IDLE:
            if (motor.move_done_pending) {
                if (dl == 0 && dr == 0) {
                    if (motor.move_stable_periods < 3) motor.move_stable_periods++;
                } else {
                    motor.move_stable_periods = 0;
                }

                if (motor.move_stable_periods >= 3) {
                    NAVIGATOR_MESSAGE msg = {0};
                    msg.type = MSG_MOTOR_MOVE_DONE;
                    if (tk_snd_mbf(navigator_mbfid, &msg, sizeof(msg), TMO_POL) >= E_OK) {
                        motor.move_done_pending = FALSE;
                        motor.move_stable_periods = 0;
                        tm_printf((UB *)"[motors] Move completed\n");
                    }
                }
            }

            if (new_cmd && (cmd.target_speed_left != 0 || cmd.target_speed_right != 0)) {
                tgt_l = CLAMP(cmd.target_speed_left, -100, 100);
                tgt_r = CLAMP(cmd.target_speed_right, -100, 100);
                turn_goal_l = (bounded_target_ticks_l > 0) ? bounded_target_ticks_l : cmd.turn_ticks_left;
                turn_goal_r = (bounded_target_ticks_r > 0) ? bounded_target_ticks_r : cmd.turn_ticks_right;
                bounded_target_ticks_l = bounded_target_ticks_r = 0;
                turn_start_l = enc_l_ticks;
                turn_start_r = enc_r_ticks;
                cruise_start_l = enc_l_ticks;
                cruise_start_r = enc_r_ticks;
                motor.stall_left = motor.stall_right = 0;
                robot_flag_assign(ROBOT_ENCODER_DEGRADED, FALSE);

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
                turn_goal_l = (bounded_target_ticks_l > 0) ? bounded_target_ticks_l : cmd.turn_ticks_left;
                turn_goal_r = (bounded_target_ticks_r > 0) ? bounded_target_ticks_r : cmd.turn_ticks_right;
                bounded_target_ticks_l = bounded_target_ticks_r = 0;

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

                if (cmd.turn_ticks_left != 0 || cmd.turn_ticks_right != 0 ||
                    bounded_target_ticks_l > 0 || bounded_target_ticks_r > 0) {
                    tgt_l = new_l;
                    tgt_r = new_r;
                    turn_goal_l = (bounded_target_ticks_l > 0) ? bounded_target_ticks_l : cmd.turn_ticks_left;
                    turn_goal_r = (bounded_target_ticks_r > 0) ? bounded_target_ticks_r : cmd.turn_ticks_right;
                    bounded_target_ticks_l = bounded_target_ticks_r = 0;
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
            }

            out_l = (tgt_l > 0) ? mag_l : ((tgt_l < 0) ? -mag_l : 0);
            out_r = (tgt_r > 0) ? mag_r : ((tgt_r < 0) ? -mag_r : 0);

            if (!encoder_degraded && stalled(out_l, out_r, dl, dr, &motor)) {
                out_l = out_r = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                robot_flags_update(FAULT_BIT_STALL, 0U);
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
                    bounded_target_ticks_l = bounded_target_ticks_r = 0;
                    motor.move_done_pending = FALSE;
                    SET_STATE(MS_BRAKE);
                    break;
                }

                INT next_goal_l = (bounded_target_ticks_l > 0) ? bounded_target_ticks_l : cmd.turn_ticks_left;
                INT next_goal_r = (bounded_target_ticks_r > 0) ? bounded_target_ticks_r : cmd.turn_ticks_right;
                bounded_target_ticks_l = bounded_target_ticks_r = 0;

                if (cmd.target_speed_left != tgt_l || cmd.target_speed_right != tgt_r ||
                    next_goal_l != turn_goal_l || next_goal_r != turn_goal_r) {
                    tgt_l = CLAMP(cmd.target_speed_left, -100, 100);
                    tgt_r = CLAMP(cmd.target_speed_right, -100, 100);
                    turn_goal_l = next_goal_l;
                    turn_goal_r = next_goal_r;
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
                motor.move_done_pending = TRUE;
                motor.move_stable_periods = 0;
                SET_STATE(MS_BRAKE);
                break;
            }

            INT mag_l = iabs(tgt_l);
            INT mag_r = iabs(tgt_r);

            // If MS_TURN is used for a straight bounded move (moveForward / moveBackward), keep heading locked!
            if (tgt_l == -tgt_r && tgt_l != 0 && !left_done && !right_done) {
                apply_straight_heading_lock(
                    mag_l, mag_r,
                    turn_start_l, turn_start_r,
                    dl, dr, &integ_units,
                    &mag_l, &mag_r);
            }

            out_l = left_done  ? 0 : ((tgt_l > 0) ? mag_l : -mag_l);
            out_r = right_done ? 0 : ((tgt_r > 0) ? mag_r : -mag_r);

            if (!encoder_degraded && stalled(out_l, out_r, dl, dr, &motor)) {
                out_l = out_r = 0;
                set_motor(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
                set_motor(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
                motor.move_done_pending = FALSE;
                robot_flags_update(FAULT_BIT_STALL, 0U);
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
                turn_goal_l = (bounded_target_ticks_l > 0) ? bounded_target_ticks_l : cmd.turn_ticks_left;
                turn_goal_r = (bounded_target_ticks_r > 0) ? bounded_target_ticks_r : cmd.turn_ticks_right;
                bounded_target_ticks_l = bounded_target_ticks_r = 0;
                turn_start_l = enc_l_ticks;
                turn_start_r = enc_r_ticks;
                cruise_start_l = enc_l_ticks;
                cruise_start_r = enc_r_ticks;
                motor.stall_left = motor.stall_right = 0;
                motor.move_done_pending = FALSE;

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
                robot_flags_update(0U, FAULT_BIT_STALL);
                SET_STATE(MS_IDLE);
            }
            break;

        case MS_ESTOP:
            if (dl == 0 && dr == 0) {
                if (motor.stop_stable_periods < 3) motor.stop_stable_periods++;
            } else {
                motor.stop_stable_periods = 0;
            }

            if (motor_stop_ack_requested && motor.stop_stable_periods >= 3) {
                NAVIGATOR_MESSAGE msg = {0};
                msg.type = MSG_MOTOR_STOPPED;
                if (tk_snd_mbf(navigator_mbfid, &msg, sizeof(msg), TMO_POL) >= E_OK) {
                    robot_flag_assign(ROBOT_STOP_ACK_REQUEST, FALSE);
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
