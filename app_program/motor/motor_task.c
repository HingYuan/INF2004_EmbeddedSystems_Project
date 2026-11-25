/**
 * @file motor_task.c
 * @brief Closed-loop PWM motor control, quadrature encoder ISR, and odometry.
 *
 * NOTE: See docs/fsm_diagrams.md for the Motor FSM diagram.
 * WARNING: Physical tire-diameter trim (MOTOR_LEFT_DRIFT_PERMILLE) and
 * odometry scale (MOTOR_MM_PER_TICK_X100) are empirically calibrated to the
 * assembled chassis.
 *
 * BARR-C Deviation (Rules 5.2.a, 6.5.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel task/interrupt/PWM APIs require native RTOS typedefs
 *   (INT, UINT, UW, ER, ID, FP, UB); TA_HLNG ISR uses standard AAPCS C entry.
 */

#include "motor_task.h"

#include <stdbool.h>
#include <stdint.h>

#include <bsp/libbsp.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>

#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"

/* Set to 1u for standalone 500 mm ruler test; 0u for normal Navigator mode. */
#define MOTOR_STAGE2_PAD_TEST           (0u)
#define MOTOR_TEST_TARGET_DIST_MM       (500)
#define MOTOR_TEST_START_CYCLES         (150)

/* 125 MHz / (6249 + 1) = 20 kHz hardware PWM. */
#define MOTOR_PWM_WRAP                  (6249u)
#define MOTOR_PWM_MAX_PCT               (100)
#define MOTOR_PWM_MIN_NEG_PCT           (-100)
#define MOTOR_PWM_SCALE_U32             (100u)
#define MOTOR_SCALE_HUNDRED             (100)
#define MOTOR_MM_PER_CM                 (10)
#define MOTOR_HALF_CM_MM                (5)
#define MOTOR_PAIR_DIVISOR              (2)
#define MOTOR_API_DRIVE_PCT             (25)
#define MOTOR_MAX_CMD_TICKS             (255)
#define MOTOR_PIN_COUNT                 (4u)

/* Timing and soft-start ramp parameters (tk_dly_tsk(2) = 20 ms = 50 Hz). */
#define MOTOR_LOOP_DELAY_MSEC           (2u)
#define MOTOR_LOOP_FREQ_HZ              (50)
#define MOTOR_MIN_START_PCT             (18)
#define MOTOR_RAMP_STEP_PCT             (2)
#define MOTOR_BRAKE_STEP_PCT            (200)
#define MOTOR_MIN_ACTIVE_PCT            (16)
#define MOTOR_STABLE_PERIODS_REQ        (3u)
#define MOTOR_PRINT_PERIOD_TICKS        (25u)

/* Heading-lock PID parameters (30.3 ticks = 1 cm). */
#define MOTOR_TICKS_PER_PCT_KP          (2)
#define MOTOR_KD_VEL_DIFF               (3)
#define MOTOR_KI_TRIM_NUM               (1)
#define MOTOR_KI_TRIM_DEN               (50)
#define MOTOR_MAX_INTEG_UNITS           (300)
#define MOTOR_MAX_TRIM_PCT              (8)
#define MOTOR_INTEG_ERR_BAND            (15)
#define MOTOR_LEFT_DRIFT_PERMILLE       (15)
#define MOTOR_PERMILLE_SCALE            (1000)

/* Fault detection thresholds at 20 ms loop period. */
#define MOTOR_STALL_MIN_PCT             (20)
#define MOTOR_STALL_PERIODS             (75u)
#define MOTOR_ENC_FAIL_TICK_DELTA       (90)
#define MOTOR_ENC_BACKUP_MIN_TICKS      (8)
#define MOTOR_ENC_CRAWL_PCT             (20)

/* 250 usec (4000 Hz) RP2040 hardware Timer Alarm 0 ISR parameters. */
#define MOTOR_ENC_ALARM_IRQ             (0u)
#define MOTOR_ENC_ALARM_PRI             (2)
#define MOTOR_ENC_ALARM_USEC            (250u)
#define MOTOR_ENC_LA_SHIFT              (0u)
#define MOTOR_ENC_LB_SHIFT              (2u)
#define MOTOR_ENC_RA_SHIFT              (4u)
#define MOTOR_ENC_RB_SHIFT              (6u)
#define MOTOR_ENC_PAIR_MASK             (3u)
#define MOTOR_PI_X100                   (314)
#define MOTOR_FULL_CIRCLE_DEG           (360)

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t state                 : 3;
    uint32_t b_has_move_done_pend  : 1;
    uint32_t b_has_reported_backup : 1;
    uint32_t move_stable_periods   : 2;
    uint32_t stop_stable_periods   : 2;
    uint32_t stall_left            : 7;
    uint32_t stall_right           : 7;
    uint32_t print_timer           : 5;
} motor_task_runtime_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint16_t filters      : 8;
    uint16_t b_is_using_b : 1;
} motor_task_enc_isr_t;

typedef struct
{
    motor_task_runtime_t runtime;
    int32_t              tgt_left;
    int32_t              tgt_right;
    int32_t              ramp_mag_left;
    int32_t              ramp_mag_right;
    int32_t              out_left;
    int32_t              out_right;
    int32_t              prev_left;
    int32_t              prev_right;
    int32_t              turn_start_left;
    int32_t              turn_start_right;
    int32_t              turn_goal_left;
    int32_t              turn_goal_right;
    int32_t              cruise_start_left;
    int32_t              cruise_start_right;
    int32_t              integ_units;
#if (0u != MOTOR_STAGE2_PAD_TEST)
    int32_t              test_cycles;
    int32_t              run_number;
#endif
} motor_task_ctx_t;

ROBOT_TASKS_STATIC_ASSERT(motor_cmd_size,
                          (4u == sizeof(motor_task_cmd_t)));
ROBOT_TASKS_STATIC_ASSERT(motor_stall_bits,
                          (MOTOR_STALL_PERIODS <= 127u));
ROBOT_TASKS_STATIC_ASSERT(motor_runtime_size,
                          (4u == sizeof(motor_task_runtime_t)));
ROBOT_TASKS_STATIC_ASSERT(motor_enc_isr_size,
                          (2u == sizeof(motor_task_enc_isr_t)));

static volatile int32_t              g_enc_left_ticks          = 0;
static volatile int32_t              g_enc_right_ticks         = 0;
static volatile uint32_t             g_buf_drop_cnt            = 0u;
static volatile int32_t              g_cur_speed_mm_per_second = 0;
static volatile int32_t              g_bounded_ticks_left      = 0;
static volatile int32_t              g_bounded_ticks_right     = 0;
static volatile int8_t               g_isr_out_left            = 0;
static volatile int8_t               g_isr_out_right           = 0;
static volatile int32_t              g_isr_ticks_la            = 0;
static volatile int32_t              g_isr_ticks_lb            = 0;
static volatile int32_t              g_isr_ticks_ra            = 0;
static volatile int32_t              g_isr_ticks_rb            = 0;
static volatile motor_task_enc_isr_t g_enc_isr_state           = { 0u, 0u };
static volatile int32_t              g_right_enc_last_a        = 0;
static volatile int32_t              g_right_enc_b_offset      = 0;

static int32_t motor_abs_i32(int32_t val);
static void motor_set_state(motor_task_runtime_t * p_rt,
                            motor_task_state_t next_state);
static void motor_take_bounded_goals(const motor_task_cmd_t * p_cmd,
                                     int32_t * p_goal_l,
                                     int32_t * p_goal_r);
static ER motor_queue_bounded_cmd(int8_t left_spd,
                                  int8_t right_spd,
                                  int32_t target_ticks);
static int32_t motor_ramp(int32_t cur_val,
                          int32_t target_val,
                          int32_t step_val);
static void motor_set_channel(uint32_t pin_a,
                              uint32_t pin_b,
                              int32_t speed_pct);
static int32_t motor_right_enc_ticks(int32_t ticks_a, int32_t ticks_b);
static void motor_count_rising_edge(uint32_t raw_lvl,
                                    int32_t dir_sign,
                                    uint8_t * p_filters,
                                    uint32_t bit_shift,
                                    volatile int32_t * p_ticks);
static void motor_apply_heading_lock(int32_t base_mag_l,
                                     int32_t base_mag_r,
                                     int32_t start_ticks_l,
                                     int32_t start_ticks_r,
                                     int32_t delta_l,
                                     int32_t delta_r,
                                     int32_t * p_integ_units,
                                     int32_t * p_out_mag_l,
                                     int32_t * p_out_mag_r);
static bool motor_check_stalled(int32_t out_l,
                                int32_t out_r,
                                int32_t delta_l,
                                int32_t delta_r,
                                motor_task_runtime_t * p_rt);
static bool motor_init_hardware(void);
static void motor_enter_turn_from_drive(motor_task_ctx_t * p_ctx,
                                        int32_t new_l,
                                        int32_t new_r,
                                        int32_t goal_l,
                                        int32_t goal_r);
static void motor_start_new_motion(motor_task_ctx_t * p_ctx,
                                   const motor_task_cmd_t * p_cmd,
                                   bool b_should_clr_degraded);
#if (0u == MOTOR_STAGE2_PAD_TEST)
static void motor_sync_estop_state(motor_task_ctx_t * p_ctx);
#endif
static void motor_step_inputs(motor_task_ctx_t * p_ctx,
                              motor_task_cmd_t * p_cmd,
                              bool * pb_has_new_cmd,
                              int32_t prev_dl,
                              int32_t prev_dr);
static void motor_update_speed_log(motor_task_ctx_t * p_ctx,
                                   int32_t delta_l,
                                   int32_t delta_r);
static void motor_update_cruise_targets(motor_task_ctx_t * p_ctx,
                                        int32_t new_l,
                                        int32_t new_r);
static void motor_update_turn_targets(motor_task_ctx_t * p_ctx,
                                      const motor_task_cmd_t * p_cmd);
static void motor_drive_turn_active(motor_task_ctx_t * p_ctx,
                                    bool b_is_left_done,
                                    bool b_is_right_done,
                                    int32_t delta_l,
                                    int32_t delta_r);
static void motor_step_idle(motor_task_ctx_t * p_ctx,
                            const motor_task_cmd_t * p_cmd,
                            bool b_has_new_cmd,
                            int32_t delta_l,
                            int32_t delta_r);
static void motor_step_accel(motor_task_ctx_t * p_ctx,
                             const motor_task_cmd_t * p_cmd,
                             bool b_has_new_cmd,
                             int32_t delta_l,
                             int32_t delta_r);
static void motor_step_cruise(motor_task_ctx_t * p_ctx,
                              const motor_task_cmd_t * p_cmd,
                              bool b_has_new_cmd,
                              int32_t delta_l,
                              int32_t delta_r);
static void motor_step_turn(motor_task_ctx_t * p_ctx,
                            const motor_task_cmd_t * p_cmd,
                            bool b_has_new_cmd,
                            int32_t delta_l,
                            int32_t delta_r);
static void motor_step_brake(motor_task_ctx_t * p_ctx,
                             const motor_task_cmd_t * p_cmd,
                             bool b_has_new_cmd);
static void motor_step_estop(motor_task_ctx_t * p_ctx,
                             int32_t delta_l,
                             int32_t delta_r);
static void motor_encoder_alarm_isr(UINT intno);

#ifndef NDEBUG
static bool motor_right_enc_self_check(void);
#endif

/**
 * @brief Clamp a signed 32-bit value into [low_limit, high_limit].
 * @param[in] value      Input value.
 * @param[in] low_limit  Minimum permitted value.
 * @param[in] high_limit Maximum permitted value.
 * @return Clamped integer value.
 */
int32_t motor_task_clamp (int32_t value, int32_t low_limit, int32_t high_limit)
{
    int32_t clamped = value;

    if (clamped < low_limit)
    {
        clamped = low_limit;
    }
    else if (clamped > high_limit)
    {
        clamped = high_limit;
    }
    else
    {
        /* Input value already lies within [low_limit, high_limit]. */
    }

    return clamped;
}

/**
 * @brief Return the motor stall, encoder, and E-stop fault bitmask.
 * @return Bitmask of MOTOR_FAULT_BIT_* flags currently asserted.
 */
uint32_t motor_task_get_fault_word (void)
{
    const uint32_t fault_mask =
        MOTOR_FAULT_BIT_STALL |
        MOTOR_FAULT_BIT_ENCODER |
        MOTOR_FAULT_BIT_ESTOP;

    return (robot_tasks_flags_get() & fault_mask);
}

/**
 * @brief Check whether an emergency stop request is active.
 * @return True if ROBOT_ESTOP_REQUEST is set, false otherwise.
 */
bool motor_task_is_estop (void)
{
    return (0u != (robot_tasks_flags_get() & ROBOT_ESTOP_REQUEST));
}

/**
 * @brief Check whether Navigator requested an E-stop acknowledgement.
 * @return True if ROBOT_STOP_ACK_REQUEST is set, false otherwise.
 */
bool motor_task_is_stop_ack_req (void)
{
    return (0u != (robot_tasks_flags_get() & ROBOT_STOP_ACK_REQUEST));
}

/**
 * @brief Check whether closed-loop encoder operation is degraded.
 * @return True if ROBOT_ENCODER_DEGRADED is set, false otherwise.
 */
bool motor_task_is_enc_degraded (void)
{
    return (0u != (robot_tasks_flags_get() & ROBOT_ENCODER_DEGRADED));
}

/**
 * @brief Read lifetime signed left encoder tick counter.
 * @return Left encoder ticks.
 */
int32_t motor_task_get_enc_left (void)
{
    return g_enc_left_ticks;
}

/**
 * @brief Read lifetime signed right encoder tick counter.
 * @return Right encoder ticks.
 */
int32_t motor_task_get_enc_right (void)
{
    return g_enc_right_ticks;
}

/**
 * @brief Read count of overwritten motor command messages.
 * @return Message buffer drop counter.
 */
uint32_t motor_task_get_drop_count (void)
{
    UINT     saved_state = robot_tasks_data_lock();
    uint32_t drop_cnt    = g_buf_drop_cnt;

    robot_tasks_data_unlock(saved_state);

    return drop_cnt;
}

/**
 * @brief Compute left wheel odometry distance in millimetres.
 * @return Left wheel distance in mm.
 */
int32_t motor_task_get_left_dist_mm (void)
{
    return (g_enc_left_ticks * MOTOR_MM_PER_TICK_X100) / MOTOR_SCALE_HUNDRED;
}

/**
 * @brief Compute right wheel odometry distance in millimetres.
 * @return Right wheel distance in mm.
 */
int32_t motor_task_get_right_dist_mm (void)
{
    return (g_enc_right_ticks * MOTOR_MM_PER_TICK_X100) / MOTOR_SCALE_HUNDRED;
}

/**
 * @brief Compute average vehicle odometry distance in millimetres.
 * @return Average distance in mm.
 */
int32_t motor_task_get_avg_dist_mm (void)
{
    return (motor_task_get_left_dist_mm() +
            motor_task_get_right_dist_mm()) / MOTOR_PAIR_DIVISOR;
}

/**
 * @brief Compute rounded average vehicle odometry distance in centimetres.
 * @return Average distance in cm.
 */
int32_t motor_task_get_avg_dist_cm (void)
{
    int32_t dist_mm = motor_task_get_avg_dist_mm();
    int32_t dist_cm = (dist_mm >= 0)
                      ? ((dist_mm + MOTOR_HALF_CM_MM) / MOTOR_MM_PER_CM)
                      : ((dist_mm - MOTOR_HALF_CM_MM) / MOTOR_MM_PER_CM);

    return dist_cm;
}

/**
 * @brief Read latest estimated linear vehicle speed in mm per second.
 * @return Estimated speed in mm/second.
 */
int32_t motor_task_get_speed (void)
{
    return g_cur_speed_mm_per_second;
}

/**
 * @brief Queue a bounded straight forward translation command.
 * @param[in] dist_mm Target forward distance in millimetres.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_move_forward (int32_t dist_mm)
{
    int32_t ticks = (dist_mm > 0)
                    ? ((dist_mm * MOTOR_SCALE_HUNDRED) / MOTOR_MM_PER_TICK_X100)
                    : 0;

    /* Fit +/-25% nominal drive PWM constants into int8_t command parameters. */
    return motor_queue_bounded_cmd((int8_t)MOTOR_API_DRIVE_PCT,
                                   (int8_t)(-MOTOR_API_DRIVE_PCT),
                                   ticks);
}

/**
 * @brief Queue a bounded straight backward translation command.
 * @param[in] dist_mm Target backward distance in millimetres.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_move_backward (int32_t dist_mm)
{
    int32_t ticks = (dist_mm > 0)
                    ? ((dist_mm * MOTOR_SCALE_HUNDRED) / MOTOR_MM_PER_TICK_X100)
                    : 0;

    /* Fit +/-25% reverse drive PWM constants into int8_t command parameters. */
    return motor_queue_bounded_cmd((int8_t)(-MOTOR_API_DRIVE_PCT),
                                   (int8_t)MOTOR_API_DRIVE_PCT,
                                   ticks);
}

/**
 * @brief Queue a bounded in-place left pivot command.
 * @param[in] angle_deg Pivot angle in degrees.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_turn_left (int32_t angle_deg)
{
    int32_t arc_mmx100 =
        (MOTOR_PI_X100 * MOTOR_WHEELBASE_MM * motor_abs_i32(angle_deg)) /
        MOTOR_FULL_CIRCLE_DEG;
    int32_t ticks      = arc_mmx100 / MOTOR_MM_PER_TICK_X100;

    /* Fit -25% pivot PWM constants into int8_t command parameters. */
    return motor_queue_bounded_cmd((int8_t)(-MOTOR_API_DRIVE_PCT),
                                   (int8_t)(-MOTOR_API_DRIVE_PCT),
                                   ticks);
}

/**
 * @brief Queue a bounded in-place right pivot command.
 * @param[in] angle_deg Pivot angle in degrees.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_turn_right (int32_t angle_deg)
{
    int32_t arc_mmx100 =
        (MOTOR_PI_X100 * MOTOR_WHEELBASE_MM * motor_abs_i32(angle_deg)) /
        MOTOR_FULL_CIRCLE_DEG;
    int32_t ticks      = arc_mmx100 / MOTOR_MM_PER_TICK_X100;

    /* Fit +25% pivot PWM constants into int8_t command parameters. */
    return motor_queue_bounded_cmd((int8_t)MOTOR_API_DRIVE_PCT,
                                   (int8_t)MOTOR_API_DRIVE_PCT,
                                   ticks);
}

/**
 * @brief Queue an immediate zero-speed stop command.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_stop (void)
{
    motor_task_cmd_t cmd         = { 0u };
    UINT             saved_state = robot_tasks_data_lock();

    g_bounded_ticks_left  = 0;
    g_bounded_ticks_right = 0;
    robot_tasks_data_unlock(saved_state);

    /* Cast 4-byte motor command union size to RTOS INT message size. */
    return tk_snd_mbf(robot_tasks_get_motor_mbf(),
                      &cmd,
                      (INT)sizeof(cmd),
                      TMO_POL);
}

/**
 * @brief Main 50 Hz closed-loop motor control and fault monitoring task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void motor_task_run_task (INT stacd, void * p_exinf)
{
    motor_task_ctx_t ctx     = { 0 };
    int32_t          delta_l = 0;
    int32_t          delta_r = 0;

    /* Explicitly ignore unused RTOS task entry start code. */
    (void)stacd;
    /* Explicitly ignore unused RTOS extended info pointer. */
    (void)p_exinf;

    /* Store initial IDLE enum state in 3-bit unsigned runtime field. */
    ctx.runtime.state = (uint32_t)MOTOR_STATE_IDLE;
    /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
    tm_printf((UB *)"[motor] Task started\n");

    if (!motor_init_hardware())
    {
        tk_ext_tsk();
    }

    for (;;)
    {
        motor_task_cmd_t cmd           = { 0u };
        bool             b_has_new_cmd = false;

        /* Range-safe: out_left is clamped to [-100, 100], fitting int8_t. */
        g_isr_out_left  = (int8_t)ctx.out_left;
        /* Range-safe: out_right is clamped to [-100, 100], fitting int8_t. */
        g_isr_out_right = (int8_t)ctx.out_right;

        motor_step_inputs(&ctx, &cmd, &b_has_new_cmd, delta_l, delta_r);

        delta_l        = g_enc_left_ticks - ctx.prev_left;
        delta_r        = g_enc_right_ticks - ctx.prev_right;
        ctx.prev_left  = g_enc_left_ticks;
        ctx.prev_right = g_enc_right_ticks;

        motor_update_speed_log(&ctx, delta_l, delta_r);

        /* Convert 3-bit runtime state field to enum for FSM dispatch. */
        switch ((motor_task_state_t)ctx.runtime.state)
        {
            case MOTOR_STATE_IDLE:
                motor_step_idle(&ctx, &cmd, b_has_new_cmd, delta_l, delta_r);
            break;

            case MOTOR_STATE_ACCEL:
                motor_step_accel(&ctx, &cmd, b_has_new_cmd, delta_l, delta_r);
            break;

            case MOTOR_STATE_CRUISE:
                motor_step_cruise(&ctx, &cmd, b_has_new_cmd, delta_l, delta_r);
            break;

            case MOTOR_STATE_TURN:
                motor_step_turn(&ctx, &cmd, b_has_new_cmd, delta_l, delta_r);
            break;

            case MOTOR_STATE_BRAKE:
                motor_step_brake(&ctx, &cmd, b_has_new_cmd);
            break;

            case MOTOR_STATE_FAULT:
                if ((b_has_new_cmd) &&
                    (0 == cmd.fields.target_speed_left) &&
                    (0 == cmd.fields.target_speed_right))
                {
                    ctx.tgt_left  = 0;
                    ctx.tgt_right = 0;
                    robot_tasks_flags_update(0u, MOTOR_FAULT_BIT_STALL);
                    motor_set_state(&ctx.runtime, MOTOR_STATE_IDLE);
                }
            break;

            case MOTOR_STATE_ESTOP:
                motor_step_estop(&ctx, delta_l, delta_r);
            break;

            default:
            break;
        }

        /* Ignore sleep return code in 50 Hz periodic control loop. */
        (void)tk_dly_tsk(MOTOR_LOOP_DELAY_MSEC);
    }
}

/**
 * @brief Return the absolute value of a signed 32-bit integer.
 * @param[in] val Input signed integer.
 * @return Non-negative magnitude of val.
 */
static int32_t motor_abs_i32 (int32_t val)
{
    return (val < 0) ? (-val) : val;
}

/**
 * @brief Transition motor FSM state and log the transition.
 * @param[in,out] p_rt       Motor runtime state structure.
 * @param[in]     next_state Target motor state enum value.
 */
static void motor_set_state (motor_task_runtime_t * p_rt,
                             motor_task_state_t next_state)
{
    /* Store enum value in 3-bit unsigned runtime field. */
    p_rt->state = (uint32_t)next_state;
    /* Cast ASCII format literal and enum state to RTOS UB* and INT. */
    tm_printf((UB *)"[motors] state=%d\n", (INT)next_state);
}

/**
 * @brief Atomically consume extended >255-tick bounded goals or wire ticks.
 * @param[in]  p_cmd    Motor command packet holding 8-bit fallback tick goals.
 * @param[out] p_goal_l Resolved left wheel tick goal.
 * @param[out] p_goal_r Resolved right wheel tick goal.
 */
static void motor_take_bounded_goals (const motor_task_cmd_t * p_cmd,
                                      int32_t * p_goal_l,
                                      int32_t * p_goal_r)
{
    UINT    saved_state = robot_tasks_data_lock();
    int32_t ext_left    = g_bounded_ticks_left;
    int32_t ext_right   = g_bounded_ticks_right;

    g_bounded_ticks_left  = 0;
    g_bounded_ticks_right = 0;
    robot_tasks_data_unlock(saved_state);

    /* Widen unsigned 8-bit wire tick field when no >255 override is queued. */
    *p_goal_l = (ext_left > 0)
                ? ext_left
                : (int32_t)p_cmd->fields.turn_ticks_left;
    /* Widen unsigned 8-bit right wire tick field when no override is queued. */
    *p_goal_r = (ext_right > 0)
                ? ext_right
                : (int32_t)p_cmd->fields.turn_ticks_right;
}

/**
 * @brief Pack and queue a bounded motor command with >255-tick support.
 * @param[in] left_spd     Left wheel speed percentage.
 * @param[in] right_spd    Right wheel speed percentage.
 * @param[in] target_ticks Bounded encoder tick goal (>= 0).
 * @return E_OK on success, or negative uT-Kernel error code.
 */
static ER motor_queue_bounded_cmd (int8_t left_spd,
                                   int8_t right_spd,
                                   int32_t target_ticks)
{
    motor_task_cmd_t cmd         = { 0u };
    int32_t          wire_ticks  = motor_task_clamp(target_ticks,
                                                    0,
                                                    MOTOR_MAX_CMD_TICKS);
    /* Range-safe: wire_ticks is clamped into [0, 255]. */
    uint8_t          clamped_tk  = (uint8_t)wire_ticks;
    UINT             saved_state = robot_tasks_data_lock();

    g_bounded_ticks_left  = target_ticks;
    g_bounded_ticks_right = target_ticks;
    robot_tasks_data_unlock(saved_state);

    cmd.fields.target_speed_left  = left_spd;
    cmd.fields.target_speed_right = right_spd;
    cmd.fields.turn_ticks_left    = clamped_tk;
    cmd.fields.turn_ticks_right   = clamped_tk;

    /* Cast 4-byte command union size to RTOS INT message size. */
    return tk_snd_mbf(robot_tasks_get_motor_mbf(),
                      &cmd,
                      (INT)sizeof(cmd),
                      TMO_POL);
}

/**
 * @brief Step cur_val toward target_val by at most step_val.
 * @param[in] cur_val    Current value.
 * @param[in] target_val Desired target value.
 * @param[in] step_val   Maximum step magnitude per period.
 * @return Next ramped value.
 */
static int32_t motor_ramp (int32_t cur_val,
                           int32_t target_val,
                           int32_t step_val)
{
    int32_t next_val = cur_val;

    if (cur_val < target_val)
    {
        next_val = ((target_val - cur_val) < step_val)
                   ? target_val
                   : (cur_val + step_val);
    }
    else if (cur_val > target_val)
    {
        next_val = ((cur_val - target_val) < step_val)
                   ? target_val
                   : (cur_val - step_val);
    }
    else
    {
        next_val = cur_val;
    }

    return next_val;
}

/**
 * @brief Apply a signed PWM speed percentage (-100..100%) to one motor bridge.
 * @param[in] pin_a     Forward PWM GPIO pin.
 * @param[in] pin_b     Reverse PWM GPIO pin.
 * @param[in] speed_pct Signed speed percentage.
 */
static void motor_set_channel (uint32_t pin_a,
                               uint32_t pin_b,
                               int32_t speed_pct)
{
    int32_t  clamped = motor_task_clamp(speed_pct,
                                        MOTOR_PWM_MIN_NEG_PCT,
                                        MOTOR_PWM_MAX_PCT);
    /* Range-safe: motor_abs_i32(clamped) is in [0, 100]. */
    uint32_t abs_pct = (uint32_t)motor_abs_i32(clamped);
    uint32_t duty    = ((MOTOR_PWM_WRAP + 1u) * abs_pct) / MOTOR_PWM_SCALE_U32;

    if (clamped > 0)
    {
        /* Cast GPIO pin and duty count to BSP PWM UINT and UW types. */
        (void)pwm_set_cc((UINT)pin_a, (UW)duty);
        /* Zero reverse bridge channel PWM duty cycle. */
        (void)pwm_set_cc((UINT)pin_b, 0u);
    }
    else if (clamped < 0)
    {
        /* Zero forward bridge channel PWM duty cycle. */
        (void)pwm_set_cc((UINT)pin_a, 0u);
        /* Cast GPIO pin and duty count to BSP PWM UINT and UW types. */
        (void)pwm_set_cc((UINT)pin_b, (UW)duty);
    }
    else
    {
        /* Zero forward bridge channel when coasting/stopped. */
        (void)pwm_set_cc((UINT)pin_a, 0u);
        /* Zero reverse bridge channel when coasting/stopped. */
        (void)pwm_set_cc((UINT)pin_b, 0u);
    }
}

/**
 * @brief Select right-wheel tick count with automatic fallback to channel B.
 * @param[in] ticks_a Filtered right encoder channel A ticks.
 * @param[in] ticks_b Filtered right encoder channel B ticks.
 * @return Continuous right-wheel tick total.
 */
static int32_t motor_right_enc_ticks (int32_t ticks_a, int32_t ticks_b)
{
    int32_t ticks_out = ticks_a;

    if (0u == g_enc_isr_state.b_is_using_b)
    {
        if (ticks_a != g_right_enc_last_a)
        {
            g_right_enc_last_a   = ticks_a;
            g_right_enc_b_offset = ticks_a - ticks_b;
        }
        else if (motor_abs_i32((ticks_b + g_right_enc_b_offset) - ticks_a) >=
                 MOTOR_ENC_BACKUP_MIN_TICKS)
        {
            g_enc_isr_state.b_is_using_b = 1u;
        }
        else
        {
            /* Channel A still within tolerance; keep using A. */
        }
    }

    if (0u != g_enc_isr_state.b_is_using_b)
    {
        ticks_out = ticks_b + g_right_enc_b_offset;
    }

    return ticks_out;
}

/**
 * @brief Two-sample debounce rising-edge counter for one encoder channel.
 * @param[in]     raw_lvl   Current GPIO level (0 or 1).
 * @param[in]     dir_sign  Direction sign (+1 or -1).
 * @param[in,out] p_filters Packed 2-bit-per-channel debounce state byte.
 * @param[in]     bit_shift Bit offset of this channel (0, 2, 4, or 6).
 * @param[in,out] p_ticks   Volatile tick counter incremented on clean rise.
 */
static void motor_count_rising_edge (uint32_t raw_lvl,
                                     int32_t dir_sign,
                                     uint8_t * p_filters,
                                     uint32_t bit_shift,
                                     volatile int32_t * p_ticks)
{
    /* Widen 8-bit packed filter state to uint32_t for bit manipulation. */
    uint32_t filters_u32 = (uint32_t)(*p_filters);
    uint32_t stable      = (filters_u32 >> bit_shift) & 1u;
    uint32_t pending     = (filters_u32 >> (bit_shift + 1u)) & 1u;
    uint32_t clear_mask  = ~(MOTOR_ENC_PAIR_MASK << bit_shift);
    uint32_t new_bits    = 0u;

    if (raw_lvl == stable)
    {
        pending = 0u;
    }
    else if (0u != pending)
    {
        stable  = raw_lvl;
        pending = 0u;

        if (0u != stable)
        {
            *p_ticks += dir_sign;
        }
    }
    else
    {
        pending = 1u;
    }

    new_bits = (stable | (pending << 1u)) << bit_shift;

    /* Range-safe: bit_shift is in {0, 2, 4, 6}, result fits in 8 bits. */
    *p_filters = (uint8_t)((filters_u32 & clear_mask) | new_bits);
}

/**
 * @brief Compute synchronized straight-line wheel PWM with heading-lock PID.
 * @param[in]     base_mag_l    Nominal left wheel PWM magnitude.
 * @param[in]     base_mag_r    Nominal right wheel PWM magnitude.
 * @param[in]     start_ticks_l Left encoder baseline tick count.
 * @param[in]     start_ticks_r Right encoder baseline tick count.
 * @param[in]     delta_l       Left wheel ticks in the last 20 ms period.
 * @param[in]     delta_r       Right wheel ticks in the last 20 ms period.
 * @param[in,out] p_integ_units Learned integral balance accumulator.
 * @param[out]    p_out_mag_l   Adjusted left PWM magnitude.
 * @param[out]    p_out_mag_r   Adjusted right PWM magnitude.
 */
static void motor_apply_heading_lock (int32_t base_mag_l,
                                      int32_t base_mag_r,
                                      int32_t start_ticks_l,
                                      int32_t start_ticks_r,
                                      int32_t delta_l,
                                      int32_t delta_r,
                                      int32_t * p_integ_units,
                                      int32_t * p_out_mag_l,
                                      int32_t * p_out_mag_r)
{
    int32_t moved_l     = motor_abs_i32(g_enc_left_ticks - start_ticks_l);
    int32_t raw_moved_r = motor_abs_i32(g_enc_right_ticks - start_ticks_r);
    int32_t moved_r     =
        (raw_moved_r * (MOTOR_PERMILLE_SCALE + MOTOR_LEFT_DRIFT_PERMILLE)) /
        MOTOR_PERMILLE_SCALE;
    int32_t diff_ticks  = moved_l - moved_r;

    if ((motor_abs_i32(diff_ticks) > MOTOR_ENC_FAIL_TICK_DELTA) &&
        ((0 == moved_l) || (0 == moved_r)))
    {
        robot_tasks_flag_assign(ROBOT_ENCODER_DEGRADED, true);
        robot_tasks_flags_update(MOTOR_FAULT_BIT_ENCODER, 0u);
    }

    if (motor_task_is_enc_degraded())
    {
        *p_out_mag_l = (base_mag_l > MOTOR_ENC_CRAWL_PCT)
                       ? MOTOR_ENC_CRAWL_PCT
                       : base_mag_l;
        *p_out_mag_r = (base_mag_r > MOTOR_ENC_CRAWL_PCT)
                       ? MOTOR_ENC_CRAWL_PCT
                       : base_mag_r;
    }
    else
    {
        int32_t vel_diff = motor_abs_i32(delta_l) - motor_abs_i32(delta_r);
        int32_t p_term   = diff_ticks / MOTOR_TICKS_PER_PCT_KP;
        int32_t d_term   = vel_diff * MOTOR_KD_VEL_DIFF;
        int32_t i_term   = 0;
        int32_t trim     = 0;

        if (motor_abs_i32(diff_ticks) <= MOTOR_INTEG_ERR_BAND)
        {
            int32_t next_integ =
                (*p_integ_units) + (diff_ticks * MOTOR_KI_TRIM_NUM);

            *p_integ_units = motor_task_clamp(next_integ,
                                              -MOTOR_MAX_INTEG_UNITS,
                                              MOTOR_MAX_INTEG_UNITS);
        }

        i_term = (*p_integ_units) / MOTOR_KI_TRIM_DEN;
        trim   = motor_task_clamp(p_term + d_term + i_term,
                                  -MOTOR_MAX_TRIM_PCT,
                                  MOTOR_MAX_TRIM_PCT);

        *p_out_mag_l = motor_task_clamp(base_mag_l - trim,
                                        MOTOR_MIN_ACTIVE_PCT,
                                        MOTOR_PWM_MAX_PCT);
        *p_out_mag_r = motor_task_clamp(base_mag_r + trim,
                                        MOTOR_MIN_ACTIVE_PCT,
                                        MOTOR_PWM_MAX_PCT);
    }
}

/**
 * @brief Update per-wheel stall counters and report dual-wheel stall fault.
 * @param[in]     out_l   Active left PWM percentage.
 * @param[in]     out_r   Active right PWM percentage.
 * @param[in]     delta_l Left ticks observed in this 20 ms period.
 * @param[in]     delta_r Right ticks observed in this 20 ms period.
 * @param[in,out] p_rt    Motor runtime state holding stall counters.
 * @return True if both wheels have stalled for MOTOR_STALL_PERIODS.
 */
static bool motor_check_stalled (int32_t out_l,
                                 int32_t out_r,
                                 int32_t delta_l,
                                 int32_t delta_r,
                                 motor_task_runtime_t * p_rt)
{
    bool b_is_left_stalled  =
        ((motor_abs_i32(out_l) >= MOTOR_STALL_MIN_PCT) && (0 == delta_l));
    bool b_is_right_stalled =
        ((motor_abs_i32(out_r) >= MOTOR_STALL_MIN_PCT) && (0 == delta_r));

    if (b_is_left_stalled)
    {
        if (p_rt->stall_left < MOTOR_STALL_PERIODS)
        {
            p_rt->stall_left++;
        }
    }
    else
    {
        p_rt->stall_left = 0u;
    }

    if (b_is_right_stalled)
    {
        if (p_rt->stall_right < MOTOR_STALL_PERIODS)
        {
            p_rt->stall_right++;
        }
    }
    else
    {
        p_rt->stall_right = 0u;
    }

    return ((p_rt->stall_left >= MOTOR_STALL_PERIODS) &&
            (p_rt->stall_right >= MOTOR_STALL_PERIODS));
}

/**
 * @brief Configure motor PWM outputs, encoder GPIOs, and Timer Alarm 0 ISR.
 * @return True if initialization succeeded, false on hardware or check fault.
 */
static bool motor_init_hardware (void)
{
    static const uint32_t out_pins[MOTOR_PIN_COUNT] =
    {
        MOTOR_LEFT_A,
        MOTOR_LEFT_B,
        MOTOR_RIGHT_A,
        MOTOR_RIGHT_B
    };
    static const uint32_t enc_pins[MOTOR_PIN_COUNT] =
    {
        MOTOR_LEFT_ENC_A,
        MOTOR_LEFT_ENC_B,
        MOTOR_RIGHT_ENC_A,
        MOTOR_RIGHT_ENC_B
    };
    bool b_is_ok = true;

    for (uint32_t idx = 0u; (idx < MOTOR_PIN_COUNT) && (b_is_ok); idx++)
    {
        /* Cast GPIO pin constant to uT-Kernel BSP UINT pin parameter. */
        UINT pin_num = (UINT)out_pins[idx];

        /* Configure 20 kHz PWM wrap (cast to BSP UW) and zero initial duty. */
        if ((pwm_set_divider(pin_num, 1u, 0u) < E_OK) ||
            (pwm_set_wrap(pin_num, (UW)MOTOR_PWM_WRAP) < E_OK) ||
            (pwm_set_pin(pin_num) < E_OK) ||
            (pwm_set_cc(pin_num, 0u) < E_OK) ||
            (pwm_set_enabled(pin_num, TRUE) < E_OK))
        {
            /* Cast ASCII format literal to uT-Kernel UB pointer. */
            tm_printf((UB *)"[motors] PWM setup failed on GP%u\n", pin_num);
            b_is_ok = false;
        }
    }

    for (uint32_t idx = 0u; (idx < MOTOR_PIN_COUNT) && (b_is_ok); idx++)
    {
        /* Cast encoder GPIO pin constant to uT-Kernel BSP UINT parameter. */
        UINT pin_num = (UINT)enc_pins[idx];

        if (gpio_set_pin(pin_num, GPIO_MODE_IN) < E_OK)
        {
            /* Cast ASCII format literal to uT-Kernel UB pointer. */
            tm_printf((UB *)"[motors] encoder GPIO setup failed on GP%u\n",
                      pin_num);
            b_is_ok = false;
        }
        else
        {
            set_w(GPIO(pin_num), GPIO_IE | GPIO_PUE | GPIO_SHEMITT);
        }
    }

#ifndef NDEBUG
    if ((b_is_ok) && (!motor_right_enc_self_check()))
    {
        robot_tasks_flags_update(MOTOR_FAULT_BIT_ENCODER, 0u);
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[motors] Encoder self-check failed\n");
        b_is_ok = false;
    }
#endif

    if (b_is_ok)
    {
        T_DINT dint_cfg = { 0 };

        dint_cfg.intatr = TA_HLNG;
        /* Cast required by uT-Kernel T_DINT.inthdr function pointer typedef. */
        dint_cfg.inthdr = (FP)motor_encoder_alarm_isr;

        if (E_OK == tk_def_int(MOTOR_ENC_ALARM_IRQ, &dint_cfg))
        {
            out_w(TIMER_INTR, 1u);
            set_w(TIMER_INTE, 1u);
            out_w(TIMER_ALARM0, in_w(TIMER_TIMERAWL) + MOTOR_ENC_ALARM_USEC);
            ClearInt(MOTOR_ENC_ALARM_IRQ);
            EnableInt(MOTOR_ENC_ALARM_IRQ, MOTOR_ENC_ALARM_PRI);
        }
    }

    return b_is_ok;
}

/**
 * @brief Transition from ACCEL or CRUISE into a bounded TURN/translation goal.
 * @param[in,out] p_ctx  Motor task context.
 * @param[in]     new_l  Clamped left wheel PWM target.
 * @param[in]     new_r  Clamped right wheel PWM target.
 * @param[in]     goal_l Left wheel encoder tick goal.
 * @param[in]     goal_r Right wheel encoder tick goal.
 */
static void motor_enter_turn_from_drive (motor_task_ctx_t * p_ctx,
                                         int32_t new_l,
                                         int32_t new_r,
                                         int32_t goal_l,
                                         int32_t goal_r)
{
    p_ctx->tgt_left         = new_l;
    p_ctx->tgt_right        = new_r;
    p_ctx->turn_goal_left   = goal_l;
    p_ctx->turn_goal_right  = goal_r;
    p_ctx->turn_start_left  = g_enc_left_ticks;
    p_ctx->turn_start_right = g_enc_right_ticks;
    p_ctx->out_left         = p_ctx->tgt_left;
    p_ctx->out_right        = p_ctx->tgt_right;
    motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, p_ctx->out_left);
    motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, p_ctx->out_right);
    motor_set_state(&p_ctx->runtime, MOTOR_STATE_TURN);
}

/**
 * @brief Start either a bounded turn/translation or an accelerated cruise.
 * @param[in,out] p_ctx                 Motor task context.
 * @param[in]     p_cmd                 Incoming non-zero motor command.
 * @param[in]     b_should_clr_degraded True when starting from IDLE.
 */
static void motor_start_new_motion (motor_task_ctx_t * p_ctx,
                                    const motor_task_cmd_t * p_cmd,
                                    bool b_should_clr_degraded)
{
    /* Widen signed 8-bit left target speed to int32_t for range clamping. */
    p_ctx->tgt_left  = motor_task_clamp(
        (int32_t)p_cmd->fields.target_speed_left,
        MOTOR_PWM_MIN_NEG_PCT,
        MOTOR_PWM_MAX_PCT);
    /* Widen signed 8-bit right target speed to int32_t for range clamping. */
    p_ctx->tgt_right = motor_task_clamp(
        (int32_t)p_cmd->fields.target_speed_right,
        MOTOR_PWM_MIN_NEG_PCT,
        MOTOR_PWM_MAX_PCT);

    motor_take_bounded_goals(p_cmd,
                             &p_ctx->turn_goal_left,
                             &p_ctx->turn_goal_right);

    p_ctx->turn_start_left     = g_enc_left_ticks;
    p_ctx->turn_start_right    = g_enc_right_ticks;
    p_ctx->cruise_start_left   = g_enc_left_ticks;
    p_ctx->cruise_start_right  = g_enc_right_ticks;
    p_ctx->runtime.stall_left  = 0u;
    p_ctx->runtime.stall_right = 0u;

    if (b_should_clr_degraded)
    {
        robot_tasks_flag_assign(ROBOT_ENCODER_DEGRADED, false);
    }
    else
    {
        p_ctx->runtime.b_has_move_done_pend = 0u;
    }

    if ((0 != p_ctx->turn_goal_left) || (0 != p_ctx->turn_goal_right))
    {
        p_ctx->out_left  = p_ctx->tgt_left;
        p_ctx->out_right = p_ctx->tgt_right;
        motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, p_ctx->out_left);
        motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, p_ctx->out_right);
        motor_set_state(&p_ctx->runtime, MOTOR_STATE_TURN);
    }
    else
    {
        int32_t abs_l = motor_abs_i32(p_ctx->tgt_left);
        int32_t abs_r = motor_abs_i32(p_ctx->tgt_right);

        p_ctx->ramp_mag_left  = (abs_l > MOTOR_MIN_START_PCT)
                                ? MOTOR_MIN_START_PCT
                                : abs_l;
        p_ctx->ramp_mag_right = (abs_r > MOTOR_MIN_START_PCT)
                                ? MOTOR_MIN_START_PCT
                                : abs_r;
        motor_set_state(&p_ctx->runtime, MOTOR_STATE_ACCEL);
    }
}

#if (0u == MOTOR_STAGE2_PAD_TEST)
/**
 * @brief Synchronize motor FSM with shared ROBOT_ESTOP_REQUEST flag.
 * @param[in,out] p_ctx Motor task context.
 */
static void motor_sync_estop_state (motor_task_ctx_t * p_ctx)
{
    if (motor_task_is_estop())
    {
        /* Compare 3-bit runtime state against MOTOR_STATE_ESTOP enum value. */
        if ((uint32_t)MOTOR_STATE_ESTOP != p_ctx->runtime.state)
        {
            UINT saved_state = robot_tasks_data_lock();

            g_bounded_ticks_left  = 0;
            g_bounded_ticks_right = 0;
            robot_tasks_data_unlock(saved_state);

            p_ctx->tgt_left                     = 0;
            p_ctx->tgt_right                    = 0;
            p_ctx->out_left                     = 0;
            p_ctx->out_right                    = 0;
            p_ctx->turn_goal_left               = 0;
            p_ctx->turn_goal_right              = 0;
            p_ctx->runtime.b_has_move_done_pend = 0u;
            p_ctx->runtime.move_stable_periods  = 0u;
            p_ctx->runtime.stop_stable_periods  = 0u;
            motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
            motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
            robot_tasks_flags_update(MOTOR_FAULT_BIT_ESTOP, 0u);
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_ESTOP);
        }
    }
    /* Release E-stop state once external ROBOT_ESTOP_REQUEST is cleared. */
    else if ((uint32_t)MOTOR_STATE_ESTOP == p_ctx->runtime.state)
    {
        robot_tasks_flags_update(0u, MOTOR_FAULT_BIT_ESTOP);
        p_ctx->runtime.stop_stable_periods = 0u;
        motor_set_state(&p_ctx->runtime, MOTOR_STATE_IDLE);
    }
    else
    {
        /* Normal running mode; keep current FSM state. */
    }
}
#endif /* 0u == MOTOR_STAGE2_PAD_TEST */

/**
 * @brief Handle E-stop transitions and drain the motor command message buffer.
 * @param[in,out] p_ctx          Motor task context.
 * @param[out]    p_cmd          Most recent queued motor command.
 * @param[out]    pb_has_new_cmd True if at least one command was received.
 * @param[in]     prev_dl        Previous period left encoder delta.
 * @param[in]     prev_dr        Previous period right encoder delta.
 */
static void motor_step_inputs (motor_task_ctx_t * p_ctx,
                               motor_task_cmd_t * p_cmd,
                               bool * pb_has_new_cmd,
                               int32_t prev_dl,
                               int32_t prev_dr)
{
    ID h_motor_mbf = robot_tasks_get_motor_mbf();

#if (0u != MOTOR_STAGE2_PAD_TEST)
    /* Drain queued commands while comparing against 4-byte INT message size. */
    while ((INT)sizeof(*p_cmd) == tk_rcv_mbf(h_motor_mbf, p_cmd, TMO_POL))
    {
        /* Discard navigator commands during standalone distance test. */
    }

    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, false);
    p_ctx->test_cycles++;

    if (MOTOR_TEST_START_CYCLES == p_ctx->test_cycles)
    {
        UINT intsts = 0u;

        DI(intsts);
        g_enc_left_ticks                     = 0;
        g_enc_right_ticks                    = 0;
        g_isr_ticks_la                       = 0;
        g_isr_ticks_lb                       = 0;
        g_isr_ticks_ra                       = 0;
        g_isr_ticks_rb                       = 0;
        g_right_enc_last_a                   = 0;
        g_right_enc_b_offset                 = 0;
        g_enc_isr_state.b_is_using_b         = 0u;
        p_ctx->runtime.b_has_reported_backup = 0u;
        p_ctx->prev_left                     = 0;
        p_ctx->prev_right                    = 0;
        EI(intsts);

        /* Fit +25% test speed constant into int8_t left speed field. */
        p_cmd->fields.target_speed_left  = (int8_t)MOTOR_API_DRIVE_PCT;
        /* Fit -25% test speed constant into int8_t right speed field. */
        p_cmd->fields.target_speed_right = (int8_t)(-MOTOR_API_DRIVE_PCT);
        p_cmd->fields.turn_ticks_left    = 0u;
        p_cmd->fields.turn_ticks_right   = 0u;
        *pb_has_new_cmd                  = true;
        /* Cast format literal and target distance to RTOS UB* and INT. */
        tm_printf((UB *)"\n=== [DISTANCE TEST] Driving to %d mm ===\n",
                  (INT)MOTOR_TEST_TARGET_DIST_MM);
    }

    if ((p_ctx->test_cycles > MOTOR_TEST_START_CYCLES) &&
        (0 == p_ctx->run_number) &&
        (motor_task_get_avg_dist_mm() >= MOTOR_TEST_TARGET_DIST_MM))
    {
        p_ctx->run_number = 1;
        p_cmd->raw        = 0u;
        *pb_has_new_cmd   = true;
    }

    /* Check whether test run stopped cleanly in MOTOR_STATE_IDLE. */
    if ((1 == p_ctx->run_number) &&
        ((uint32_t)MOTOR_STATE_IDLE == p_ctx->runtime.state) &&
        (0 == prev_dl) &&
        (0 == prev_dr))
    {
        p_ctx->run_number = 2;
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"\n=== [DISTANCE TEST COMPLETE] ===\n");
        /* Cast format literal and odometry distances to RTOS UB* and INT. */
        tm_printf((UB *)"Reported: %d cm (L=%d mm, R=%d mm, Avg=%d mm)\n",
                  (INT)motor_task_get_avg_dist_cm(),
                  (INT)motor_task_get_left_dist_mm(),
                  (INT)motor_task_get_right_dist_mm(),
                  (INT)motor_task_get_avg_dist_mm());
        /* Cast format literal and final tick totals to RTOS UB* and INT. */
        tm_printf((UB *)"Final Ticks: L=%d, R=%d (Avg=%d ticks)\n",
                  (INT)g_enc_left_ticks,
                  (INT)g_enc_right_ticks,
                  (INT)((g_enc_left_ticks + g_enc_right_ticks) /
                        MOTOR_PAIR_DIVISOR));
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"Measure physical distance with a ruler now!\n");
    }
#else
    /* Explicitly ignore unused previous left delta in normal Navigator mode. */
    (void)prev_dl;
    /* Explicitly ignore unused previous right delta in normal mode. */
    (void)prev_dr;

    motor_sync_estop_state(p_ctx);

    /* Drain motor queue keeping latest 4-byte command (INT size comparison). */
    while ((INT)sizeof(*p_cmd) == tk_rcv_mbf(h_motor_mbf, p_cmd, TMO_POL))
    {
        if (*pb_has_new_cmd)
        {
            UINT saved_state = robot_tasks_data_lock();

            g_buf_drop_cnt++;
            robot_tasks_data_unlock(saved_state);
        }

        *pb_has_new_cmd = true;
    }
#endif /* MOTOR_STAGE2_PAD_TEST */
}

/**
 * @brief Update live speed estimate, encoder backup notice, and periodic log.
 * @param[in,out] p_ctx   Motor task context.
 * @param[in]     delta_l Left encoder delta ticks over the last 20 ms period.
 * @param[in]     delta_r Right encoder delta ticks over the last 20 ms period.
 */
static void motor_update_speed_log (motor_task_ctx_t * p_ctx,
                                    int32_t delta_l,
                                    int32_t delta_r)
{
    int32_t avg_period_ticks =
        (motor_abs_i32(delta_l) + motor_abs_i32(delta_r)) / MOTOR_PAIR_DIVISOR;

    if ((0u != g_enc_isr_state.b_is_using_b) &&
        (0u == p_ctx->runtime.b_has_reported_backup))
    {
        p_ctx->runtime.b_has_reported_backup = 1u;
        /* Cast format literal and ISR tick counters to RTOS UB* and INT. */
        tm_printf((UB *)"[motors] Right encoder A missing; "
                        "using B (A=%d B=%d)\n",
                  (INT)g_isr_ticks_ra,
                  (INT)g_isr_ticks_rb);
    }

    g_cur_speed_mm_per_second =
        (avg_period_ticks * MOTOR_LOOP_FREQ_HZ * MOTOR_MM_PER_TICK_X100) /
        MOTOR_SCALE_HUNDRED;

    p_ctx->runtime.print_timer++;

    if (p_ctx->runtime.print_timer >= MOTOR_PRINT_PERIOD_TICKS)
    {
        p_ctx->runtime.print_timer = 0u;

        if ((0 != delta_l) ||
            (0 != delta_r) ||
            (0 != p_ctx->out_left) ||
            (0 != p_ctx->out_right))
        {
            int32_t run_l =
                motor_abs_i32(g_enc_left_ticks - p_ctx->cruise_start_left);
            int32_t run_r =
                motor_abs_i32(g_enc_right_ticks - p_ctx->cruise_start_right);

            /* Cast format literal and encoder/PWM values to UB* and INT. */
            tm_printf((UB *)"[dist] %d cm | L=%d(A:%d,B:%d) "
                            "R=%d(A:%d,B:%d) err=%d | PWM L=%d R=%d\n",
                      (INT)motor_task_get_avg_dist_cm(),
                      (INT)run_l,
                      (INT)g_isr_ticks_la,
                      (INT)g_isr_ticks_lb,
                      (INT)run_r,
                      (INT)g_isr_ticks_ra,
                      (INT)g_isr_ticks_rb,
                      (INT)(run_l - run_r),
                      (INT)p_ctx->out_left,
                      (INT)p_ctx->out_right);
        }
    }
}

/**
 * @brief Update straight/curve cruise targets and reset baseline on change.
 * @param[in,out] p_ctx Motor task context.
 * @param[in]     new_l Clamped left wheel speed target.
 * @param[in]     new_r Clamped right wheel speed target.
 */
static void motor_update_cruise_targets (motor_task_ctx_t * p_ctx,
                                         int32_t new_l,
                                         int32_t new_r)
{
    if ((new_l != p_ctx->tgt_left) || (new_r != p_ctx->tgt_right))
    {
        p_ctx->cruise_start_left  = g_enc_left_ticks;
        p_ctx->cruise_start_right = g_enc_right_ticks;
    }

    p_ctx->tgt_left  = new_l;
    p_ctx->tgt_right = new_r;
}

/**
 * @brief Refresh bounded turn targets when a new non-zero command differs.
 * @param[in,out] p_ctx Motor task context.
 * @param[in]     p_cmd Incoming non-zero motor command.
 */
static void motor_update_turn_targets (motor_task_ctx_t * p_ctx,
                                       const motor_task_cmd_t * p_cmd)
{
    /* Widen signed 8-bit left command speed to int32_t. */
    int32_t cmd_l       = (int32_t)p_cmd->fields.target_speed_left;
    /* Widen signed 8-bit right command speed to int32_t. */
    int32_t cmd_r       = (int32_t)p_cmd->fields.target_speed_right;
    int32_t next_goal_l = 0;
    int32_t next_goal_r = 0;

    motor_take_bounded_goals(p_cmd, &next_goal_l, &next_goal_r);

    if ((cmd_l != p_ctx->tgt_left) ||
        (cmd_r != p_ctx->tgt_right) ||
        (next_goal_l != p_ctx->turn_goal_left) ||
        (next_goal_r != p_ctx->turn_goal_right))
    {
        p_ctx->tgt_left         = motor_task_clamp(cmd_l,
                                                   MOTOR_PWM_MIN_NEG_PCT,
                                                   MOTOR_PWM_MAX_PCT);
        p_ctx->tgt_right        = motor_task_clamp(cmd_r,
                                                   MOTOR_PWM_MIN_NEG_PCT,
                                                   MOTOR_PWM_MAX_PCT);
        p_ctx->turn_goal_left   = next_goal_l;
        p_ctx->turn_goal_right  = next_goal_r;
        p_ctx->turn_start_left  = g_enc_left_ticks;
        p_ctx->turn_start_right = g_enc_right_ticks;
    }
}

/**
 * @brief Drive motors during an incomplete bounded turn or translation.
 * @param[in,out] p_ctx           Motor task context.
 * @param[in]     b_is_left_done  True if left wheel reached its tick goal.
 * @param[in]     b_is_right_done True if right wheel reached its tick goal.
 * @param[in]     delta_l         Left encoder delta ticks.
 * @param[in]     delta_r         Right encoder delta ticks.
 */
static void motor_drive_turn_active (motor_task_ctx_t * p_ctx,
                                     bool b_is_left_done,
                                     bool b_is_right_done,
                                     int32_t delta_l,
                                     int32_t delta_r)
{
    int32_t mag_l = motor_abs_i32(p_ctx->tgt_left);
    int32_t mag_r = motor_abs_i32(p_ctx->tgt_right);

    if ((p_ctx->tgt_left == (-p_ctx->tgt_right)) &&
        (0 != p_ctx->tgt_left) &&
        (!b_is_left_done) &&
        (!b_is_right_done))
    {
        motor_apply_heading_lock(mag_l,
                                 mag_r,
                                 p_ctx->turn_start_left,
                                 p_ctx->turn_start_right,
                                 delta_l,
                                 delta_r,
                                 &p_ctx->integ_units,
                                 &mag_l,
                                 &mag_r);
    }

    p_ctx->out_left  = b_is_left_done
                       ? 0
                       : ((p_ctx->tgt_left > 0) ? mag_l : (-mag_l));
    p_ctx->out_right = b_is_right_done
                       ? 0
                       : ((p_ctx->tgt_right > 0) ? mag_r : (-mag_r));

    if ((!motor_task_is_enc_degraded()) &&
        (motor_check_stalled(p_ctx->out_left,
                             p_ctx->out_right,
                             delta_l,
                             delta_r,
                             &p_ctx->runtime)))
    {
        p_ctx->out_left                     = 0;
        p_ctx->out_right                    = 0;
        motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
        motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
        p_ctx->runtime.b_has_move_done_pend = 0u;
        robot_tasks_flags_update(MOTOR_FAULT_BIT_STALL, 0u);
        motor_set_state(&p_ctx->runtime, MOTOR_STATE_FAULT);
    }
    else
    {
        motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, p_ctx->out_left);
        motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, p_ctx->out_right);
    }
}

/**
 * @brief Handle MOTOR_STATE_IDLE settle acknowledgement and motion launch.
 * @param[in,out] p_ctx         Motor task context.
 * @param[in]     p_cmd         Latest motor command.
 * @param[in]     b_has_new_cmd True if a new command arrived this tick.
 * @param[in]     delta_l       Left encoder delta ticks.
 * @param[in]     delta_r       Right encoder delta ticks.
 */
static void motor_step_idle (motor_task_ctx_t * p_ctx,
                             const motor_task_cmd_t * p_cmd,
                             bool b_has_new_cmd,
                             int32_t delta_l,
                             int32_t delta_r)
{
    if (0u != p_ctx->runtime.b_has_move_done_pend)
    {
        bool b_are_wheels_still = ((0 == delta_l) && (0 == delta_r));

        if ((b_are_wheels_still) &&
            (p_ctx->runtime.move_stable_periods < MOTOR_STABLE_PERIODS_REQ))
        {
            p_ctx->runtime.move_stable_periods++;
        }
        else if (!b_are_wheels_still)
        {
            p_ctx->runtime.move_stable_periods = 0u;
        }
        else
        {
            /* Already reached required settled periods. */
        }
    }

    if ((0u != p_ctx->runtime.b_has_move_done_pend) &&
        (p_ctx->runtime.move_stable_periods >= MOTOR_STABLE_PERIODS_REQ))
    {
        navigator_task_msg_t done_msg = { 0 };

        /* Store MSG_MOTOR_MOVE_DONE enum in 4-bit unsigned message field. */
        done_msg.type = (uint32_t)MSG_MOTOR_MOVE_DONE;

        /* Cast Navigator message size to RTOS INT parameter. */
        if (tk_snd_mbf(robot_tasks_get_nav_mbf(),
                       &done_msg,
                       (INT)sizeof(done_msg),
                       TMO_POL) >= E_OK)
        {
            p_ctx->runtime.b_has_move_done_pend = 0u;
            p_ctx->runtime.move_stable_periods  = 0u;
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[motors] Move completed\n");
        }
    }

    if ((b_has_new_cmd) &&
        ((0 != p_cmd->fields.target_speed_left) ||
         (0 != p_cmd->fields.target_speed_right)))
    {
        motor_start_new_motion(p_ctx, p_cmd, true);
    }
}

/**
 * @brief Handle MOTOR_STATE_ACCEL soft-start ramping and heading lock.
 * @param[in,out] p_ctx         Motor task context.
 * @param[in]     p_cmd         Latest motor command.
 * @param[in]     b_has_new_cmd True if a new command arrived this tick.
 * @param[in]     delta_l       Left encoder delta ticks.
 * @param[in]     delta_r       Right encoder delta ticks.
 */
static void motor_step_accel (motor_task_ctx_t * p_ctx,
                              const motor_task_cmd_t * p_cmd,
                              bool b_has_new_cmd,
                              int32_t delta_l,
                              int32_t delta_r)
{
    bool b_should_continue = true;

    if (b_has_new_cmd)
    {
        /* Widen signed 8-bit left command speed to int32_t for clamping. */
        int32_t new_l  = motor_task_clamp(
            (int32_t)p_cmd->fields.target_speed_left,
            MOTOR_PWM_MIN_NEG_PCT,
            MOTOR_PWM_MAX_PCT);
        /* Widen signed 8-bit right command speed to int32_t for clamping. */
        int32_t new_r  = motor_task_clamp(
            (int32_t)p_cmd->fields.target_speed_right,
            MOTOR_PWM_MIN_NEG_PCT,
            MOTOR_PWM_MAX_PCT);
        int32_t goal_l = 0;
        int32_t goal_r = 0;

        motor_take_bounded_goals(p_cmd, &goal_l, &goal_r);

        if ((0 == new_l) && (0 == new_r))
        {
            p_ctx->turn_goal_left  = 0;
            p_ctx->turn_goal_right = 0;
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_BRAKE);
            b_should_continue      = false;
        }
        else if ((0 != goal_l) || (0 != goal_r))
        {
            motor_enter_turn_from_drive(p_ctx, new_l, new_r, goal_l, goal_r);
            b_should_continue = false;
        }
        else
        {
            p_ctx->turn_goal_left  = 0;
            p_ctx->turn_goal_right = 0;
            motor_update_cruise_targets(p_ctx, new_l, new_r);
        }
    }

    if (b_should_continue)
    {
        int32_t mag_l = 0;
        int32_t mag_r = 0;

        p_ctx->ramp_mag_left  = motor_ramp(p_ctx->ramp_mag_left,
                                           motor_abs_i32(p_ctx->tgt_left),
                                           MOTOR_RAMP_STEP_PCT);
        p_ctx->ramp_mag_right = motor_ramp(p_ctx->ramp_mag_right,
                                           motor_abs_i32(p_ctx->tgt_right),
                                           MOTOR_RAMP_STEP_PCT);
        mag_l                 = p_ctx->ramp_mag_left;
        mag_r                 = p_ctx->ramp_mag_right;

        if ((p_ctx->tgt_left == (-p_ctx->tgt_right)) && (0 != p_ctx->tgt_left))
        {
            motor_apply_heading_lock(p_ctx->ramp_mag_left,
                                     p_ctx->ramp_mag_right,
                                     p_ctx->cruise_start_left,
                                     p_ctx->cruise_start_right,
                                     delta_l,
                                     delta_r,
                                     &p_ctx->integ_units,
                                     &mag_l,
                                     &mag_r);
        }

        p_ctx->out_left  = (p_ctx->tgt_left > 0)
                           ? mag_l
                           : ((p_ctx->tgt_left < 0) ? (-mag_l) : 0);
        p_ctx->out_right = (p_ctx->tgt_right > 0)
                           ? mag_r
                           : ((p_ctx->tgt_right < 0) ? (-mag_r) : 0);

        motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, p_ctx->out_left);
        motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, p_ctx->out_right);

        if ((p_ctx->ramp_mag_left == motor_abs_i32(p_ctx->tgt_left)) &&
            (p_ctx->ramp_mag_right == motor_abs_i32(p_ctx->tgt_right)))
        {
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_CRUISE);
        }
    }
}

/**
 * @brief Handle MOTOR_STATE_CRUISE steady driving, heading lock, and stall.
 * @param[in,out] p_ctx         Motor task context.
 * @param[in]     p_cmd         Latest motor command.
 * @param[in]     b_has_new_cmd True if a new command arrived this tick.
 * @param[in]     delta_l       Left encoder delta ticks.
 * @param[in]     delta_r       Right encoder delta ticks.
 */
static void motor_step_cruise (motor_task_ctx_t * p_ctx,
                               const motor_task_cmd_t * p_cmd,
                               bool b_has_new_cmd,
                               int32_t delta_l,
                               int32_t delta_r)
{
    bool b_should_continue = true;

    if (b_has_new_cmd)
    {
        /* Widen signed 8-bit left command speed to int32_t for clamping. */
        int32_t new_l  = motor_task_clamp(
            (int32_t)p_cmd->fields.target_speed_left,
            MOTOR_PWM_MIN_NEG_PCT,
            MOTOR_PWM_MAX_PCT);
        /* Widen signed 8-bit right command speed to int32_t for clamping. */
        int32_t new_r  = motor_task_clamp(
            (int32_t)p_cmd->fields.target_speed_right,
            MOTOR_PWM_MIN_NEG_PCT,
            MOTOR_PWM_MAX_PCT);
        int32_t goal_l = 0;
        int32_t goal_r = 0;

        motor_take_bounded_goals(p_cmd, &goal_l, &goal_r);

        if ((0 == new_l) && (0 == new_r))
        {
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_BRAKE);
            b_should_continue = false;
        }
        else if ((0 != goal_l) || (0 != goal_r))
        {
            motor_enter_turn_from_drive(p_ctx, new_l, new_r, goal_l, goal_r);
            b_should_continue = false;
        }
        else
        {
            motor_update_cruise_targets(p_ctx, new_l, new_r);
        }
    }

    if (b_should_continue)
    {
        int32_t mag_l = motor_abs_i32(p_ctx->tgt_left);
        int32_t mag_r = motor_abs_i32(p_ctx->tgt_right);

        if ((p_ctx->tgt_left == (-p_ctx->tgt_right)) && (0 != p_ctx->tgt_left))
        {
            motor_apply_heading_lock(mag_l,
                                     mag_r,
                                     p_ctx->cruise_start_left,
                                     p_ctx->cruise_start_right,
                                     delta_l,
                                     delta_r,
                                     &p_ctx->integ_units,
                                     &mag_l,
                                     &mag_r);
        }

        p_ctx->out_left  = (p_ctx->tgt_left > 0)
                           ? mag_l
                           : ((p_ctx->tgt_left < 0) ? (-mag_l) : 0);
        p_ctx->out_right = (p_ctx->tgt_right > 0)
                           ? mag_r
                           : ((p_ctx->tgt_right < 0) ? (-mag_r) : 0);

        if ((!motor_task_is_enc_degraded()) &&
            (motor_check_stalled(p_ctx->out_left,
                                 p_ctx->out_right,
                                 delta_l,
                                 delta_r,
                                 &p_ctx->runtime)))
        {
            p_ctx->out_left  = 0;
            p_ctx->out_right = 0;
            motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
            motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
            robot_tasks_flags_update(MOTOR_FAULT_BIT_STALL, 0u);
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_FAULT);
        }
        else
        {
            motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, p_ctx->out_left);
            motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, p_ctx->out_right);
        }
    }
}

/**
 * @brief Handle MOTOR_STATE_TURN bounded translations and in-place pivots.
 * @param[in,out] p_ctx         Motor task context.
 * @param[in]     p_cmd         Latest motor command.
 * @param[in]     b_has_new_cmd True if a new command arrived this tick.
 * @param[in]     delta_l       Left encoder delta ticks.
 * @param[in]     delta_r       Right encoder delta ticks.
 */
static void motor_step_turn (motor_task_ctx_t * p_ctx,
                             const motor_task_cmd_t * p_cmd,
                             bool b_has_new_cmd,
                             int32_t delta_l,
                             int32_t delta_r)
{
    bool b_should_continue = true;

    if (b_has_new_cmd)
    {
        if ((0 == p_cmd->fields.target_speed_left) &&
            (0 == p_cmd->fields.target_speed_right))
        {
            UINT saved_state = robot_tasks_data_lock();

            g_bounded_ticks_left  = 0;
            g_bounded_ticks_right = 0;
            robot_tasks_data_unlock(saved_state);

            p_ctx->tgt_left                     = 0;
            p_ctx->tgt_right                    = 0;
            p_ctx->turn_goal_left               = 0;
            p_ctx->turn_goal_right              = 0;
            p_ctx->runtime.b_has_move_done_pend = 0u;
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_BRAKE);
            b_should_continue                   = false;
        }
        else
        {
            motor_update_turn_targets(p_ctx, p_cmd);
        }
    }

    if (b_should_continue)
    {
        bool b_is_left_done  =
            ((0 == p_ctx->turn_goal_left) ||
             (motor_abs_i32(g_enc_left_ticks - p_ctx->turn_start_left) >=
              p_ctx->turn_goal_left));
        bool b_is_right_done =
            ((0 == p_ctx->turn_goal_right) ||
             (motor_abs_i32(g_enc_right_ticks - p_ctx->turn_start_right) >=
              p_ctx->turn_goal_right));

        if ((b_is_left_done) && (b_is_right_done))
        {
            p_ctx->out_left                     = 0;
            p_ctx->out_right                    = 0;
            motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, 0);
            motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, 0);
            p_ctx->turn_goal_left               = 0;
            p_ctx->turn_goal_right              = 0;
            p_ctx->runtime.b_has_move_done_pend = 1u;
            p_ctx->runtime.move_stable_periods  = 0u;
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_BRAKE);
        }
        else
        {
            motor_drive_turn_active(p_ctx,
                                    b_is_left_done,
                                    b_is_right_done,
                                    delta_l,
                                    delta_r);
        }
    }
}

/**
 * @brief Handle MOTOR_STATE_BRAKE active braking or immediate re-launch.
 * @param[in,out] p_ctx         Motor task context.
 * @param[in]     p_cmd         Latest motor command.
 * @param[in]     b_has_new_cmd True if a new command arrived this tick.
 */
static void motor_step_brake (motor_task_ctx_t * p_ctx,
                              const motor_task_cmd_t * p_cmd,
                              bool b_has_new_cmd)
{
    if ((b_has_new_cmd) &&
        ((0 != p_cmd->fields.target_speed_left) ||
         (0 != p_cmd->fields.target_speed_right)))
    {
        motor_start_new_motion(p_ctx, p_cmd, false);
    }
    else
    {
        p_ctx->out_left  = motor_ramp(p_ctx->out_left,
                                      0,
                                      MOTOR_BRAKE_STEP_PCT);
        p_ctx->out_right = motor_ramp(p_ctx->out_right,
                                      0,
                                      MOTOR_BRAKE_STEP_PCT);
        motor_set_channel(MOTOR_LEFT_A, MOTOR_LEFT_B, p_ctx->out_left);
        motor_set_channel(MOTOR_RIGHT_A, MOTOR_RIGHT_B, p_ctx->out_right);

        if ((0 == p_ctx->out_left) && (0 == p_ctx->out_right))
        {
            p_ctx->tgt_left  = 0;
            p_ctx->tgt_right = 0;
            motor_set_state(&p_ctx->runtime, MOTOR_STATE_IDLE);
        }
    }
}

/**
 * @brief Handle MOTOR_STATE_ESTOP wheel settle confirmation to Navigator.
 * @param[in,out] p_ctx   Motor task context.
 * @param[in]     delta_l Left encoder delta ticks.
 * @param[in]     delta_r Right encoder delta ticks.
 */
static void motor_step_estop (motor_task_ctx_t * p_ctx,
                              int32_t delta_l,
                              int32_t delta_r)
{
    bool b_are_wheels_still = ((0 == delta_l) && (0 == delta_r));

    if ((b_are_wheels_still) &&
        (p_ctx->runtime.stop_stable_periods < MOTOR_STABLE_PERIODS_REQ))
    {
        p_ctx->runtime.stop_stable_periods++;
    }
    else if (!b_are_wheels_still)
    {
        p_ctx->runtime.stop_stable_periods = 0u;
    }
    else
    {
        /* Already reached required settled periods. */
    }

    if ((motor_task_is_stop_ack_req()) &&
        (p_ctx->runtime.stop_stable_periods >= MOTOR_STABLE_PERIODS_REQ))
    {
        navigator_task_msg_t stop_msg = { 0 };

        /* Store MSG_MOTOR_STOPPED enum in 4-bit unsigned message field. */
        stop_msg.type = (uint32_t)MSG_MOTOR_STOPPED;

        /* Cast Navigator message size to RTOS INT parameter. */
        if (tk_snd_mbf(robot_tasks_get_nav_mbf(),
                       &stop_msg,
                       (INT)sizeof(stop_msg),
                       TMO_POL) >= E_OK)
        {
            robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, false);
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[motors] Stop confirmed\n");
        }
    }
}

#ifndef NDEBUG
/**
 * @brief Debug-only self-check verifying right encoder channel B backup.
 * @return True if all assertions pass, false otherwise.
 */
static bool motor_right_enc_self_check (void)
{
    bool b_is_ok = false;

    b_is_ok = ((12 == motor_right_enc_ticks(12, 13)) &&
               (0u == g_enc_isr_state.b_is_using_b) &&
               (12 == motor_right_enc_ticks(12, 20)) &&
               (0u == g_enc_isr_state.b_is_using_b) &&
               (20 == motor_right_enc_ticks(12, 21)) &&
               (0u != g_enc_isr_state.b_is_using_b) &&
               (0 == motor_right_enc_ticks(12, 1)) &&
               (-60 == motor_right_enc_ticks(12, -59)) &&
               (-60 == motor_right_enc_ticks(-60, -59)));

    /* Restore clean encoder state prior to enabling Timer Alarm 0 IRQ. */
    g_right_enc_last_a           = 0;
    g_right_enc_b_offset         = 0;
    g_enc_isr_state.b_is_using_b = 0u;

    b_is_ok = ((b_is_ok) &&
               (0 == motor_right_enc_ticks(0, 0)) &&
               (0u == g_enc_isr_state.b_is_using_b));

    return b_is_ok;
}
#endif /* NDEBUG */

/**
 * @brief 4000 Hz RP2040 Timer Alarm 0 ISR sampling quadrature encoders.
 *
 * BARR-C Deviation (Rule 6.5.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: micro T-Kernel 3.0 dispatches TA_HLNG interrupt handlers via
 *   its assembly vector trampoline using the standard ARM AAPCS C calling
 *   convention; GCC __attribute__((interrupt)) must not be applied.
 *
 * @param[in] intno RP2040 interrupt vector number (0 = TIMER_IRQ_0).
 */
static void motor_encoder_alarm_isr (UINT intno)
{
    uint32_t gpio_bits = 0u;
    uint32_t raw_la    = 0u;
    uint32_t raw_lb    = 0u;
    uint32_t raw_ra    = 0u;
    uint32_t raw_rb    = 0u;
    int32_t  dir_l     = 1;
    int32_t  dir_r     = 1;
    uint8_t  filters   = 0u;

    out_w(TIMER_INTR, 1u);
    out_w(TIMER_ALARM0, in_w(TIMER_TIMERAWL) + MOTOR_ENC_ALARM_USEC);
    ClearInt(intno);

    /* Range-safe: GPIO_IN is a 32-bit unsigned hardware register. */
    gpio_bits = (uint32_t)in_w(GPIO_IN);
    raw_la    = (gpio_bits >> MOTOR_LEFT_ENC_A) & 1u;
    raw_lb    = (gpio_bits >> MOTOR_LEFT_ENC_B) & 1u;
    raw_ra    = (gpio_bits >> MOTOR_RIGHT_ENC_A) & 1u;
    raw_rb    = (gpio_bits >> MOTOR_RIGHT_ENC_B) & 1u;

    /* Left forward is positive PWM; right forward is negative PWM. */
    dir_l = (g_isr_out_left > 0) ? 1 : ((g_isr_out_left < 0) ? (-1) : 1);
    dir_r = (g_isr_out_right < 0) ? 1 : ((g_isr_out_right > 0) ? (-1) : 1);

    /* Range-safe: filters bit-field is 8 bits wide. */
    filters = (uint8_t)g_enc_isr_state.filters;

    motor_count_rising_edge(raw_la,
                            dir_l,
                            &filters,
                            MOTOR_ENC_LA_SHIFT,
                            &g_isr_ticks_la);
    motor_count_rising_edge(raw_lb,
                            dir_l,
                            &filters,
                            MOTOR_ENC_LB_SHIFT,
                            &g_isr_ticks_lb);
    motor_count_rising_edge(raw_ra,
                            dir_r,
                            &filters,
                            MOTOR_ENC_RA_SHIFT,
                            &g_isr_ticks_ra);
    motor_count_rising_edge(raw_rb,
                            dir_r,
                            &filters,
                            MOTOR_ENC_RB_SHIFT,
                            &g_isr_ticks_rb);

    g_enc_isr_state.filters = filters;
    g_enc_left_ticks        = g_isr_ticks_la;
    g_enc_right_ticks       = motor_right_enc_ticks(g_isr_ticks_ra,
                                                    g_isr_ticks_rb);
}

/*** end of file ***/
