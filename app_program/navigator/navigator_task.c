/**
 * @file navigator_task.c
 * @brief Central Navigator state machine and shared telemetry snapshot owner.
 *
 * NOTE: See docs/fsm_diagrams.md for Navigator and Bypass FSM diagrams.
 * WARNING: Intersection advance/turn timings (250 ms, 900 ms, 30 ms, 50 ms)
 * are empirically tuned for chassis speed and wheel friction.
 */

#include "navigator_task.h"

#include <stdbool.h>
#include <stdint.h>

#include <tm/tmonitor.h>

#include "../motor/motor_task.h"
#include "../network/mqtt_client.h"
#include "../robot_tasks.h"
#include "../ultrasonic/bypass_helpers.h"
#include "../ultrasonic/ultrasonic_task.h"

#define NAV_STARTUP_GRACE_MSEC          (2000u)
#define NAV_SPAM_MITIGATION_LIMIT       (10u)
#define NAV_ADVANCE_PRE_DELAY_MSEC      (250u)
#define NAV_ADVANCE_RUN_DELAY_MSEC      (900u)
#define NAV_ADVANCE_POST_DELAY_MSEC     (30u)
#define NAV_TURN_SETTLE_DELAY_MSEC      (50u)
#define NAV_TURN_STOP_DELAY_MSEC        (50u)
#define NAV_ROTATIONS_BIT_WIDTH         (3u)
#define NAV_SONAR_CENTER_RAY            (3u)
#define NAV_SONAR_MIN_CM                (2u)
#define NAV_SONAR_MAX_CM                (400u)
#define NAV_REENTRY_BLACK_REQ           (2u)
#define NAV_HUMP_MIN_WHEEL_SPD          (10)
#define NAV_HUMP_ROLL_THRESH            (10)
#define NAV_TRACK_ANGLE_DEG             (45)
#define NAV_ASCII_LOWER_A               ((uint8_t)'a')
#define NAV_ASCII_LOWER_Z               ((uint8_t)'z')
#define NAV_ASCII_CASE_DELTA            (32u)

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t m_mode                  : 4;
    uint32_t prev_ir_state           : 3;
    uint32_t b_reentry               : 1;
    uint32_t reentry_dir             : 3;
    uint32_t instruction_idx         : 3;
    uint32_t instruction_idx_barcode : 3;
    uint32_t mapping_increment       : 3;
    uint32_t num_rotations_raw       : 3;
    uint32_t b_hump_pop              : 1;
    uint32_t spam_mitigation         : 4;
    uint32_t b_scan_requested        : 1;
    uint32_t b_live_sonar_valid      : 1;
    uint32_t live_sonar_status       : 3;
    uint32_t turning_rampup_cnt      : 6;
    uint32_t last_decoded_char       : 7;
    uint32_t last_obstacle_dist_cm   : 9;
    uint32_t last_closest_cm         : 9;
    uint32_t last_closest_ang_raw    : 7;
    uint32_t live_sonar_dist_cm      : 9;
} navigator_task_state_t;

typedef struct
{
    navigator_task_state_t       nav;
    motor_task_cmd_t             mot_cmd;
    uint8_t                      barcode_chars[NAV_NUMBER_OF_COMMANDS];
    navigator_task_route_t       barcode_insts;
    navigator_task_route_t       instructions;
    navigator_task_width_t       measured_width;
    navigator_task_bypass_ctx_t  bypass;
} navigator_task_ctx_t;

ROBOT_TASKS_STATIC_ASSERT(nav_state_size,
                          (10u == sizeof(navigator_task_state_t)));

static volatile navigator_task_telemetry_t g_telemetry_snapshot = { 0u };

static void nav_clear_mbf (void);
static void nav_drive_wheels (motor_task_cmd_t * p_cmd,
                              int8_t left_spd,
                              int8_t right_spd);
static void nav_handle_wifi_comms (navigator_task_ctx_t * p_ctx,
                                   const navigator_task_msg_t * p_msg);
static void nav_handle_line_ramp (navigator_task_ctx_t * p_ctx,
                                  navigator_task_ir_state_t curr_state);
static void nav_handle_line_ir (navigator_task_ctx_t * p_ctx,
                                navigator_task_ir_state_t curr_state);
static void nav_handle_line_following (navigator_task_ctx_t * p_ctx,
                                       const navigator_task_msg_t * p_msg);
static void nav_handle_hump (navigator_task_ctx_t * p_ctx,
                             const navigator_task_msg_t * p_msg);
static void nav_handle_decision_advance (navigator_task_ctx_t * p_ctx);
static void nav_start_turn_decision (navigator_task_ctx_t * p_ctx,
                                     navigator_task_dir_t decision);
static void nav_step_turn_progress (navigator_task_ctx_t * p_ctx,
                                    navigator_task_ir_state_t curr_state,
                                    navigator_task_dir_t decision);
static void nav_handle_decision_turn (navigator_task_ctx_t * p_ctx,
                                      const navigator_task_msg_t * p_msg);
static void nav_handle_assess_profile (
    navigator_task_ctx_t * p_ctx,
    const navigator_task_obs_profile_t * p_prof);
static void nav_handle_assess_move_done (navigator_task_ctx_t * p_ctx);
static void nav_handle_obstacle_assess (navigator_task_ctx_t * p_ctx,
                                        const navigator_task_msg_t * p_msg);
static void nav_wall_handle_turn_in (navigator_task_ctx_t * p_ctx);
static void nav_wall_dispatch_phase (navigator_task_ctx_t * p_ctx);
static void nav_handle_wall_move_done (navigator_task_ctx_t * p_ctx);
static void nav_finish_line_reentry (navigator_task_ctx_t * p_ctx);
static void nav_handle_wall_ir (navigator_task_ctx_t * p_ctx,
                                navigator_task_ir_state_t curr_state);
static void nav_handle_wall_track (
    navigator_task_ctx_t * p_ctx,
    const navigator_task_sonar_probe_t * p_probe);
static void nav_handle_wall_hugging (navigator_task_ctx_t * p_ctx,
                                     const navigator_task_msg_t * p_msg);
static void nav_dispatch_mode (navigator_task_ctx_t * p_ctx,
                               const navigator_task_msg_t * p_msg);
static void nav_process_message (navigator_task_ctx_t * p_ctx,
                                 const navigator_task_msg_t * p_msg,
                                 bool b_bypass_active);
static void nav_publish_telemetry (const navigator_task_ctx_t * p_ctx,
                                   bool b_bypass_active);

/**
 * @brief Copy the latest telemetry snapshot under the shared data lock.
 * @param[out] p_snapshot Destination snapshot structure.
 */
void navigator_task_read_telem (navigator_task_telemetry_t * p_snapshot)
{
    UINT saved_state = robot_tasks_data_lock();

    *p_snapshot = g_telemetry_snapshot;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Read the current IMU event code from shared telemetry.
 * @return IMU event enum value (0..3).
 */
uint8_t navigator_task_get_imu_event (void)
{
    UINT    saved_state = robot_tasks_data_lock();
    uint8_t event_val   = (uint8_t)g_telemetry_snapshot.imu_event;

    robot_tasks_data_unlock(saved_state);

    return event_val;
}

/**
 * @brief Read the latest compass heading in degrees from shared telemetry.
 * @return Heading angle (0..359 degrees).
 */
int16_t navigator_task_get_heading (void)
{
    UINT    saved_state = robot_tasks_data_lock();
    /* Range-safe: heading_deg is 0..359 (9 bits). */
    int16_t heading_deg = (int16_t)g_telemetry_snapshot.heading_deg;

    robot_tasks_data_unlock(saved_state);

    return heading_deg;
}

/**
 * @brief Update live ultrasonic distance fields in shared telemetry.
 * @param[in] b_valid True if distance reading is valid.
 * @param[in] status  Ultrasonic status enum value.
 * @param[in] dist_cm Distance in cm (zeroed when invalid).
 */
void navigator_task_set_sonar_telem (bool b_valid,
                                     uint8_t status,
                                     uint32_t dist_cm)
{
    UINT saved_state = robot_tasks_data_lock();

    g_telemetry_snapshot.obs_dist_valid = b_valid ? 1u : 0u;
    g_telemetry_snapshot.obs_status     = status;
    /* Range-safe: valid ultrasonic distances are in 0..400 cm (9 bits). */
    g_telemetry_snapshot.obs_dist_cm    = b_valid ? ((uint16_t)dist_cm) : 0u;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Update IMU live hump height in tenths of a millimetre.
 * @param[in] height_dmm Clamped height (0..680).
 */
void navigator_task_set_imu_height (int16_t height_dmm)
{
    UINT saved_state = robot_tasks_data_lock();

    /* Range-safe: callers pass values clamped to [0, 680]. */
    g_telemetry_snapshot.height_dmm = (uint16_t)height_dmm;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Update IMU event code in shared telemetry.
 * @param[in] imu_event IMU event enum value (0..3).
 */
void navigator_task_set_imu_event (uint8_t imu_event)
{
    UINT saved_state = robot_tasks_data_lock();

    g_telemetry_snapshot.imu_event = imu_event;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Update IMU FSM state code in shared telemetry.
 * @param[in] imu_state IMU state enum value (0..3).
 */
void navigator_task_set_imu_state (uint8_t imu_state)
{
    UINT saved_state = robot_tasks_data_lock();

    g_telemetry_snapshot.imu_state = imu_state;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Atomically update IMU pitch, roll, heading, and optional state.
 * @param[in] pitch_ddeg  Pitch angle in tenths of a degree (-1800..1800).
 * @param[in] roll_ddeg   Roll angle in tenths of a degree (-1800..1800).
 * @param[in] heading_deg Compass heading in degrees (0..359).
 * @param[in] b_set_state True to also update imu_state.
 * @param[in] imu_state   IMU state enum value when b_set_state is true.
 */
void navigator_task_set_imu_pose (int16_t pitch_ddeg,
                                  int16_t roll_ddeg,
                                  int16_t heading_deg,
                                  bool b_set_state,
                                  uint8_t imu_state)
{
    UINT saved_state = robot_tasks_data_lock();

    g_telemetry_snapshot.pitch_ddeg_raw =
        (uint16_t)robot_tasks_pack_signed((int32_t)pitch_ddeg,
                                          NAV_TILT_BIT_WIDTH);
    g_telemetry_snapshot.roll_ddeg_raw =
        (uint16_t)robot_tasks_pack_signed((int32_t)roll_ddeg,
                                          NAV_TILT_BIT_WIDTH);
    /* Range-safe: heading_deg is normalized to [0, 359]. */
    g_telemetry_snapshot.heading_deg = (uint16_t)heading_deg;

    if (b_set_state)
    {
        g_telemetry_snapshot.imu_state = imu_state;
    }

    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Atomically latch IMU lifted/overturned telemetry fields.
 * @param[in] pitch_ddeg Pitch angle in tenths of a degree (-1800..1800).
 * @param[in] roll_ddeg  Roll angle in tenths of a degree (-1800..1800).
 * @param[in] imu_state  IMU state enum value.
 * @param[in] imu_event  IMU event enum value.
 */
void navigator_task_set_imu_stop (int16_t pitch_ddeg,
                                  int16_t roll_ddeg,
                                  uint8_t imu_state,
                                  uint8_t imu_event)
{
    UINT saved_state = robot_tasks_data_lock();

    g_telemetry_snapshot.pitch_ddeg_raw =
        (uint16_t)robot_tasks_pack_signed((int32_t)pitch_ddeg,
                                          NAV_TILT_BIT_WIDTH);
    g_telemetry_snapshot.roll_ddeg_raw =
        (uint16_t)robot_tasks_pack_signed((int32_t)roll_ddeg,
                                          NAV_TILT_BIT_WIDTH);
    g_telemetry_snapshot.height_dmm = 0u;
    g_telemetry_snapshot.imu_state  = imu_state;
    g_telemetry_snapshot.imu_event  = imu_event;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Central Navigator coordination task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void navigator_task (INT stacd, void * p_exinf)
{
    navigator_task_ctx_t ctx   = { 0 };
    bool                 b_run = true;

    (void)stacd;
    (void)p_exinf;

    ctx.nav.m_mode            = (uint32_t)MODE_WIFI_SEND_COMMS;
    ctx.nav.prev_ir_state     = (uint32_t)IR_STATE_NULL;
    ctx.nav.reentry_dir       = (uint32_t)DIR_FRONT;
    ctx.nav.live_sonar_status = (uint32_t)SONAR_INVALID;

    for (uint32_t idx = 0u; idx < NAV_NUMBER_OF_COMMANDS; idx++)
    {
        navigator_task_route_set(&ctx.instructions, idx, DIR_INVALID);
    }

    (void)tk_dly_tsk(NAV_STARTUP_GRACE_MSEC);
    nav_clear_mbf();

#ifndef NDEBUG
    if ((!bypass_helpers_self_check()) ||
        (!bypass_helpers_width_check()) ||
        (!mqtt_client_codec_check()))
    {
        robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
        tm_printf((UB *)"[bypass] Self-check failed\n");
        b_run = false;
    }
#endif

    while (b_run)
    {
        navigator_task_msg_t nav_msg  = { 0 };
        bool b_bypass_active =
            (((uint32_t)MODE_OBSTACLE_ASSESS == ctx.nav.m_mode) ||
             ((uint32_t)MODE_WALL_HUGGING == ctx.nav.m_mode));
        TMO wait_tmo = b_bypass_active ? ((TMO)BYPASS_POLL_MSEC) : TMO_FEVR;
        INT msg_size = tk_rcv_mbf(robot_tasks_get_nav_mbf(),
                                  (void *)&nav_msg,
                                  wait_tmo);

        if (b_bypass_active && (E_TMOUT == msg_size))
        {
            nav_msg.type = (uint32_t)MSG_BYPASS_TICK;
            msg_size     = (INT)sizeof(nav_msg);
        }

        if (b_bypass_active &&
            (bypass_helpers_has_hw_fault() ||
             (motor_task_is_estop() && (0u == ctx.bypass.owns_obs_stop)) ||
             ((msg_size < E_OK) && (E_TMOUT != msg_size))))
        {
            bypass_helpers_abort(&ctx.bypass,
                                 &ctx.mot_cmd,
                                 "hardware or Navigator receive fault");
            ctx.nav.m_mode  = (uint32_t)MODE_SAFE_STOP;
            b_bypass_active = false;
        }

        if ((INT)sizeof(navigator_task_msg_t) != msg_size)
        {
            /* Unexpected RTOS message buffer error or partial read. */
        }
        else
        {
            nav_process_message(&ctx, &nav_msg, b_bypass_active);
            nav_publish_telemetry(&ctx, b_bypass_active);
        }
    }
}

/**
 * @brief Drain all pending stale messages from the Navigator message buffer.
 */
static void nav_clear_mbf (void)
{
    navigator_task_msg_t discard_msg = { 0 };
    ID                   h_nav_mbf   = robot_tasks_get_nav_mbf();

    while (tk_rcv_mbf(h_nav_mbf, &discard_msg, TMO_POL) >= E_OK)
    {
        /* Discard stale queued messages before starting the next phase. */
    }
}

/**
 * @brief Populate wheel speed command with zero ticks and send blocking.
 * @param[out] p_cmd     Motor command structure updated in place.
 * @param[in]  left_spd  Left wheel speed percentage.
 * @param[in]  right_spd Right wheel speed percentage.
 */
static void nav_drive_wheels (motor_task_cmd_t * p_cmd,
                              int8_t left_spd,
                              int8_t right_spd)
{
    p_cmd->fields.target_speed_left  = left_spd;
    p_cmd->fields.target_speed_right = right_spd;
    p_cmd->fields.turn_ticks_left    = 0u;
    p_cmd->fields.turn_ticks_right   = 0u;

    (void)tk_snd_mbf(robot_tasks_get_motor_mbf(),
                     (void *)p_cmd,
                     (INT)sizeof(motor_task_cmd_t),
                     TMO_FEVR);
}

/**
 * @brief Handle MODE_WIFI_SEND_COMMS barcode-to-direction mapping messages.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_handle_wifi_comms (navigator_task_ctx_t * p_ctx,
                                   const navigator_task_msg_t * p_msg)
{
    p_ctx->nav.instruction_idx = 0u;
    nav_drive_wheels(&p_ctx->mot_cmd,
                     (int8_t)NAV_STOP_SPEED,
                     (int8_t)NAV_STOP_SPEED);

    switch ((navigator_task_msg_type_t)p_msg->type)
    {
        case MSG_WIFI_BARCODE_MAPPING:
        {
            uint32_t             raw_map = (uint32_t)p_msg->data.packed_mapping;
            navigator_task_dir_t dir     =
                (navigator_task_dir_t)(raw_map >> 7u);
            /* Range-safe: low 7 bits hold ASCII character 0..127. */
            uint8_t code_ch = (uint8_t)(raw_map & 0x007Fu);

            if ((code_ch >= NAV_ASCII_LOWER_A) &&
                (code_ch <= NAV_ASCII_LOWER_Z))
            {
                code_ch = (uint8_t)(code_ch - NAV_ASCII_CASE_DELTA);
            }

            tm_printf((UB *)"[nav] mapping received: %c -> %d\n",
                      (INT)code_ch,
                      (INT)dir);

            if ((p_ctx->nav.mapping_increment < NAV_NUMBER_OF_COMMANDS) &&
                (dir <= DIR_STOP))
            {
                p_ctx->barcode_chars[p_ctx->nav.mapping_increment] = code_ch;
                navigator_task_route_set(&p_ctx->barcode_insts,
                                         p_ctx->nav.mapping_increment,
                                         dir);
                p_ctx->nav.mapping_increment++;

                if (p_ctx->nav.mapping_increment >= NAV_NUMBER_OF_COMMANDS)
                {
                    p_ctx->nav.m_mode = (uint32_t)MODE_LINE_FOLLOWING;
                }
            }
        }
        break;

        default:
        break;
    }
}

/**
 * @brief Ramp differential turn power if vehicle remains off-center too long.
 * @param[in,out] p_ctx      Navigator context.
 * @param[in]     curr_state Current line-following IR state.
 */
static void nav_handle_line_ramp (navigator_task_ctx_t * p_ctx,
                                  navigator_task_ir_state_t curr_state)
{
    if ((IR_STATE_B_W == curr_state) || (IR_STATE_W_B == curr_state))
    {
        if (p_ctx->nav.turning_rampup_cnt < NAV_TURNING_RAMPUP_LIMIT)
        {
            p_ctx->nav.turning_rampup_cnt++;
        }

        if (NAV_TURNING_RAMPUP_LIMIT == p_ctx->nav.turning_rampup_cnt)
        {
            int8_t left_pwr  = (IR_STATE_B_W == curr_state)
                               ? (int8_t)NAV_LEFT_TURN_L_RAMP
                               : (int8_t)NAV_RIGHT_TURN_L_RAMP;
            int8_t right_pwr = (IR_STATE_B_W == curr_state)
                               ? (int8_t)(-NAV_LEFT_TURN_R_RAMP)
                               : (int8_t)(-NAV_RIGHT_TURN_R_RAMP);

            p_ctx->nav.turning_rampup_cnt++;
            nav_drive_wheels(&p_ctx->mot_cmd, left_pwr, right_pwr);
        }
    }
}

/**
 * @brief Process one line-following IR sensor state update.
 * @param[in,out] p_ctx      Navigator context.
 * @param[in]     curr_state Latest two-sensor IR state.
 */
static void nav_handle_line_ir (navigator_task_ctx_t * p_ctx,
                                navigator_task_ir_state_t curr_state)
{
    bool b_reentry_hit = ((0u != p_ctx->nav.b_reentry) &&
                          (IR_STATE_W_W != curr_state) &&
                          (IR_STATE_NULL != curr_state));

    if (b_reentry_hit)
    {
        p_ctx->nav.prev_ir_state      = (uint32_t)IR_STATE_B_B;
        p_ctx->nav.num_rotations_raw  = 0u;
        p_ctx->nav.turning_rampup_cnt = 0u;
        p_ctx->nav.m_mode             = (uint32_t)MODE_DECISION_MAKING_ADVANCE;
        tm_printf((UB *)"[nav] Reentry line detected; preparing alignment "
                  "turn\n");
    }
    else if (((uint32_t)curr_state != p_ctx->nav.prev_ir_state) ||
             (0u != p_ctx->nav.b_reentry))
    {
        if (IR_STATE_W_W == curr_state)
        {
            p_ctx->nav.turning_rampup_cnt = 0u;
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)NAV_FORWARD_SPEED,
                             (int8_t)(-NAV_FORWARD_SPEED));
        }
        else if (IR_STATE_W_B == curr_state)
        {
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)NAV_RIGHT_TURN_L_BASE,
                             (int8_t)(-NAV_RIGHT_TURN_R_BASE));
        }
        else if (IR_STATE_B_W == curr_state)
        {
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)NAV_LEFT_TURN_L_BASE,
                             (int8_t)(-NAV_LEFT_TURN_R_BASE));
        }
        else if (IR_STATE_B_B == curr_state)
        {
            p_ctx->nav.turning_rampup_cnt = 0u;
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)NAV_STOP_SPEED,
                             (int8_t)(-NAV_STOP_SPEED));
            p_ctx->nav.m_mode = (uint32_t)MODE_DECISION_MAKING_ADVANCE;
        }
        else
        {
            /* Erroneous IR state ignored cleanly. */
        }

        p_ctx->nav.prev_ir_state = (uint32_t)curr_state;
    }
    else
    {
        nav_handle_line_ramp(p_ctx, curr_state);
    }
}

/**
 * @brief Handle MODE_LINE_FOLLOWING sensor, barcode, obstacle, and IMU events.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_handle_line_following (navigator_task_ctx_t * p_ctx,
                                       const navigator_task_msg_t * p_msg)
{
    switch ((navigator_task_msg_type_t)p_msg->type)
    {
        case MSG_IR_UPDATE:
            nav_handle_line_ir(p_ctx,
                               (navigator_task_ir_state_t)p_msg->data.ir_state);
        break;

        case MSG_BARCODE_DECODED:
            p_ctx->nav.last_decoded_char = p_msg->data.decoded_char;
#ifndef NDEBUG
            tm_printf((UB *)"HELLO %c\n", (INT)p_msg->data.decoded_char);
#endif
            for (uint32_t idx = 0u; idx < NAV_NUMBER_OF_COMMANDS; idx++)
            {
                if (p_ctx->nav.last_decoded_char ==
                    (uint32_t)p_ctx->barcode_chars[idx])
                {
                    navigator_task_route_set(
                        &p_ctx->instructions,
                        p_ctx->nav.instruction_idx_barcode,
                        navigator_task_route_get(&p_ctx->barcode_insts, idx));
                    p_ctx->nav.instruction_idx_barcode++;
                }
            }
        break;

        case MSG_OBSTACLE_DETECTED:
            p_ctx->nav.last_obstacle_dist_cm = p_msg->data.obs_dist_cm;
            p_ctx->measured_width = (navigator_task_width_t)
            {
                .width_cm = 0u,
                .status   = (uint32_t)WIDTH_SCANNING
            };
            p_ctx->bypass = (navigator_task_bypass_ctx_t)
            {
                .phase         = (uint32_t)BYPASS_IDLE,
                .owns_obs_stop = 1u,
                .deadline_usec =
                    robot_tasks_time_usec() + (BYPASS_RESPONSE_MSEC * 1000u)
            };
            p_ctx->nav.b_scan_requested   = 0u;
            p_ctx->mot_cmd.raw            = 0u;
            p_ctx->nav.b_live_sonar_valid = 0u;
            p_ctx->nav.live_sonar_dist_cm = 0u;
            p_ctx->nav.live_sonar_status  = (uint32_t)SONAR_INVALID;
            robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, true);
            p_ctx->nav.m_mode = (uint32_t)MODE_OBSTACLE_ASSESS;
            tm_printf((UB *)"[nav] Obstacle detected\n");
        break;

        case MSG_IMU_UPDATE:
            if ((uint32_t)IMU_MSG_HUMP_DETECTED == p_msg->data.imu_msg.event)
            {
                nav_drive_wheels(&p_ctx->mot_cmd,
                                 (int8_t)NAV_FORWARD_SPEED_HUMP,
                                 (int8_t)(-NAV_FORWARD_SPEED_HUMP));
                p_ctx->nav.m_mode = (uint32_t)MODE_GOING_OVER_HUMP;
            }
        break;

        default:
        break;
    }
}

/**
 * @brief Handle MODE_GOING_OVER_HUMP IMU pitch/roll balancing and exit.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_handle_hump (navigator_task_ctx_t * p_ctx,
                             const navigator_task_msg_t * p_msg)
{
    if ((uint32_t)MSG_IMU_UPDATE == p_msg->type)
    {
        int32_t left_spd  = NAV_FORWARD_SPEED_HUMP;
        int32_t right_spd = -NAV_FORWARD_SPEED_HUMP;
        int32_t pitch_deg = robot_tasks_sign_extend(
            p_msg->data.imu_msg.pitch_ddeg_raw, NAV_TILT_BIT_WIDTH) / 10;
        int32_t roll_deg  = robot_tasks_sign_extend(
            p_msg->data.imu_msg.roll_ddeg_raw, NAV_TILT_BIT_WIDTH) / 10;

        switch ((navigator_task_imu_event_t)p_msg->data.imu_msg.event)
        {
            case IMU_MSG_HUMP_DETECTED:
                if (pitch_deg < 0)
                {
                    int32_t adj_l = ((left_spd + pitch_deg) >
                                     NAV_HUMP_MIN_WHEEL_SPD)
                                    ? (left_spd + pitch_deg)
                                    : NAV_HUMP_MIN_WHEEL_SPD;

                    /* Range-safe: adj_l is bounded in [10, 30]. */
                    p_ctx->mot_cmd.fields.target_speed_left  = (int8_t)adj_l;
                    p_ctx->mot_cmd.fields.target_speed_right =
                        (int8_t)right_spd;
                }
                else
                {
                    int32_t adj_r = ((right_spd + pitch_deg) <
                                     (-NAV_HUMP_MIN_WHEEL_SPD))
                                    ? (right_spd + pitch_deg)
                                    : (-NAV_HUMP_MIN_WHEEL_SPD);

                    /* Range-safe: adj_r is bounded in [-30, -10]. */
                    p_ctx->mot_cmd.fields.target_speed_left  = (int8_t)left_spd;
                    p_ctx->mot_cmd.fields.target_speed_right = (int8_t)adj_r;
                }

                if ((roll_deg < (-NAV_HUMP_ROLL_THRESH)) &&
                    (0u == p_ctx->nav.b_hump_pop))
                {
                    p_ctx->nav.b_hump_pop = 1u;
                }
                else if ((roll_deg > NAV_HUMP_ROLL_THRESH) &&
                         (0u != p_ctx->nav.b_hump_pop))
                {
                    p_ctx->nav.b_hump_pop = 0u;
                }
                else
                {
                    /* Roll within deadband; retain hump_pop flag. */
                }

                p_ctx->mot_cmd.fields.turn_ticks_left  = 0u;
                p_ctx->mot_cmd.fields.turn_ticks_right = 0u;
                p_ctx->nav.spam_mitigation++;

                if (p_ctx->nav.spam_mitigation > NAV_SPAM_MITIGATION_LIMIT)
                {
                    (void)tk_snd_mbf(robot_tasks_get_motor_mbf(),
                                     (void *)&p_ctx->mot_cmd,
                                     (INT)sizeof(motor_task_cmd_t),
                                     TMO_FEVR);
                    p_ctx->nav.spam_mitigation = 0u;
                }
            break;

            case IMU_MSG_NO_HUMP:
                p_ctx->nav.b_hump_pop    = 0u;
                p_ctx->nav.prev_ir_state = (uint32_t)IR_STATE_NULL;
                p_ctx->nav.m_mode        = (uint32_t)MODE_LINE_FOLLOWING;
                nav_drive_wheels(&p_ctx->mot_cmd,
                                 (int8_t)NAV_FORWARD_SPEED,
                                 (int8_t)(-NAV_FORWARD_SPEED));
            break;

            default:
            break;
        }
    }
}

/**
 * @brief Advance past an intersection to center the wheel axle before turning.
 * @param[in,out] p_ctx Navigator context.
 */
static void nav_handle_decision_advance (navigator_task_ctx_t * p_ctx)
{
    (void)tk_dly_tsk(NAV_ADVANCE_PRE_DELAY_MSEC);
    nav_drive_wheels(&p_ctx->mot_cmd,
                     (int8_t)NAV_FORWARD_SPEED,
                     (int8_t)(-NAV_FORWARD_SPEED));
    (void)tk_dly_tsk(NAV_ADVANCE_RUN_DELAY_MSEC);
    nav_drive_wheels(&p_ctx->mot_cmd,
                     (int8_t)NAV_STOP_SPEED,
                     (int8_t)NAV_STOP_SPEED);

    p_ctx->nav.m_mode = (uint32_t)MODE_DECISION_MAKING_TURN;
    nav_clear_mbf();
    (void)tk_dly_tsk(NAV_ADVANCE_POST_DELAY_MSEC);
}

/**
 * @brief Initiate a junction pivot or resume straight travel for decision.
 * @param[in,out] p_ctx    Navigator context.
 * @param[in]     decision Route or re-entry turn direction.
 */
static void nav_start_turn_decision (navigator_task_ctx_t * p_ctx,
                                     navigator_task_dir_t decision)
{
    switch (decision)
    {
        case DIR_BACK:
            p_ctx->nav.num_rotations_raw =
                robot_tasks_pack_signed(2, NAV_ROTATIONS_BIT_WIDTH);
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)(-NAV_LEFT_PIVOT_L),
                             (int8_t)(-NAV_RIGHT_PIVOT_R));
        break;

        case DIR_LEFT:
            p_ctx->nav.num_rotations_raw =
                robot_tasks_pack_signed(1, NAV_ROTATIONS_BIT_WIDTH);
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)(-NAV_LEFT_PIVOT_L),
                             (int8_t)(-NAV_LEFT_PIVOT_R));
        break;

        case DIR_RIGHT:
            p_ctx->nav.num_rotations_raw =
                robot_tasks_pack_signed(1, NAV_ROTATIONS_BIT_WIDTH);
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)NAV_RIGHT_PIVOT_L,
                             (int8_t)NAV_RIGHT_PIVOT_R);
        break;

        case DIR_STOP:
            p_ctx->nav.num_rotations_raw = 0u;
            /* FALLTHROUGH: preserve existing DIR_STOP handling as DIR_FRONT. */

        default:
            p_ctx->nav.num_rotations_raw = 0u;
            nav_drive_wheels(&p_ctx->mot_cmd,
                             (int8_t)NAV_FORWARD_SPEED,
                             (int8_t)(-NAV_FORWARD_SPEED));
            p_ctx->nav.prev_ir_state = (uint32_t)IR_STATE_NULL;
            p_ctx->nav.m_mode        = (uint32_t)MODE_LINE_FOLLOWING;

            if (0u == p_ctx->nav.b_reentry)
            {
                p_ctx->nav.instruction_idx++;

                if (p_ctx->nav.instruction_idx >= NAV_NUMBER_OF_COMMANDS)
                {
                    p_ctx->nav.instruction_idx--;
                }
            }

            p_ctx->nav.b_reentry = 0u;
        break;
    }
}

/**
 * @brief Track line-crossing rotations until the junction pivot completes.
 * @param[in,out] p_ctx      Navigator context.
 * @param[in]     curr_state Latest IR line state.
 * @param[in]     decision   Active turn direction.
 */
static void nav_step_turn_progress (navigator_task_ctx_t * p_ctx,
                                    navigator_task_ir_state_t curr_state,
                                    navigator_task_dir_t decision)
{
    int32_t num_rot = robot_tasks_sign_extend(p_ctx->nav.num_rotations_raw,
                                              NAV_ROTATIONS_BIT_WIDTH);
    bool b_left_cross =
        ((IR_STATE_B_W == curr_state) &&
         ((uint32_t)IR_STATE_W_W == p_ctx->nav.prev_ir_state) &&
         ((DIR_LEFT == decision) || (DIR_BACK == decision)));
    bool b_right_cross =
        ((IR_STATE_W_B == curr_state) &&
         ((uint32_t)IR_STATE_W_W == p_ctx->nav.prev_ir_state) &&
         (DIR_RIGHT == decision));

    if (b_left_cross)
    {
        num_rot--;
    }
    else if (b_right_cross)
    {
        num_rot--;
    }
    else
    {
        /* No line crossing transition observed on this IR sample. */
    }

    p_ctx->nav.num_rotations_raw =
        robot_tasks_pack_signed(num_rot, NAV_ROTATIONS_BIT_WIDTH);

    if (num_rot <= 0)
    {
        (void)tk_dly_tsk(NAV_TURN_SETTLE_DELAY_MSEC);
        nav_drive_wheels(&p_ctx->mot_cmd,
                         (int8_t)NAV_STOP_SPEED,
                         (int8_t)NAV_STOP_SPEED);
        (void)tk_dly_tsk(NAV_TURN_STOP_DELAY_MSEC);

        if ((0u == p_ctx->nav.b_reentry) &&
            ((p_ctx->nav.instruction_idx + 1u) < NAV_NUMBER_OF_COMMANDS))
        {
            p_ctx->nav.instruction_idx++;
        }

        p_ctx->nav.num_rotations_raw = 0u;
        p_ctx->nav.prev_ir_state     = (uint32_t)IR_STATE_NULL;
        p_ctx->nav.m_mode            = (uint32_t)MODE_LINE_FOLLOWING;
        p_ctx->nav.b_reentry         = 0u;
    }
}

/**
 * @brief Handle MODE_DECISION_MAKING_TURN IR updates at junctions and reentry.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_handle_decision_turn (navigator_task_ctx_t * p_ctx,
                                      const navigator_task_msg_t * p_msg)
{
    if ((uint32_t)MSG_IR_UPDATE == p_msg->type)
    {
        navigator_task_ir_state_t curr_state =
            (navigator_task_ir_state_t)p_msg->data.ir_state;
        navigator_task_dir_t decision = DIR_INVALID;
        int32_t num_rot = robot_tasks_sign_extend(p_ctx->nav.num_rotations_raw,
                                                  NAV_ROTATIONS_BIT_WIDTH);

        if ((DIR_INVALID == navigator_task_route_get(
                 &p_ctx->instructions, p_ctx->nav.instruction_idx_barcode)) &&
            (p_ctx->nav.instruction_idx > 0u))
        {
            p_ctx->nav.instruction_idx--;
        }
        else if ((DIR_INVALID == navigator_task_route_get(
                      &p_ctx->instructions, p_ctx->nav.instruction_idx)) &&
                 (0u == p_ctx->nav.instruction_idx))
        {
            p_ctx->nav.instruction_idx = 0u;
        }
        else
        {
            /* Current route instruction index remains valid. */
        }

        decision = navigator_task_route_get(&p_ctx->instructions,
                                            p_ctx->nav.instruction_idx);

        if (0u != p_ctx->nav.b_reentry)
        {
            decision = (navigator_task_dir_t)p_ctx->nav.reentry_dir;
        }

        if (((uint32_t)IR_STATE_B_B == p_ctx->nav.prev_ir_state) &&
            (!((0u != p_ctx->nav.b_reentry) && (num_rot > 0))))
        {
            nav_start_turn_decision(p_ctx, decision);
        }
        else
        {
            nav_step_turn_progress(p_ctx, curr_state, decision);
        }

        if (p_ctx->nav.prev_ir_state != (uint32_t)curr_state)
        {
            p_ctx->nav.prev_ir_state = (uint32_t)curr_state;
        }
    }
}

/**
 * @brief Process first or second stationary obstacle profile during assessment.
 * @param[in,out] p_ctx  Navigator context.
 * @param[in]     p_prof Completed ultrasonic obstacle profile.
 */
static void nav_handle_assess_profile (
    navigator_task_ctx_t * p_ctx,
    const navigator_task_obs_profile_t * p_prof)
{
    if (0u != p_ctx->nav.b_scan_requested)
    {
        bool b_front_valid = false;

        p_ctx->nav.b_scan_requested   = 0u;
        p_ctx->nav.last_closest_cm    = p_prof->closest_cm;
        p_ctx->nav.last_closest_ang_raw = p_prof->closest_ang_raw;
        p_ctx->nav.live_sonar_status  =
            (uint32_t)navigator_task_ray_status(p_prof, NAV_SONAR_CENTER_RAY);
        b_front_valid =
            (((uint32_t)SONAR_OK == p_ctx->nav.live_sonar_status) &&
             (p_prof->front_cm >= NAV_SONAR_MIN_CM) &&
             (p_prof->front_cm <= NAV_SONAR_MAX_CM));
        p_ctx->nav.b_live_sonar_valid = b_front_valid ? 1u : 0u;
        p_ctx->nav.live_sonar_dist_cm = b_front_valid ? p_prof->front_cm : 0u;

        switch ((navigator_task_bypass_phase_t)p_ctx->bypass.phase)
        {
            case BYPASS_IDLE:
                p_ctx->measured_width =
                    bypass_helpers_estimate_width(p_prof, NULL);

                if ((uint32_t)WIDTH_UNAVAILABLE == p_ctx->measured_width.status)
                {
                    p_ctx->measured_width.status = (uint32_t)WIDTH_SCANNING;
                }

                bypass_helpers_report_width(p_ctx->measured_width);
                p_ctx->bypass.first_scan = *p_prof;
                p_ctx->bypass.front_cm   = p_ctx->nav.last_obstacle_dist_cm;

                if ((p_prof->front_cm >= NAV_SONAR_MIN_CM) &&
                    (p_prof->front_cm <=
                     (SONAR_OBSTACLE_TRIG_CM + BYPASS_PROFILE_DEPTH_CM)))
                {
                    p_ctx->bypass.front_cm = p_prof->front_cm;
                }

                if ((p_ctx->bypass.front_cm + BYPASS_PIVOT_FROM_FRONT_CM) <
                    (BYPASS_SWEPT_RADIUS_CM + SONAR_WALL_BODY_GAP_CM))
                {
                    bypass_helpers_abort(&p_ctx->bypass,
                                         &p_ctx->mot_cmd,
                                         "insufficient room for initial pivot");
                }
                else
                {
                    bypass_helpers_begin(&p_ctx->bypass,
                                         &p_ctx->mot_cmd,
                                         BYPASS_WAIT_RELEASE,
                                         0,
                                         0,
                                         0u);
                }
            break;

            case BYPASS_SCAN_SECOND:
                p_ctx->measured_width = bypass_helpers_estimate_width(
                    &p_ctx->bypass.first_scan, p_prof);
                bypass_helpers_report_width(p_ctx->measured_width);
                bypass_helpers_plan_width(&p_ctx->bypass, p_prof);
                bypass_helpers_begin(&p_ctx->bypass,
                                     &p_ctx->mot_cmd,
                                     BYPASS_SCAN_RETURN,
                                     BYPASS_API_POWER,
                                     -BYPASS_API_POWER,
                                     BYPASS_SCAN_BACK_MM);
            break;

            default:
            break;
        }
    }
}

/**
 * @brief Handle motor move completion during two-scan obstacle assessment.
 * @param[in,out] p_ctx Navigator context.
 */
static void nav_handle_assess_move_done (navigator_task_ctx_t * p_ctx)
{
    if (0u != p_ctx->bypass.move_pending)
    {
        p_ctx->bypass.move_pending = 0u;

        switch ((navigator_task_bypass_phase_t)p_ctx->bypass.phase)
        {
            case BYPASS_SCAN_BACK:
                p_ctx->bypass.phase = (uint32_t)BYPASS_SCAN_SECOND;
                p_ctx->bypass.deadline_usec =
                    robot_tasks_time_usec() + (BYPASS_SCAN_MSEC * 1000u);
                p_ctx->nav.b_scan_requested = 1u;

                if (tk_set_flg(robot_tasks_get_ranger_flg(),
                               SONAR_SCAN_REQUEST) < E_OK)
                {
                    bypass_helpers_abort(&p_ctx->bypass,
                                         &p_ctx->mot_cmd,
                                         "second scan request");
                }
            break;

            case BYPASS_SCAN_RETURN:
            {
                int32_t side = navigator_task_bypass_side(&p_ctx->bypass);

                bypass_helpers_track_side(&p_ctx->bypass, &p_ctx->mot_cmd);
                bypass_helpers_pivot(&p_ctx->bypass,
                                     &p_ctx->mot_cmd,
                                     BYPASS_TURN_OUT,
                                     side);
                p_ctx->nav.m_mode = (uint32_t)MODE_WALL_HUGGING;
            }
            break;

            default:
            break;
        }
    }
}

/**
 * @brief Handle MODE_OBSTACLE_ASSESS events and stationary scan deadlines.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_handle_obstacle_assess (navigator_task_ctx_t * p_ctx,
                                        const navigator_task_msg_t * p_msg)
{
    switch ((navigator_task_msg_type_t)p_msg->type)
    {
        case MSG_MOTOR_STOPPED:
            if ((0u == p_ctx->nav.b_scan_requested) &&
                ((uint32_t)BYPASS_IDLE == p_ctx->bypass.phase))
            {
                if (tk_set_flg(robot_tasks_get_ranger_flg(),
                               SONAR_SCAN_REQUEST) < E_OK)
                {
                    bypass_helpers_abort(&p_ctx->bypass,
                                         &p_ctx->mot_cmd,
                                         "scan request");
                }
                else
                {
                    p_ctx->nav.b_scan_requested = 1u;
                    p_ctx->bypass.deadline_usec =
                        robot_tasks_time_usec() + (BYPASS_SCAN_MSEC * 1000u);
                }
            }
        break;

        case MSG_OBSTACLE_PROFILED:
            nav_handle_assess_profile(p_ctx, &p_msg->data.obs_profile);
        break;

        case MSG_MOTOR_MOVE_DONE:
            nav_handle_assess_move_done(p_ctx);
        break;

        default:
        break;
    }

    if (((uint32_t)BYPASS_WAIT_RELEASE == p_ctx->bypass.phase) &&
        (0u == (motor_task_get_fault_word() & MOTOR_FAULT_BIT_ESTOP)))
    {
        bypass_helpers_begin(&p_ctx->bypass,
                             &p_ctx->mot_cmd,
                             BYPASS_SCAN_BACK,
                             -BYPASS_API_POWER,
                             BYPASS_API_POWER,
                             BYPASS_SCAN_BACK_MM);
    }

    /* Modular signed difference preserves 32-bit microsecond wraparound. */
    if (((uint32_t)MODE_OBSTACLE_ASSESS == p_ctx->nav.m_mode) &&
        ((uint32_t)BYPASS_ABORT != p_ctx->bypass.phase) &&
        (0u != p_ctx->bypass.deadline_usec) &&
        (((int32_t)(robot_tasks_time_usec() -
                    p_ctx->bypass.deadline_usec)) >= 0))
    {
        bypass_helpers_abort(&p_ctx->bypass,
                             &p_ctx->mot_cmd,
                             "stop acknowledgement or scan timeout");
    }

    if ((uint32_t)BYPASS_ABORT == p_ctx->bypass.phase)
    {
        p_ctx->nav.m_mode = (uint32_t)MODE_SAFE_STOP;
    }
}

/**
 * @brief Handle BYPASS_TURN_IN completion: search for line or return probe.
 * @param[in,out] p_ctx Navigator context.
 */
static void nav_wall_handle_turn_in (navigator_task_ctx_t * p_ctx)
{
    if (0u != p_ctx->bypass.depth_clr_pend)
    {
        p_ctx->bypass.depth_clr_pend = 0u;
        p_ctx->bypass.black_cnt      = 0u;
        bypass_helpers_begin(&p_ctx->bypass,
                             &p_ctx->mot_cmd,
                             BYPASS_FIND_LINE,
                             BYPASS_API_POWER,
                             -BYPASS_API_POWER,
                             p_ctx->bypass.shift_mm +
                             (SONAR_ROBOT_LENGTH_CM * 10u));
    }
    else
    {
        bypass_helpers_probe(&p_ctx->bypass,
                             &p_ctx->mot_cmd,
                             BYPASS_RETURN_CHECK);
    }
}

/**
 * @brief Dispatch next rectangular bypass leg after one leg completes.
 * @param[in,out] p_ctx Navigator context.
 */
static void nav_wall_dispatch_phase (navigator_task_ctx_t * p_ctx)
{
    int32_t side = navigator_task_bypass_side(&p_ctx->bypass);

    switch ((navigator_task_bypass_phase_t)p_ctx->bypass.phase)
    {
        case BYPASS_TURN_OUT:
            bypass_helpers_begin(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 BYPASS_SHIFT_OUT,
                                 BYPASS_API_POWER,
                                 -BYPASS_API_POWER,
                                 p_ctx->bypass.planned_shift_mm);
        break;

        case BYPASS_SHIFT_OUT:
            p_ctx->bypass.shift_mm += p_ctx->bypass.leg_mm;
            bypass_helpers_pivot(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 BYPASS_TURN_PARALLEL,
                                 -side);
        break;

        case BYPASS_TURN_PARALLEL:
            bypass_helpers_probe(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 BYPASS_WIDTH_CHECK);
        break;

        case BYPASS_WIDTH_ALIGN:
            if (0u != p_ctx->bypass.width_clr_pend)
            {
                p_ctx->bypass.width_clr_pend = 0u;
                bypass_helpers_follow_next(&p_ctx->bypass, &p_ctx->mot_cmd);
            }
            else
            {
                bypass_helpers_probe(&p_ctx->bypass,
                                     &p_ctx->mot_cmd,
                                     BYPASS_WIDTH_CHECK);
            }
        break;

        case BYPASS_WIDTH_TURN_OUT:
            bypass_helpers_begin(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 BYPASS_WIDTH_EXTEND,
                                 BYPASS_API_POWER,
                                 -BYPASS_API_POWER,
                                 (0u != p_ctx->bypass.width_clr_pend)
                                     ? BYPASS_PIVOT_CLEAR_MM
                                     : BYPASS_WIDTH_STEP_MM);
        break;

        case BYPASS_WIDTH_EXTEND:
            p_ctx->bypass.shift_mm += p_ctx->bypass.leg_mm;
            bypass_helpers_pivot(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 BYPASS_WIDTH_ALIGN,
                                 -side);
        break;

        case BYPASS_ADVANCE:
            p_ctx->bypass.forward_mm += p_ctx->bypass.leg_mm;
            bypass_helpers_follow_next(&p_ctx->bypass, &p_ctx->mot_cmd);
        break;

        case BYPASS_TURN_IN:
            nav_wall_handle_turn_in(p_ctx);
        break;

        case BYPASS_DEPTH_TURN_BACK:
            bypass_helpers_track_side(&p_ctx->bypass, &p_ctx->mot_cmd);
            bypass_helpers_begin(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 BYPASS_ADVANCE,
                                 BYPASS_API_POWER,
                                 -BYPASS_API_POWER,
                                 (0u != p_ctx->bypass.depth_clr_pend)
                                     ? BYPASS_DEPTH_CLEAR_MM
                                     : BYPASS_FORWARD_STEP_MM);
        break;

        case BYPASS_FIND_LINE:
            bypass_helpers_abort(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 "line not found within search distance");
        break;

        default:
        break;
    }
}

/**
 * @brief Advance rectangular bypass state machine on MSG_MOTOR_MOVE_DONE.
 * @param[in,out] p_ctx Navigator context.
 */
static void nav_handle_wall_move_done (navigator_task_ctx_t * p_ctx)
{
    if (0u != p_ctx->bypass.move_pending)
    {
        p_ctx->bypass.move_pending = 0u;

        if (bypass_helpers_complete_leg(&p_ctx->bypass, &p_ctx->mot_cmd))
        {
            nav_wall_dispatch_phase(p_ctx);
        }
    }
}

/**
 * @brief Stop bypass search and re-enter line following after line reacquired.
 * @param[in,out] p_ctx Navigator context.
 */
static void nav_finish_line_reentry (navigator_task_ctx_t * p_ctx)
{
    bypass_helpers_begin(&p_ctx->bypass,
                         &p_ctx->mot_cmd,
                         BYPASS_LINE_FOUND,
                         0,
                         0,
                         0u);

    if (((uint32_t)BYPASS_ABORT != p_ctx->bypass.phase) &&
        (tk_set_flg(robot_tasks_get_ranger_flg(),
                    SONAR_RESUME_FLAG) >= E_OK))
    {
        int32_t side = navigator_task_bypass_side(&p_ctx->bypass);

        p_ctx->nav.b_scan_requested   = 0u;
        p_ctx->nav.prev_ir_state      = (uint32_t)IR_STATE_NULL;
        p_ctx->nav.turning_rampup_cnt = 0u;
        p_ctx->nav.m_mode             = (uint32_t)MODE_LINE_FOLLOWING;
        p_ctx->nav.reentry_dir        = (side > 0)
                                        ? (uint32_t)DIR_LEFT
                                        : (uint32_t)DIR_RIGHT;
        p_ctx->nav.b_reentry          = 1u;

        tm_printf((UB *)"[bypass] Line found; reentry side=%d, align=%s\n",
                  (INT)side,
                  ((uint32_t)DIR_LEFT == p_ctx->nav.reentry_dir)
                      ? "left"
                      : "right");
    }
    else if ((uint32_t)BYPASS_ABORT != p_ctx->bypass.phase)
    {
        bypass_helpers_abort(&p_ctx->bypass,
                             &p_ctx->mot_cmd,
                             "forward sonar resume");
    }
    else
    {
        /* Bypass already aborted inside bypass_helpers_begin. */
    }
}

/**
 * @brief Detect two consecutive black IR samples during BYPASS_FIND_LINE.
 * @param[in,out] p_ctx      Navigator context.
 * @param[in]     curr_state Latest IR line state.
 */
static void nav_handle_wall_ir (navigator_task_ctx_t * p_ctx,
                                navigator_task_ir_state_t curr_state)
{
    if ((uint32_t)BYPASS_FIND_LINE == p_ctx->bypass.phase)
    {
        bool b_black = ((IR_STATE_B_W == curr_state) ||
                        (IR_STATE_W_B == curr_state) ||
                        (IR_STATE_B_B == curr_state));

        if (b_black && (p_ctx->bypass.black_cnt < NAV_REENTRY_BLACK_REQ))
        {
            p_ctx->bypass.black_cnt++;
        }
        else if (!b_black)
        {
            p_ctx->bypass.black_cnt = 0u;
        }
        else
        {
            /* Required consecutive black samples already reached. */
        }

        if (p_ctx->bypass.black_cnt >= NAV_REENTRY_BLACK_REQ)
        {
            nav_finish_line_reentry(p_ctx);
        }
    }
}

/**
 * @brief Validate freshness and status of side-tracking ultrasonic replies.
 * @param[in,out] p_ctx   Navigator context.
 * @param[in]     p_probe Incoming sonar tracking probe sample.
 */
static void nav_handle_wall_track (
    navigator_task_ctx_t * p_ctx,
    const navigator_task_sonar_probe_t * p_probe)
{
    uint32_t now_usec   = robot_tasks_time_usec();
    int32_t  probe_ang  = robot_tasks_sign_extend(p_probe->angle_deg_raw,
                                                  NAV_ANGLE_BIT_WIDTH);
    int32_t  expect_ang = (-NAV_TRACK_ANGLE_DEG) *
                          navigator_task_bypass_side(&p_ctx->bypass);
    /* Modular subtraction preserves 32-bit wraparound timestamp order. */
    int32_t  since_req  = (int32_t)(p_probe->sampled_at_usec -
                                    p_ctx->bypass.requested_usec);
    int32_t  since_last = (int32_t)(p_probe->sampled_at_usec -
                                    p_ctx->bypass.last_sample_usec);
    uint32_t age_usec   = now_usec - p_probe->sampled_at_usec;
    bool b_fresh = ((0u != p_ctx->bypass.tracking_active) &&
                    (probe_ang == expect_ang) &&
                    (since_req >= 0) &&
                    (since_last > 0) &&
                    (age_usec < (BYPASS_TRACK_STALE_MSEC * 1000u)));

    if (b_fresh)
    {
        bool b_hw_fault =
            (((uint32_t)SONAR_SENSOR_FAULT == p_probe->status) ||
             ((uint32_t)SONAR_SERVO_FAULT == p_probe->status));
        bool b_known_st =
            (((uint32_t)SONAR_OK == p_probe->status) ||
             ((uint32_t)SONAR_NO_ECHO == p_probe->status) ||
             ((uint32_t)SONAR_TOO_CLOSE == p_probe->status) ||
             ((uint32_t)SONAR_UNSTABLE == p_probe->status));
        bool b_bad_ok =
            (((uint32_t)SONAR_OK == p_probe->status) &&
             ((0u == p_probe->b_valid) ||
              (p_probe->dist_cm < NAV_SONAR_MIN_CM) ||
              (p_probe->dist_cm > NAV_SONAR_MAX_CM)));

        if (b_hw_fault)
        {
            bypass_helpers_abort(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 "sonar hardware");
        }
        else if (b_known_st && (!b_bad_ok))
        {
            p_ctx->bypass.last_sample_usec = p_probe->sampled_at_usec;
            p_ctx->bypass.last_track_usec  = p_probe->sampled_at_usec;
        }
        else
        {
            /* Ignore out-of-range or unknown tracking statuses. */
        }
    }
}

/**
 * @brief Handle MODE_WALL_HUGGING rectangular bypass events and watchdogs.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_handle_wall_hugging (navigator_task_ctx_t * p_ctx,
                                     const navigator_task_msg_t * p_msg)
{
    switch ((navigator_task_msg_type_t)p_msg->type)
    {
        case MSG_MOTOR_MOVE_DONE:
            nav_handle_wall_move_done(p_ctx);
        break;

        case MSG_SONAR_PROBED:
        {
            bool b_valid =
                ((0u != p_msg->data.sonar_probe.b_valid) &&
                 ((uint32_t)SONAR_OK == p_msg->data.sonar_probe.status) &&
                 (p_msg->data.sonar_probe.dist_cm >= NAV_SONAR_MIN_CM) &&
                 (p_msg->data.sonar_probe.dist_cm <= NAV_SONAR_MAX_CM));

            p_ctx->nav.live_sonar_status  = p_msg->data.sonar_probe.status;
            p_ctx->nav.b_live_sonar_valid = b_valid ? 1u : 0u;
            p_ctx->nav.live_sonar_dist_cm =
                b_valid ? p_msg->data.sonar_probe.dist_cm : 0u;
            bypass_helpers_probe_result(&p_ctx->bypass,
                                        &p_ctx->mot_cmd,
                                        &p_msg->data.sonar_probe);
        }
        break;

        case MSG_IR_UPDATE:
            nav_handle_wall_ir(p_ctx,
                               (navigator_task_ir_state_t)p_msg->data.ir_state);
        break;

        case MSG_OBSTACLE_DETECTED:
            /* Discard queued front detections while sonar is parked. */
        break;

        case MSG_TRACKING_OBSTACLE:
            nav_handle_wall_track(p_ctx, &p_msg->data.sonar_probe);
        break;

        default:
        break;
    }

    if ((uint32_t)MODE_WALL_HUGGING == p_ctx->nav.m_mode)
    {
        uint32_t now_usec = robot_tasks_time_usec();

        if (((uint32_t)BYPASS_ABORT != p_ctx->bypass.phase) &&
            (0u != p_ctx->bypass.tracking_active) &&
            ((now_usec - p_ctx->bypass.last_track_usec) >=
             (BYPASS_TRACK_STALE_MSEC * 1000u)))
        {
            bypass_helpers_abort(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 "side tracking messages stopped");
        }

        if (((uint32_t)BYPASS_ABORT != p_ctx->bypass.phase) &&
            (0u != p_ctx->bypass.deadline_usec) &&
            (((int32_t)(now_usec - p_ctx->bypass.deadline_usec)) >= 0))
        {
            bypass_helpers_abort(&p_ctx->bypass,
                                 &p_ctx->mot_cmd,
                                 "motor movement timeout");
        }

        if ((uint32_t)BYPASS_ABORT == p_ctx->bypass.phase)
        {
            p_ctx->nav.m_mode = (uint32_t)MODE_SAFE_STOP;
        }
    }
}

/**
 * @brief Dispatch one message to the active Navigator movement mode handler.
 * @param[in,out] p_ctx Navigator context.
 * @param[in]     p_msg Received message.
 */
static void nav_dispatch_mode (navigator_task_ctx_t * p_ctx,
                               const navigator_task_msg_t * p_msg)
{
    switch ((navigator_task_mode_t)p_ctx->nav.m_mode)
    {
        case MODE_WIFI_SEND_COMMS:
            nav_handle_wifi_comms(p_ctx, p_msg);
        break;

        case MODE_LINE_FOLLOWING:
            nav_handle_line_following(p_ctx, p_msg);
        break;

        case MODE_GOING_OVER_HUMP:
            nav_handle_hump(p_ctx, p_msg);
        break;

        case MODE_DECISION_MAKING_ADVANCE:
            nav_handle_decision_advance(p_ctx);
        break;

        case MODE_DECISION_MAKING_TURN:
            nav_handle_decision_turn(p_ctx, p_msg);
        break;

        case MODE_OBSTACLE_ASSESS:
            nav_handle_obstacle_assess(p_ctx, p_msg);
        break;

        case MODE_WALL_HUGGING:
            nav_handle_wall_hugging(p_ctx, p_msg);
        break;

        case MODE_REENTRY_ADVANCE:
        case MODE_REENTRY_PIVOT:
        default:
        break;
    }
}

/**
 * @brief Process global priority messages or dispatch to the active FSM mode.
 * @param[in,out] p_ctx           Navigator context.
 * @param[in]     p_msg           Received message.
 * @param[in]     b_bypass_active True if currently assessing or bypassing.
 */
static void nav_process_message (navigator_task_ctx_t * p_ctx,
                                 const navigator_task_msg_t * p_msg,
                                 bool b_bypass_active)
{
    if (((uint32_t)MSG_REMOTE_CMD == p_msg->type) && (!b_bypass_active))
    {
        if ((!motor_task_is_estop()) &&
            ((uint32_t)MODE_WALL_HUGGING != p_ctx->nav.m_mode) &&
            ((uint32_t)MODE_SAFE_STOP != p_ctx->nav.m_mode))
        {
            p_ctx->mot_cmd = p_msg->data.remote_cmd;
            (void)tk_snd_mbf(robot_tasks_get_motor_mbf(),
                             (void *)&p_ctx->mot_cmd,
                             (INT)sizeof(motor_task_cmd_t),
                             TMO_FEVR);
        }
    }
    else if ((!b_bypass_active) &&
             ((uint32_t)MSG_IMU_UPDATE == p_msg->type) &&
             ((uint32_t)IMU_MSG_LIFTED_OVERTURNED == p_msg->data.imu_msg.event))
    {
        robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
        p_ctx->nav.m_mode = (uint32_t)MODE_SAFE_STOP;
        p_ctx->mot_cmd.fields.target_speed_left  = (int8_t)NAV_STOP_SPEED;
        p_ctx->mot_cmd.fields.target_speed_right = (int8_t)NAV_STOP_SPEED;
        (void)tk_snd_mbf(robot_tasks_get_motor_mbf(),
                         (void *)&p_ctx->mot_cmd,
                         (INT)sizeof(motor_task_cmd_t),
                         TMO_POL);
        tm_printf((UB *)"[nav] STOP: robot lifted/overturned\n");
    }
    else if ((!b_bypass_active) &&
             ((uint32_t)MSG_SONAR_DISTANCE == p_msg->type))
    {
        p_ctx->nav.b_live_sonar_valid = p_msg->data.sonar_dist.b_valid;
        p_ctx->nav.live_sonar_status  = p_msg->data.sonar_dist.status;

        if (0u != p_ctx->nav.b_live_sonar_valid)
        {
            p_ctx->nav.live_sonar_dist_cm = p_msg->data.sonar_dist.dist_cm;
        }
    }
    else
    {
        nav_dispatch_mode(p_ctx, p_msg);
    }

    if (((uint32_t)MODE_OBSTACLE_ASSESS != p_ctx->nav.m_mode) &&
        ((uint32_t)WIDTH_SCANNING == p_ctx->measured_width.status))
    {
        p_ctx->measured_width.status = (uint32_t)WIDTH_UNAVAILABLE;
    }
}

/**
 * @brief Atomically copy Navigator state into the shared telemetry snapshot.
 * @param[in] p_ctx           Navigator context.
 * @param[in] b_bypass_active True if currently assessing or bypassing.
 */
static void nav_publish_telemetry (const navigator_task_ctx_t * p_ctx,
                                   bool b_bypass_active)
{
    UINT saved_state = robot_tasks_data_lock();

    g_telemetry_snapshot.speed_left =
        p_ctx->mot_cmd.fields.target_speed_left;
    g_telemetry_snapshot.speed_right =
        p_ctx->mot_cmd.fields.target_speed_right;
    g_telemetry_snapshot.ir_state         = p_ctx->nav.prev_ir_state;
    g_telemetry_snapshot.movement_mode    = p_ctx->nav.m_mode;
    g_telemetry_snapshot.bypass_phase     = p_ctx->bypass.phase;
    g_telemetry_snapshot.last_barcode     = p_ctx->nav.last_decoded_char;
    g_telemetry_snapshot.inst_idx         = p_ctx->nav.instruction_idx;
    g_telemetry_snapshot.enc_left         = motor_task_get_enc_left();
    g_telemetry_snapshot.enc_right        = motor_task_get_enc_right();
    g_telemetry_snapshot.dist_cm          = motor_task_get_avg_dist_cm();
    g_telemetry_snapshot.closest_cm =
        (uint16_t)p_ctx->nav.last_closest_cm;
    g_telemetry_snapshot.closest_ang_raw  =
        (uint16_t)p_ctx->nav.last_closest_ang_raw;
    g_telemetry_snapshot.est_width_cm     =
        (uint16_t)p_ctx->measured_width.width_cm;
    g_telemetry_snapshot.obs_width_status = p_ctx->measured_width.status;
    g_telemetry_snapshot.seq_num++;
    g_telemetry_snapshot.obs_dist_valid   = p_ctx->nav.b_live_sonar_valid;
    g_telemetry_snapshot.obs_status       = p_ctx->nav.live_sonar_status;

    if (0u != p_ctx->nav.b_live_sonar_valid)
    {
        g_telemetry_snapshot.obs_dist_cm =
            (uint16_t)p_ctx->nav.live_sonar_dist_cm;
    }
    else if (b_bypass_active)
    {
        g_telemetry_snapshot.obs_dist_cm = 0u;
    }
    else
    {
        g_telemetry_snapshot.obs_dist_cm =
            (uint16_t)p_ctx->nav.last_obstacle_dist_cm;
    }

    robot_tasks_data_unlock(saved_state);
}

/*** end of file ***/
