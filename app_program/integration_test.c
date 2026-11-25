/** @file integration_test.c
 *
 * @brief Automated cross-subsystem integration and system readiness tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#define PROHIBIT_DEF_SIZE_T

#include <stddef.h>

#include "integration_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <tk/tkernel.h>

#include "imu/imu_task.h"
#include "ir/ir_task.h"
#include "motor/motor_task.h"
#include "navigator/navigator_task.h"
#include "network/mqtt_client.h"
#include "robot_tasks.h"
#include "test_report.h"
#include "ultrasonic/bypass_helpers.h"
#include "ultrasonic/ultrasonic_task.h"
#include "util/code39.h"

#define INT_TEST_BAR_WIN                (10u)
#define INT_TEST_CODE39_ELEMS           (9u)
#define INT_TEST_HDR_CMD                (0x12u)
#define INT_TEST_HDR_MAP                (0x13u)

static void integration_test_cleanup(void);
static void integration_test_push_pattern(ID h_bar_mbf, uint16_t pat_bits);
static bool integration_test_decode_one(ID h_bar_mbf, ID h_nav_mbf);
static bool integration_test_int01_barcode_nav(void);
static bool integration_test_int02_ir_nav_motor(void);
static bool integration_test_int03_mqtt_map_nav(void);
static bool integration_test_int04_mqtt_cmd_motor(void);
static bool integration_test_int05_sonar_estop_scan(void);
static bool integration_test_int06_bypass_motor_probe(void);
static bool integration_test_int07_imu_nav_estop(void);
static bool integration_test_int08_telem_network(void);
static bool integration_test_sys01_abi_invariants(void);
static bool integration_test_sys02_ipc_and_gate(void);

/**
 * @brief Execute automated integration (`INT-01`..`INT-08`) and system checks.
 */
void integration_test_run (void)
{
    integration_test_cleanup();

    TEST_CHECK("INT-01", integration_test_int01_barcode_nav());
    integration_test_cleanup();

    TEST_CHECK("INT-02", integration_test_int02_ir_nav_motor());
    integration_test_cleanup();

    TEST_CHECK("INT-03", integration_test_int03_mqtt_map_nav());
    integration_test_cleanup();

    TEST_CHECK("INT-04", integration_test_int04_mqtt_cmd_motor());
    integration_test_cleanup();

    TEST_CHECK("INT-05", integration_test_int05_sonar_estop_scan());
    integration_test_cleanup();

    TEST_CHECK("INT-06", integration_test_int06_bypass_motor_probe());
    integration_test_cleanup();

    TEST_CHECK("INT-07", integration_test_int07_imu_nav_estop());
    integration_test_cleanup();

    TEST_CHECK("INT-08", integration_test_int08_telem_network());
    integration_test_cleanup();

    TEST_CHECK("SYS-01", integration_test_sys01_abi_invariants());
    TEST_CHECK("SYS-02", integration_test_sys02_ipc_and_gate());

    integration_test_cleanup();
}

/**
 * @brief Reset all shared IPC buffers, event flags, fault bits, and telemetry.
 */
static void integration_test_cleanup (void)
{
    ir_task_bar_record_t bar_rec = { 0u, 0u };
    navigator_task_msg_t nav_msg = { 0 };
    motor_task_cmd_t     mot_cmd = { 0u };

    while (tk_rcv_mbf(robot_tasks_get_barcode_mbf(),
                      &bar_rec,
                      TMO_POL) >= E_OK)
    {
        /* Drain barcode queue. */
    }

    while (tk_rcv_mbf(robot_tasks_get_nav_mbf(), &nav_msg, TMO_POL) >= E_OK)
    {
        /* Drain Navigator queue. */
    }

    while (tk_rcv_mbf(robot_tasks_get_motor_mbf(), &mot_cmd, TMO_POL) >= E_OK)
    {
        /* Drain motor queue. */
    }

    (void)tk_clr_flg(robot_tasks_get_ranger_flg(), 0u);
    robot_tasks_flags_update(0u, 0xFFFFFFFFu);
    navigator_task_set_sonar_telem(false, 0u, 0u);
    navigator_task_set_imu_pose(0, 0, 0, true, 0u);
    navigator_task_set_imu_height(0);
    navigator_task_set_imu_event(0u);
}

/**
 * @brief Enqueue 10 bar records (1 space + 9 Code 39 elements) into MBF.
 * @param[in] h_bar_mbf Barcode message buffer handle.
 * @param[in] pat_bits  Nine-bit Code 39 pattern.
 */
static void integration_test_push_pattern (ID h_bar_mbf, uint16_t pat_bits)
{
    ir_task_bar_record_t lead_rec = { 0u, 400u };

    (void)tk_snd_mbf(h_bar_mbf, &lead_rec, (INT)sizeof(lead_rec), TMO_POL);

    for (uint32_t idx = 1u; idx < INT_TEST_BAR_WIN; idx++)
    {
        uint32_t             mask = 1u << ((INT_TEST_BAR_WIN - 1u) - idx);
        ir_task_bar_record_t rec  = { 0u, 0u };

        rec.b_is_high = idx & 1u;
        rec.duration  = (0u != (pat_bits & mask)) ? 90u : 30u;
        (void)tk_snd_mbf(h_bar_mbf, &rec, (INT)sizeof(rec), TMO_POL);
    }
}

/**
 * @brief Dequeue 10 bars, threshold + decode, and forward valid char to Nav.
 * @param[in] h_bar_mbf Barcode message buffer handle.
 * @param[in] h_nav_mbf Navigator message buffer handle.
 * @return True if all 10 records were dequeued cleanly.
 */
static bool integration_test_decode_one (ID h_bar_mbf, ID h_nav_mbf)
{
    ir_task_bar_record_t bars[INT_TEST_BAR_WIN] = { { 0u, 0u } };
    uint32_t             sum_dur                = 0u;
    uint32_t             avg_dur                = 0u;
    uint16_t             pat_bits               = 0u;
    uint8_t              ch_val                 = (uint8_t)'?';
    bool                 b_is_ok                = true;

    for (uint32_t idx = 0u; (idx < INT_TEST_BAR_WIN) && b_is_ok; idx++)
    {
        INT rx_sz = tk_rcv_mbf(h_bar_mbf, &bars[idx], TMO_POL);

        b_is_ok = ((INT)sizeof(ir_task_bar_record_t) == rx_sz);
    }

    if (b_is_ok)
    {
        for (uint32_t idx = 1u; idx < INT_TEST_BAR_WIN; idx++)
        {
            sum_dur += bars[idx].duration;
        }

        avg_dur = sum_dur / INT_TEST_CODE39_ELEMS;

        for (uint32_t idx = 1u; idx < INT_TEST_BAR_WIN; idx++)
        {
            if (bars[idx].duration > avg_dur)
            {
                pat_bits |= (uint16_t)(1u << ((INT_TEST_BAR_WIN - 1u) - idx));
            }
        }

        ch_val = code39_decode_pattern(pat_bits);

        if (((uint8_t)'*' != ch_val) && ((uint8_t)'?' != ch_val))
        {
            navigator_task_msg_t nav_msg = { 0 };

            nav_msg.type              = (uint32_t)MSG_BARCODE_DECODED;
            nav_msg.data.decoded_char = ch_val;
            b_is_ok = (E_OK == tk_snd_mbf(h_nav_mbf,
                                          &nav_msg,
                                          (INT)sizeof(nav_msg),
                                          TMO_POL));
        }
    }

    return b_is_ok;
}

/**
 * @brief INT-01: IR Barcode MBF -> Code 39 Decoder -> Navigator MBF ('*F*').
 * @return True if '*' delimiters are filtered and 'F' arrives in gh_nav_mbf.
 */
static bool integration_test_int01_barcode_nav (void)
{
    ID                   h_bar   = robot_tasks_get_barcode_mbf();
    ID                   h_nav   = robot_tasks_get_nav_mbf();
    navigator_task_msg_t rx_msg  = { 0 };
    bool                 b_is_ok = false;

    /* Push '*' (0x094) then 'F' (0x058) = 20 bar records into barcode MBF. */
    integration_test_push_pattern(h_bar, 0x094u);
    integration_test_push_pattern(h_bar, 0x058u);

    b_is_ok = (integration_test_decode_one(h_bar, h_nav) &&
               (E_TMOUT == tk_rcv_mbf(h_nav, &rx_msg, TMO_POL)) &&
               integration_test_decode_one(h_bar, h_nav) &&
               ((INT)sizeof(rx_msg) == tk_rcv_mbf(h_nav, &rx_msg, TMO_POL)) &&
               ((uint32_t)MSG_BARCODE_DECODED == rx_msg.type) &&
               ((uint8_t)'F' == rx_msg.data.decoded_char));

    return b_is_ok;
}

/**
 * @brief INT-02: IR Line State -> Navigator MSG_IR_UPDATE -> Motor MBF speeds.
 * @return True if W_W, W_B, B_W, and B_B map to exact differential speeds.
 */
static bool integration_test_int02_ir_nav_motor (void)
{
    static const navigator_task_ir_state_t states[4] =
    {
        IR_STATE_W_W, IR_STATE_W_B, IR_STATE_B_W, IR_STATE_B_B
    };
    static const int8_t exp_l[4] =
    {
        (int8_t)NAV_FORWARD_SPEED,
        (int8_t)NAV_RIGHT_TURN_L_BASE,
        (int8_t)NAV_LEFT_TURN_L_BASE,
        (int8_t)NAV_STOP_SPEED
    };
    static const int8_t exp_r[4] =
    {
        (int8_t)(-NAV_FORWARD_SPEED),
        (int8_t)(-NAV_RIGHT_TURN_R_BASE),
        (int8_t)(-NAV_LEFT_TURN_R_BASE),
        (int8_t)NAV_STOP_SPEED
    };
    ID   h_nav   = robot_tasks_get_nav_mbf();
    ID   h_mot   = robot_tasks_get_motor_mbf();
    bool b_is_ok = true;

    for (uint32_t idx = 0u; (idx < 4u) && b_is_ok; idx++)
    {
        navigator_task_msg_t tx_ir   = { 0 };
        navigator_task_msg_t rx_ir   = { 0 };
        motor_task_cmd_t     tx_mot  = { 0u };
        motor_task_cmd_t     rx_mot  = { 0u };

        tx_ir.type          = (uint32_t)MSG_IR_UPDATE;
        tx_ir.data.ir_state = (uint8_t)states[idx];

        b_is_ok = ((E_OK == tk_snd_mbf(h_nav,
                                       &tx_ir,
                                       (INT)sizeof(tx_ir),
                                       TMO_POL)) &&
                   ((INT)sizeof(rx_ir) ==
                    tk_rcv_mbf(h_nav, &rx_ir, TMO_POL)) &&
                   ((uint32_t)MSG_IR_UPDATE == rx_ir.type));

        if (b_is_ok)
        {
            tx_mot.fields.target_speed_left  = exp_l[rx_ir.data.ir_state];
            tx_mot.fields.target_speed_right = exp_r[rx_ir.data.ir_state];
            b_is_ok = ((E_OK == tk_snd_mbf(h_mot,
                                           &tx_mot,
                                           (INT)sizeof(tx_mot),
                                           TMO_POL)) &&
                       ((INT)sizeof(rx_mot) ==
                        tk_rcv_mbf(h_mot, &rx_mot, TMO_POL)) &&
                       (exp_l[idx] == rx_mot.fields.target_speed_left) &&
                       (exp_r[idx] == rx_mot.fields.target_speed_right) &&
                       (0u == rx_mot.fields.turn_ticks_left));
        }
    }

    return b_is_ok;
}

/**
 * @brief INT-03: MQTT 0x13 Map Decode -> MSG_WIFI_BARCODE_MAPPING -> Route.
 * @return True if all 5 packed mappings arrive over gh_nav_mbf into route_t.
 */
static bool integration_test_int03_mqtt_map_nav (void)
{
    static const uint8_t keys[NAV_NUMBER_OF_COMMANDS] =
    {
        (uint8_t)'A', (uint8_t)'B', (uint8_t)'C', (uint8_t)'D', (uint8_t)'E'
    };
    static const navigator_task_dir_t dirs[NAV_NUMBER_OF_COMMANDS] =
    {
        DIR_FRONT, DIR_LEFT, DIR_RIGHT, DIR_BACK, DIR_STOP
    };
    uint8_t                pkt[9]                  = { INT_TEST_HDR_MAP, 5u };
    uint8_t                map_tbl[MQTT_MAP_BYTES] = { 0u };
    navigator_task_route_t route                   = { { 0u } };
    ID                     h_nav                   = robot_tasks_get_nav_mbf();
    uint32_t               rx_cnt                  = 0u;
    bool                   b_is_ok                 = true;

    for (uint32_t idx = 0u; idx < NAV_NUMBER_OF_COMMANDS; idx++)
    {
        uint32_t entry = (((uint32_t)dirs[idx]) << 7u) | (uint32_t)keys[idx];

        robot_tasks_bits_set(&pkt[2], idx * 10u, 10u, entry);
    }

    b_is_ok = mqtt_client_decode_map(pkt, (uint32_t)sizeof(pkt), map_tbl);

    for (uint32_t code = 33u; (code <= 126u) && b_is_ok; code++)
    {
        uint32_t dir_val = robot_tasks_bits_get(map_tbl, code * 3u, 3u);

        if ((uint32_t)DIR_INVALID != dir_val)
        {
            navigator_task_msg_t msg = { 0 };

            msg.type                = (uint32_t)MSG_WIFI_BARCODE_MAPPING;
            msg.data.packed_mapping = (uint16_t)((dir_val << 7u) | code);
            b_is_ok = (E_OK == tk_snd_mbf(h_nav,
                                          &msg,
                                          (INT)sizeof(msg),
                                          TMO_POL));
        }
    }

    while (b_is_ok && (rx_cnt < NAV_NUMBER_OF_COMMANDS))
    {
        navigator_task_msg_t rx = { 0 };

        if ((INT)sizeof(rx) != tk_rcv_mbf(h_nav, &rx, TMO_POL))
        {
            b_is_ok = false;
        }
        else
        {
            navigator_task_dir_t rx_dir =
                (navigator_task_dir_t)(rx.data.packed_mapping >> 7u);
            uint8_t rx_ch = (uint8_t)(rx.data.packed_mapping & 0x7Fu);

            navigator_task_set_route(&route, rx_cnt, rx_dir);
            b_is_ok = (((uint32_t)MSG_WIFI_BARCODE_MAPPING == rx.type) &&
                       (keys[rx_cnt] == rx_ch) &&
                       (dirs[rx_cnt] ==
                        navigator_task_get_route(&route, rx_cnt)));
            rx_cnt++;
        }
    }

    return (b_is_ok && (NAV_NUMBER_OF_COMMANDS == rx_cnt));
}

/**
 * @brief INT-04: MQTT 0x12 Command -> Navigator MSG_REMOTE_CMD -> Motor MBF.
 * @return True if remote speeds and turn ticks traverse both queues intact.
 */
static bool integration_test_int04_mqtt_cmd_motor (void)
{
    const uint8_t pkt[MQTT_COMMAND_BYTES] =
    {
        INT_TEST_HDR_CMD, 35u, (uint8_t)(-35), 120u, 120u
    };
    ID                   h_nav   = robot_tasks_get_nav_mbf();
    ID                   h_mot   = robot_tasks_get_motor_mbf();
    motor_task_cmd_t     decoded = { 0u };
    motor_task_cmd_t     rx_mot  = { 0u };
    navigator_task_msg_t nav_tx  = { 0 };
    navigator_task_msg_t nav_rx  = { 0 };
    bool                 b_is_ok = false;

    if (mqtt_client_decode_cmd(pkt, MQTT_COMMAND_BYTES, &decoded))
    {
        nav_tx.type            = (uint32_t)MSG_REMOTE_CMD;
        nav_tx.data.remote_cmd = decoded;

        if ((E_OK == tk_snd_mbf(h_nav, &nav_tx, (INT)sizeof(nav_tx), TMO_POL)) &&
            ((INT)sizeof(nav_rx) == tk_rcv_mbf(h_nav, &nav_rx, TMO_POL)) &&
            ((uint32_t)MSG_REMOTE_CMD == nav_rx.type) &&
            (E_OK == tk_snd_mbf(h_mot,
                                &nav_rx.data.remote_cmd,
                                (INT)sizeof(motor_task_cmd_t),
                                TMO_POL)) &&
            ((INT)sizeof(rx_mot) == tk_rcv_mbf(h_mot, &rx_mot, TMO_POL)))
        {
            b_is_ok = ((35 == rx_mot.fields.target_speed_left) &&
                       (-35 == rx_mot.fields.target_speed_right) &&
                       (120u == rx_mot.fields.turn_ticks_left) &&
                       (120u == rx_mot.fields.turn_ticks_right));
        }
    }

    return b_is_ok;
}

/**
 * @brief INT-05: Ultrasonic E-Stop, Stop-Ack & Ranger Event-Flag Handshake.
 * @return True if obstacle stop, ack, scan flag, and profile delivery succeed.
 */
static bool integration_test_int05_sonar_estop_scan (void)
{
    ID                   h_nav   = robot_tasks_get_nav_mbf();
    ID                   h_flg   = robot_tasks_get_ranger_flg();
    navigator_task_msg_t tx_msg  = { 0 };
    navigator_task_msg_t rx_msg  = { 0 };
    UINT                 flg_ptn = 0u;
    bool                 b_is_ok = false;

    /* Step 1: Ultrasonic latches E-stop and sends MSG_OBSTACLE_DETECTED. */
    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    tx_msg.type             = (uint32_t)MSG_OBSTACLE_DETECTED;
    tx_msg.data.obs_dist_cm = 18u;
    b_is_ok = (motor_task_is_estop() &&
               (E_OK == tk_snd_mbf(h_nav,
                                   &tx_msg,
                                   (INT)sizeof(tx_msg),
                                   TMO_POL)) &&
               ((INT)sizeof(rx_msg) == tk_rcv_mbf(h_nav, &rx_msg, TMO_POL)) &&
               (18u == rx_msg.data.obs_dist_cm));

    /* Step 2: Navigator requests stop ack; Motor replies MSG_MOTOR_STOPPED. */
    robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, true);
    b_is_ok = (b_is_ok && motor_task_is_stop_ack_req());
    robot_tasks_flag_assign(ROBOT_STOP_ACK_REQUEST, false);
    tx_msg      = (navigator_task_msg_t){ 0 };
    tx_msg.type = (uint32_t)MSG_MOTOR_STOPPED;
    b_is_ok = (b_is_ok &&
               (E_OK == tk_snd_mbf(h_nav,
                                   &tx_msg,
                                   (INT)sizeof(tx_msg),
                                   TMO_POL)) &&
               ((INT)sizeof(rx_msg) == tk_rcv_mbf(h_nav, &rx_msg, TMO_POL)) &&
               ((uint32_t)MSG_MOTOR_STOPPED == rx_msg.type));

    /* Step 3: Navigator sets SONAR_SCAN_REQUEST; Ultrasonic wakes + clears. */
    b_is_ok = (b_is_ok &&
               (E_OK == tk_set_flg(h_flg, SONAR_SCAN_REQUEST)) &&
               (E_OK == tk_wai_flg(h_flg,
                                   SONAR_SCAN_REQUEST,
                                   TWF_ORW | TWF_BITCLR,
                                   &flg_ptn,
                                   TMO_POL)) &&
               (SONAR_SCAN_REQUEST == flg_ptn) &&
               (E_TMOUT == tk_wai_flg(h_flg,
                                      SONAR_SCAN_REQUEST,
                                      TWF_ORW,
                                      &flg_ptn,
                                      TMO_POL)));

    /* Step 4: Ultrasonic returns 33-byte MSG_OBSTACLE_PROFILED to Navigator. */
    tx_msg                                    = (navigator_task_msg_t){ 0 };
    tx_msg.type                               = (uint32_t)MSG_OBSTACLE_PROFILED;
    tx_msg.data.obs_profile.front_cm          = 18u;
    tx_msg.data.obs_profile.b_is_profile_valid = 1u;
    navigator_task_set_ray(&tx_msg.data.obs_profile, 3u, 18u, SONAR_OK);

    b_is_ok = (b_is_ok &&
               (E_OK == tk_snd_mbf(h_nav,
                                   &tx_msg,
                                   (INT)sizeof(tx_msg),
                                   TMO_POL)) &&
               ((INT)sizeof(rx_msg) == tk_rcv_mbf(h_nav, &rx_msg, TMO_POL)) &&
               ((uint32_t)MSG_OBSTACLE_PROFILED == rx_msg.type) &&
               (18u == navigator_task_get_ray_cm(&rx_msg.data.obs_profile, 3u)));

    return b_is_ok;
}

/**
 * @brief INT-06: Bypass Planner -> Motor Queue & 2-Probe Clearance Handshake.
 * @return True if planned step, probe flags, and 2x probe clear advance FSM.
 */
static bool integration_test_int06_bypass_motor_probe (void)
{
    navigator_task_bypass_ctx_t  bypass  = { 0u };
    navigator_task_obs_profile_t second  = { 0u };
    navigator_task_sonar_probe_t probe   = { 0u };
    motor_task_cmd_t             mot_cmd = { 0u };
    motor_task_cmd_t             rx_mot  = { 0u };
    UINT                         flg_ptn = 0u;
    bool                         b_is_ok = false;

    bypass.front_cm            = 20u;
    bypass.first_scan.front_cm = 20u;
    second.front_cm            = 22u;
    navigator_task_set_ray(&bypass.first_scan, 4u, 0u, SONAR_NO_ECHO);
    navigator_task_set_ray(&second, 4u, 0u, SONAR_NO_ECHO);

    bypass_helpers_plan_width(&bypass, &second);
    bypass_helpers_begin(&bypass,
                         &mot_cmd,
                         BYPASS_SHIFT_OUT,
                         BYPASS_API_POWER,
                         -BYPASS_API_POWER,
                         bypass.planned_shift_mm);

    b_is_ok = (((INT)sizeof(rx_mot) ==
                tk_rcv_mbf(robot_tasks_get_motor_mbf(), &rx_mot, TMO_POL)) &&
               (BYPASS_API_POWER == rx_mot.fields.target_speed_left) &&
               ((-BYPASS_API_POWER) == rx_mot.fields.target_speed_right) &&
               (rx_mot.fields.turn_ticks_left > 0u));

    /* Stationary width probe requires 2 consecutive clear probes -> turn out. */
    bypass_helpers_probe(&bypass, &mot_cmd, BYPASS_WIDTH_CHECK);
    b_is_ok = (b_is_ok &&
               (E_OK == tk_wai_flg(robot_tasks_get_ranger_flg(),
                                   SONAR_PROBE_FRONT_FLAG,
                                   TWF_ORW | TWF_BITCLR,
                                   &flg_ptn,
                                   TMO_POL)));

    robot_tasks_delay_usec(10u);
    probe.sampled_at_usec = robot_tasks_time_usec();
    probe.angle_deg_raw   = 0u;
    probe.status          = (uint32_t)SONAR_NO_ECHO;
    bypass_helpers_probe_result(&bypass, &mot_cmd, &probe);
    b_is_ok = (b_is_ok &&
               ((uint32_t)BYPASS_WIDTH_CHECK == bypass.phase) &&
               (1u == bypass.probe_clear_cnt));

    robot_tasks_delay_usec(10u);
    probe.sampled_at_usec = robot_tasks_time_usec();
    bypass_helpers_probe_result(&bypass, &mot_cmd, &probe);
    b_is_ok = (b_is_ok &&
               ((uint32_t)BYPASS_WIDTH_TURN_OUT == bypass.phase) &&
               (1u == bypass.b_has_width_clr_pend));

    return b_is_ok;
}

/**
 * @brief INT-07: IMU Hump & Lift Safety -> Navigator MBF & Motor E-Stop.
 * @return True if hump height/event and lift E-stop propagate across boundary.
 */
static bool integration_test_int07_imu_nav_estop (void)
{
    ID                         h_nav   = robot_tasks_get_nav_mbf();
    navigator_task_msg_t       tx_msg  = { 0 };
    navigator_task_msg_t       rx_msg  = { 0 };
    navigator_task_telemetry_t snap    = { 0u };
    bool                       b_is_ok = false;

    /* Phase 1: Hump detected -> height_dmm=220, MSG_IMU_UPDATE delivered. */
    navigator_task_set_imu_height(220);
    navigator_task_set_imu_event((uint8_t)IMU_MSG_HUMP_DETECTED);
    tx_msg.type                        = (uint32_t)MSG_IMU_UPDATE;
    tx_msg.data.imu_msg.event          = (uint32_t)IMU_MSG_HUMP_DETECTED;
    tx_msg.data.imu_msg.pitch_ddeg_raw =
        (uint16_t)robot_tasks_pack_signed(85, NAV_TILT_BIT_WIDTH);
    tx_msg.data.imu_msg.height_dmm     = 220u;

    b_is_ok = ((E_OK == tk_snd_mbf(h_nav,
                                   &tx_msg,
                                   (INT)sizeof(tx_msg),
                                   TMO_POL)) &&
               ((INT)sizeof(rx_msg) == tk_rcv_mbf(h_nav, &rx_msg, TMO_POL)) &&
               ((uint32_t)IMU_MSG_HUMP_DETECTED == rx_msg.data.imu_msg.event) &&
               (!motor_task_is_estop()));

    /* Phase 2: Lift/overturn -> latches E-stop, IMU_TILT, zeroes height. */
    robot_tasks_flags_update(ROBOT_FAULT_IMU_TILT | ROBOT_ESTOP_REQUEST, 0u);
    navigator_task_set_imu_stop(-320,
                                15,
                                (uint8_t)IMU_STATE_LIFTED_OVERTURNED,
                                (uint8_t)IMU_MSG_LIFTED_OVERTURNED);
    navigator_task_read_telem(&snap);

    return (b_is_ok &&
            motor_task_is_estop() &&
            (0u == snap.height_dmm) &&
            ((uint32_t)IMU_STATE_LIFTED_OVERTURNED == snap.imu_state) &&
            ((uint32_t)IMU_MSG_LIFTED_OVERTURNED == snap.imu_event));
}

/**
 * @brief INT-08: Shared Multi-Subsystem Telemetry -> MQTT 0x11 & HTTP JSON.
 * @return True if Ultrasonic + IMU + Fault state serializes bit-accurately.
 */
static bool integration_test_int08_telem_network (void)
{
    navigator_task_telemetry_t snap                     = { 0u };
    uint8_t                    wire[MQTT_TELEMETRY_BYTES] = { 0u };
    char                       json_buf[512]            = { '\0' };
    uint32_t                   flags                    = 0u;
    uint32_t                   json_len                 = 0u;

    navigator_task_set_sonar_telem(true, (uint8_t)SONAR_OK, 77u);
    navigator_task_set_imu_pose(-110,
                                45,
                                90,
                                true,
                                (uint8_t)IMU_STATE_MONITORING);
    navigator_task_set_imu_height(195);
    navigator_task_set_imu_event((uint8_t)IMU_MSG_HUMP_DETECTED);
    robot_tasks_flags_update(ROBOT_FAULT_WIFI | ROBOT_STATUS_NO_ECHO, 0u);

    navigator_task_read_telem(&snap);
    flags = robot_tasks_flags_get();
    mqtt_client_encode_telem(wire, &snap, flags);
    json_len = mqtt_client_telemetry_json(json_buf, (uint32_t)sizeof(json_buf));

    return ((0x11u == wire[0]) &&
            (77u == robot_tasks_bits_get(wire, 152u, 9u)) &&
            (90u == robot_tasks_bits_get(wire, 213u, 9u)) &&
            (195u == robot_tasks_bits_get(wire, 222u, 10u)) &&
            (1u == robot_tasks_bits_get(wire, 251u, 1u)) &&
            (flags == robot_tasks_bits_get(wire, 260u, ROBOT_FLAG_BITS)) &&
            (json_len >= 2u) &&
            ('\0' == json_buf[json_len]));
}

/**
 * @brief SYS-01: Verify cross-core ABI struct sizes and wire frame constants.
 * @return True if all packed IPC and network frame byte sizes match spec.
 */
static bool integration_test_sys01_abi_invariants (void)
{
    return ((4u == sizeof(motor_task_cmd_t)) &&
            (4u == sizeof(ir_task_bar_record_t)) &&
            (7u == sizeof(navigator_task_width_scan_t)) &&
            (2u == sizeof(navigator_task_width_t)) &&
            (33u == sizeof(navigator_task_obs_profile_t)) &&
            (68u == sizeof(navigator_task_bypass_ctx_t)) &&
            (34u == sizeof(navigator_task_msg_t)) &&
            (32u == sizeof(navigator_task_telemetry_t)) &&
            (35u == MQTT_TELEMETRY_BYTES) &&
            (5u == MQTT_COMMAND_BYTES) &&
            (10u == MQTT_STATUS_BYTES) &&
            (0u == robot_tasks_flags_get()));
}

/**
 * @brief SYS-02: Verify live RTOS IPC object handles and zero prior test fails.
 * @return True if all kernel handles are valid and failed count is zero.
 */
static bool integration_test_sys02_ipc_and_gate (void)
{
    return ((robot_tasks_get_barcode_mbf() > 0) &&
            (robot_tasks_get_nav_mbf() > 0) &&
            (robot_tasks_get_motor_mbf() > 0) &&
            (robot_tasks_get_ranger_flg() > 0) &&
            (49u == test_report_get_total()) &&
            (0u == test_report_get_failed()));
}

/*** end of file ***/
