/** @file network_test.c
 *
 * @brief Subsystem 5 (Wi-Fi, Binary MQTT v1 & Web Dashboard) unit tests.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#define PROHIBIT_DEF_SIZE_T

#include <stddef.h>

#include "network_test.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../motor/motor_task.h"
#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"
#include "../test_report.h"
#include "index_html.h"
#include "mqtt_client.h"

#define NET_TEST_HDR_TELEM              (0x11u)
#define NET_TEST_HDR_CMD                (0x12u)
#define NET_TEST_HDR_MAP                (0x13u)
#define NET_TEST_TELEM_START_BIT        (8u)
#define NET_TEST_TELEM_END_BIT          (273u)
#define NET_TEST_TELEM_PAD_MASK         (0xF8u)
#define NET_TEST_MAX_TCP_BODY           (65535u)

static bool network_test_chk_field(const uint8_t * p_bytes,
                                   uint32_t * p_offset,
                                   uint32_t bit_width,
                                   uint32_t expect_val);
static bool network_test_check_cmd_codec(void);
static bool network_test_check_map_valid(void);
static bool network_test_check_map_guards(void);
static bool network_test_check_telem_codec(void);
static bool network_test_check_json_builders(void);
static bool network_test_check_dashboard_json(void);
static bool network_test_check_telem_json(void);
static bool network_test_check_html_asset(void);

/**
 * @brief Execute all automated unit tests (`S5-U01`..`S5-U08`) for Subsystem 5.
 */
void network_test_run (void)
{
    TEST_CHECK("S5-U01", network_test_check_cmd_codec());
    TEST_CHECK("S5-U02", network_test_check_map_valid());
    TEST_CHECK("S5-U03", network_test_check_map_guards());
    TEST_CHECK("S5-U04", network_test_check_telem_codec());
    TEST_CHECK("S5-U05", network_test_check_json_builders());
    TEST_CHECK("S5-U06", network_test_check_dashboard_json());
    TEST_CHECK("S5-U07", network_test_check_telem_json());
    TEST_CHECK("S5-U08", network_test_check_html_asset());
}

/**
 * @brief Extract and verify one bit-packed field in a 0x11 telemetry frame.
 * @param[in]     p_bytes    Encoded 35-byte wire frame.
 * @param[in,out] p_offset   Running bit offset advanced by bit_width.
 * @param[in]     bit_width  Field width in bits (1..32).
 * @param[in]     expect_val Expected unsigned bit pattern.
 * @return True if extracted bits match expect_val masked to bit_width.
 */
static bool network_test_chk_field (const uint8_t * p_bytes,
                                    uint32_t * p_offset,
                                    uint32_t bit_width,
                                    uint32_t expect_val)
{
    uint32_t mask     = 0xFFFFFFFFu >> (32u - bit_width);
    uint32_t observed = robot_tasks_bits_get(p_bytes, *p_offset, bit_width);

    *p_offset += bit_width;

    return (observed == (expect_val & mask));
}

/**
 * @brief S5-U01: Verify binary 0x12 motor command decode and bounds rejection.
 * @return True if valid frames decode and out-of-range/bad frames fail.
 */
static bool network_test_check_cmd_codec (void)
{
    uint8_t cmd_ok[MQTT_COMMAND_BYTES] =
    {
        NET_TEST_HDR_CMD, 100u, (uint8_t)156u, 255u, 7u
    };
    uint8_t cmd_zero[MQTT_COMMAND_BYTES] =
    {
        NET_TEST_HDR_CMD, 0u, 0u, 0u, 0u
    };
    uint8_t cmd_bad[MQTT_COMMAND_BYTES] =
    {
        NET_TEST_HDR_CMD, 101u, 0u, 0u, 0u
    };
    motor_task_cmd_t decoded = { 0u };
    bool             b_is_ok = false;

    b_is_ok = (mqtt_client_decode_cmd(cmd_ok, MQTT_COMMAND_BYTES, &decoded) &&
               (100 == decoded.fields.target_speed_left) &&
               (-100 == decoded.fields.target_speed_right) &&
               (255u == decoded.fields.turn_ticks_left) &&
               (7u == decoded.fields.turn_ticks_right));

    b_is_ok = (b_is_ok &&
               mqtt_client_decode_cmd(cmd_zero, MQTT_COMMAND_BYTES, &decoded) &&
               (0u == decoded.raw));

    /* Speed +101, speed -101 (155u), wrong hdr, bad lengths, and NULL fail. */
    b_is_ok = (b_is_ok &&
               (!mqtt_client_decode_cmd(cmd_bad,
                                        MQTT_COMMAND_BYTES,
                                        &decoded)));
    cmd_bad[1] = 0u;
    cmd_bad[2] = 155u;
    b_is_ok = (b_is_ok &&
               (!mqtt_client_decode_cmd(cmd_bad,
                                        MQTT_COMMAND_BYTES,
                                        &decoded)));
    cmd_bad[2] = 0u;
    cmd_bad[0] = NET_TEST_HDR_TELEM;
    b_is_ok = (b_is_ok &&
               (!mqtt_client_decode_cmd(cmd_bad,
                                        MQTT_COMMAND_BYTES,
                                        &decoded)) &&
               (!mqtt_client_decode_cmd(cmd_ok, 4u, &decoded)) &&
               (!mqtt_client_decode_cmd(cmd_ok, 6u, &decoded)) &&
               (!mqtt_client_decode_cmd(NULL, MQTT_COMMAND_BYTES, &decoded)) &&
               (!mqtt_client_decode_cmd(cmd_ok, MQTT_COMMAND_BYTES, NULL)));

    return b_is_ok;
}

/**
 * @brief S5-U02: Verify binary 0x13 barcode map decode and lowercase folding.
 * @return True if 5-entry bit-packed frame and case normalization succeed.
 */
static bool network_test_check_map_valid (void)
{
    /* 5 entries x 10 bits = 50 bits -> 7 payload bytes + 2 hdr = 9 bytes. */
    static const uint8_t keys[5] =
    {
        (uint8_t)'a', (uint8_t)'f', (uint8_t)'Z', (uint8_t)'0', (uint8_t)'9'
    };
    static const navigator_task_dir_t dirs[5] =
    {
        DIR_FRONT, DIR_LEFT, DIR_RIGHT, DIR_BACK, DIR_STOP
    };
    uint8_t pkt[9]                  = { NET_TEST_HDR_MAP, 5u };
    uint8_t map_tbl[MQTT_MAP_BYTES] = { 0u };

    for (uint32_t idx = 0u; idx < 5u; idx++)
    {
        uint32_t entry = (((uint32_t)dirs[idx]) << 7u) | (uint32_t)keys[idx];

        robot_tasks_bits_set(&pkt[2], idx * 10u, 10u, entry);
    }

    return (mqtt_client_decode_map(pkt, (uint32_t)sizeof(pkt), map_tbl) &&
            ((uint32_t)DIR_FRONT ==
             robot_tasks_bits_get(map_tbl, ((uint32_t)'A') * 3u, 3u)) &&
            ((uint32_t)DIR_LEFT ==
             robot_tasks_bits_get(map_tbl, ((uint32_t)'F') * 3u, 3u)) &&
            ((uint32_t)DIR_RIGHT ==
             robot_tasks_bits_get(map_tbl, ((uint32_t)'Z') * 3u, 3u)) &&
            ((uint32_t)DIR_BACK ==
             robot_tasks_bits_get(map_tbl, ((uint32_t)'0') * 3u, 3u)) &&
            ((uint32_t)DIR_STOP ==
             robot_tasks_bits_get(map_tbl, ((uint32_t)'9') * 3u, 3u)) &&
            ((uint32_t)DIR_INVALID ==
             robot_tasks_bits_get(map_tbl, ((uint32_t)'B') * 3u, 3u)));
}

/**
 * @brief S5-U03: Verify 0x13 map rejection of duplicates, quotes, and bad pad.
 * @return True if every malformed or unsafe 0x13 frame is rejected.
 */
static bool network_test_check_map_guards (void)
{
    uint8_t map_tbl[MQTT_MAP_BYTES] = { 0u };
    uint8_t pkt_1[4]                = { NET_TEST_HDR_MAP, 1u, 0u, 0u };
    uint8_t pkt_dup[5]              = { NET_TEST_HDR_MAP, 2u, 0u, 0u, 0u };
    bool    b_is_ok                 = true;

    /* Valid 1-entry ('A' -> DIR_LEFT), then corrupt padding bit -> fail. */
    robot_tasks_bits_set(&pkt_1[2],
                         0u,
                         10u,
                         (((uint32_t)DIR_LEFT) << 7u) | (uint32_t)'A');
    b_is_ok = mqtt_client_decode_map(pkt_1, 4u, map_tbl);
    pkt_1[3] |= 0x80u;
    b_is_ok = (b_is_ok && (!mqtt_client_decode_map(pkt_1, 4u, map_tbl)));

    /* Disallowed JSON quote '"', backslash '\\', space 32, and dir=5 fail. */
    pkt_1[2] = 0u;
    pkt_1[3] = 0u;
    robot_tasks_bits_set(&pkt_1[2], 0u, 10u, (uint32_t)'"');
    b_is_ok = (b_is_ok && (!mqtt_client_decode_map(pkt_1, 4u, map_tbl)));

    pkt_1[2] = 0u;
    pkt_1[3] = 0u;
    robot_tasks_bits_set(&pkt_1[2], 0u, 10u, (uint32_t)'\\');
    b_is_ok = (b_is_ok && (!mqtt_client_decode_map(pkt_1, 4u, map_tbl)));

    pkt_1[2] = 0u;
    pkt_1[3] = 0u;
    robot_tasks_bits_set(&pkt_1[2],
                         0u,
                         10u,
                         (((uint32_t)DIR_INVALID) << 7u) | (uint32_t)'A');
    b_is_ok = (b_is_ok && (!mqtt_client_decode_map(pkt_1, 4u, map_tbl)));

    /* Duplicate 'A' and 'a' after case folding -> fail. */
    robot_tasks_bits_set(&pkt_dup[2], 0u, 10u, (uint32_t)'A');
    robot_tasks_bits_set(&pkt_dup[2], 10u, 10u, (uint32_t)'a');
    b_is_ok = (b_is_ok &&
               (!mqtt_client_decode_map(pkt_dup, 5u, map_tbl)) &&
               (!mqtt_client_decode_map(pkt_dup, 4u, map_tbl)) &&
               (!mqtt_client_decode_map(NULL, 4u, map_tbl)));

    return b_is_ok;
}

/**
 * @brief S5-U04: Verify 35-byte 0x11 binary telemetry wire encoding & padding.
 * @return True if all 25 bitfields and 5 zero tail padding bits match.
 */
static bool network_test_check_telem_codec (void)
{
    navigator_task_telemetry_t snap                     = { 0u };
    uint8_t                    wire[MQTT_TELEMETRY_BYTES] = { 0xFFu };
    uint32_t                   all_flags = (1u << ROBOT_FLAG_BITS) - 1u;
    uint32_t                   bit_off   = NET_TEST_TELEM_START_BIT;
    bool                       b_is_ok   = true;

    snap.seq_num          = 0xA5A55A5Au;
    snap.enc_left         = INT32_MIN;
    snap.enc_right        = INT32_MAX;
    snap.dist_cm          = -42;
    snap.speed_left       = -100;
    snap.speed_right      = 100;
    snap.obs_dist_cm      = 400u;
    snap.closest_cm       = 19u;
    snap.closest_ang_raw  =
        (uint16_t)robot_tasks_pack_signed(-45, NAV_ANGLE_BIT_WIDTH);
    snap.est_width_cm     = 800u;
    snap.pitch_ddeg_raw   =
        (uint16_t)robot_tasks_pack_signed(-1800, NAV_TILT_BIT_WIDTH);
    snap.roll_ddeg_raw    =
        (uint16_t)robot_tasks_pack_signed(1800, NAV_TILT_BIT_WIDTH);
    snap.heading_deg      = 359u;
    snap.height_dmm       = 680u;
    snap.movement_mode    = (uint32_t)MODE_SAFE_STOP;
    snap.bypass_phase     = (uint32_t)BYPASS_RETURN_CHECK;
    snap.ir_state         = (uint32_t)IR_STATE_B_B;
    snap.last_barcode     = (uint32_t)'F';
    snap.inst_idx            = 4u;
    snap.b_is_obs_dist_valid = 1u;
    snap.obs_status          = (uint32_t)SONAR_OK;
    snap.imu_state        = 3u;
    snap.imu_event        = 3u;
    snap.obs_width_status = (uint32_t)WIDTH_ESTIMATED;

    mqtt_client_encode_telem(wire, &snap, all_flags);

    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 32u,
                                                 snap.seq_num));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 32u,
                                                 (uint32_t)snap.enc_left));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 32u,
                                                 (uint32_t)snap.enc_right));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 32u,
                                                 (uint32_t)snap.dist_cm));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 8u,
                                                 (uint32_t)snap.speed_left));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 8u,
                                                 (uint32_t)snap.speed_right));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 9u,
                                                 snap.obs_dist_cm));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 9u,
                                                 snap.closest_cm));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 7u,
                                                 snap.closest_ang_raw));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 10u,
                                                 snap.est_width_cm));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 12u,
                                                 snap.pitch_ddeg_raw));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 12u,
                                                 snap.roll_ddeg_raw));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 9u,
                                                 snap.heading_deg));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 10u,
                                                 snap.height_dmm));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 4u,
                                                 snap.movement_mode));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 5u,
                                                 snap.bypass_phase));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 3u,
                                                 snap.ir_state));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 7u,
                                                 snap.last_barcode));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 3u,
                                                 snap.inst_idx));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 1u,
                                                 snap.b_is_obs_dist_valid));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 3u,
                                                 snap.obs_status));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 2u,
                                                 snap.imu_state));
    b_is_ok = (b_is_ok && network_test_chk_field(wire, &bit_off, 2u,
                                                 snap.imu_event));

    return (b_is_ok &&
            (NET_TEST_HDR_TELEM == wire[0]) &&
            (NET_TEST_TELEM_END_BIT == (bit_off + ROBOT_FLAG_BITS)) &&
            (all_flags ==
             robot_tasks_bits_get(wire, bit_off, ROBOT_FLAG_BITS)) &&
            ((uint32_t)WIDTH_ESTIMATED ==
             robot_tasks_bits_get(wire, bit_off + ROBOT_FLAG_BITS, 2u)) &&
            (0u == (wire[MQTT_TELEMETRY_BYTES - 1u] & NET_TEST_TELEM_PAD_MASK)));
}

/**
 * @brief S5-U05: Verify bounded JSON char/text/number builders and guard byte.
 * @return True if 0, UINT32_MAX, and tight buffer boundaries never overrun.
 */
static bool network_test_check_json_builders (void)
{
    char     buf[32]  = { '\0' };
    char     tiny[6]  = { 'X', 'X', 'X', 'X', 'X', '\x7F' };
    uint32_t len      = 0u;
    uint32_t tiny_len = 0u;

    mqtt_client_json_text(buf, (uint32_t)sizeof(buf), &len, "v=");
    mqtt_client_json_number(buf, (uint32_t)sizeof(buf), &len, 0u);
    mqtt_client_json_char(buf, (uint32_t)sizeof(buf), &len, ',');
    mqtt_client_json_number(buf, (uint32_t)sizeof(buf), &len, 4294967295u);
    buf[len] = '\0';

    /* Write 10 chars into 5-byte usable buffer; canary at [5] must hold. */
    mqtt_client_json_text(tiny, 5u, &tiny_len, "0123456789");
    tiny[4] = '\0';

    return ((14u == len) &&
            (0 == strcmp(buf, "v=0,4294967295")) &&
            (10u == tiny_len) &&
            (0 == strcmp(tiny, "0123")) &&
            ('\x7F' == tiny[5]));
}

/**
 * @brief S5-U06: Verify /api/mqtt JSON formatter structure and truncation.
 * @return True if output is valid JSON object and small buffers stay safe.
 */
static bool network_test_check_dashboard_json (void)
{
    char     out[256] = { '\0' };
    char     small[8] = { '\0' };
    uint32_t out_len  = mqtt_client_dashboard_json(out, (uint32_t)sizeof(out));
    uint32_t sm_len   =
        mqtt_client_dashboard_json(small, (uint32_t)sizeof(small));

    return ((0u == mqtt_client_dashboard_json(out, 0u)) &&
            (out_len > 20u) &&
            (out_len == (uint32_t)strlen(out)) &&
            ('{' == out[0]) &&
            ('}' == out[out_len - 1u]) &&
            (NULL != strstr(out, "\"enabled\":")) &&
            (NULL != strstr(out, "\"mapping\":{")) &&
            (7u == sm_len) &&
            ('\0' == small[7]));
}

/**
 * @brief S5-U07: Verify /api/telemetry JSON formatter and barcode lookup guard.
 * @return True if telemetry JSON is NUL-terminated and invalid lookups fail.
 */
static bool network_test_check_telem_json (void)
{
    char             out[512] = { '\0' };
    char             tiny[4]  = { 'A', 'A', 'A', 'A' };
    motor_task_cmd_t cmd      = { 0u };
    uint32_t         out_len  =
        mqtt_client_telemetry_json(out, (uint32_t)sizeof(out));
    uint32_t         tiny_len = mqtt_client_telemetry_json(tiny, 2u);

    return ((0u == mqtt_client_telemetry_json(out, 0u)) &&
            (out_len >= 2u) &&
            (out_len == (uint32_t)strlen(out)) &&
            ('{' == out[0]) &&
            ('}' == out[out_len - 1u]) &&
            (1u == tiny_len) &&
            ('\0' == tiny[1]) &&
            (!mqtt_client_get_barcode_cmd((uint8_t)'A', NULL)) &&
            (!mqtt_client_get_barcode_cmd(0u, &cmd)));
}

/**
 * @brief S5-U08: Verify embedded dashboard HTML asset size and header tag.
 * @return True if HTML body matches declared length and fits 16-bit TCP limit.
 */
static bool network_test_check_html_asset (void)
{
    const char * p_body = index_html_get_body();
    uint32_t     length = index_html_get_length();

    return ((NULL != p_body) &&
            (INDEX_HTML_BODY_LEN == length) &&
            (length > 100u) &&
            (length <= NET_TEST_MAX_TCP_BODY) &&
            (0 == strncmp(p_body, "<!DOCTYPE html>", 15u)) &&
            ('\0' == p_body[length]));
}

/*** end of file ***/
