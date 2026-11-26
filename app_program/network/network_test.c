/**
 * @file network_test.c
 * @brief Subsystem 5 (Wi-Fi, Binary MQTT v1 & Web Dashboard) unit tests.
 */

#include "network_test.h"

#include <stdbool.h>
#include <stddef.h>
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
#define NET_TEST_FULL_U32_MASK          (0xFFFFFFFFu)
#define NET_TEST_U32_BITS               (32u)
#define NET_TEST_MAX_TCP_BODY           (65535u)
#define NET_TEST_MAP_VALID_ENTRIES      (5u)
#define NET_TEST_MAP_PKT_5_BYTES        (9u)
#define NET_TEST_MAP_PKT_1_BYTES        (4u)
#define NET_TEST_MAP_PKT_2_BYTES        (5u)
#define NET_TEST_MAP_DATA_IDX           (2u)
#define NET_TEST_CMD_HDR_IDX            (0u)
#define NET_TEST_CMD_LEFT_IDX           (1u)
#define NET_TEST_CMD_RIGHT_IDX          (2u)
#define NET_TEST_JSON_BUF_LEN           (32u)
#define NET_TEST_TINY_GUARD_LEN         (6u)
#define NET_TEST_TINY_CANARY_IDX        (5u)
#define NET_TEST_TINY_NUL_IDX           (4u)
#define NET_TEST_DASH_BUF_LEN           (256u)
#define NET_TEST_SMALL_BUF_LEN          (8u)
#define NET_TEST_TELEM_BUF_LEN          (512u)
#define NET_TEST_TINY_TELEM_LEN         (4u)
#define NET_TEST_DOCTYPE_LEN            (15u)

static bool network_test_chk_field(const uint8_t * p_bytes,
                                   uint32_t * p_offset,
                                   uint32_t bit_width,
                                   uint32_t expect_val);
static bool network_test_check_cmd_codec(void);
static bool network_test_check_map_valid(void);
static bool network_test_check_map_guards(void);
static void network_test_fill_telem_snap(navigator_task_telemetry_t * p_snap);
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
    uint32_t mask     =
        NET_TEST_FULL_U32_MASK >> (NET_TEST_U32_BITS - bit_width);
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
    /* Byte 156u encodes -100 in 8-bit two's-complement wire format. */
    uint8_t          cmd_ok[MQTT_COMMAND_BYTES]   =
    {
        NET_TEST_HDR_CMD, 100u, (uint8_t)156u, 255u, 7u
    };
    uint8_t          cmd_zero[MQTT_COMMAND_BYTES] =
    {
        NET_TEST_HDR_CMD, 0u, 0u, 0u, 0u
    };
    uint8_t          cmd_bad[MQTT_COMMAND_BYTES]  =
    {
        NET_TEST_HDR_CMD, 101u, 0u, 0u, 0u
    };
    motor_task_cmd_t decoded                      = { 0u };
    bool             b_is_ok                      = false;

    b_is_ok = ((mqtt_client_decode_cmd(cmd_ok, MQTT_COMMAND_BYTES, &decoded)) &&
               (100 == decoded.fields.target_speed_left) &&
               (-100 == decoded.fields.target_speed_right) &&
               (255u == decoded.fields.turn_ticks_left) &&
               (7u == decoded.fields.turn_ticks_right));

    b_is_ok = ((b_is_ok) &&
               (mqtt_client_decode_cmd(cmd_zero,
                                       MQTT_COMMAND_BYTES,
                                       &decoded)) &&
               (0u == decoded.raw));

    /* Speed +101, speed -101 (155u), wrong hdr, bad lengths, and NULL fail. */
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_cmd(cmd_bad,
                                        MQTT_COMMAND_BYTES,
                                        &decoded)));
    cmd_bad[NET_TEST_CMD_LEFT_IDX]  = 0u;
    cmd_bad[NET_TEST_CMD_RIGHT_IDX] = 155u;
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_cmd(cmd_bad,
                                        MQTT_COMMAND_BYTES,
                                        &decoded)));
    cmd_bad[NET_TEST_CMD_RIGHT_IDX] = 0u;
    cmd_bad[NET_TEST_CMD_HDR_IDX]   = NET_TEST_HDR_TELEM;
    b_is_ok = ((b_is_ok) &&
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
    /* Cast ASCII barcode character literals to uint8_t test array elements. */
    static const uint8_t              keys[NET_TEST_MAP_VALID_ENTRIES] =
    {
        (uint8_t)'a', (uint8_t)'f', (uint8_t)'Z', (uint8_t)'0', (uint8_t)'9'
    };
    static const navigator_task_dir_t dirs[NET_TEST_MAP_VALID_ENTRIES] =
    {
        DIR_FRONT, DIR_LEFT, DIR_RIGHT, DIR_BACK, DIR_STOP
    };
    uint8_t                           pkt[NET_TEST_MAP_PKT_5_BYTES]    =
    {
        NET_TEST_HDR_MAP, 5u
    };
    uint8_t                           map_tbl[MQTT_MAP_BYTES]          = { 0u };

    for (uint32_t idx = 0u; idx < NET_TEST_MAP_VALID_ENTRIES; idx++)
    {
        /* Pack 3-bit direction << 7 | 7-bit ASCII key into 10-bit wire slot. */
        uint32_t entry = (((uint32_t)dirs[idx]) << 7u) | (uint32_t)keys[idx];

        robot_tasks_bits_set(&pkt[NET_TEST_MAP_DATA_IDX],
                             idx * 10u,
                             10u,
                             entry);
    }

    /* Cast 9-byte packet sizeof and expected direction enums to uint32_t. */
    return ((mqtt_client_decode_map(pkt, (uint32_t)sizeof(pkt), map_tbl)) &&
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
    uint8_t map_tbl[MQTT_MAP_BYTES]          = { 0u };
    uint8_t pkt_1[NET_TEST_MAP_PKT_1_BYTES]  = { NET_TEST_HDR_MAP, 1u, 0u, 0u };
    uint8_t pkt_dup[NET_TEST_MAP_PKT_2_BYTES] =
    {
        NET_TEST_HDR_MAP, 2u, 0u, 0u, 0u
    };
    bool    b_is_ok                          = true;

    /* Valid 1-entry ('A' -> DIR_LEFT), then corrupt padding bit -> fail. */
    robot_tasks_bits_set(&pkt_1[NET_TEST_MAP_DATA_IDX],
                         0u,
                         10u,
                         (((uint32_t)DIR_LEFT) << 7u) | (uint32_t)'A');
    b_is_ok = mqtt_client_decode_map(pkt_1, NET_TEST_MAP_PKT_1_BYTES, map_tbl);
    pkt_1[NET_TEST_MAP_PKT_1_BYTES - 1u] |= 0x80u;
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_map(pkt_1,
                                        NET_TEST_MAP_PKT_1_BYTES,
                                        map_tbl)));

    /* Disallowed JSON quote '"', backslash '\\', and dir=5 fail. */
    pkt_1[NET_TEST_MAP_DATA_IDX]      = 0u;
    pkt_1[NET_TEST_MAP_DATA_IDX + 1u] = 0u;
    /* Widen quote ASCII literal to uint32_t 10-bit wire entry. */
    robot_tasks_bits_set(&pkt_1[NET_TEST_MAP_DATA_IDX],
                         0u,
                         10u,
                         (uint32_t)'"');
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_map(pkt_1,
                                        NET_TEST_MAP_PKT_1_BYTES,
                                        map_tbl)));

    pkt_1[NET_TEST_MAP_DATA_IDX]      = 0u;
    pkt_1[NET_TEST_MAP_DATA_IDX + 1u] = 0u;
    /* Widen backslash ASCII literal to uint32_t 10-bit wire entry. */
    robot_tasks_bits_set(&pkt_1[NET_TEST_MAP_DATA_IDX],
                         0u,
                         10u,
                         (uint32_t)'\\');
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_map(pkt_1,
                                        NET_TEST_MAP_PKT_1_BYTES,
                                        map_tbl)));

    pkt_1[NET_TEST_MAP_DATA_IDX]      = 0u;
    pkt_1[NET_TEST_MAP_DATA_IDX + 1u] = 0u;
    /* Pack invalid direction DIR_INVALID (5) with 'A' into 10-bit entry. */
    robot_tasks_bits_set(&pkt_1[NET_TEST_MAP_DATA_IDX],
                         0u,
                         10u,
                         (((uint32_t)DIR_INVALID) << 7u) | (uint32_t)'A');
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_map(pkt_1,
                                        NET_TEST_MAP_PKT_1_BYTES,
                                        map_tbl)));

    /* Duplicate 'A' and 'a' after case folding -> fail. */
    robot_tasks_bits_set(&pkt_dup[NET_TEST_MAP_DATA_IDX],
                         0u,
                         10u,
                         (uint32_t)'A');
    /* Pack lowercase 'a' into second 10-bit entry to trigger collision. */
    robot_tasks_bits_set(&pkt_dup[NET_TEST_MAP_DATA_IDX],
                         10u,
                         10u,
                         (uint32_t)'a');
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_map(pkt_dup,
                                        NET_TEST_MAP_PKT_2_BYTES,
                                        map_tbl)) &&
               (!mqtt_client_decode_map(pkt_dup,
                                        NET_TEST_MAP_PKT_1_BYTES,
                                        map_tbl)) &&
               (!mqtt_client_decode_map(NULL,
                                        NET_TEST_MAP_PKT_1_BYTES,
                                        map_tbl)));

    return b_is_ok;
}

/**
 * @brief Populate synthetic telemetry snapshot for S5-U04 wire codec test.
 * @param[out] p_snap Telemetry snapshot populated with boundary test values.
 */
static void network_test_fill_telem_snap (navigator_task_telemetry_t * p_snap)
{
    p_snap->seq_num             = 0xA5A55A5Au;
    p_snap->enc_left            = INT32_MIN;
    p_snap->enc_right           = INT32_MAX;
    p_snap->dist_cm             = -42;
    p_snap->speed_left          = -100;
    p_snap->speed_right         = 100;
    p_snap->obs_dist_cm         = 400u;
    p_snap->closest_cm          = 19u;
    /* Narrow 7-bit packed signed angle (-45) into uint16_t bit-field. */
    p_snap->closest_ang_raw     =
        (uint16_t)robot_tasks_pack_signed(-45, NAV_ANGLE_BIT_WIDTH);
    p_snap->est_width_cm        = 800u;
    /* Narrow 12-bit packed signed pitch (-1800) into uint16_t bit-field. */
    p_snap->pitch_ddeg_raw      =
        (uint16_t)robot_tasks_pack_signed(-1800, NAV_TILT_BIT_WIDTH);
    /* Narrow 12-bit packed signed roll (+1800) into uint16_t bit-field. */
    p_snap->roll_ddeg_raw       =
        (uint16_t)robot_tasks_pack_signed(1800, NAV_TILT_BIT_WIDTH);
    p_snap->heading_deg         = 359u;
    p_snap->height_dmm          = 680u;
    /* Store boundary enum values in unsigned telemetry bit-fields. */
    p_snap->movement_mode       = (uint32_t)MODE_SAFE_STOP;
    /* Store BYPASS_RETURN_CHECK enum in 5-bit unsigned field. */
    p_snap->bypass_phase        = (uint32_t)BYPASS_RETURN_CHECK;
    /* Store IR_STATE_B_B enum in 3-bit unsigned field. */
    p_snap->ir_state            = (uint32_t)IR_STATE_B_B;
    /* Store ASCII 'F' in 7-bit unsigned barcode field. */
    p_snap->last_barcode        = (uint32_t)'F';
    p_snap->inst_idx            = 4u;
    p_snap->b_is_obs_dist_valid = 1u;
    /* Store SONAR_OK enum in 3-bit unsigned status field. */
    p_snap->obs_status          = (uint32_t)SONAR_OK;
    p_snap->imu_state           = 3u;
    p_snap->imu_event           = 3u;
    /* Store WIDTH_ESTIMATED enum in 2-bit unsigned width status field. */
    p_snap->obs_width_status    = (uint32_t)WIDTH_ESTIMATED;
}

/**
 * @brief S5-U04: Verify 35-byte 0x11 binary telemetry wire encoding & padding.
 * @return True if all 25 bitfields and 5 zero tail padding bits match.
 */
static bool network_test_check_telem_codec (void)
{
    navigator_task_telemetry_t snap                       = { 0u };
    uint8_t                    wire[MQTT_TELEMETRY_BYTES] = { 0xFFu };
    uint32_t                   all_flags = (1u << ROBOT_FLAG_BITS) - 1u;
    uint32_t                   bit_off   = NET_TEST_TELEM_START_BIT;
    bool                       b_is_ok   = true;

    network_test_fill_telem_snap(&snap);
    mqtt_client_encode_telem(wire, &snap, all_flags);

    b_is_ok = ((b_is_ok) &&
               (network_test_chk_field(wire, &bit_off, 32u, snap.seq_num)));
    /* Verify two's-complement 32-bit signed enc_left, enc_right, dist_cm. */
    b_is_ok = ((b_is_ok) &&
               (network_test_chk_field(wire, &bit_off, 32u,
                                       (uint32_t)snap.enc_left)) &&
               (network_test_chk_field(wire, &bit_off, 32u,
                                       (uint32_t)snap.enc_right)) &&
               (network_test_chk_field(wire, &bit_off, 32u,
                                       (uint32_t)snap.dist_cm)));
    /* Verify 8-bit signed wheel speeds and unsigned sensor bit-fields. */
    b_is_ok = ((b_is_ok) &&
               (network_test_chk_field(wire, &bit_off, 8u,
                                       (uint32_t)snap.speed_left)) &&
               (network_test_chk_field(wire, &bit_off, 8u,
                                       (uint32_t)snap.speed_right)) &&
               (network_test_chk_field(wire, &bit_off, 9u,
                                       snap.obs_dist_cm)) &&
               (network_test_chk_field(wire, &bit_off, 9u,
                                       snap.closest_cm)) &&
               (network_test_chk_field(wire, &bit_off, 7u,
                                       snap.closest_ang_raw)) &&
               (network_test_chk_field(wire, &bit_off, 10u,
                                       snap.est_width_cm)) &&
               (network_test_chk_field(wire, &bit_off, 12u,
                                       snap.pitch_ddeg_raw)) &&
               (network_test_chk_field(wire, &bit_off, 12u,
                                       snap.roll_ddeg_raw)) &&
               (network_test_chk_field(wire, &bit_off, 9u,
                                       snap.heading_deg)) &&
               (network_test_chk_field(wire, &bit_off, 10u,
                                       snap.height_dmm)) &&
               (network_test_chk_field(wire, &bit_off, 4u,
                                       snap.movement_mode)) &&
               (network_test_chk_field(wire, &bit_off, 5u,
                                       snap.bypass_phase)) &&
               (network_test_chk_field(wire, &bit_off, 3u, snap.ir_state)) &&
               (network_test_chk_field(wire, &bit_off, 7u,
                                       snap.last_barcode)) &&
               (network_test_chk_field(wire, &bit_off, 3u, snap.inst_idx)) &&
               (network_test_chk_field(wire, &bit_off, 1u,
                                       snap.b_is_obs_dist_valid)) &&
               (network_test_chk_field(wire, &bit_off, 3u, snap.obs_status)) &&
               (network_test_chk_field(wire, &bit_off, 2u, snap.imu_state)) &&
               (network_test_chk_field(wire, &bit_off, 2u, snap.imu_event)));

    /* Verify header byte, 13-bit flag word, 2-bit width status, and 0 pad. */
    return ((b_is_ok) &&
            (NET_TEST_HDR_TELEM == wire[NET_TEST_CMD_HDR_IDX]) &&
            (NET_TEST_TELEM_END_BIT == (bit_off + ROBOT_FLAG_BITS)) &&
            (all_flags ==
             robot_tasks_bits_get(wire, bit_off, ROBOT_FLAG_BITS)) &&
            ((uint32_t)WIDTH_ESTIMATED ==
             robot_tasks_bits_get(wire, bit_off + ROBOT_FLAG_BITS, 2u)) &&
            (0u == (wire[MQTT_TELEMETRY_BYTES - 1u] &
                    NET_TEST_TELEM_PAD_MASK)));
}

/**
 * @brief S5-U05: Verify bounded JSON char/text/number builders and guard byte.
 * @return True if 0, UINT32_MAX, and tight buffer boundaries never overrun.
 */
static bool network_test_check_json_builders (void)
{
    char     buf[NET_TEST_JSON_BUF_LEN]    = { '\0' };
    char     tiny[NET_TEST_TINY_GUARD_LEN] =
    {
        'X', 'X', 'X', 'X', 'X', '\x7F'
    };
    uint32_t len                           = 0u;
    uint32_t tiny_len                      = 0u;

    /* Cast 32-byte destination buffer sizeof to uint32_t capacity args. */
    mqtt_client_json_text(buf, (uint32_t)sizeof(buf), &len, "v=");
    /* Format zero integer into bounded JSON buffer. */
    mqtt_client_json_number(buf, (uint32_t)sizeof(buf), &len, 0u);
    /* Append comma separator into bounded JSON buffer. */
    mqtt_client_json_char(buf, (uint32_t)sizeof(buf), &len, ',');
    /* Format UINT32_MAX into bounded JSON buffer. */
    mqtt_client_json_number(buf, (uint32_t)sizeof(buf), &len, 4294967295u);
    buf[len] = '\0';

    /* Write 10 chars into 5-byte usable buffer; canary at [5] must hold. */
    mqtt_client_json_text(tiny, 5u, &tiny_len, "0123456789");
    tiny[NET_TEST_TINY_NUL_IDX] = '\0';

    return ((14u == len) &&
            (0 == strcmp(buf, "v=0,4294967295")) &&
            (10u == tiny_len) &&
            (0 == strcmp(tiny, "0123")) &&
            ('\x7F' == tiny[NET_TEST_TINY_CANARY_IDX]));
}

/**
 * @brief S5-U06: Verify /api/mqtt JSON formatter structure and truncation.
 * @return True if output is valid JSON object and small buffers stay safe.
 */
static bool network_test_check_dashboard_json (void)
{
    char     out[NET_TEST_DASH_BUF_LEN]    = { '\0' };
    char     small[NET_TEST_SMALL_BUF_LEN] = { '\0' };
    /* Cast 256-byte buffer sizeof to uint32_t capacity parameter. */
    uint32_t out_len = mqtt_client_dashboard_json(out, (uint32_t)sizeof(out));
    /* Cast 8-byte small buffer sizeof to uint32_t capacity parameter. */
    uint32_t sm_len  =
        mqtt_client_dashboard_json(small, (uint32_t)sizeof(small));

    /* Cast strlen(out) to uint32_t to verify exact JSON length match. */
    return ((0u == mqtt_client_dashboard_json(out, 0u)) &&
            (out_len > 20u) &&
            (out_len == (uint32_t)strlen(out)) &&
            ('{' == out[0]) &&
            ('}' == out[out_len - 1u]) &&
            (NULL != strstr(out, "\"enabled\":")) &&
            (NULL != strstr(out, "\"mapping\":{")) &&
            (7u == sm_len) &&
            ('\0' == small[NET_TEST_SMALL_BUF_LEN - 1u]));
}

/**
 * @brief S5-U07: Verify /api/telemetry JSON formatter and barcode lookup guard.
 * @return True if telemetry JSON is NUL-terminated and invalid lookups fail.
 */
static bool network_test_check_telem_json (void)
{
    char             out[NET_TEST_TELEM_BUF_LEN]   = { '\0' };
    char             tiny[NET_TEST_TINY_TELEM_LEN] = { 'A', 'A', 'A', 'A' };
    motor_task_cmd_t cmd                           = { 0u };
    /* Cast 512-byte buffer sizeof to uint32_t capacity parameter. */
    uint32_t         out_len  =
        mqtt_client_telemetry_json(out, (uint32_t)sizeof(out));
    uint32_t         tiny_len = mqtt_client_telemetry_json(tiny, 2u);

    /* Cast strlen(out) and ASCII 'A' to uint32_t and uint8_t for checks. */
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
            (0 == strncmp(p_body, "<!DOCTYPE html>", NET_TEST_DOCTYPE_LEN)) &&
            ('\0' == p_body[length]));
}

/*** end of file ***/
