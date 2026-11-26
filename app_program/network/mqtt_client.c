/**
 * @file mqtt_client.c
 * @brief Binary MQTT v1 telemetry/command codec, client poller, and JSON API.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel message-buffer/console APIs and lwIP MQTT callbacks
 *   require native RTOS/lwIP typedefs (INT, UINT, UB, u8_t, u16_t, u32_t,
 *   err_t).
 */

#include "mqtt_client.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <tm/tmonitor.h>

#if TM_MQTT
#include "lwip/apps/mqtt.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "mqtt_config.h"
#endif

/* Binary MQTT v1 wire protocol framing constants. */
#define MQTT_HDR_TELEMETRY              (0x11u)
#define MQTT_HDR_COMMAND                (0x12u)
#define MQTT_HDR_MAPPING                (0x13u)
#define MQTT_HDR_STATUS                 (0x14u)
#define MQTT_HDR_BYTE_IDX               (0u)
#define MQTT_CMD_LEFT_IDX               (1u)
#define MQTT_CMD_RIGHT_IDX              (2u)
#define MQTT_CMD_LTICKS_IDX             (3u)
#define MQTT_CMD_RTICKS_IDX             (4u)
#define MQTT_MAP_COUNT_IDX              (1u)
#define MQTT_MAP_DATA_IDX               (2u)
#define MQTT_MAP_MIN_HEADER_LEN         (2u)
#define MQTT_BITS_PER_BYTE              (8u)
#define MQTT_BYTE_ROUND_UP_BITS         (7u)
#define MQTT_CMD_MIN_SPEED              (-100)
#define MQTT_CMD_MAX_SPEED              (100)
#define MQTT_ASCII_TABLE_COUNT          (128u)
#define MQTT_ASCII_MASK_7BIT            (127u)
#define MQTT_ASCII_PRINT_MIN            (33u)
#define MQTT_ASCII_PRINT_MAX            (126u)
#define MQTT_ASCII_SPACE_MIN            (32u)
#define MQTT_ASCII_LOWER_A              ((uint32_t)'a')
#define MQTT_ASCII_LOWER_Z              ((uint32_t)'z')
#define MQTT_ASCII_CASE_DELTA           ((uint32_t)('a' - 'A'))
#define MQTT_MAP_ENTRY_BITS             (10u)
#define MQTT_MAP_DIR_SHIFT              (7u)
#define MQTT_MAP_DIR_BITS               (3u)
#define MQTT_TELEM_START_BIT            (8u)
#define MQTT_TELEM_U32_BITS             (32u)
#define MQTT_TELEM_DIST_BITS            (9u)
#define MQTT_TELEM_EST_WIDTH_BITS       (10u)
#define MQTT_TELEM_HEIGHT_BITS          (10u)
#define MQTT_TELEM_MODE_BITS            (4u)
#define MQTT_TELEM_PHASE_BITS           (5u)
#define MQTT_TELEM_IR_BITS              (3u)
#define MQTT_TELEM_BARCODE_BITS         (7u)
#define MQTT_TELEM_INST_BITS            (3u)
#define MQTT_TELEM_VALID_BITS           (1u)
#define MQTT_TELEM_SONAR_ST_BITS        (3u)
#define MQTT_TELEM_IMU_BITS             (2u)
#define MQTT_TELEM_WIDTH_ST_BITS        (2u)
#define MQTT_TELEM_FLAGS_BIT_END        (273u)
#define MQTT_TELEM_PAD_MASK             (0xF8u)
#define MQTT_JSON_DIGITS_MAX            (11u)
#define MQTT_DECIMAL_RADIX              (10u)
#define MQTT_HEX_NIBBLE_SHIFT           (4u)
#define MQTT_HEX_NIBBLE_MASK            (0x0Fu)
#define MQTT_CARDINAL_COUNT             (8u)
#define MQTT_CARDINAL_HALF_STEP         (22u)
#define MQTT_CARDINAL_STEP_DEG          (45u)
#define MQTT_CARDINAL_MASK              (7u)
#define MQTT_DEGREES_FULL               (360)
#define MQTT_TEST_MAP_PKT_LEN           (4u)
#define MQTT_TEST_BAD_PAD_BIT           (0x80u)
#define MQTT_FULL_U32_MASK              (0xFFFFFFFFu)

#if TM_MQTT
#define MQTT_RETRY_MSEC                 (5000u)
#define MQTT_TOPIC_MAX                  (96u)
#define MQTT_MOVE_CFG_TOPIC_DEF         "robot/config/movement"
#define MQTT_STATUS_TOPIC_DEF           "robot/status"
#define MQTT_STATUS_INTERVAL_MSEC       (5000u)
#define MQTT_TELEM_INTERVAL_MSEC        (500u)
#define MQTT_KEEP_ALIVE_SECONDS         (30u)
#define MQTT_BARCODE_DRIVE_PCT          (40)
#define MQTT_STATUS_ONLINE_BIT          (8u)
#define MQTT_STATUS_UPTIME_BIT          (9u)
#define MQTT_STATUS_SEQ_BIT             (41u)
#define MQTT_USEC_PER_MSEC_U64          (1000u)

#ifndef MQTT_MOVEMENT_CONFIG_TOPIC
#define MQTT_MOVEMENT_CONFIG_TOPIC      MQTT_MOVE_CFG_TOPIC_DEF
#endif

#ifndef MQTT_STATUS_TOPIC
#define MQTT_STATUS_TOPIC               MQTT_STATUS_TOPIC_DEF
#endif

typedef struct mqtt_connect_client_info_t mqtt_client_conn_info_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t b_is_configured    : 1;
    uint32_t b_is_pending       : 1;
    uint32_t b_is_connected     : 1;
    uint32_t b_is_payload_valid : 1;
    uint32_t payload_length     : 7;
    uint32_t payload_expected   : 7;
} mqtt_client_runtime_t;

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t b_is_ready : 1;
    uint8_t  directions[MQTT_MAP_BYTES];
} mqtt_client_map_t;

ROBOT_TASKS_STATIC_ASSERT(mqtt_runtime_size,
                          (3u == sizeof(mqtt_client_runtime_t)));
ROBOT_TASKS_STATIC_ASSERT(mqtt_map_size,
                          ((MQTT_MAP_BYTES + 1u) == sizeof(mqtt_client_map_t)));

extern uint64_t time_us_64(void);

static const uint8_t g_offline_will[MQTT_STATUS_BYTES] = { MQTT_HDR_STATUS };

static mqtt_client_t *           gp_mqtt_client                   = NULL;
static ip_addr_t                 g_mqtt_broker                    = { 0 };
static mqtt_client_conn_info_t   g_mqtt_info                      = { 0 };
static char                      g_mqtt_topic[MQTT_TOPIC_MAX]     = { '\0' };
static uint8_t                   g_mqtt_payload[MQTT_PAYLOAD_MAX] = { 0u };
static mqtt_client_runtime_t     g_mqtt_state                     = { 0u };
static uint32_t                  g_mqtt_msg_cnt                   = 0u;
static uint32_t                  g_mqtt_next_retry                = 0u;
static uint32_t                  g_mqtt_next_telem                = 0u;
static uint32_t                  g_mqtt_next_status               = 0u;
static int32_t                   g_mqtt_last_error                = 0;
static volatile mqtt_client_map_t g_movement_map                  = { 0u };
#endif /* TM_MQTT */

static bool mqtt_decode_map_entries(const uint8_t * p_bytes,
                                    uint32_t count,
                                    uint8_t * p_map_buf);
static void mqtt_put_field(uint8_t * p_bytes,
                           uint32_t * p_offset,
                           uint32_t bit_width,
                           uint32_t field_val);
#ifndef NDEBUG
static bool mqtt_chk_field(const uint8_t * p_bytes,
                           uint32_t * p_offset,
                           uint32_t bit_width,
                           uint32_t expect_val);
static void mqtt_fill_test_telem_snap(navigator_task_telemetry_t * p_snap);
static bool mqtt_check_telem_frame(void);
#endif

#if TM_MQTT
static void mqtt_json_barcode_char(char * p_output,
                                   uint32_t capacity,
                                   uint32_t * p_length,
                                   uint32_t barcode_code);
static void mqtt_json_append_sensors(
    char * p_output,
    uint32_t capacity,
    uint32_t * p_length,
    const navigator_task_telemetry_t * p_snap);
static void mqtt_poll_connected(uint32_t now_msec);
static void mqtt_json_append_map(char * p_output,
                                 uint32_t capacity,
                                 uint32_t * p_length,
                                 const uint8_t * p_map_copy,
                                 bool b_is_ready);
static bool mqtt_build_move_cmd(navigator_task_dir_t dir,
                                motor_task_cmd_t * p_cmd);
static const char * mqtt_movement_name(navigator_task_dir_t dir);
static void mqtt_forward_map_entries(const uint8_t * p_next_map);
static bool mqtt_update_move_map(const uint8_t * p_payload, uint32_t length);
static uint32_t mqtt_now_msec(void);
static void mqtt_on_request_done(void * p_arg, err_t err_code);
static void mqtt_dispatch_remote_cmd(void);
static void mqtt_dispatch_complete_msg(void);
static void mqtt_append_rx_chunk(const u8_t * p_data,
                                 u16_t length,
                                 u8_t flags);
static void mqtt_on_data(void * p_arg,
                         const u8_t * p_data,
                         u16_t length,
                         u8_t flags);
static void mqtt_on_incoming_pub(void * p_arg,
                                 const char * p_topic,
                                 u32_t total_length);
static void mqtt_on_connection(mqtt_client_t * p_client,
                               void * p_arg,
                               mqtt_connection_status_t status);
static void mqtt_begin_connect(void);
static void mqtt_json_signed(char * p_output,
                             uint32_t capacity,
                             uint32_t * p_length,
                             int32_t num_val);
static void mqtt_append_tenths(char * p_output,
                               uint32_t capacity,
                               uint32_t * p_length,
                               int16_t tenths_val);
static void mqtt_json_quoted(char * p_output,
                             uint32_t capacity,
                             uint32_t * p_length,
                             const char * p_text);
static const char * mqtt_heading_cardinal(int16_t deg_val);
#endif /* TM_MQTT */

/**
 * @brief Decode a 5-byte binary MQTT v1 motor command (0x12).
 * @param[in]  p_bytes Source frame bytes.
 * @param[in]  length  Frame length in bytes.
 * @param[out] p_cmd   Decoded motor command structure.
 * @return True if valid 0x12 command, false otherwise.
 */
bool mqtt_client_decode_cmd (const uint8_t * p_bytes,
                             uint32_t length,
                             motor_task_cmd_t * p_cmd)
{
    bool b_is_valid = false;

    if ((NULL != p_bytes) &&
        (NULL != p_cmd) &&
        (MQTT_COMMAND_BYTES == length) &&
        (MQTT_HDR_COMMAND == p_bytes[MQTT_HDR_BYTE_IDX]))
    {
        /* Widen 8-bit two's-complement left speed byte for sign extension. */
        int32_t left_spd  = robot_tasks_sign_extend(
            (uint32_t)p_bytes[MQTT_CMD_LEFT_IDX], MQTT_BITS_PER_BYTE);
        /* Widen 8-bit two's-complement right speed byte for sign extension. */
        int32_t right_spd = robot_tasks_sign_extend(
            (uint32_t)p_bytes[MQTT_CMD_RIGHT_IDX], MQTT_BITS_PER_BYTE);

        if ((left_spd >= MQTT_CMD_MIN_SPEED) &&
            (left_spd <= MQTT_CMD_MAX_SPEED) &&
            (right_spd >= MQTT_CMD_MIN_SPEED) &&
            (right_spd <= MQTT_CMD_MAX_SPEED))
        {
            p_cmd->raw                       = 0u;
            /* Range-safe: left_spd is validated in [-100, 100]. */
            p_cmd->fields.target_speed_left  = (int8_t)left_spd;
            /* Range-safe: right_spd is validated in [-100, 100]. */
            p_cmd->fields.target_speed_right = (int8_t)right_spd;
            p_cmd->fields.turn_ticks_left    = p_bytes[MQTT_CMD_LTICKS_IDX];
            p_cmd->fields.turn_ticks_right   = p_bytes[MQTT_CMD_RTICKS_IDX];
            b_is_valid                       = true;
        }
    }

    return b_is_valid;
}

/**
 * @brief Decode a binary MQTT v1 barcode-to-direction mapping (0x13).
 * @param[in]  p_bytes   Source frame bytes.
 * @param[in]  length    Frame length in bytes.
 * @param[out] p_map_buf Output 48-byte packed 3-bit-per-ASCII direction table.
 * @return True if valid 0x13 mapping, false otherwise.
 */
bool mqtt_client_decode_map (const uint8_t * p_bytes,
                             uint32_t length,
                             uint8_t * p_map_buf)
{
    bool b_is_valid = false;

    if ((NULL != p_bytes) &&
        (NULL != p_map_buf) &&
        (length >= MQTT_MAP_MIN_HEADER_LEN) &&
        (MQTT_HDR_MAPPING == p_bytes[MQTT_HDR_BYTE_IDX]) &&
        (0u != p_bytes[MQTT_MAP_COUNT_IDX]) &&
        (p_bytes[MQTT_MAP_COUNT_IDX] <= MQTT_MAPPING_MAX))
    {
        /* Widen 8-bit entry count (1..66) to uint32_t for bit math. */
        uint32_t count      = (uint32_t)p_bytes[MQTT_MAP_COUNT_IDX];
        uint32_t total_bits = count * MQTT_MAP_ENTRY_BITS;
        uint32_t expect_len = MQTT_MAP_MIN_HEADER_LEN +
                              ((total_bits + MQTT_BYTE_ROUND_UP_BITS) /
                               MQTT_BITS_PER_BYTE);
        uint32_t used_bits  = total_bits % MQTT_BITS_PER_BYTE;
        /* Widen final frame byte to verify unused tail padding bits are 0. */
        bool     b_is_pad_ok =
            ((0u == used_bits) ||
             (0u == (((uint32_t)p_bytes[length - 1u]) >> used_bits)));

        if ((expect_len == length) && (b_is_pad_ok))
        {
            b_is_valid = mqtt_decode_map_entries(p_bytes, count, p_map_buf);
        }
    }

    return b_is_valid;
}

/**
 * @brief Encode a telemetry snapshot and flag word into a 35-byte 0x11 frame.
 * @param[out] p_out_bytes Destination 35-byte wire buffer.
 * @param[in]  p_snapshot  Telemetry snapshot to serialize.
 * @param[in]  flags_word  Shared 13-bit robot fault/control word.
 */
void mqtt_client_encode_telem (uint8_t * p_out_bytes,
                               const navigator_task_telemetry_t * p_snapshot,
                               uint32_t flags_word)
{
    uint32_t bit_off = MQTT_TELEM_START_BIT;

    /* Zero all 35 wire bytes including trailing 5 zero padding bits. */
    (void)memset(p_out_bytes, 0, MQTT_TELEMETRY_BYTES);
    p_out_bytes[MQTT_HDR_BYTE_IDX] = MQTT_HDR_TELEMETRY;

    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_U32_BITS, p_snapshot->seq_num);
    /* Modulo-2^32 unsigned cast preserves 32-bit two's-complement wire bits. */
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_U32_BITS, (uint32_t)p_snapshot->enc_left);
    /* Modulo-2^32 unsigned cast preserves right encoder wire bits. */
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_U32_BITS, (uint32_t)p_snapshot->enc_right);
    /* Modulo-2^32 unsigned cast preserves signed distance wire bits. */
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_U32_BITS, (uint32_t)p_snapshot->dist_cm);
    /* Modulo-2^32 unsigned cast preserves 8-bit signed left speed bits. */
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_BITS_PER_BYTE, (uint32_t)p_snapshot->speed_left);
    /* Modulo-2^32 unsigned cast preserves 8-bit signed right speed bits. */
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_BITS_PER_BYTE, (uint32_t)p_snapshot->speed_right);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_DIST_BITS, p_snapshot->obs_dist_cm);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_DIST_BITS, p_snapshot->closest_cm);
    mqtt_put_field(p_out_bytes, &bit_off,
                   NAV_ANGLE_BIT_WIDTH, p_snapshot->closest_ang_raw);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_EST_WIDTH_BITS, p_snapshot->est_width_cm);
    mqtt_put_field(p_out_bytes, &bit_off,
                   NAV_TILT_BIT_WIDTH, p_snapshot->pitch_ddeg_raw);
    mqtt_put_field(p_out_bytes, &bit_off,
                   NAV_TILT_BIT_WIDTH, p_snapshot->roll_ddeg_raw);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_DIST_BITS, p_snapshot->heading_deg);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_HEIGHT_BITS, p_snapshot->height_dmm);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_MODE_BITS, p_snapshot->movement_mode);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_PHASE_BITS, p_snapshot->bypass_phase);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_IR_BITS, p_snapshot->ir_state);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_BARCODE_BITS, p_snapshot->last_barcode);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_INST_BITS, p_snapshot->inst_idx);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_VALID_BITS, p_snapshot->b_is_obs_dist_valid);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_SONAR_ST_BITS, p_snapshot->obs_status);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_IMU_BITS, p_snapshot->imu_state);
    mqtt_put_field(p_out_bytes, &bit_off,
                   MQTT_TELEM_IMU_BITS, p_snapshot->imu_event);

    robot_tasks_bits_set(p_out_bytes, bit_off, ROBOT_FLAG_BITS, flags_word);
    robot_tasks_bits_set(p_out_bytes,
                         bit_off + ROBOT_FLAG_BITS,
                         MQTT_TELEM_WIDTH_ST_BITS,
                         p_snapshot->obs_width_status);
}

#ifndef NDEBUG
/**
 * @brief Run debug-only self-checks on MQTT v1 command, map, and telem codecs.
 * @return True if all wire codec checks pass, false otherwise.
 */
bool mqtt_client_codec_check (void)
{
    /* Byte 156u encodes -100 in 8-bit two's-complement wire format. */
    uint8_t          cmd_buf[MQTT_COMMAND_BYTES]    =
    {
        MQTT_HDR_COMMAND, 100u, (uint8_t)156u, 255u, 7u
    };
    motor_task_cmd_t decoded                        = { 0u };
    uint8_t          map_pkt[MQTT_TEST_MAP_PKT_LEN] =
    {
        MQTT_HDR_MAPPING, 1u, 0x41u, 0x01u
    };
    uint8_t          map_tbl[MQTT_MAP_BYTES]        = { 0u };
    bool             b_is_ok                        = true;

    /* Cast static 5-byte command buffer sizeof to uint32_t length argument. */
    b_is_ok = ((b_is_ok) &&
               (mqtt_client_decode_cmd(cmd_buf,
                                       (uint32_t)sizeof(cmd_buf),
                                       &decoded)) &&
               (100 == decoded.fields.target_speed_left) &&
               (-100 == decoded.fields.target_speed_right) &&
               (255u == decoded.fields.turn_ticks_left) &&
               (7u == decoded.fields.turn_ticks_right));

    cmd_buf[MQTT_CMD_LEFT_IDX] = 101u;
    /* Verify out-of-range speed (+101) and truncated length are rejected. */
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_cmd(cmd_buf,
                                        (uint32_t)sizeof(cmd_buf),
                                        &decoded)) &&
               (!mqtt_client_decode_cmd(cmd_buf,
                                        ((uint32_t)sizeof(cmd_buf)) - 1u,
                                        &decoded)));

    /* Verify 'A' -> DIR_LEFT and unmapped 'B' -> DIR_INVALID. */
    b_is_ok = ((b_is_ok) &&
               (mqtt_client_decode_map(map_pkt,
                                       (uint32_t)sizeof(map_pkt),
                                       map_tbl)) &&
               ((uint32_t)DIR_LEFT ==
                robot_tasks_bits_get(map_tbl,
                                     ((uint32_t)'A') * MQTT_MAP_DIR_BITS,
                                     MQTT_MAP_DIR_BITS)) &&
               ((uint32_t)DIR_INVALID ==
                robot_tasks_bits_get(map_tbl,
                                     ((uint32_t)'B') * MQTT_MAP_DIR_BITS,
                                     MQTT_MAP_DIR_BITS)));

    map_pkt[MQTT_TEST_MAP_PKT_LEN - 1u] |= MQTT_TEST_BAD_PAD_BIT;
    /* Verify non-zero tail padding bit is rejected and check 0x11 frame. */
    b_is_ok = ((b_is_ok) &&
               (!mqtt_client_decode_map(map_pkt,
                                        (uint32_t)sizeof(map_pkt),
                                        map_tbl)) &&
               (mqtt_check_telem_frame()));

    return b_is_ok;
}
#endif /* NDEBUG */

/**
 * @brief Append one ASCII character to a bounded JSON output buffer.
 * @param[out]    p_output Destination character buffer.
 * @param[in]     capacity Total buffer capacity in bytes.
 * @param[in,out] p_length Running character count (incremented even if full).
 * @param[in]     ch_val   Character to append.
 */
void mqtt_client_json_char (char * p_output,
                            uint32_t capacity,
                            uint32_t * p_length,
                            char ch_val)
{
    if (((*p_length) + 1u) < capacity)
    {
        p_output[*p_length] = ch_val;
    }

    (*p_length)++;
}

/**
 * @brief Append a NUL-terminated string to a bounded JSON output buffer.
 * @param[out]    p_output Destination character buffer.
 * @param[in]     capacity Total buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     p_text   NUL-terminated string to append.
 */
void mqtt_client_json_text (char * p_output,
                            uint32_t capacity,
                            uint32_t * p_length,
                            const char * p_text)
{
    const char * p_cur = p_text;

    while ('\0' != (*p_cur))
    {
        mqtt_client_json_char(p_output, capacity, p_length, *p_cur);
        p_cur++;
    }
}

/**
 * @brief Append an unsigned decimal integer to a bounded JSON output buffer.
 * @param[out]    p_output Destination character buffer.
 * @param[in]     capacity Total buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     num_val  Unsigned 32-bit integer value to format.
 */
void mqtt_client_json_number (char * p_output,
                              uint32_t capacity,
                              uint32_t * p_length,
                              uint32_t num_val)
{
    char     digits[MQTT_JSON_DIGITS_MAX] = { '\0' };
    uint32_t digit_cnt                    = 0u;
    uint32_t rem_val                      = num_val;

    if (0u == rem_val)
    {
        digits[0] = '0';
        digit_cnt = 1u;
    }
    else
    {
        while ((0u != rem_val) && (digit_cnt < MQTT_JSON_DIGITS_MAX))
        {
            /* Range-safe: rem_val % 10u is 0..9, mapping to '0'..'9'. */
            digits[digit_cnt] =
                (char)(((uint32_t)'0') + (rem_val % MQTT_DECIMAL_RADIX));
            digit_cnt++;
            rem_val /= MQTT_DECIMAL_RADIX;
        }
    }

    while (digit_cnt > 0u)
    {
        digit_cnt--;
        mqtt_client_json_char(p_output, capacity, p_length, digits[digit_cnt]);
    }
}

#if TM_MQTT
/**
 * @brief Look up a motor command mapped to a decoded barcode ASCII character.
 * @param[in]  barcode_ch ASCII barcode character.
 * @param[out] p_cmd      Output motor command if mapped.
 * @return True if barcode_ch has a valid mapped command, false otherwise.
 */
bool mqtt_client_get_barcode_cmd (uint8_t barcode_ch, motor_task_cmd_t * p_cmd)
{
    /* Widen 8-bit ASCII barcode byte to uint32_t table index. */
    uint32_t key_code = (uint32_t)barcode_ch;
    bool     b_is_ok  = false;

    if ((key_code >= MQTT_ASCII_LOWER_A) && (key_code <= MQTT_ASCII_LOWER_Z))
    {
        key_code -= MQTT_ASCII_CASE_DELTA;
    }

    if ((NULL != p_cmd) &&
        (key_code >= MQTT_ASCII_PRINT_MIN) &&
        (key_code <= MQTT_ASCII_PRINT_MAX))
    {
        UINT                 saved_state = robot_tasks_data_lock();
        navigator_task_dir_t dir_val     = DIR_INVALID;

        if (0u != g_movement_map.b_is_ready)
        {
            uint8_t map_copy[MQTT_MAP_BYTES] = { 0u };

            for (uint32_t idx = 0u; idx < MQTT_MAP_BYTES; idx++)
            {
                map_copy[idx] = g_movement_map.directions[idx];
            }

            /* Range-safe: stored map values are 0..DIR_INVALID. */
            dir_val = (navigator_task_dir_t)robot_tasks_bits_get(
                map_copy, key_code * MQTT_MAP_DIR_BITS, MQTT_MAP_DIR_BITS);
        }

        robot_tasks_data_unlock(saved_state);
        b_is_ok = mqtt_build_move_cmd(dir_val, p_cmd);
    }

    return b_is_ok;
}

/**
 * @brief Format current shared telemetry snapshot as JSON for HTTP /api.
 * @param[out] p_output Destination character buffer.
 * @param[in]  capacity Buffer capacity in bytes (including NUL terminator).
 * @return Length of formatted JSON string excluding NUL terminator.
 */
uint32_t mqtt_client_telemetry_json (char * p_output, uint32_t capacity)
{
    uint32_t                   length = 0u;
    navigator_task_telemetry_t snap   = { 0u };

    navigator_task_read_telem(&snap);

    if (0u != capacity)
    {
        mqtt_client_json_char(p_output, capacity, &length, '{');

        mqtt_client_json_text(p_output, capacity, &length, "\"mode\":");
        mqtt_client_json_number(p_output,
                                capacity,
                                &length,
                                snap.movement_mode);
        mqtt_client_json_text(p_output, capacity, &length, ",\"phase\":");
        mqtt_client_json_number(p_output, capacity, &length, snap.bypass_phase);
        mqtt_client_json_text(p_output, capacity, &length, ",\"ir\":");
        mqtt_client_json_number(p_output, capacity, &length, snap.ir_state);

        mqtt_client_json_text(p_output, capacity, &length, ",\"barcode\":\"");
        mqtt_json_barcode_char(p_output, capacity, &length, snap.last_barcode);
        mqtt_client_json_text(p_output, capacity, &length, "\"");
        mqtt_client_json_text(p_output, capacity, &length, ",\"inst_idx\":");
        mqtt_client_json_number(p_output, capacity, &length, snap.inst_idx);

        mqtt_client_json_text(p_output, capacity, &length, ",\"speed_l\":");
        /* Widen signed 8-bit left speed to int32_t for JSON formatting. */
        mqtt_json_signed(p_output,
                         capacity,
                         &length,
                         (int32_t)snap.speed_left);
        mqtt_client_json_text(p_output, capacity, &length, ",\"speed_r\":");
        /* Widen signed 8-bit right speed to int32_t for JSON formatting. */
        mqtt_json_signed(p_output,
                         capacity,
                         &length,
                         (int32_t)snap.speed_right);
        mqtt_client_json_text(p_output, capacity, &length, ",\"enc_l\":");
        mqtt_json_signed(p_output, capacity, &length, snap.enc_left);
        mqtt_client_json_text(p_output, capacity, &length, ",\"enc_r\":");
        mqtt_json_signed(p_output, capacity, &length, snap.enc_right);
        mqtt_client_json_text(p_output, capacity, &length, ",\"dist_cm\":");
        mqtt_json_signed(p_output, capacity, &length, snap.dist_cm);

        mqtt_json_append_sensors(p_output, capacity, &length, &snap);

        mqtt_client_json_text(p_output, capacity, &length, ",\"seq\":");
        mqtt_client_json_number(p_output, capacity, &length, snap.seq_num);
        mqtt_client_json_char(p_output, capacity, &length, '}');

        if (length >= capacity)
        {
            length = capacity - 1u;
        }

        p_output[length] = '\0';
    }

    return length;
}

/**
 * @brief Poll lwIP MQTT client connection, telemetry publish, and keepalive.
 * @param[in] p_netif Active lwIP network interface.
 */
void mqtt_client_poll (mqtt_client_netif_t * p_netif)
{
    bool b_is_link_up = ((NULL != p_netif) &&
                         (0u != netif_is_up(p_netif)) &&
                         (0u != netif_is_link_up(p_netif)));

    robot_tasks_flag_assign(ROBOT_FAULT_WIFI, !b_is_link_up);
    robot_tasks_flag_assign(ROBOT_FAULT_MQTT,
                            (!b_is_link_up) ||
                            (0u == g_mqtt_state.b_is_connected));

    if (b_is_link_up)
    {
        uint32_t now_msec = mqtt_now_msec();

        /* Modular signed difference handles 32-bit retry timer wraparound. */
        if ((0u == g_mqtt_state.b_is_connected) &&
            (0u == g_mqtt_state.b_is_pending) &&
            (((int32_t)(now_msec - g_mqtt_next_retry)) >= 0))
        {
            mqtt_begin_connect();
        }
        else if (0u != g_mqtt_state.b_is_connected)
        {
            mqtt_poll_connected(now_msec);
        }
        else
        {
            /* Retry backoff or connection handshake in progress. */
        }
    }
}

/**
 * @brief Publish offline will status and disconnect on Wi-Fi link loss.
 */
void mqtt_client_link_down (void)
{
    if (NULL != gp_mqtt_client)
    {
        if (0u != g_mqtt_state.b_is_connected)
        {
            /* Best-effort offline LWT publish; cast 10-byte size to u16_t. */
            (void)mqtt_publish(gp_mqtt_client,
                               MQTT_STATUS_TOPIC,
                               g_offline_will,
                               (u16_t)sizeof(g_offline_will),
                               MQTT_QOS,
                               1u,
                               mqtt_on_request_done,
                               NULL);
        }

        mqtt_disconnect(gp_mqtt_client);
    }

    g_mqtt_state.b_is_connected     = 0u;
    g_mqtt_state.b_is_pending       = 0u;
    g_mqtt_state.b_is_payload_valid = 0u;
    robot_tasks_flags_update(ROBOT_FAULT_WIFI | ROBOT_FAULT_MQTT, 0u);
    g_mqtt_next_retry               = mqtt_now_msec() + MQTT_RETRY_MSEC;
}

/**
 * @brief Format MQTT status and decoded barcode mapping table as JSON.
 * @param[out] p_output Destination character buffer.
 * @param[in]  capacity Buffer capacity in bytes (including NUL terminator).
 * @return Length of formatted JSON string excluding NUL terminator.
 */
uint32_t mqtt_client_dashboard_json (char * p_output, uint32_t capacity)
{
    static const char hex_digits[]             = "0123456789ABCDEF";
    uint32_t          length                   = 0u;
    uint8_t           map_copy[MQTT_MAP_BYTES] = { 0u };
    bool              b_is_ready               = false;
    UINT              saved_state              = robot_tasks_data_lock();

    b_is_ready = (0u != g_movement_map.b_is_ready);

    for (uint32_t idx = 0u; idx < MQTT_MAP_BYTES; idx++)
    {
        map_copy[idx] = g_movement_map.directions[idx];
    }

    robot_tasks_data_unlock(saved_state);

    if (0u != capacity)
    {
        const char * p_conn =
            (0u != g_mqtt_state.b_is_connected) ? "true" : "false";

        mqtt_client_json_char(p_output, capacity, &length, '{');
        mqtt_client_json_text(p_output,
                              capacity,
                              &length,
                              "\"enabled\":true,\"connected\":");
        mqtt_client_json_text(p_output, capacity, &length, p_conn);
        mqtt_client_json_text(p_output, capacity, &length, ",\"messages\":");
        mqtt_client_json_number(p_output, capacity, &length, g_mqtt_msg_cnt);
        mqtt_client_json_text(p_output, capacity, &length, ",\"topic\":\"");
        mqtt_json_quoted(p_output, capacity, &length, g_mqtt_topic);
        mqtt_client_json_text(p_output, capacity, &length, "\",\"payload\":\"");

        for (uint32_t idx = 0u; idx < g_mqtt_state.payload_length; idx++)
        {
            if (0u != idx)
            {
                mqtt_client_json_char(p_output, capacity, &length, ' ');
            }

            mqtt_client_json_char(
                p_output,
                capacity,
                &length,
                hex_digits[g_mqtt_payload[idx] >> MQTT_HEX_NIBBLE_SHIFT]);
            mqtt_client_json_char(
                p_output,
                capacity,
                &length,
                hex_digits[g_mqtt_payload[idx] & MQTT_HEX_NIBBLE_MASK]);
        }

        mqtt_client_json_text(p_output, capacity, &length, "\",\"error\":");
        mqtt_json_signed(p_output, capacity, &length, g_mqtt_last_error);
        mqtt_client_json_text(p_output, capacity, &length, ",\"mapping\":{");
        mqtt_json_append_map(p_output,
                             capacity,
                             &length,
                             map_copy,
                             b_is_ready);
        mqtt_client_json_char(p_output, capacity, &length, '}');
        mqtt_client_json_char(p_output, capacity, &length, '}');

        if (length >= capacity)
        {
            length = capacity - 1u;
        }

        p_output[length] = '\0';
    }

    return length;
}

#else

/**
 * @brief No-op link-down stub when TM_MQTT is disabled.
 */
void mqtt_client_link_down (void)
{
    /* No MQTT client active when TM_MQTT is zero. */
}

/**
 * @brief No-op poll stub when TM_MQTT is disabled.
 * @param[in] p_netif Network interface pointer (unused).
 */
void mqtt_client_poll (mqtt_client_netif_t * p_netif)
{
    /* Explicitly ignore network interface when TM_MQTT is disabled. */
    (void)p_netif;
}

/**
 * @brief Format static disabled MQTT status JSON when TM_MQTT is disabled.
 * @param[out] p_output Destination buffer.
 * @param[in]  capacity Buffer capacity in bytes.
 * @return Output string length excluding NUL terminator.
 */
uint32_t mqtt_client_dashboard_json (char * p_output, uint32_t capacity)
{
    static const char disabled_json[] =
        "{\"enabled\":false,\"connected\":false,\"messages\":0,"
        "\"topic\":\"\",\"payload\":\"\",\"error\":0,\"mapping\":{}}";
    uint32_t          length          = 0u;

    if (0u != capacity)
    {
        /* Cast static JSON literal sizeof to uint32_t length excluding NUL. */
        length = ((uint32_t)sizeof(disabled_json)) - 1u;

        if (length >= capacity)
        {
            length = capacity - 1u;
        }

        /* Copy bounded JSON body into caller buffer. */
        (void)memcpy(p_output, disabled_json, length);
        p_output[length] = '\0';
    }

    return length;
}

/**
 * @brief Format empty JSON telemetry object when TM_MQTT is disabled.
 * @param[out] p_output Destination buffer.
 * @param[in]  capacity Buffer capacity in bytes.
 * @return Output string length excluding NUL terminator.
 */
uint32_t mqtt_client_telemetry_json (char * p_output, uint32_t capacity)
{
    static const char empty_json[] = "{}";
    uint32_t          length       = 0u;

    if (0u != capacity)
    {
        /* Cast static "{}" literal sizeof to uint32_t length excluding NUL. */
        length = ((uint32_t)sizeof(empty_json)) - 1u;

        if (length >= capacity)
        {
            length = capacity - 1u;
        }

        /* Copy bounded "{}" JSON body into caller buffer. */
        (void)memcpy(p_output, empty_json, length);
        p_output[length] = '\0';
    }

    return length;
}

/**
 * @brief Always return false for barcode lookup when TM_MQTT is disabled.
 * @param[in]  barcode_ch Barcode character (unused).
 * @param[out] p_cmd      Motor command pointer (unused).
 * @return Always false.
 */
bool mqtt_client_get_barcode_cmd (uint8_t barcode_ch, motor_task_cmd_t * p_cmd)
{
    /* Explicitly ignore barcode character when TM_MQTT is disabled. */
    (void)barcode_ch;
    /* Explicitly ignore output command pointer when TM_MQTT is disabled. */
    (void)p_cmd;

    return false;
}

#endif /* TM_MQTT */

/**
 * @brief Populate the 48-byte barcode mapping table from 10-bit wire entries.
 * @param[in]  p_bytes   Wire frame starting with header and entry count.
 * @param[in]  count     Number of 10-bit entries in the payload.
 * @param[out] p_map_buf Output 48-byte packed 3-bit-per-ASCII direction table.
 * @return True if all entries are valid and unique, false otherwise.
 */
static bool mqtt_decode_map_entries (const uint8_t * p_bytes,
                                     uint32_t count,
                                     uint8_t * p_map_buf)
{
    bool b_is_valid = true;

    /* Clear 48-byte table before initializing all 128 slots to DIR_INVALID. */
    (void)memset(p_map_buf, 0, MQTT_MAP_BYTES);

    for (uint32_t key_idx = 0u; key_idx < MQTT_ASCII_TABLE_COUNT; key_idx++)
    {
        /* Store 3-bit DIR_INVALID sentinel in every ASCII slot. */
        robot_tasks_bits_set(p_map_buf,
                             key_idx * MQTT_MAP_DIR_BITS,
                             MQTT_MAP_DIR_BITS,
                             (uint32_t)DIR_INVALID);
    }

    for (uint32_t entry_idx = 0u;
         (entry_idx < count) && (b_is_valid);
         entry_idx++)
    {
        uint32_t entry    = robot_tasks_bits_get(
            &p_bytes[MQTT_MAP_DATA_IDX],
            entry_idx * MQTT_MAP_ENTRY_BITS,
            MQTT_MAP_ENTRY_BITS);
        uint32_t key_code = entry & MQTT_ASCII_MASK_7BIT;
        uint32_t dir_val  = entry >> MQTT_MAP_DIR_SHIFT;

        if ((key_code >= MQTT_ASCII_LOWER_A) &&
            (key_code <= MQTT_ASCII_LOWER_Z))
        {
            key_code -= MQTT_ASCII_CASE_DELTA;
        }

        /* Reject non-printable, JSON-unsafe, invalid-dir, or duplicate keys. */
        if ((key_code < MQTT_ASCII_PRINT_MIN) ||
            (key_code > MQTT_ASCII_PRINT_MAX) ||
            ((uint32_t)'"' == key_code) ||
            ((uint32_t)'\\' == key_code) ||
            (dir_val > (uint32_t)DIR_STOP) ||
            ((uint32_t)DIR_INVALID !=
             robot_tasks_bits_get(p_map_buf,
                                  key_code * MQTT_MAP_DIR_BITS,
                                  MQTT_MAP_DIR_BITS)))
        {
            b_is_valid = false;
        }
        else
        {
            robot_tasks_bits_set(p_map_buf,
                                 key_code * MQTT_MAP_DIR_BITS,
                                 MQTT_MAP_DIR_BITS,
                                 dir_val);
        }
    }

    return b_is_valid;
}

/**
 * @brief Write one telemetry field into the little-endian wire bit stream.
 * @param[out]    p_bytes   Destination frame buffer.
 * @param[in,out] p_offset  Running bit offset advanced by bit_width.
 * @param[in]     bit_width Field width in bits (1..32).
 * @param[in]     field_val Unsigned field bit pattern.
 */
static void mqtt_put_field (uint8_t * p_bytes,
                            uint32_t * p_offset,
                            uint32_t bit_width,
                            uint32_t field_val)
{
    robot_tasks_bits_set(p_bytes, *p_offset, bit_width, field_val);
    *p_offset += bit_width;
}

#ifndef NDEBUG
/**
 * @brief Verify one telemetry bit-stream field against its expected value.
 * @param[in]     p_bytes    Encoded frame buffer.
 * @param[in,out] p_offset   Running bit offset advanced by bit_width.
 * @param[in]     bit_width  Field width in bits (1..32).
 * @param[in]     expect_val Expected raw unsigned bit pattern.
 * @return True if extracted bits match expect_val masked to bit_width.
 */
static bool mqtt_chk_field (const uint8_t * p_bytes,
                            uint32_t * p_offset,
                            uint32_t bit_width,
                            uint32_t expect_val)
{
    uint32_t mask     =
        MQTT_FULL_U32_MASK >> (MQTT_TELEM_U32_BITS - bit_width);
    uint32_t observed = robot_tasks_bits_get(p_bytes, *p_offset, bit_width);

    *p_offset += bit_width;

    return (observed == (expect_val & mask));
}

/**
 * @brief Populate extreme boundary telemetry values for 0x11 codec self-check.
 * @param[out] p_snap Telemetry snapshot populated with boundary test values.
 */
static void mqtt_fill_test_telem_snap (navigator_task_telemetry_t * p_snap)
{
    p_snap->seq_num             = MQTT_FULL_U32_MASK;
    p_snap->enc_left            = INT32_MIN;
    p_snap->enc_right           = INT32_MAX;
    p_snap->dist_cm             = -1;
    p_snap->speed_left          = -100;
    p_snap->speed_right         = 100;
    p_snap->obs_dist_cm         = 400u;
    p_snap->closest_cm          = 400u;
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
    /* Store highest bypass phase enum in 5-bit unsigned field. */
    p_snap->bypass_phase        = (uint32_t)BYPASS_RETURN_CHECK;
    /* Store IR_STATE_NULL enum in 3-bit unsigned field. */
    p_snap->ir_state            = (uint32_t)IR_STATE_NULL;
    p_snap->last_barcode        = MQTT_ASCII_MASK_7BIT;
    p_snap->inst_idx            = 5u;
    p_snap->b_is_obs_dist_valid = 1u;
    p_snap->obs_status          = 6u;
    p_snap->imu_state           = 3u;
    p_snap->imu_event           = 3u;
    /* Store WIDTH_ESTIMATED enum in 2-bit unsigned field. */
    p_snap->obs_width_status    = (uint32_t)WIDTH_ESTIMATED;
}

/**
 * @brief Verify 35-byte binary telemetry encoding bit offsets and padding.
 * @return True if every bit-packed field round-trips identically.
 */
static bool mqtt_check_telem_frame (void)
{
    navigator_task_telemetry_t snap                           = { 0u };
    uint8_t                    telem_buf[MQTT_TELEMETRY_BYTES] = { 0u };
    uint32_t                   all_flags = (1u << ROBOT_FLAG_BITS) - 1u;
    uint32_t                   bit_off   = MQTT_TELEM_START_BIT;
    bool                       b_is_ok   = true;

    mqtt_fill_test_telem_snap(&snap);
    mqtt_client_encode_telem(telem_buf, &snap, all_flags);

    b_is_ok = ((b_is_ok) &&
               (mqtt_chk_field(telem_buf, &bit_off, 32u, snap.seq_num)));
    /* Verify two's-complement 32-bit signed enc_left, enc_right, dist_cm. */
    b_is_ok = ((b_is_ok) &&
               (mqtt_chk_field(telem_buf, &bit_off, 32u,
                               (uint32_t)snap.enc_left)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 32u,
                               (uint32_t)snap.enc_right)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 32u,
                               (uint32_t)snap.dist_cm)));
    /* Verify 8-bit signed wheel speeds and unsigned sensor bit-fields. */
    b_is_ok = ((b_is_ok) &&
               (mqtt_chk_field(telem_buf, &bit_off, 8u,
                               (uint32_t)snap.speed_left)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 8u,
                               (uint32_t)snap.speed_right)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 9u, snap.obs_dist_cm)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 9u, snap.closest_cm)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 7u,
                               snap.closest_ang_raw)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 10u,
                               snap.est_width_cm)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 12u,
                               snap.pitch_ddeg_raw)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 12u,
                               snap.roll_ddeg_raw)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 9u, snap.heading_deg)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 10u, snap.height_dmm)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 4u,
                               snap.movement_mode)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 5u, snap.bypass_phase)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 3u, snap.ir_state)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 7u, snap.last_barcode)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 3u, snap.inst_idx)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 1u,
                               snap.b_is_obs_dist_valid)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 3u, snap.obs_status)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 2u, snap.imu_state)) &&
               (mqtt_chk_field(telem_buf, &bit_off, 2u, snap.imu_event)));

    /* Verify header byte, 13-bit flag word, 2-bit width status, and 0 pad. */
    b_is_ok = ((b_is_ok) &&
               (MQTT_HDR_TELEMETRY == telem_buf[MQTT_HDR_BYTE_IDX]) &&
               (MQTT_TELEM_FLAGS_BIT_END == (bit_off + ROBOT_FLAG_BITS)) &&
               (all_flags ==
                robot_tasks_bits_get(telem_buf, bit_off, ROBOT_FLAG_BITS)) &&
               ((uint32_t)WIDTH_ESTIMATED ==
                robot_tasks_bits_get(telem_buf,
                                     bit_off + ROBOT_FLAG_BITS,
                                     MQTT_TELEM_WIDTH_ST_BITS)) &&
               (0u == (telem_buf[MQTT_TELEMETRY_BYTES - 1u] &
                       MQTT_TELEM_PAD_MASK)) &&
               (INT32_MIN == robot_tasks_sign_extend(0x80000000u, 32u)));

    return b_is_ok;
}
#endif /* NDEBUG */

#if TM_MQTT
/**
 * @brief Append an escaped printable ASCII barcode character to JSON output.
 * @param[out]    p_output     Destination buffer.
 * @param[in]     capacity     Buffer capacity in bytes.
 * @param[in,out] p_length     Running character count.
 * @param[in]     barcode_code Raw 7-bit barcode ASCII value.
 */
static void mqtt_json_barcode_char (char * p_output,
                                    uint32_t capacity,
                                    uint32_t * p_length,
                                    uint32_t barcode_code)
{
    if ((barcode_code >= MQTT_ASCII_SPACE_MIN) &&
        (barcode_code <= MQTT_ASCII_PRINT_MAX))
    {
        /* Range-safe: printable ASCII 32..126 fits in char. */
        char bar_ch = (char)barcode_code;

        if (('"' == bar_ch) || ('\\' == bar_ch))
        {
            mqtt_client_json_char(p_output, capacity, p_length, '\\');
        }

        mqtt_client_json_char(p_output, capacity, p_length, bar_ch);
    }
}

/**
 * @brief Append ultrasonic and IMU JSON fields for the telemetry endpoint.
 * @param[out]    p_output Destination buffer.
 * @param[in]     capacity Buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     p_snap   Telemetry snapshot.
 */
static void mqtt_json_append_sensors (
    char * p_output,
    uint32_t capacity,
    uint32_t * p_length,
    const navigator_task_telemetry_t * p_snap)
{
    int32_t closest_deg = robot_tasks_sign_extend(p_snap->closest_ang_raw,
                                                  NAV_ANGLE_BIT_WIDTH);
    /* Range-safe: 12-bit signed pitch lies in [-1800, 1800]. */
    int16_t pitch_ddeg  = (int16_t)robot_tasks_sign_extend(
        p_snap->pitch_ddeg_raw, NAV_TILT_BIT_WIDTH);
    /* Range-safe: 12-bit signed roll lies in [-1800, 1800]. */
    int16_t roll_ddeg   = (int16_t)robot_tasks_sign_extend(
        p_snap->roll_ddeg_raw, NAV_TILT_BIT_WIDTH);
    /* Range-safe: heading_deg is 0..359, fitting positive int16_t. */
    int16_t heading_deg = (int16_t)p_snap->heading_deg;
    /* Range-safe: height_dmm is 0..680, fitting positive int16_t. */
    int16_t height_dmm  = (int16_t)p_snap->height_dmm;

    mqtt_client_json_text(p_output, capacity, p_length, ",\"obs_dist\":");
    mqtt_client_json_number(p_output, capacity, p_length, p_snap->obs_dist_cm);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"obs_dist_valid\":");
    mqtt_client_json_number(p_output,
                            capacity,
                            p_length,
                            p_snap->b_is_obs_dist_valid);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"obs_status\":");
    mqtt_client_json_number(p_output, capacity, p_length, p_snap->obs_status);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"closest_cm\":");
    mqtt_client_json_number(p_output, capacity, p_length, p_snap->closest_cm);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"closest_deg\":");
    mqtt_json_signed(p_output, capacity, p_length, closest_deg);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"obs_width\":");

    /* Compare 2-bit width status field against WIDTH_ESTIMATED enum. */
    if ((uint32_t)WIDTH_ESTIMATED == p_snap->obs_width_status)
    {
        mqtt_client_json_number(p_output,
                                capacity,
                                p_length,
                                p_snap->est_width_cm);
    }
    else
    {
        mqtt_client_json_text(p_output, capacity, p_length, "null");
    }

    mqtt_client_json_text(p_output,
                          capacity,
                          p_length,
                          ",\"obs_width_status\":");
    mqtt_client_json_number(p_output,
                            capacity,
                            p_length,
                            p_snap->obs_width_status);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"pitch\":");
    mqtt_append_tenths(p_output, capacity, p_length, pitch_ddeg);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"roll\":");
    mqtt_append_tenths(p_output, capacity, p_length, roll_ddeg);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"heading\":");
    /* Widen 16-bit heading angle to int32_t for JSON number formatting. */
    mqtt_json_signed(p_output, capacity, p_length, (int32_t)heading_deg);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"heading_dir\":\"");
    mqtt_client_json_text(p_output,
                          capacity,
                          p_length,
                          mqtt_heading_cardinal(heading_deg));
    mqtt_client_json_text(p_output, capacity, p_length, "\"");
    mqtt_client_json_text(p_output, capacity, p_length, ",\"height_mm\":");
    mqtt_append_tenths(p_output, capacity, p_length, height_dmm);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"imu_state\":");
    mqtt_client_json_number(p_output, capacity, p_length, p_snap->imu_state);
    mqtt_client_json_text(p_output, capacity, p_length, ",\"imu_event\":");
    mqtt_client_json_number(p_output, capacity, p_length, p_snap->imu_event);
}

/**
 * @brief Publish periodic binary telemetry and retained status when connected.
 * @param[in] now_msec Current monotonic uptime in milliseconds.
 */
static void mqtt_poll_connected (uint32_t now_msec)
{
    navigator_task_telemetry_t snap = { 0u };

    navigator_task_read_telem(&snap);

    /* Modular signed difference handles 32-bit telemetry timer wraparound. */
    if (((int32_t)(now_msec - g_mqtt_next_telem)) >= 0)
    {
        uint8_t payload[MQTT_TELEMETRY_BYTES] = { 0u };
        err_t   pub_err                       = ERR_OK;

        mqtt_client_encode_telem(payload, &snap, robot_tasks_flags_get());
        /* Cast 35-byte telemetry buffer size to lwIP u16_t payload length. */
        pub_err = mqtt_publish(gp_mqtt_client,
                               MQTT_PUBLISH_TOPIC,
                               payload,
                               (u16_t)sizeof(payload),
                               MQTT_QOS,
                               0u,
                               mqtt_on_request_done,
                               NULL);

        if (ERR_OK != pub_err)
        {
            /* Widen lwIP signed err_t status code to int32_t diagnostic. */
            g_mqtt_last_error = (int32_t)pub_err;
        }

        g_mqtt_next_telem = now_msec + MQTT_TELEM_INTERVAL_MSEC;
    }

    /* Modular signed difference handles 32-bit status timer wraparound. */
    if (((int32_t)(now_msec - g_mqtt_next_status)) >= 0)
    {
        uint8_t status_pkt[MQTT_STATUS_BYTES] = { MQTT_HDR_STATUS };
        err_t   pub_err                       = ERR_OK;

        robot_tasks_bits_set(status_pkt,
                             MQTT_STATUS_ONLINE_BIT,
                             MQTT_TELEM_VALID_BITS,
                             1u);
        robot_tasks_bits_set(status_pkt,
                             MQTT_STATUS_UPTIME_BIT,
                             MQTT_TELEM_U32_BITS,
                             now_msec);
        robot_tasks_bits_set(status_pkt,
                             MQTT_STATUS_SEQ_BIT,
                             MQTT_TELEM_U32_BITS,
                             snap.seq_num);
        /* Cast 10-byte status buffer size to lwIP u16_t payload length. */
        pub_err = mqtt_publish(gp_mqtt_client,
                               MQTT_STATUS_TOPIC,
                               status_pkt,
                               (u16_t)sizeof(status_pkt),
                               MQTT_QOS,
                               1u,
                               mqtt_on_request_done,
                               NULL);

        if (ERR_OK != pub_err)
        {
            /* Widen lwIP signed err_t status code to int32_t diagnostic. */
            g_mqtt_last_error = (int32_t)pub_err;
        }

        g_mqtt_next_status = now_msec + MQTT_STATUS_INTERVAL_MSEC;
    }
}

/**
 * @brief Append active barcode-to-action pairs to the MQTT dashboard JSON.
 * @param[out]    p_output   Destination buffer.
 * @param[in]     capacity   Buffer capacity in bytes.
 * @param[in,out] p_length   Running character count.
 * @param[in]     p_map_copy Copied 48-byte direction mapping table.
 * @param[in]     b_is_ready True if mapping table has been populated.
 */
static void mqtt_json_append_map (char * p_output,
                                  uint32_t capacity,
                                  uint32_t * p_length,
                                  const uint8_t * p_map_copy,
                                  bool b_is_ready)
{
    bool b_is_first = true;

    for (uint32_t key_code = MQTT_ASCII_PRINT_MIN;
         (b_is_ready) && (key_code <= MQTT_ASCII_PRINT_MAX);
         key_code++)
    {
        /* Convert 3-bit stored map entry to direction enum. */
        navigator_task_dir_t dir =
            (navigator_task_dir_t)robot_tasks_bits_get(
                p_map_copy, key_code * MQTT_MAP_DIR_BITS, MQTT_MAP_DIR_BITS);

        if (dir <= DIR_STOP)
        {
            /* Range-safe: key_code is printable ASCII 33..126. */
            char key_ch = (char)key_code;

            if (!b_is_first)
            {
                mqtt_client_json_char(p_output, capacity, p_length, ',');
            }

            mqtt_client_json_char(p_output, capacity, p_length, '"');
            mqtt_client_json_char(p_output, capacity, p_length, key_ch);
            mqtt_client_json_text(p_output, capacity, p_length, "\":\"");
            mqtt_client_json_text(p_output,
                                  capacity,
                                  p_length,
                                  mqtt_movement_name(dir));
            mqtt_client_json_char(p_output, capacity, p_length, '"');
            b_is_first = false;
        }
    }
}

/**
 * @brief Map a Navigator direction enum to a remote barcode motor command.
 * @param[in]  dir   Direction enum value.
 * @param[out] p_cmd Output motor command structure.
 * @return True if direction is valid (0..DIR_STOP), false otherwise.
 */
static bool mqtt_build_move_cmd (navigator_task_dir_t dir,
                                 motor_task_cmd_t * p_cmd)
{
    bool b_is_valid = true;

    p_cmd->raw = 0u;

    switch (dir)
    {
        case DIR_FRONT:
            /* Fit +40% forward left speed constant into int8_t field. */
            p_cmd->fields.target_speed_left  = (int8_t)MQTT_BARCODE_DRIVE_PCT;
            /* Fit -40% forward right speed constant into int8_t field. */
            p_cmd->fields.target_speed_right =
                (int8_t)(-MQTT_BARCODE_DRIVE_PCT);
        break;

        case DIR_BACK:
            /* Fit -40% reverse left speed constant into int8_t field. */
            p_cmd->fields.target_speed_left  =
                (int8_t)(-MQTT_BARCODE_DRIVE_PCT);
            /* Fit +40% reverse right speed constant into int8_t field. */
            p_cmd->fields.target_speed_right = (int8_t)MQTT_BARCODE_DRIVE_PCT;
        break;

        case DIR_LEFT:
            /* Fit -40% left-pivot speed constant into left int8_t field. */
            p_cmd->fields.target_speed_left  =
                (int8_t)(-MQTT_BARCODE_DRIVE_PCT);
            /* Fit -40% left-pivot speed constant into right int8_t field. */
            p_cmd->fields.target_speed_right =
                (int8_t)(-MQTT_BARCODE_DRIVE_PCT);
        break;

        case DIR_RIGHT:
            /* Fit +40% right-pivot speed constant into left int8_t field. */
            p_cmd->fields.target_speed_left  = (int8_t)MQTT_BARCODE_DRIVE_PCT;
            /* Fit +40% right-pivot speed constant into right int8_t field. */
            p_cmd->fields.target_speed_right = (int8_t)MQTT_BARCODE_DRIVE_PCT;
        break;

        case DIR_STOP:
            /* Zeroed command stops both motors. */
        break;

        default:
            b_is_valid = false;
        break;
    }

    return b_is_valid;
}

/**
 * @brief Return the JSON action string for a direction enum value.
 * @param[in] dir Direction enum value.
 * @return Action label string.
 */
static const char * mqtt_movement_name (navigator_task_dir_t dir)
{
    const char * p_label = "UNKNOWN";

    switch (dir)
    {
        case DIR_FRONT:
            p_label = "MOVE_FORWARD";
        break;

        case DIR_BACK:
            p_label = "MOVE_BACKWARD";
        break;

        case DIR_LEFT:
            p_label = "TURN_LEFT";
        break;

        case DIR_RIGHT:
            p_label = "TURN_RIGHT";
        break;

        case DIR_STOP:
            p_label = "STOP";
        break;

        case DIR_INVALID:
        default:
            p_label = "UNKNOWN";
        break;
    }

    return p_label;
}

/**
 * @brief Forward every mapped barcode entry to the Navigator message buffer.
 * @param[in] p_next_map Newly decoded 48-byte packed direction table.
 */
static void mqtt_forward_map_entries (const uint8_t * p_next_map)
{
    for (uint32_t key_code = MQTT_ASCII_PRINT_MIN;
         key_code <= MQTT_ASCII_PRINT_MAX;
         key_code++)
    {
        uint32_t dir_code = robot_tasks_bits_get(
            p_next_map, key_code * MQTT_MAP_DIR_BITS, MQTT_MAP_DIR_BITS);

        /* Forward only populated ASCII entries (not DIR_INVALID). */
        if ((uint32_t)DIR_INVALID != dir_code)
        {
            navigator_task_msg_t nav_msg = { 0 };

            /* Store MSG_WIFI_BARCODE_MAPPING enum in 4-bit message field. */
            nav_msg.type                = (uint32_t)MSG_WIFI_BARCODE_MAPPING;
            /* Range-safe: 3-bit dir << 7 | 7-bit ASCII fits in uint16_t. */
            nav_msg.data.packed_mapping =
                (uint16_t)((dir_code << MQTT_MAP_DIR_SHIFT) | key_code);

            /* Cast Navigator message size to RTOS INT parameter. */
            if (tk_snd_mbf(robot_tasks_get_nav_mbf(),
                           &nav_msg,
                           (INT)sizeof(nav_msg),
                           TMO_POL) < E_OK)
            {
                /* Cast ASCII literal to uT-Kernel UB pointer for console. */
                tm_printf((UB *)"[mqtt] mapping message dropped\n");
            }
        }
    }
}

/**
 * @brief Store a decoded mapping table and forward entries to Navigator.
 * @param[in] p_payload Raw binary 0x13 mapping payload.
 * @param[in] length    Payload length in bytes.
 * @return True if mapping decoded cleanly, false if malformed.
 */
static bool mqtt_update_move_map (const uint8_t * p_payload, uint32_t length)
{
    uint8_t next_map[MQTT_MAP_BYTES] = { 0u };
    bool    b_is_ok                  = mqtt_client_decode_map(p_payload,
                                                              length,
                                                              next_map);

    if (b_is_ok)
    {
        UINT saved_state = robot_tasks_data_lock();

        for (uint32_t idx = 0u; idx < MQTT_MAP_BYTES; idx++)
        {
            g_movement_map.directions[idx] = next_map[idx];
        }

        g_movement_map.b_is_ready = 1u;
        robot_tasks_data_unlock(saved_state);
        mqtt_forward_map_entries(next_map);
    }

    return b_is_ok;
}

/**
 * @brief Read monotonic uptime in milliseconds from the 64-bit usec timer.
 * @return Wrapping 32-bit millisecond uptime counter.
 */
static uint32_t mqtt_now_msec (void)
{
    /* Range-safe: deliberate modulo-2^32 wrapping millisecond timer. */
    return (uint32_t)(time_us_64() / MQTT_USEC_PER_MSEC_U64);
}

/**
 * @brief lwIP MQTT publish/subscribe completion callback.
 * @param[in] p_arg    User argument (unused).
 * @param[in] err_code lwIP status code.
 */
static void mqtt_on_request_done (void * p_arg, err_t err_code)
{
    /* Explicitly ignore unused lwIP callback user argument. */
    (void)p_arg;

    if (ERR_OK != err_code)
    {
        /* Widen lwIP signed err_t status code to int32_t diagnostic. */
        g_mqtt_last_error = (int32_t)err_code;
    }
}

/**
 * @brief Decode and forward a completed binary MQTT motor command to Navigator.
 */
static void mqtt_dispatch_remote_cmd (void)
{
    navigator_task_msg_t nav_msg = { 0 };
    motor_task_cmd_t     mot_cmd = { 0u };

    if (!mqtt_client_decode_cmd(g_mqtt_payload,
                                g_mqtt_state.payload_length,
                                &mot_cmd))
    {
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[mqtt] invalid binary command ignored\n");
    }
    else
    {
        /* Store MSG_REMOTE_CMD enum in 4-bit Navigator message field. */
        nav_msg.type            = (uint32_t)MSG_REMOTE_CMD;
        nav_msg.data.remote_cmd = mot_cmd;

        /* Cast Navigator message size to RTOS INT parameter. */
        if (tk_snd_mbf(robot_tasks_get_nav_mbf(),
                       &nav_msg,
                       (INT)sizeof(nav_msg),
                       TMO_POL) < E_OK)
        {
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[mqtt] navigator queue full, cmd dropped\n");
        }
    }
}

/**
 * @brief Dispatch one fully assembled MQTT message based on its topic.
 */
static void mqtt_dispatch_complete_msg (void)
{
    g_mqtt_msg_cnt++;

    if (0 == strcmp(g_mqtt_topic, MQTT_MOVEMENT_CONFIG_TOPIC))
    {
        if (!mqtt_update_move_map(g_mqtt_payload, g_mqtt_state.payload_length))
        {
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[mqtt] invalid binary mapping ignored\n");
        }
    }
    else if (0 == strcmp(g_mqtt_topic, MQTT_SUBSCRIBE_TOPIC))
    {
        mqtt_dispatch_remote_cmd();
    }
    else
    {
        /* Ignore publications on unexpected topics. */
    }
}

/**
 * @brief Append a validated payload chunk and dispatch on MQTT_DATA_FLAG_LAST.
 * @param[in] p_data Fragment byte pointer.
 * @param[in] length Fragment length in bytes.
 * @param[in] flags  lwIP fragment flags.
 */
static void mqtt_append_rx_chunk (const u8_t * p_data,
                                  u16_t length,
                                  u8_t flags)
{
    if (0u != length)
    {
        /* Copy validated fragment into static MQTT payload buffer. */
        (void)memcpy(&g_mqtt_payload[g_mqtt_state.payload_length],
                     p_data,
                     length);
    }

    /* Widen validated fragment length to accumulate 7-bit payload_length. */
    g_mqtt_state.payload_length += (uint32_t)length;

    if (0u != (flags & MQTT_DATA_FLAG_LAST))
    {
        g_mqtt_state.b_is_payload_valid = 0u;

        if (g_mqtt_state.payload_length == g_mqtt_state.payload_expected)
        {
            mqtt_dispatch_complete_msg();
        }
    }
}

/**
 * @brief lwIP MQTT incoming publish payload fragment callback.
 * @param[in] p_arg  User argument (unused).
 * @param[in] p_data Fragment byte pointer.
 * @param[in] length Fragment length in bytes.
 * @param[in] flags  lwIP fragment flags (MQTT_DATA_FLAG_LAST on final chunk).
 */
static void mqtt_on_data (void * p_arg,
                          const u8_t * p_data,
                          u16_t length,
                          u8_t flags)
{
    /* Explicitly ignore unused lwIP callback user argument. */
    (void)p_arg;

    if (0u != g_mqtt_state.b_is_payload_valid)
    {
        /* Widen u16_t fragment length to check against expected total bytes. */
        bool b_is_bad_chunk =
            (((0u != length) && (NULL == p_data)) ||
             ((g_mqtt_state.payload_length + (uint32_t)length) >
              g_mqtt_state.payload_expected));

        if (b_is_bad_chunk)
        {
            g_mqtt_state.b_is_payload_valid = 0u;
        }
        else
        {
            mqtt_append_rx_chunk(p_data, length, flags);
        }
    }
}

/**
 * @brief lwIP MQTT incoming publish header callback validating topic and size.
 * @param[in] p_arg        User argument (unused).
 * @param[in] p_topic      Null-terminated topic string.
 * @param[in] total_length Declared total payload length in bytes.
 */
static void mqtt_on_incoming_pub (void * p_arg,
                                  const char * p_topic,
                                  u32_t total_length)
{
    /* Cast bounded topic strlen result to uint32_t for length check. */
    uint32_t topic_len = (NULL != p_topic) ? ((uint32_t)strlen(p_topic)) : 0u;

    /* Explicitly ignore unused lwIP callback user argument. */
    (void)p_arg;

    g_mqtt_state.payload_length     = 0u;
    g_mqtt_state.payload_expected   = 0u;
    g_mqtt_state.b_is_payload_valid = 0u;
    g_mqtt_topic[0]                 = '\0';

    if ((0u != topic_len) &&
        (topic_len < MQTT_TOPIC_MAX) &&
        (0u != total_length) &&
        (total_length <= MQTT_PAYLOAD_MAX))
    {
        /* Copy NUL-terminated topic string into fixed topic buffer. */
        (void)memcpy(g_mqtt_topic, p_topic, topic_len + 1u);
        g_mqtt_state.payload_expected   = total_length;
        g_mqtt_state.b_is_payload_valid = 1u;
    }
}

/**
 * @brief lwIP MQTT broker connection status callback.
 * @param[in] p_client lwIP MQTT client handle (unused).
 * @param[in] p_arg    User argument (unused).
 * @param[in] status   Connection status code.
 */
static void mqtt_on_connection (mqtt_client_t * p_client,
                                void * p_arg,
                                mqtt_connection_status_t status)
{
    /* Explicitly ignore lwIP client pointer (matches gp_mqtt_client). */
    (void)p_client;
    /* Explicitly ignore unused lwIP callback user argument. */
    (void)p_arg;

    g_mqtt_state.b_is_pending = 0u;
    robot_tasks_flag_assign(ROBOT_FAULT_MQTT,
                            (MQTT_CONNECT_ACCEPTED != status));

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_mqtt_state.b_is_connected = 1u;
        g_mqtt_last_error           = 0;
        g_mqtt_next_telem           = 0u;
        g_mqtt_next_status          = 0u;
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[mqtt] connected to " MQTT_BROKER_IP "\n");

        if (ERR_OK != mqtt_subscribe(gp_mqtt_client,
                                     MQTT_SUBSCRIBE_TOPIC,
                                     MQTT_QOS,
                                     mqtt_on_request_done,
                                     NULL))
        {
            /* Store lwIP ERR_CONN code as int32_t diagnostic. */
            g_mqtt_last_error = (int32_t)ERR_CONN;
        }

        if (ERR_OK != mqtt_subscribe(gp_mqtt_client,
                                     MQTT_MOVEMENT_CONFIG_TOPIC,
                                     MQTT_QOS,
                                     mqtt_on_request_done,
                                     NULL))
        {
            /* Store lwIP ERR_CONN code as int32_t diagnostic. */
            g_mqtt_last_error = (int32_t)ERR_CONN;
        }
    }
    else
    {
        g_mqtt_state.b_is_connected = 0u;
        g_mqtt_next_retry           = mqtt_now_msec() + MQTT_RETRY_MSEC;
        /* Store lwIP connection status enum as int32_t diagnostic. */
        g_mqtt_last_error           = (int32_t)status;
        /* Cast format literal and connection status to RTOS UB* and INT. */
        tm_printf((UB *)"[mqtt] disconnected, status=%d\n", (INT)status);
    }
}

/**
 * @brief Allocate lwIP MQTT client, configure LWT, and connect to broker.
 */
static void mqtt_begin_connect (void)
{
    bool b_is_ready = true;

    if (NULL == gp_mqtt_client)
    {
        gp_mqtt_client = mqtt_client_new();

        if (NULL == gp_mqtt_client)
        {
            /* Store lwIP ERR_MEM code as int32_t diagnostic. */
            g_mqtt_last_error = (int32_t)ERR_MEM;
            g_mqtt_next_retry = mqtt_now_msec() + MQTT_RETRY_MSEC;
            b_is_ready        = false;
        }
        else
        {
            mqtt_set_inpub_callback(gp_mqtt_client,
                                    mqtt_on_incoming_pub,
                                    mqtt_on_data,
                                    NULL);
        }
    }

    if ((b_is_ready) && (0u == g_mqtt_state.b_is_configured))
    {
        if (0 == ipaddr_aton(MQTT_BROKER_IP, &g_mqtt_broker))
        {
            /* Store lwIP ERR_ARG code as int32_t diagnostic. */
            g_mqtt_last_error = (int32_t)ERR_ARG;
            g_mqtt_next_retry = mqtt_now_msec() + MQTT_RETRY_MSEC;
            b_is_ready        = false;
        }
        else
        {
            g_mqtt_info.client_id       = MQTT_CLIENT_ID;
            g_mqtt_info.client_user     =
                ('\0' != MQTT_USERNAME[0]) ? MQTT_USERNAME : NULL;
            g_mqtt_info.client_pass     =
                ('\0' != MQTT_PASSWORD[0]) ? MQTT_PASSWORD : NULL;
            g_mqtt_info.keep_alive      = MQTT_KEEP_ALIVE_SECONDS;
            g_mqtt_info.will_topic      = MQTT_STATUS_TOPIC;
            /* Cast required by lwIP mqtt_connect_client_info_t::will_msg. */
            g_mqtt_info.will_msg        = (const char *)g_offline_will;
            /* Cast 10-byte LWT buffer size to lwIP u16_t length field. */
            g_mqtt_info.will_msg_len    = (u16_t)sizeof(g_offline_will);
            g_mqtt_info.will_qos        = 0u;
            g_mqtt_info.will_retain     = 1u;
            g_mqtt_state.b_is_configured = 1u;
        }
    }

    if (b_is_ready)
    {
        g_mqtt_state.b_is_pending = 1u;

        if (ERR_OK != mqtt_client_connect(gp_mqtt_client,
                                          &g_mqtt_broker,
                                          MQTT_BROKER_PORT,
                                          mqtt_on_connection,
                                          NULL,
                                          &g_mqtt_info))
        {
            g_mqtt_state.b_is_pending = 0u;
            /* Store lwIP ERR_CONN code as int32_t diagnostic. */
            g_mqtt_last_error         = (int32_t)ERR_CONN;
            g_mqtt_next_retry         = mqtt_now_msec() + MQTT_RETRY_MSEC;
        }
    }
}

/**
 * @brief Append a signed 32-bit integer (including INT32_MIN) as JSON text.
 * @param[out]    p_output Destination buffer.
 * @param[in]     capacity Buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     num_val  Signed 32-bit value.
 */
static void mqtt_json_signed (char * p_output,
                              uint32_t capacity,
                              uint32_t * p_length,
                              int32_t num_val)
{
    uint32_t magnitude = 0u;

    if (num_val < 0)
    {
        mqtt_client_json_char(p_output, capacity, p_length, '-');
        /* Range-safe: -(num_val + 1) avoids signed overflow for INT32_MIN. */
        magnitude = ((uint32_t)(-(num_val + 1))) + 1u;
    }
    else
    {
        /* Range-safe: non-negative num_val converts directly to uint32_t. */
        magnitude = (uint32_t)num_val;
    }

    mqtt_client_json_number(p_output, capacity, p_length, magnitude);
}

/**
 * @brief Append a tenths fixed-point int16_t as decimal JSON (e.g. -12.3).
 * @param[out]    p_output   Destination buffer.
 * @param[in]     capacity   Buffer capacity in bytes.
 * @param[in,out] p_length   Running character count.
 * @param[in]     tenths_val Value scaled by 10.
 */
static void mqtt_append_tenths (char * p_output,
                                uint32_t capacity,
                                uint32_t * p_length,
                                int16_t tenths_val)
{
    uint32_t magnitude = 0u;

    if (tenths_val < 0)
    {
        mqtt_client_json_char(p_output, capacity, p_length, '-');
        /* Range-safe: widening to int32_t prevents INT16_MIN negation UB. */
        magnitude = (uint32_t)(-((int32_t)tenths_val));
    }
    else
    {
        /* Range-safe: non-negative tenths_val converts to uint32_t. */
        magnitude = (uint32_t)tenths_val;
    }

    mqtt_client_json_number(p_output,
                            capacity,
                            p_length,
                            magnitude / MQTT_DECIMAL_RADIX);
    mqtt_client_json_char(p_output, capacity, p_length, '.');
    mqtt_client_json_number(p_output,
                            capacity,
                            p_length,
                            magnitude % MQTT_DECIMAL_RADIX);
}

/**
 * @brief Append an escaped JSON string body (escaping quotes and backslashes).
 * @param[out]    p_output Destination buffer.
 * @param[in]     capacity Buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     p_text   Null-terminated source string.
 */
static void mqtt_json_quoted (char * p_output,
                              uint32_t capacity,
                              uint32_t * p_length,
                              const char * p_text)
{
    const char * p_cur = p_text;

    while ('\0' != (*p_cur))
    {
        char ch_val = *p_cur;

        if (('"' == ch_val) || ('\\' == ch_val))
        {
            mqtt_client_json_char(p_output, capacity, p_length, '\\');
        }

        mqtt_client_json_char(p_output, capacity, p_length, ch_val);
        p_cur++;
    }
}

/**
 * @brief Convert compass heading in degrees to an 8-point cardinal label.
 * @param[in] deg_val Compass heading in degrees.
 * @return Null-terminated cardinal string ("N", "NE", ..., "NW").
 */
static const char * mqtt_heading_cardinal (int16_t deg_val)
{
    static const char * const cardinal_labels[MQTT_CARDINAL_COUNT] =
    {
        "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };
    /* Widen 16-bit heading angle to int32_t for positive modulo-360 math. */
    int32_t  norm_deg = ((((int32_t)deg_val) % MQTT_DEGREES_FULL) +
                         MQTT_DEGREES_FULL) % MQTT_DEGREES_FULL;
    /* Range-safe: norm_deg is normalized into [0, 359]. */
    uint32_t dir_idx  =
        ((((uint32_t)norm_deg) + MQTT_CARDINAL_HALF_STEP) /
         MQTT_CARDINAL_STEP_DEG) & MQTT_CARDINAL_MASK;

    return cardinal_labels[dir_idx];
}
#endif /* TM_MQTT */

/*** end of file ***/
