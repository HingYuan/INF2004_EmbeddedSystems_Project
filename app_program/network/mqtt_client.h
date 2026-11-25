/**
 * @file mqtt_client.h
 * @brief Binary MQTT v1 telemetry/command codec and JSON dashboard formatter.
 *
 * NOTE: MQTT v1 wire ABI uses explicit little-endian bit streams (never raw
 * compiler struct layouts):
 *   0x11 telemetry (35 bytes): header + 267 data bits + 5 zero padding bits.
 *   0x12 command    (5 bytes): hdr, left %, right %, left ticks, right ticks.
 *   0x13 mapping  (2..85 B)  : hdr, count (1..66), count x (ASCII:7 + dir:3).
 *   0x14 status    (10 bytes): header, online(1), uptime_msec(32), seq(32).
 */

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "../motor/motor_task.h"
#include "../navigator/navigator_task.h"

typedef struct netif mqtt_client_netif_t;

#define MQTT_COMMAND_BYTES              (5u)
#define MQTT_TELEMETRY_BYTES            (35u)
#define MQTT_STATUS_BYTES               (10u)
#define MQTT_MAPPING_MAX                (66u)
#define MQTT_MAP_BYTES                  (48u)
#define MQTT_PAYLOAD_MAX                \
    (2u + (((MQTT_MAPPING_MAX * 10u) + 7u) / 8u))

/**
 * @brief Decode a 5-byte binary MQTT v1 motor command (0x12).
 * @param[in]  p_bytes Source frame bytes.
 * @param[in]  length  Frame length in bytes.
 * @param[out] p_cmd   Decoded motor command structure.
 * @return True if valid 0x12 command, false otherwise.
 */
bool mqtt_client_decode_cmd(const uint8_t * p_bytes,
                            uint32_t length,
                            motor_task_cmd_t * p_cmd);

/**
 * @brief Decode a binary MQTT v1 barcode-to-direction mapping (0x13).
 * @param[in]  p_bytes   Source frame bytes.
 * @param[in]  length    Frame length in bytes.
 * @param[out] p_map_buf Output 48-byte packed 3-bit-per-ASCII direction table.
 * @return True if valid 0x13 mapping, false otherwise.
 */
bool mqtt_client_decode_map(const uint8_t * p_bytes,
                            uint32_t length,
                            uint8_t * p_map_buf);

/**
 * @brief Encode a telemetry snapshot and flag word into a 35-byte 0x11 frame.
 * @param[out] p_out_bytes Destination 35-byte wire buffer.
 * @param[in]  p_snapshot  Telemetry snapshot to serialize.
 * @param[in]  flags_word  Shared 13-bit robot fault/control word.
 */
void mqtt_client_encode_telem(uint8_t * p_out_bytes,
                              const navigator_task_telemetry_t * p_snapshot,
                              uint32_t flags_word);

#ifndef NDEBUG
/**
 * @brief Run debug-only self-checks on MQTT v1 command, map, and telem codecs.
 * @return True if all wire codec checks pass, false otherwise.
 */
bool mqtt_client_codec_check(void);
#endif

/**
 * @brief Publish offline will status and disconnect on Wi-Fi link loss.
 */
void mqtt_client_link_down(void);

/**
 * @brief Poll lwIP MQTT client connection, telemetry publish, and keepalive.
 * @param[in] p_netif Active lwIP network interface.
 */
void mqtt_client_poll(mqtt_client_netif_t * p_netif);

/**
 * @brief Format MQTT status and decoded barcode mapping table as JSON.
 * @param[out] p_output Destination character buffer.
 * @param[in]  capacity Buffer capacity in bytes (including NUL terminator).
 * @return Length of formatted JSON string excluding NUL terminator.
 */
uint32_t mqtt_client_dashboard_json(char * p_output, uint32_t capacity);

/**
 * @brief Format current shared telemetry snapshot as JSON for HTTP /api.
 * @param[out] p_output Destination character buffer.
 * @param[in]  capacity Buffer capacity in bytes (including NUL terminator).
 * @return Length of formatted JSON string excluding NUL terminator.
 */
uint32_t mqtt_client_telemetry_json(char * p_output, uint32_t capacity);

/**
 * @brief Look up a motor command mapped to a decoded barcode ASCII character.
 * @param[in]  barcode_ch ASCII barcode character.
 * @param[out] p_cmd      Output motor command if mapped.
 * @return True if barcode_ch has a valid mapped command, false otherwise.
 */
bool mqtt_client_get_barcode_cmd(uint8_t barcode_ch, motor_task_cmd_t * p_cmd);

/**
 * @brief Append one ASCII character to a bounded JSON output buffer.
 * @param[out]    p_output Destination character buffer.
 * @param[in]     capacity Total buffer capacity in bytes.
 * @param[in,out] p_length Running character count (incremented even if full).
 * @param[in]     ch_val   Character to append.
 */
void mqtt_client_json_char(char * p_output,
                           uint32_t capacity,
                           uint32_t * p_length,
                           char ch_val);

/**
 * @brief Append a NUL-terminated string to a bounded JSON output buffer.
 * @param[out]    p_output Destination character buffer.
 * @param[in]     capacity Total buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     p_text   NUL-terminated string to append.
 */
void mqtt_client_json_text(char * p_output,
                           uint32_t capacity,
                           uint32_t * p_length,
                           const char * p_text);

/**
 * @brief Append an unsigned decimal integer to a bounded JSON output buffer.
 * @param[out]    p_output Destination character buffer.
 * @param[in]     capacity Total buffer capacity in bytes.
 * @param[in,out] p_length Running character count.
 * @param[in]     num_val  Unsigned 32-bit integer value to format.
 */
void mqtt_client_json_number(char * p_output,
                             uint32_t capacity,
                             uint32_t * p_length,
                             uint32_t num_val);

#endif /* MQTT_CLIENT_H */

/*** end of file ***/
