#ifndef UTK_MQTT_CLIENT_H
#define UTK_MQTT_CLIENT_H

#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif

#include "../Motor/Motor_task.h"
#include "../Navigator/Navigator_task.h"
struct netif;

/* MQTT v1: explicit little-endian bit streams (never compiler struct images).
 * First byte: version 1 in high nibble, kind in low nibble.
 * 0x12 command: header, signed left %, signed right %, left ticks, right ticks.
 *    Exactly 5 bytes; speeds -100..100, independent ticks 0..255.
 *    Example forward at 40%: 12 28 D8 00 00.
 * 0x13 mapping: header, count (1..66), count entries of ASCII:7 + direction:3.
 *    Directions retain Navigator values 0..4. Unused final bits must be zero.
 *    Example A -> LEFT: 13 01 41 01. Case folded; duplicate keys rejected.
 * 0x11 telemetry: fields in order below, widths in parentheses:
 *    seq(32), enc_l(32 signed), enc_r(32 signed), dist_cm(32 signed),
 *    speed_l/r(8 signed each), distance(9), closest(9), closest_angle(7 signed),
 *    width(10), pitch/roll(12 signed each, 0.1 degree), heading(9),
 *    height(10, 0.1 mm), mode(4), phase(5), IR(3), barcode(7), route_idx(3),
 *    sonar_valid(1), sonar_status(3), IMU_state/event(2 each), fault/control(13).
 *    273 bits = 35 bytes including header; final 7 padding bits zero.
 * 0x14 status: header, online(1), uptime_ms(32), telemetry_seq(32) = 10 bytes.
 * HTTP /api/telemetry and /api/mqtt stay JSON for the existing dashboard.
 */
#define MQTT_COMMAND_BYTES   5U
#define MQTT_TELEMETRY_BYTES 35U
#define MQTT_STATUS_BYTES   10U
#define MQTT_MAPPING_MAX    66U
#define MQTT_MAP_BYTES      48U
#define MQTT_PAYLOAD_MAX    (2U + (MQTT_MAPPING_MAX * 10U + 7U) / 8U)

typedef struct __attribute__((packed)) {
    UW configured : 1;
    UW pending : 1;
    UW connected : 1;
    UW payload_valid : 1;
    UW payload_length : 7;
    UW payload_expected : 7;
} MQTT_RUNTIME_STATE;

/* Only this map crosses cores; every access holds robot_data_lock. */
typedef struct __attribute__((packed)) {
    UW ready : 1;
    UB directions[MQTT_MAP_BYTES];
} MQTT_MOVEMENT_MAP;

BOOL mqtt_decode_command(const UB *bytes, SZ length, MOTOR_COMMAND *command);
BOOL mqtt_decode_mapping(const UB *bytes, SZ length, UB map[MQTT_MAP_BYTES]);
void mqtt_encode_telemetry(UB bytes[MQTT_TELEMETRY_BYTES],
                          const TELEMETRY_SNAPSHOT *snapshot, UW flags);
#ifndef NDEBUG
BOOL mqtt_codec_self_check(void);
#endif
void mqtt_dashboard_link_down(void);

void mqtt_dashboard_poll(struct netif *netif);
SZ mqtt_dashboard_json(char *output, SZ capacity);
SZ telemetry_json(char *output, SZ capacity);
BOOL mqtt_get_movement_for_barcode(char barcode, MOTOR_COMMAND *command);

/* JSON helpers used by telemetry_json / mqtt_dashboard_json */
void json_char(char *output, SZ capacity, SZ *length, char value);
void json_text(char *output, SZ capacity, SZ *length, const char *text);
void json_number(char *output, SZ capacity, SZ *length, UW value);

#endif
