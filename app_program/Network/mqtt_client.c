/* Prevent syslib.h from redefining size_t so stddef.h (unsigned int) wins */
#define PROHIBIT_DEF_SIZE_T

#include <stddef.h>
#include "mqtt_client.h"
#include <string.h>

#include <tm/tmonitor.h>


BOOL mqtt_decode_command(const UB *bytes, SZ length, MOTOR_COMMAND *command)
{
    if (!bytes || !command || length != MQTT_COMMAND_BYTES || bytes[0] != 0x12U)
        return FALSE;
    W left = robot_sign_extend(bytes[1], 8U);
    W right = robot_sign_extend(bytes[2], 8U);
    if (left < -100 || left > 100 || right < -100 || right > 100) return FALSE;
    *command = (MOTOR_COMMAND){.target_speed_left = left, .target_speed_right = right,
                              .turn_ticks_left = bytes[3], .turn_ticks_right = bytes[4]};
    return TRUE;
}

BOOL mqtt_decode_mapping(const UB *bytes, SZ length, UB map[MQTT_MAP_BYTES])
{
    if (!bytes || !map || length < 2 || bytes[0] != 0x13U ||
        bytes[1] == 0U || bytes[1] > MQTT_MAPPING_MAX ||
        length != (SZ)(2U + (bytes[1] * 10U + 7U) / 8U)) return FALSE;
    UINT bits = bytes[1] * 10U, used = bits % 8U;
    if (used && (bytes[length - 1] >> used) != 0U) return FALSE;
    memset(map, 0, MQTT_MAP_BYTES);
    for (UINT i = 0; i < 128U; ++i) robot_bits_set(map, i * 3U, 3U, DIR_INVALID);
    for (UINT i = 0; i < bytes[1]; ++i) {
        UW entry = robot_bits_get(bytes + 2, i * 10U, 10U);
        UINT key = entry & 127U, direction = entry >> 7;
        if (key >= 'a' && key <= 'z') key -= 'a' - 'A';
        if (key < 33U || key > 126U || key == '"' || key == '\\' ||
            direction > DIR_STOP || robot_bits_get(map, key * 3U, 3U) != DIR_INVALID)
            return FALSE;
        robot_bits_set(map, key * 3U, 3U, direction);
    }
    return TRUE;
}

void mqtt_encode_telemetry(UB bytes[MQTT_TELEMETRY_BYTES],
                          const TELEMETRY_SNAPSHOT *s, UW flags)
{
    UINT offset = 8U;
    memset(bytes, 0, MQTT_TELEMETRY_BYTES);
    bytes[0] = 0x11U;
#define FIELD(member, width) do { robot_bits_set(bytes, offset, width, (UW)s->member); offset += width; } while (0)
    FIELD(seq, 32); FIELD(enc_l, 32); FIELD(enc_r, 32); FIELD(dist_cm, 32);
    FIELD(speed_left, 8); FIELD(speed_right, 8);
    FIELD(obstacle_distance_cm, 9); FIELD(closest_cm, 9); FIELD(closest_angle_deg, 7);
    FIELD(estimated_width_cm, 10); FIELD(pitch_ddeg, 12); FIELD(roll_ddeg, 12);
    FIELD(heading_deg, 9); FIELD(height_dmm, 10); FIELD(movement_mode, 4);
    FIELD(bypass_phase, 5); FIELD(ir_state, 3); FIELD(last_barcode, 7);
    FIELD(instruction_idx, 3); FIELD(obstacle_distance_valid, 1);
    FIELD(obstacle_status, 3); FIELD(imu_state, 2); FIELD(imu_event, 2);
#undef FIELD
    robot_bits_set(bytes, offset, ROBOT_FLAG_BITS, flags);
    robot_bits_set(bytes, offset + ROBOT_FLAG_BITS, 2U, s->obstacle_width_status);
}

#ifndef NDEBUG
BOOL mqtt_codec_self_check(void)
{
    UB command[] = {0x12U, 100U, (UB)-100, 255U, 7U};
    MOTOR_COMMAND decoded = {0};
    if (!mqtt_decode_command(command, sizeof(command), &decoded) ||
        decoded.target_speed_left != 100 || decoded.target_speed_right != -100 ||
        decoded.turn_ticks_left != 255 || decoded.turn_ticks_right != 7) return FALSE;
    command[1] = 101;
    if (mqtt_decode_command(command, sizeof(command), &decoded) ||
        mqtt_decode_command(command, sizeof(command) - 1, &decoded)) return FALSE;
    UB mapping[] = {0x13U, 1U, 0x41U, 0x01U}, map[MQTT_MAP_BYTES];
    if (!mqtt_decode_mapping(mapping, sizeof(mapping), map) ||
        robot_bits_get(map, 'A' * 3U, 3U) != DIR_LEFT ||
        robot_bits_get(map, 'B' * 3U, 3U) != DIR_INVALID) return FALSE;
    mapping[3] |= 0x80U;
    if (mqtt_decode_mapping(mapping, sizeof(mapping), map)) return FALSE;
    TELEMETRY_SNAPSHOT s = {.seq = 0xffffffffUL, .enc_l = (-2147483647L - 1),
        .enc_r = 2147483647L, .dist_cm = -1, .speed_left = -100, .speed_right = 100,
        .obstacle_distance_cm = 400, .closest_cm = 400, .closest_angle_deg = -45,
        .estimated_width_cm = 800, .pitch_ddeg = -1800, .roll_ddeg = 1800,
        .heading_deg = 359, .height_dmm = 680, .movement_mode = MODE_SAFE_STOP,
        .bypass_phase = BYPASS_RETURN_CHECK, .ir_state = STATE_NULL, .last_barcode = 127,
        .instruction_idx = 5, .obstacle_distance_valid = 1, .obstacle_status = 6,
        .imu_state = 3, .imu_event = 3, .obstacle_width_status = WIDTH_ESTIMATED};
    UB bytes[MQTT_TELEMETRY_BYTES];
    mqtt_encode_telemetry(bytes, &s, (1UL << ROBOT_FLAG_BITS) - 1U);
    UINT offset = 8;
#define CHECK(member, width) do { if (robot_bits_get(bytes, offset, width) != \
    ((UW)s.member & (0xffffffffUL >> (32U - width)))) return FALSE; offset += width; } while (0)
    CHECK(seq, 32); CHECK(enc_l, 32); CHECK(enc_r, 32); CHECK(dist_cm, 32);
    CHECK(speed_left, 8); CHECK(speed_right, 8); CHECK(obstacle_distance_cm, 9);
    CHECK(closest_cm, 9); CHECK(closest_angle_deg, 7); CHECK(estimated_width_cm, 10);
    CHECK(pitch_ddeg, 12); CHECK(roll_ddeg, 12); CHECK(heading_deg, 9); CHECK(height_dmm, 10);
    CHECK(movement_mode, 4); CHECK(bypass_phase, 5); CHECK(ir_state, 3); CHECK(last_barcode, 7);
    CHECK(instruction_idx, 3); CHECK(obstacle_distance_valid, 1); CHECK(obstacle_status, 3);
    CHECK(imu_state, 2); CHECK(imu_event, 2);
#undef CHECK
    return bytes[0] == 0x11U && offset + ROBOT_FLAG_BITS == 273U &&
           robot_bits_get(bytes, offset, ROBOT_FLAG_BITS) == (1UL << ROBOT_FLAG_BITS) - 1U &&
           robot_bits_get(bytes, offset + ROBOT_FLAG_BITS, 2U) == WIDTH_ESTIMATED &&
           (bytes[MQTT_TELEMETRY_BYTES - 1] & 0xf8U) == 0U &&
           robot_sign_extend(0x80000000UL, 32U) == (-2147483647L - 1);
}
#endif

#if TM_MQTT

#include "lwip/apps/mqtt.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "mqtt_config.h"
#include <tk/tkernel.h>
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"

extern UD time_us_64(void);

#define MQTT_RETRY_MS       5000U
#define MQTT_TOPIC_MAX      96U
#define MQTT_MOVEMENT_CONFIG_TOPIC_DEFAULT "robot/config/movement"
#define MQTT_STATUS_TOPIC_DEFAULT "robot/status"
#define MQTT_STATUS_INTERVAL_MS 5000U

static mqtt_client_t *mqtt_client;
static ip_addr_t mqtt_broker;
static struct mqtt_connect_client_info_t mqtt_info;
static char mqtt_topic[MQTT_TOPIC_MAX];
static UB mqtt_payload[MQTT_PAYLOAD_MAX];
static MQTT_RUNTIME_STATE mqtt_state;
static UW mqtt_message_count;
static UW mqtt_next_retry_ms;
static UW mqtt_next_telemetry_ms;
static UW mqtt_next_status_ms;
static int mqtt_last_error;
static MQTT_MOVEMENT_MAP movement_map;

#ifndef MQTT_MOVEMENT_CONFIG_TOPIC
#define MQTT_MOVEMENT_CONFIG_TOPIC MQTT_MOVEMENT_CONFIG_TOPIC_DEFAULT
#endif
#ifndef MQTT_STATUS_TOPIC
#define MQTT_STATUS_TOPIC MQTT_STATUS_TOPIC_DEFAULT
#endif

static BOOL movement_command(DIRECTION direction, MOTOR_COMMAND *command)
{
    *command = (MOTOR_COMMAND){0};
    switch (direction) {
    case DIR_FRONT: command->target_speed_left = 40; command->target_speed_right = -40; break;
    case DIR_BACK: command->target_speed_left = -40; command->target_speed_right = 40; break;
    case DIR_LEFT: command->target_speed_left = -40; command->target_speed_right = -40; break;
    case DIR_RIGHT: command->target_speed_left = 40; command->target_speed_right = 40; break;
    case DIR_STOP: break;
    default: return FALSE;
    }
    return TRUE;
}

static const char *movement_name(DIRECTION direction)
{
    static const char *const names[] = {"MOVE_FORWARD", "MOVE_BACKWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"};
    return direction <= DIR_STOP ? names[direction] : "UNKNOWN";
}

static BOOL update_movement_mapping(const UB *payload, SZ length)
{
    UB next[MQTT_MAP_BYTES];
    if (!mqtt_decode_mapping(payload, length, next)) return FALSE;
    UINT saved = robot_data_lock();
    memcpy(movement_map.directions, next, sizeof(next));
    movement_map.ready = TRUE;
    robot_data_unlock(saved);
    for (UINT key = 33U; key <= 126U; ++key) {
        DIRECTION direction = robot_bits_get(next, key * 3U, 3U);
        if (direction == DIR_INVALID) continue;
        NAVIGATOR_MESSAGE message = {.type = MSG_WIFI_BARCODE_MAPPING};
        message.data.packed_command_mapping = (direction << 7) | key;
        if (tk_snd_mbf(navigator_mbfid, &message, sizeof(message), TMO_POL) < E_OK)
            tm_printf((UB *)"[mqtt] mapping message dropped\n");
    }
    return TRUE;
}

BOOL mqtt_get_movement_for_barcode(char barcode, MOTOR_COMMAND *command)
{
    UINT key = (UB)barcode;
    if (key >= 'a' && key <= 'z') key -= 'a' - 'A';
    if (!command || key < 33U || key > 126U) return FALSE;
    UINT saved = robot_data_lock();
    DIRECTION direction = movement_map.ready
        ? robot_bits_get(movement_map.directions, key * 3U, 3U) : DIR_INVALID;
    robot_data_unlock(saved);
    return movement_command(direction, command);
}

static UW mqtt_now_ms(void)
{
    return (UW)(time_us_64() / 1000U);
}

static void mqtt_request_done(void *arg, err_t error)
{
    (void)arg;
    if(error != ERR_OK) mqtt_last_error = error;
}

static void mqtt_data(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    (void)arg;
    if (!mqtt_state.payload_valid) return;
    if ((length && !data) || (UW)mqtt_state.payload_length + length > mqtt_state.payload_expected) {
        mqtt_state.payload_valid = FALSE;
        return;
    }
    if (length) memcpy(mqtt_payload + mqtt_state.payload_length, data, length);
    mqtt_state.payload_length += length;
    if (!(flags & MQTT_DATA_FLAG_LAST)) return;
    mqtt_state.payload_valid = FALSE;
    if (mqtt_state.payload_length != mqtt_state.payload_expected) return;
    mqtt_message_count++;
    if (!strcmp(mqtt_topic, MQTT_MOVEMENT_CONFIG_TOPIC)) {
        if (!update_movement_mapping(mqtt_payload, mqtt_state.payload_length))
            tm_printf((UB *)"[mqtt] invalid binary mapping ignored\n");
    } else if (!strcmp(mqtt_topic, MQTT_SUBSCRIBE_TOPIC)) {
        NAVIGATOR_MESSAGE command = {.type = MSG_REMOTE_CMD};
        MOTOR_COMMAND motor = {0};
        if (!mqtt_decode_command(mqtt_payload, mqtt_state.payload_length, &motor)) {
            tm_printf((UB *)"[mqtt] invalid binary command ignored\n");
            return;
        }
        command.data.remote_cmd = motor;
        if (tk_snd_mbf(navigator_mbfid, &command, sizeof(command), TMO_POL) < E_OK)
            tm_printf((UB *)"[mqtt] navigator queue full, cmd dropped\n");
    }
}

static void mqtt_incoming_publish(void *arg, const char *topic, u32_t total_length)
{
    (void)arg;
    SZ length = topic ? strlen(topic) : 0U;
    mqtt_state.payload_length = 0;
    mqtt_state.payload_expected = 0;
    mqtt_state.payload_valid = FALSE;
    mqtt_topic[0] = '\0';
    if (!length || length >= MQTT_TOPIC_MAX || !total_length || total_length > MQTT_PAYLOAD_MAX)
        return;
    memcpy(mqtt_topic, topic, length + 1);
    mqtt_state.payload_expected = total_length;
    mqtt_state.payload_valid = TRUE;
}

static void mqtt_connection(mqtt_client_t *client, void *arg,
                            mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;
    mqtt_state.pending = 0U;
    robot_flag_assign(ROBOT_FAULT_MQTT, status != MQTT_CONNECT_ACCEPTED);
    if(status == MQTT_CONNECT_ACCEPTED) {
        mqtt_state.connected = 1U;
        mqtt_last_error = 0;
        mqtt_next_telemetry_ms = 0U;
        mqtt_next_status_ms = 0U;
        tm_printf((UB *)"[mqtt] connected to " MQTT_BROKER_IP "\n");
        if(mqtt_subscribe(mqtt_client, MQTT_SUBSCRIBE_TOPIC, MQTT_QOS,
                          mqtt_request_done, NULL) != ERR_OK) {
            mqtt_last_error = ERR_CONN;
        }
        if(mqtt_subscribe(mqtt_client, MQTT_MOVEMENT_CONFIG_TOPIC, MQTT_QOS,
                          mqtt_request_done, NULL) != ERR_OK) {
            mqtt_last_error = ERR_CONN;
        }
    } else {
        mqtt_state.connected = 0U;
        mqtt_next_retry_ms = mqtt_now_ms() + MQTT_RETRY_MS;
        mqtt_last_error = (int)status;
        tm_printf((UB *)"[mqtt] disconnected, status=%d\n", (int)status);
    }
}

static void mqtt_begin(void)
{
    if(mqtt_client == NULL) {
        mqtt_client = mqtt_client_new();
        if(mqtt_client == NULL) {
            mqtt_last_error = ERR_MEM;
            mqtt_next_retry_ms = mqtt_now_ms() + MQTT_RETRY_MS;
            return;
        }
        mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish,
                    mqtt_data, NULL);
    }

    if(!mqtt_state.configured) {
        if(!ipaddr_aton(MQTT_BROKER_IP, &mqtt_broker)) {
            mqtt_last_error = ERR_ARG;
            mqtt_next_retry_ms = mqtt_now_ms() + MQTT_RETRY_MS;
            return;
        }
        mqtt_info.client_id = MQTT_CLIENT_ID;
        mqtt_info.client_user = MQTT_USERNAME[0] != '\0' ? MQTT_USERNAME : NULL;
        mqtt_info.client_pass = MQTT_PASSWORD[0] != '\0' ? MQTT_PASSWORD : NULL;
        mqtt_info.keep_alive = 30U;
        mqtt_info.will_topic = MQTT_STATUS_TOPIC;
        static const UB offline[MQTT_STATUS_BYTES] = {0x14U};
        mqtt_info.will_msg = (const char *)offline;
        mqtt_info.will_msg_len = sizeof(offline);
        mqtt_info.will_qos = 0U;
        mqtt_info.will_retain = 1U;
        mqtt_state.configured = 1U;
    }

    mqtt_state.pending = 1U;
    if(mqtt_client_connect(mqtt_client, &mqtt_broker, MQTT_BROKER_PORT,
                           mqtt_connection, NULL, &mqtt_info) != ERR_OK) {
        mqtt_state.pending = 0U;
        mqtt_last_error = ERR_CONN;
        mqtt_next_retry_ms = mqtt_now_ms() + MQTT_RETRY_MS;
    }
}

static void json_signed_number(char *output, SZ capacity, SZ *length, W value)
{
    UW magnitude = value < 0 ? (UW)(-(value + 1)) + 1U : (UW)value;
    if (value < 0) json_char(output, capacity, length, '-');
    json_number(output, capacity, length, magnitude);
}

static void json_fixed10(char *output, SZ capacity, SZ *length, H value)
{
    UINT magnitude = value < 0 ? -(INT)value : value;
    if (value < 0) json_char(output, capacity, length, '-');
    json_number(output, capacity, length, magnitude / 10U);
    json_char(output, capacity, length, '.');
    json_number(output, capacity, length, magnitude % 10U);
}

static const char *heading_to_cardinal(H deg)
{
    static const char *const dirs[8] = {
        "N", "NE", "E", "SE", "S", "SW", "W", "NW"
    };
    int normalized = ((int)deg % 360 + 360) % 360;
    int idx = ((normalized + 22) / 45) & 7;
    return dirs[idx];
}

SZ telemetry_json(char *output, SZ capacity)
{
    SZ length = 0U;
    TELEMETRY_SNAPSHOT snapshot;
    telemetry_read(&snapshot);

    if(capacity == 0U) return 0U;
    json_char(output, capacity, &length, '{');

    // High Priority: Navigation & Motion
    json_text(output, capacity, &length, "\"mode\":");
    json_number(output, capacity, &length, (UW)snapshot.movement_mode);
    json_text(output, capacity, &length, ",\"phase\":");
    json_number(output, capacity, &length, (UW)snapshot.bypass_phase);
    json_text(output, capacity, &length, ",\"ir\":");
    json_number(output, capacity, &length, (UW)snapshot.ir_state);

    char barcode_str[2] = { snapshot.last_barcode, '\0' };
    json_text(output, capacity, &length, ",\"barcode\":\"");
    if (barcode_str[0] >= 32 && barcode_str[0] <= 126) {
        if (barcode_str[0] == '"' || barcode_str[0] == '\\') json_char(output, capacity, &length, '\\');
        json_char(output, capacity, &length, barcode_str[0]);
    }
    json_text(output, capacity, &length, "\"");

    json_text(output, capacity, &length, ",\"inst_idx\":");
    json_number(output, capacity, &length, (UW)snapshot.instruction_idx);

    json_text(output, capacity, &length, ",\"speed_l\":");
    json_signed_number(output, capacity, &length, (W)snapshot.speed_left);
    json_text(output, capacity, &length, ",\"speed_r\":");
    json_signed_number(output, capacity, &length, (W)snapshot.speed_right);
    json_text(output, capacity, &length, ",\"enc_l\":");
    json_signed_number(output, capacity, &length, snapshot.enc_l);
    json_text(output, capacity, &length, ",\"enc_r\":");
    json_signed_number(output, capacity, &length, snapshot.enc_r);
    json_text(output, capacity, &length, ",\"dist_cm\":");
    json_signed_number(output, capacity, &length, snapshot.dist_cm);

    // Sensor Specific: Ultrasonic
    json_text(output, capacity, &length, ",\"obs_dist\":");
    json_number(output, capacity, &length, snapshot.obstacle_distance_cm);
    json_text(output, capacity, &length, ",\"obs_dist_valid\":");
    json_number(output, capacity, &length, snapshot.obstacle_distance_valid);
    json_text(output, capacity, &length, ",\"obs_status\":");
    json_number(output, capacity, &length, (UW)snapshot.obstacle_status);
    json_text(output, capacity, &length, ",\"closest_cm\":");
    json_number(output, capacity, &length, snapshot.closest_cm);
    json_text(output, capacity, &length, ",\"closest_deg\":");
    json_signed_number(output, capacity, &length, snapshot.closest_angle_deg);
    json_text(output, capacity, &length, ",\"obs_width\":");
    if (snapshot.obstacle_width_status == WIDTH_ESTIMATED)
        json_number(output, capacity, &length, snapshot.estimated_width_cm);
    else
        json_text(output, capacity, &length, "null");
    json_text(output, capacity, &length, ",\"obs_width_status\":");
    json_number(output, capacity, &length, snapshot.obstacle_width_status);

    // Sensor Specific: IMU
    json_text(output, capacity, &length, ",\"pitch\":");
    json_fixed10(output, capacity, &length, snapshot.pitch_ddeg);
    json_text(output, capacity, &length, ",\"roll\":");
    json_fixed10(output, capacity, &length, snapshot.roll_ddeg);
    json_text(output, capacity, &length, ",\"heading\":");
    json_signed_number(output, capacity, &length, (W)snapshot.heading_deg);
    json_text(output, capacity, &length, ",\"heading_dir\":\"");
    json_text(output, capacity, &length, heading_to_cardinal(snapshot.heading_deg));
    json_text(output, capacity, &length, "\"");
    json_text(output, capacity, &length, ",\"height_mm\":");
    json_fixed10(output, capacity, &length, snapshot.height_dmm);
    json_text(output, capacity, &length, ",\"imu_state\":");
    json_number(output, capacity, &length, (UW)snapshot.imu_state);
    json_text(output, capacity, &length, ",\"imu_event\":");
    json_number(output, capacity, &length, (UW)snapshot.imu_event);

    json_text(output, capacity, &length, ",\"seq\":");
    json_number(output, capacity, &length, snapshot.seq);
    json_char(output, capacity, &length, '}');

    if(length >= capacity) length = capacity - 1U;
    output[length] = '\0';
    return length;
}

void mqtt_dashboard_poll(struct netif *netif)
{
    BOOL link = netif && netif_is_up(netif) && netif_is_link_up(netif);
    robot_flag_assign(ROBOT_FAULT_WIFI, !link);
    robot_flag_assign(ROBOT_FAULT_MQTT, !link || !mqtt_state.connected);
    if (!link) return;
    UW now = mqtt_now_ms();
    if (!mqtt_state.connected && !mqtt_state.pending) {
        if ((W)(now - mqtt_next_retry_ms) >= 0) mqtt_begin();
        return;
    }
    if (!mqtt_state.connected) return;
    TELEMETRY_SNAPSHOT snapshot;
    telemetry_read(&snapshot);
    if ((W)(now - mqtt_next_telemetry_ms) >= 0) {
        UB payload[MQTT_TELEMETRY_BYTES];
        mqtt_encode_telemetry(payload, &snapshot, robot_flags_get());
        err_t result = mqtt_publish(mqtt_client, MQTT_PUBLISH_TOPIC, payload,
                                    sizeof(payload), MQTT_QOS, 0, mqtt_request_done, NULL);
        if (result != ERR_OK) mqtt_last_error = result;
        mqtt_next_telemetry_ms = now + 500U;
    }
    if ((W)(now - mqtt_next_status_ms) >= 0) {
        UB payload[MQTT_STATUS_BYTES] = {0x14U};
        robot_bits_set(payload, 8, 1, 1);
        robot_bits_set(payload, 9, 32, now);
        robot_bits_set(payload, 41, 32, snapshot.seq);
        err_t result = mqtt_publish(mqtt_client, MQTT_STATUS_TOPIC, payload,
                                    sizeof(payload), MQTT_QOS, 1, mqtt_request_done, NULL);
        if (result != ERR_OK) mqtt_last_error = result;
        mqtt_next_status_ms = now + MQTT_STATUS_INTERVAL_MS;
    }
}

void mqtt_dashboard_link_down(void)
{
    if (mqtt_client) {
        if (mqtt_state.connected) {
            static const UB offline[MQTT_STATUS_BYTES] = {0x14U};
            (void)mqtt_publish(mqtt_client, MQTT_STATUS_TOPIC, offline, sizeof(offline),
                               MQTT_QOS, 1, mqtt_request_done, NULL);
        }
        mqtt_disconnect(mqtt_client);
    }
    mqtt_state.connected = mqtt_state.pending = 0;
    mqtt_state.payload_valid = 0;
    robot_flags_update(ROBOT_FAULT_WIFI | ROBOT_FAULT_MQTT, 0);
    mqtt_next_retry_ms = mqtt_now_ms() + MQTT_RETRY_MS;
}

void json_char(char *output, SZ capacity, SZ *length, char value)
{
    if(*length + 1U < capacity) output[*length] = value;
    (*length)++;
}

void json_text(char *output, SZ capacity, SZ *length,
               const char *text)
{
    while (*text != '\0') {
        json_char(output, capacity, length, *text++);
    }
}

static void json_quoted_text(char *output, SZ capacity, SZ *length,
                             const char *text)
{
    while (*text != '\0') {
        char value = *text++;
        if (value == '"' || value == '\\') {
            json_char(output, capacity, length, '\\');
        }
        json_char(output, capacity, length, value);
    }
}

void json_number(char *output, SZ capacity, SZ *length,
                        UW value)
{
    char digits[11];
    SZ count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while(value != 0U && count < sizeof(digits));
    while(count > 0U) json_char(output, capacity, length, digits[--count]);
}

SZ mqtt_dashboard_json(char *output, SZ capacity)
{
    SZ length = 0U;
    UB map[MQTT_MAP_BYTES];
    UINT saved = robot_data_lock();
    BOOL ready = movement_map.ready;
    memcpy(map, movement_map.directions, sizeof(map));
    robot_data_unlock(saved);
    unsigned int i;
    BOOL first = TRUE;

    if(capacity == 0U) return 0U;
    json_char(output, capacity, &length, '{');
    json_text(output, capacity, &length, "\"enabled\":true,\"connected\":");
    json_text(output, capacity, &length, mqtt_state.connected ? "true" : "false");
    json_text(output, capacity, &length, ",\"messages\":");
    json_number(output, capacity, &length, mqtt_message_count);
    json_text(output, capacity, &length, ",\"topic\":\"");
    json_quoted_text(output, capacity, &length, mqtt_topic);
    json_text(output, capacity, &length, "\",\"payload\":\"");
    /* Preserve embedded zero bytes when showing binary messages. */
    static const char hex[] = "0123456789ABCDEF";
    for (UINT n = 0; n < mqtt_state.payload_length; ++n) {
        if (n) json_char(output, capacity, &length, ' ');
        json_char(output, capacity, &length, hex[mqtt_payload[n] >> 4]);
        json_char(output, capacity, &length, hex[mqtt_payload[n] & 15U]);
    }
    json_text(output, capacity, &length, "\",\"error\":");
    json_signed_number(output, capacity, &length, mqtt_last_error);
    json_text(output, capacity, &length, ",\"mapping\":{");
    for(i = 33U; i <= 126U; i++) {
        DIRECTION direction = robot_bits_get(map, i * 3U, 3U);
        if(ready && direction <= DIR_STOP) {
            if(!first) json_char(output, capacity, &length, ',');
            json_char(output, capacity, &length, '"');
            json_char(output, capacity, &length, (char)i);
            json_text(output, capacity, &length, "\":\"");
            json_text(output, capacity, &length,
                      movement_name(direction));
            json_char(output, capacity, &length, '"');
            first = FALSE;
        }
    }
    json_char(output, capacity, &length, '}');
    json_char(output, capacity, &length, '}');

    if(length >= capacity) length = capacity - 1U;
    output[length] = '\0';
    return length;
}

#else

void mqtt_dashboard_link_down(void) {}

void mqtt_dashboard_poll(struct netif *netif)
{
    (void)netif;
}

SZ mqtt_dashboard_json(char *output, SZ capacity)
{
    static const char disabled[] =
        "{\"enabled\":false,\"connected\":false,\"messages\":0,"\
        "\"topic\":\"\",\"payload\":\"\",\"error\":0,\"mapping\":{}}";
    SZ length = sizeof(disabled) - 1U;

    if(capacity == 0U) return 0U;
    if(length >= capacity) length = capacity - 1U;
    memcpy(output, disabled, length);
    output[length] = '\0';
    return length;
}

SZ telemetry_json(char *output, SZ capacity)
{
    static const char empty[] = "{}";
    SZ length = sizeof(empty) - 1U;

    if(capacity == 0U) return 0U;
    if(length >= capacity) length = capacity - 1U;
    memcpy(output, empty, length);
    output[length] = '\0';
    return length;
}

BOOL mqtt_get_movement_for_barcode(char barcode, MOTOR_COMMAND *command)
{
    (void)barcode;
    (void)command;
    return FALSE;
}

#endif
