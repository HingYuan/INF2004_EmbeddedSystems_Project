/* Prevent syslib.h from redefining size_t so stddef.h (unsigned int) wins */
#define PROHIBIT_DEF_SIZE_T

#include <stddef.h>
#include "mqtt_client.h"
#include <string.h>

#include <tm/tmonitor.h>

#if TM_MQTT

#include "lwip/apps/mqtt.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "mqtt_config.h"
#include <tk/tkernel.h>
#include "../robot_tasks.h"
#include "../Navigator/Navigator_task.h"

extern uint64_t time_us_64(void);

#define MQTT_RETRY_MS       5000U
#define MQTT_TOPIC_MAX      96U
#define MQTT_PAYLOAD_MAX    256U
#define MQTT_MOVEMENT_CONFIG_TOPIC_DEFAULT "robot/config/movement"
#define MQTT_STATUS_TOPIC_DEFAULT "robot/status"
#define MQTT_STATUS_INTERVAL_MS 5000U

static mqtt_client_t *mqtt_client;
static ip_addr_t mqtt_broker;
static struct mqtt_connect_client_info_t mqtt_info;
static char mqtt_topic[MQTT_TOPIC_MAX];
static char mqtt_payload[MQTT_PAYLOAD_MAX];
static SZ mqtt_payload_length;
static uint32_t mqtt_message_count;
static uint32_t mqtt_next_retry_ms;
static uint32_t mqtt_next_telemetry_ms;
static uint32_t mqtt_next_status_ms;
static uint8_t mqtt_configured;
static uint8_t mqtt_connect_pending;
static uint8_t mqtt_connected;
static int mqtt_last_error;
static MOTOR_COMMAND movement_commands[2][128];
static uint8_t movement_valid[2][128];
static volatile uint8_t movement_active;

#ifndef MQTT_MOVEMENT_CONFIG_TOPIC
#define MQTT_MOVEMENT_CONFIG_TOPIC MQTT_MOVEMENT_CONFIG_TOPIC_DEFAULT
#endif
#ifndef MQTT_STATUS_TOPIC
#define MQTT_STATUS_TOPIC MQTT_STATUS_TOPIC_DEFAULT
#endif

static const char *skip_json_space(const char *text)
{
    while(*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') text++;
    return text;
}

static BOOL json_string(const char **cursor, char *output, SZ capacity)
{
    const char *text = *cursor;
    SZ length = 0U;

    if(*text++ != '"') return FALSE;
    while(*text != '\0' && *text != '"') {
        if(*text == '\\' || length + 1U >= capacity) return FALSE;
        output[length++] = *text++;
    }
    if(*text != '"') return FALSE;
    output[length] = '\0';
    *cursor = text + 1;
    return TRUE;
}

static BOOL movement_command(const char *name, MOTOR_COMMAND *command)
{
    command->turn_ticks_left = 0;
    command->turn_ticks_right = 0;
    if(strcmp(name, "MOVE_FORWARD") == 0) {
        command->target_speed_left = 40;
        command->target_speed_right = -40;
    } else if(strcmp(name, "MOVE_BACKWARD") == 0) {
        command->target_speed_left = -40;
        command->target_speed_right = 40;
    } else if(strcmp(name, "TURN_LEFT") == 0) {
        command->target_speed_left = -40;
        command->target_speed_right = -40;
    } else if(strcmp(name, "TURN_RIGHT") == 0) {
        command->target_speed_left = 40;
        command->target_speed_right = 40;
    } else if(strcmp(name, "STOP") == 0) {
        command->target_speed_left = 0;
        command->target_speed_right = 0;
    } else {
        return FALSE;
    }
    return TRUE;
}

static const char *movement_name(const MOTOR_COMMAND *command)
{
    if(command->target_speed_left == 40 &&
       command->target_speed_right == -40) return "MOVE_FORWARD";
    if(command->target_speed_left == -40 &&
       command->target_speed_right == 40) return "MOVE_BACKWARD";
    if(command->target_speed_left == -40 &&
       command->target_speed_right == -40) return "TURN_LEFT";
    if(command->target_speed_left == 40 &&
       command->target_speed_right == 40) return "TURN_RIGHT";
    if(command->target_speed_left == 0 &&
       command->target_speed_right == 0) return "STOP";
    return "UNKNOWN";
}

static DIRECTION movement_direction(const MOTOR_COMMAND *command)
{
    if(command->target_speed_left == 40 &&
       command->target_speed_right == -40) return DIR_FRONT;
    if(command->target_speed_left == -40 &&
       command->target_speed_right == 40) return DIR_BACK;
    if(command->target_speed_left == -40 &&
       command->target_speed_right == -40) return DIR_LEFT;
    if(command->target_speed_left == 40 &&
       command->target_speed_right == 40) return DIR_RIGHT;
    if(command->target_speed_left == 0 &&
       command->target_speed_right == 0) return DIR_STOP;
    return DIR_INVALID;
}

static BOOL update_movement_mapping(const char *payload)
{
    const char *cursor = skip_json_space(payload);
    uint8_t next = (uint8_t)(movement_active ^ 1U);
    uint8_t count = 0U;
    SZ key_length;
    char key[8];
    char value[24];

    memset(movement_valid[next], 0, sizeof(movement_valid[next]));
    if(*cursor++ != '{') return FALSE;
    cursor = skip_json_space(cursor);
    if(*cursor == '}') return FALSE;

    while(1) {
        cursor = skip_json_space(cursor);
        if(!json_string(&cursor, key, sizeof(key))) return FALSE;
        if(key[0] >= 'a' && key[0] <= 'z') key[0] = (char)(key[0] - 'a' + 'A');
        key_length = strlen(key);
        if(key_length != 1U || (unsigned char)key[0] < 33U ||
           (unsigned char)key[0] > 126U || key[0] == '"' || key[0] == '\\') {
            return FALSE;
        }
        cursor = skip_json_space(cursor);
        if(*cursor++ != ':') return FALSE;
        cursor = skip_json_space(cursor);
        if(!json_string(&cursor, value, sizeof(value)) ||
           !movement_command(value, &movement_commands[next][(unsigned char)key[0]]) ||
           movement_valid[next][(unsigned char)key[0]]) {
            return FALSE;
        }
        movement_valid[next][(unsigned char)key[0]] = 1U;
        count++;
        cursor = skip_json_space(cursor);
        if(*cursor == '}') {
            cursor = skip_json_space(cursor + 1);
            if(*cursor != '\0' || count == 0U) return FALSE;
            movement_active = next;
            for(SZ key = 33U; key <= 126U; key++) {
                if(movement_valid[next][key]) {
                    NAVIGATOR_MESSAGE message = {0};
                    DIRECTION direction =
                        movement_direction(&movement_commands[next][key]);
                    if(direction == DIR_INVALID) return FALSE;
                    message.type = MSG_WIFI_BARCODE_MAPPING;
                    message.data.packed_command_mapping =
                        (UH)(((UH)direction << 8) | (UH)key);
                    if(tk_snd_mbf(navigator_mbfid, &message,
                                  sizeof(message), TMO_POL) < E_OK) {
                        tm_printf((UB *)"[mqtt] mapping message dropped\n");
                    }
                }
            }
            return TRUE;
        }
        if(*cursor++ != ',') return FALSE;
    }
}

BOOL mqtt_get_movement_for_barcode(char barcode, MOTOR_COMMAND *command)
{
    uint8_t active;
    unsigned char index = (unsigned char)barcode;

    if(index >= 'a' && index <= 'z') index = (unsigned char)(index - 'a' + 'A');
    if(command == NULL || index < 33U || index > 126U) return FALSE;
    active = movement_active;
    if(!movement_valid[active][index]) return FALSE;
    *command = movement_commands[active][index];
    return TRUE;
}

static uint32_t mqtt_now_ms(void)
{
    return (uint32_t)(time_us_64() / 1000U);
}

static void mqtt_request_done(void *arg, err_t error)
{
    (void)arg;
    if(error != ERR_OK) mqtt_last_error = error;
}

static void mqtt_data(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    SZ available;

    (void)arg;
    available = MQTT_PAYLOAD_MAX - 1U - mqtt_payload_length;
    if(data != NULL && length > 0U && available > 0U) {
        if((SZ)length > available) length = (u16_t)available;
        memcpy(&mqtt_payload[mqtt_payload_length], data, length);
        mqtt_payload_length += length;
    }

    if((flags & MQTT_DATA_FLAG_LAST) != 0U) {
        mqtt_payload[mqtt_payload_length] = '\0';
        mqtt_message_count++;
        tm_printf((UB *)"[mqtt] #%u %s: %s\n",
                  mqtt_message_count, mqtt_topic, mqtt_payload);

        if(strcmp(mqtt_topic, MQTT_MOVEMENT_CONFIG_TOPIC) == 0) {
            if(!update_movement_mapping(mqtt_payload)) {
                tm_printf((UB *)"[mqtt] invalid movement mapping ignored\n");
            }
            return;
        }

        /* If message is a command for the robot, parse L,R,T and send to navigator */
        if(strcmp(mqtt_topic, MQTT_SUBSCRIBE_TOPIC) != 0) return;
        int l = 0, r = 0, t = 0;
        char *p = mqtt_payload;
        int sign = 1, val = 0;

        /* Parse speed left */
        while(*p == ' ') p++;
        if(*p == '-') { sign = -1; p++; } else if(*p == '+') p++;
        while(*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
        l = sign * val;

        if(*p == ',') {
            p++;
            while(*p == ' ') p++;
            sign = 1; val = 0;
            if(*p == '-') { sign = -1; p++; } else if(*p == '+') p++;
            while(*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
            r = sign * val;

            if(*p == ',') {
                p++;
                while(*p == ' ') p++;
                sign = 1; val = 0;
                if(*p == '-') { sign = -1; p++; } else if(*p == '+') p++;
                while(*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
                t = sign * val;
            }

            NAVIGATOR_MESSAGE nav_cmd;
            nav_cmd.type = MSG_REMOTE_CMD;
            nav_cmd.data.remote_cmd.target_speed_left = (int16_t)l;
            nav_cmd.data.remote_cmd.target_speed_right = (int16_t)r;
            nav_cmd.data.remote_cmd.turn_ticks_left = (int16_t)t;

            ER er = tk_snd_mbf(navigator_mbfid, &nav_cmd, sizeof(NAVIGATOR_MESSAGE), TMO_POL);
            if(er < E_OK) {
                tm_printf((UB *)"[mqtt] navigator queue full, cmd dropped\n");
            }
        }
    }
}

static void mqtt_incoming_publish(void *arg, const char *topic,
                                  u32_t total_length)
{
    SZ length;

    (void)arg;
    length = strlen(topic != NULL ? topic : "");
    if(length >= MQTT_TOPIC_MAX) length = MQTT_TOPIC_MAX - 1U;
    if(topic != NULL) memcpy(mqtt_topic, topic, length);
    mqtt_topic[length] = '\0';
    mqtt_payload_length = 0U;
    mqtt_payload[0] = '\0';
    (void)total_length;
}

static void mqtt_connection(mqtt_client_t *client, void *arg,
                            mqtt_connection_status_t status)
{
    (void)client;
    (void)arg;
    mqtt_connect_pending = 0U;
    if(status == MQTT_CONNECT_ACCEPTED) {
        mqtt_connected = 1U;
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
        mqtt_connected = 0U;
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

    if(!mqtt_configured) {
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
        mqtt_info.will_msg = "{\"online\":false}";
        mqtt_info.will_msg_len = 0U;
        mqtt_info.will_qos = 0U;
        mqtt_info.will_retain = 1U;
        mqtt_configured = 1U;
    }

    mqtt_connect_pending = 1U;
    if(mqtt_client_connect(mqtt_client, &mqtt_broker, MQTT_BROKER_PORT,
                           mqtt_connection, NULL, &mqtt_info) != ERR_OK) {
        mqtt_connect_pending = 0U;
        mqtt_last_error = ERR_CONN;
        mqtt_next_retry_ms = mqtt_now_ms() + MQTT_RETRY_MS;
    }
}

static void json_signed_number(char *output, SZ capacity, SZ *length,
                               int32_t value)
{
    if(value < 0) {
        json_char(output, capacity, length, '-');
        value = -value;
    }
    json_number(output, capacity, length, (uint32_t)value);
}

static void json_float(char *output, SZ capacity, SZ *length, float value)
{
    if(value < 0.0f) {
        json_char(output, capacity, length, '-');
        value = -value;
    }
    int32_t whole = (int32_t)value;
    int32_t frac = (int32_t)((value - (float)whole) * 10.0f);
    if(frac < 0) frac = 0;
    json_number(output, capacity, length, (uint32_t)whole);
    json_char(output, capacity, length, '.');
    json_number(output, capacity, length, (uint32_t)frac);
}

SZ telemetry_json(char *output, SZ capacity)
{
    SZ length = 0U;

    if(capacity == 0U) return 0U;
    json_char(output, capacity, &length, '{');

    // High Priority: Navigation & Motion
    json_text(output, capacity, &length, "\"mode\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.movement_mode);
    json_text(output, capacity, &length, ",\"phase\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.bypass_phase);
    json_text(output, capacity, &length, ",\"ir\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.ir_state);

    char barcode_str[2] = { telemetry_snapshot.last_barcode, '\0' };
    json_text(output, capacity, &length, ",\"barcode\":\"");
    if (barcode_str[0] >= 32 && barcode_str[0] <= 126) {
        json_char(output, capacity, &length, barcode_str[0]);
    }
    json_text(output, capacity, &length, "\"");

    json_text(output, capacity, &length, ",\"inst_idx\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.instruction_idx);

    json_text(output, capacity, &length, ",\"speed_l\":");
    json_signed_number(output, capacity, &length, (int32_t)telemetry_snapshot.speed_left);
    json_text(output, capacity, &length, ",\"speed_r\":");
    json_signed_number(output, capacity, &length, (int32_t)telemetry_snapshot.speed_right);
    json_text(output, capacity, &length, ",\"enc_l\":");
    json_signed_number(output, capacity, &length, telemetry_snapshot.enc_l);
    json_text(output, capacity, &length, ",\"enc_r\":");
    json_signed_number(output, capacity, &length, telemetry_snapshot.enc_r);
    json_text(output, capacity, &length, ",\"dist_cm\":");
    json_signed_number(output, capacity, &length, telemetry_snapshot.dist_cm);

    // Sensor Specific: Ultrasonic
    json_text(output, capacity, &length, ",\"obs_dist\":");
    json_number(output, capacity, &length, telemetry_snapshot.obstacle_distance_cm);
    json_text(output, capacity, &length, ",\"obs_dist_valid\":");
    json_number(output, capacity, &length, telemetry_snapshot.obstacle_distance_valid);
    json_text(output, capacity, &length, ",\"closest_cm\":");
    json_number(output, capacity, &length, telemetry_snapshot.closest_cm);
    json_text(output, capacity, &length, ",\"closest_deg\":");
    json_signed_number(output, capacity, &length, telemetry_snapshot.closest_angle_deg);
    json_text(output, capacity, &length, ",\"obs_width\":");
    json_number(output, capacity, &length, telemetry_snapshot.estimated_width_cm);

    // Sensor Specific: IMU
    json_text(output, capacity, &length, ",\"pitch\":");
    json_float(output, capacity, &length, telemetry_snapshot.pitch_deg);
    json_text(output, capacity, &length, ",\"roll\":");
    json_float(output, capacity, &length, telemetry_snapshot.roll_deg);
    json_text(output, capacity, &length, ",\"height_mm\":");
    json_float(output, capacity, &length, telemetry_snapshot.estimated_height_mm);
    json_text(output, capacity, &length, ",\"imu_state\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.imu_state);
    json_text(output, capacity, &length, ",\"imu_event\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.imu_event);

    json_text(output, capacity, &length, ",\"seq\":");
    json_number(output, capacity, &length, telemetry_snapshot.seq);
    json_char(output, capacity, &length, '}');

    if(length >= capacity) length = capacity - 1U;
    output[length] = '\0';
    return length;
}

void mqtt_dashboard_poll(struct netif *netif)
{
    uint32_t now;

    if(netif == NULL || !netif_is_up(netif) || !netif_is_link_up(netif)) return;

    now = mqtt_now_ms();

    if(!mqtt_connected && !mqtt_connect_pending) {
        if((int32_t)(now - mqtt_next_retry_ms) >= 0) {
            mqtt_begin();
        }
        return;
    }

    if(mqtt_connected) {
        if((int32_t)(now - mqtt_next_telemetry_ms) >= 0) {
            char telem_buf[320];
            SZ len = telemetry_json(telem_buf, sizeof(telem_buf));
            if(len > 0) {
                err_t result = mqtt_publish(
                    mqtt_client, MQTT_PUBLISH_TOPIC, telem_buf, (u16_t)len,
                    MQTT_QOS, 0, mqtt_request_done, NULL);
                if(result != ERR_OK) {
                    mqtt_last_error = result;
                    tm_printf((UB *)"[mqtt] telemetry publish failed: %d\n",
                              result);
                }
            }
            mqtt_next_telemetry_ms = now + 500U;
        }

        if((int32_t)(now - mqtt_next_status_ms) >= 0) {
            char status_payload[96];
            int length = tm_sprintf(
                (UB *)status_payload,
                (UB *)"{\"online\":true,\"uptime_ms\":%u,\"telemetry_seq\":%u}",
                now, telemetry_snapshot.seq);
            err_t result = mqtt_publish(
                mqtt_client, MQTT_STATUS_TOPIC, status_payload,
                (u16_t)length, MQTT_QOS, 1, mqtt_request_done, NULL);
            if(result != ERR_OK) {
                mqtt_last_error = result;
                tm_printf((UB *)"[mqtt] status publish failed: %d\n", result);
            }
            mqtt_next_status_ms = now + MQTT_STATUS_INTERVAL_MS;
        }
    }
}

void mqtt_dashboard_link_down(void)
{
    if(mqtt_client != NULL && mqtt_connected) {
        static const char offline[] = "{\"online\":false}";
        err_t result = mqtt_publish(
            mqtt_client, MQTT_STATUS_TOPIC, offline,
            (u16_t)(sizeof(offline) - 1U), MQTT_QOS, 1,
            mqtt_request_done, NULL);
        if(result != ERR_OK) {
            mqtt_last_error = result;
            tm_printf((UB *)"[mqtt] offline status publish failed: %d\n",
                      result);
        }
        mqtt_disconnect(mqtt_client);
    }
    mqtt_connected = 0U;
    mqtt_connect_pending = 0U;
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
                        uint32_t value)
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
    uint8_t active = movement_active;
    unsigned int i;
    BOOL first = TRUE;

    if(capacity == 0U) return 0U;
    json_char(output, capacity, &length, '{');
    json_text(output, capacity, &length, "\"enabled\":true,\"connected\":");
    json_text(output, capacity, &length, mqtt_connected ? "true" : "false");
    json_text(output, capacity, &length, ",\"messages\":");
    json_number(output, capacity, &length, mqtt_message_count);
    json_text(output, capacity, &length, ",\"topic\":\"");
    json_quoted_text(output, capacity, &length, mqtt_topic);
    json_text(output, capacity, &length, "\",\"payload\":\"");
    json_quoted_text(output, capacity, &length, mqtt_payload);
    json_text(output, capacity, &length, "\",\"error\":");
    json_number(output, capacity, &length, (uint32_t)mqtt_last_error);
    json_text(output, capacity, &length, ",\"mapping\":{");
    for(i = 33U; i <= 126U; i++) {
        if(movement_valid[active][i]) {
            if(!first) json_char(output, capacity, &length, ',');
            json_char(output, capacity, &length, '"');
            json_char(output, capacity, &length, (char)i);
            json_text(output, capacity, &length, "\":\"");
            json_text(output, capacity, &length,
                      movement_name(&movement_commands[active][i]));
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

void mqtt_dashboard_poll(struct netif *netif)
{
    (void)netif;
}

void mqtt_dashboard_link_down(void)
{
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
