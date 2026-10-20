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

static mqtt_client_t *mqtt_client;
static ip_addr_t mqtt_broker;
static struct mqtt_connect_client_info_t mqtt_info;
static char mqtt_topic[MQTT_TOPIC_MAX];
static char mqtt_payload[MQTT_PAYLOAD_MAX];
static size_t mqtt_payload_length;
static uint32_t mqtt_message_count;
static uint32_t mqtt_next_retry_ms;
static uint8_t mqtt_configured;
static uint8_t mqtt_connect_pending;
static uint8_t mqtt_connected;
static int mqtt_last_error;

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
    size_t available;

    (void)arg;
    available = MQTT_PAYLOAD_MAX - 1U - mqtt_payload_length;
    if(data != NULL && length > 0U && available > 0U) {
        if((size_t)length > available) length = (u16_t)available;
        memcpy(&mqtt_payload[mqtt_payload_length], data, length);
        mqtt_payload_length += length;
    }

    if((flags & MQTT_DATA_FLAG_LAST) != 0U) {
        mqtt_payload[mqtt_payload_length] = '\0';
        mqtt_message_count++;
        tm_printf((UB *)"[mqtt] #%u %s: %s\n",
                  mqtt_message_count, mqtt_topic, mqtt_payload);

        /* If message is a command for the robot, parse L,R,T and send to navigator */
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
            nav_cmd.data.remote_cmd.turn_ticks = (int16_t)t;

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
    size_t length;

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
        tm_printf((UB *)"[mqtt] connected to " MQTT_BROKER_IP "\n");
        if(mqtt_subscribe(mqtt_client, MQTT_SUBSCRIBE_TOPIC, MQTT_QOS,
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
        mqtt_info.will_topic = NULL;
        mqtt_info.will_msg = NULL;
        mqtt_info.will_msg_len = 0U;
        mqtt_info.will_qos = 0U;
        mqtt_info.will_retain = 0U;
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

#define MQTT_TELEMETRY_INTERVAL_MS 500U
static uint32_t mqtt_next_telemetry_ms;

static void json_signed_number(char *output, size_t capacity, size_t *length,
                               int32_t value)
{
    if(value < 0) {
        json_char(output, capacity, length, '-');
        value = -value;
    }
    json_number(output, capacity, length, (uint32_t)value);
}

size_t telemetry_json(char *output, size_t capacity)
{
    size_t length = 0U;

    if(capacity == 0U) return 0U;
    json_char(output, capacity, &length, '{');
    json_text(output, capacity, &length, "\"speed_l\":");
    json_signed_number(output, capacity, &length, (int32_t)telemetry_snapshot.speed_left);
    json_text(output, capacity, &length, ",\"speed_r\":");
    json_signed_number(output, capacity, &length, (int32_t)telemetry_snapshot.speed_right);
    json_text(output, capacity, &length, ",\"ir\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.ir_state);
    json_text(output, capacity, &length, ",\"mode\":");
    json_number(output, capacity, &length, (uint32_t)telemetry_snapshot.movement_mode);
    json_text(output, capacity, &length, ",\"enc_l\":");
    json_signed_number(output, capacity, &length, telemetry_snapshot.enc_l);
    json_text(output, capacity, &length, ",\"enc_r\":");
    json_signed_number(output, capacity, &length, telemetry_snapshot.enc_r);
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
            char telem_buf[160];
            size_t len = telemetry_json(telem_buf, sizeof(telem_buf));
            if(len > 0) {
#ifdef MQTT_PUBLISH_TOPIC
                mqtt_publish(mqtt_client, MQTT_PUBLISH_TOPIC, telem_buf, (u16_t)len,
                             MQTT_QOS, 0, mqtt_request_done, NULL);
#else
                mqtt_publish(mqtt_client, "robot/telemetry", telem_buf, (u16_t)len,
                             MQTT_QOS, 0, mqtt_request_done, NULL);
#endif
            }
            mqtt_next_telemetry_ms = now + MQTT_TELEMETRY_INTERVAL_MS;
        }
    }
}

static void json_char(char *output, size_t capacity, size_t *length, char value)
{
    if(*length + 1U < capacity) output[*length] = value;
    (*length)++;
}

static void json_text(char *output, size_t capacity, size_t *length,
                      const char *text)
{
    while(*text != '\0') {
        char value = *text++;
        if(value == '"' || value == '\\') {
            json_char(output, capacity, length, '\\');
        } else if((unsigned char)value < 0x20U) {
            value = ' ';
        }
        json_char(output, capacity, length, value);
    }
}

static void json_number(char *output, size_t capacity, size_t *length,
                        uint32_t value)
{
    char digits[11];
    size_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while(value != 0U && count < sizeof(digits));
    while(count > 0U) json_char(output, capacity, length, digits[--count]);
}

size_t mqtt_dashboard_json(char *output, size_t capacity)
{
    size_t length = 0U;

    if(capacity == 0U) return 0U;
    json_char(output, capacity, &length, '{');
    json_text(output, capacity, &length, "\"enabled\":true,\"connected\":");
    json_text(output, capacity, &length, mqtt_connected ? "true" : "false");
    json_text(output, capacity, &length, ",\"messages\":");
    json_number(output, capacity, &length, mqtt_message_count);
    json_text(output, capacity, &length, ",\"topic\":\"");
    json_text(output, capacity, &length, mqtt_topic);
    json_text(output, capacity, &length, "\",\"payload\":\"");
    json_text(output, capacity, &length, mqtt_payload);
    json_text(output, capacity, &length, "\",\"error\":");
    json_number(output, capacity, &length, (uint32_t)mqtt_last_error);
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

size_t mqtt_dashboard_json(char *output, size_t capacity)
{
    static const char disabled[] =
        "{\"enabled\":false,\"connected\":false,\"messages\":0,"\
        "\"topic\":\"\",\"payload\":\"\",\"error\":0}";
    size_t length = sizeof(disabled) - 1U;

    if(capacity == 0U) return 0U;
    if(length >= capacity) length = capacity - 1U;
    memcpy(output, disabled, length);
    output[length] = '\0';
    return length;
}

size_t telemetry_json(char *output, size_t capacity)
{
    static const char empty[] =
        "{\"speed_l\":0,\"speed_r\":0,\"ir\":0,\"mode\":0,\"enc_l\":0,\"enc_r\":0,\"seq\":0}";
    size_t length = sizeof(empty) - 1U;

    if(capacity == 0U) return 0U;
    if(length >= capacity) length = capacity - 1U;
    memcpy(output, empty, length);
    output[length] = '\0';
    return length;
}

#endif
