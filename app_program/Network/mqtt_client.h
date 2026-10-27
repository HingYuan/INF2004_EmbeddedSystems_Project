#ifndef UTK_MQTT_CLIENT_H
#define UTK_MQTT_CLIENT_H

#include <stddef.h>
#include <stdint.h>

struct netif;

void mqtt_dashboard_poll(struct netif *netif);
size_t mqtt_dashboard_json(char *output, size_t capacity);
size_t telemetry_json(char *output, size_t capacity);

/* JSON helpers used by telemetry_json / mqtt_dashboard_json */
void json_char(char *output, size_t capacity, size_t *length, char value);
void json_text(char *output, size_t capacity, size_t *length, const char *text);
void json_number(char *output, size_t capacity, size_t *length, uint32_t value);

#endif
