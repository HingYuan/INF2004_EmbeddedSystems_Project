#ifndef UTK_MQTT_CLIENT_H
#define UTK_MQTT_CLIENT_H

#include <stddef.h>

struct netif;

void mqtt_dashboard_poll(struct netif *netif);
size_t mqtt_dashboard_json(char *output, size_t capacity);
size_t telemetry_json(char *output, size_t capacity);

#endif
