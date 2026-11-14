#ifndef UTK_MQTT_CLIENT_H
#define UTK_MQTT_CLIENT_H

#ifndef PROHIBIT_DEF_SIZE_T
#define PROHIBIT_DEF_SIZE_T
#endif

#include "../Motor/Motor_task.h"
struct netif;

void mqtt_dashboard_poll(struct netif *netif);
SZ mqtt_dashboard_json(char *output, SZ capacity);
SZ telemetry_json(char *output, SZ capacity);
BOOL mqtt_get_movement_for_barcode(char barcode, MOTOR_COMMAND *command);

/* JSON helpers used by telemetry_json / mqtt_dashboard_json */
void json_char(char *output, SZ capacity, SZ *length, char value);
void json_text(char *output, SZ capacity, SZ *length, const char *text);
void json_number(char *output, SZ capacity, SZ *length, uint32_t value);

#endif
