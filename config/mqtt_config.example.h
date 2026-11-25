/**
 * @file mqtt_config.example.h
 * @brief Example MQTT broker configuration template.
 *
 * NOTE: Copy this file to config/mqtt_config.h, update the broker settings,
 * and set UTK_MQTT_CONFIG_SET to 1u before building with MQTT=1.
 *
 * BARR-C Deviation (Rules 4.1.a, 4.1.b, 4.2.a, 4.2.b):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: This template header uses the .example.h suffix, is copied to
 *   untracked config/mqtt_config.h by the developer, has no matching .c file,
 *   and uses include guard MQTT_CONFIG_H matching its target filename.
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define UTK_MQTT_CONFIG_SET         (0u)

#if (0u == UTK_MQTT_CONFIG_SET)
#error "Set broker values and UTK_MQTT_CONFIG_SET to 1u in mqtt_config.h."
#endif

#define MQTT_BROKER_IP              "192.168.1.10"
#define MQTT_BROKER_PORT            (1883u)
#define MQTT_CLIENT_ID              "utk-picow"
#define MQTT_SUBSCRIBE_TOPIC        "robot/command"
#define MQTT_MOVEMENT_CONFIG_TOPIC  "robot/config/movement"
#define MQTT_PUBLISH_TOPIC          "robot/telemetry"
#define MQTT_STATUS_TOPIC           "robot/status"
#define MQTT_USERNAME               ""
#define MQTT_PASSWORD               ""
#define MQTT_QOS                    (0u)

#endif /* MQTT_CONFIG_H */

/*** end of file ***/
