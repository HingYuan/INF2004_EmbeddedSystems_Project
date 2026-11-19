#ifndef UTK_MQTT_CONFIG_H
#define UTK_MQTT_CONFIG_H

/* Copy this file to config/mqtt_config.h and set the broker values. */
#define UTK_MQTT_CONFIG_SET 0

#if !UTK_MQTT_CONFIG_SET
#error "config/mqtt_config.h: set the broker address, topic, and UTK_MQTT_CONFIG_SET to 1."
#endif

#define MQTT_BROKER_IP        "192.168.1.10"
#define MQTT_BROKER_PORT      1883
#define MQTT_CLIENT_ID        "utk-picow"
#define MQTT_SUBSCRIBE_TOPIC  "robot/command"
#define MQTT_MOVEMENT_CONFIG_TOPIC "robot/config/movement"
#define MQTT_PUBLISH_TOPIC    "robot/telemetry"
#define MQTT_STATUS_TOPIC     "robot/status"
#define MQTT_USERNAME         ""
#define MQTT_PASSWORD         ""
#define MQTT_QOS              0

#endif
