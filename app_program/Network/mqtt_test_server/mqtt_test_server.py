"""Publish repeatable test messages and retained movement mapping.

This is a test publisher, not a broker. Run it against Mosquitto, EMQX,
or another MQTT broker that the Pico W can reach.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="MQTT broker IPv4 address or hostname")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--topic", default="dashboard/test")
    parser.add_argument("--movement-topic", default="robot/config/movement")
    parser.add_argument(
        "--movement-config",
        default='{"a":"MOVE_FORWARD","b":"MOVE_BACKWARD","c":"TURN_LEFT","d":"TURN_RIGHT","e":"STOP"}',
        help="JSON movement mapping to publish retained",
    )
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--username")
    parser.add_argument("--password")
    return parser.parse_args()


def main() -> int:
    arguments = parse_args()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="utk-dashboard-test")
    if arguments.username is not None:
        client.username_pw_set(arguments.username, arguments.password or "")

    try:
        client.connect(arguments.host, arguments.port, keepalive=30)
    except OSError as error:
        print(f"Could not connect to MQTT broker: {error}", file=sys.stderr)
        return 1

    client.loop_start()
    try:
        movement_config = json.loads(arguments.movement_config)
        if not isinstance(movement_config, dict):
            raise ValueError("movement config must be a JSON object")
        result = client.publish(
            arguments.movement_topic,
            json.dumps(movement_config, separators=(",", ":")),
            qos=1,
            retain=True,
        )
        result.wait_for_publish()
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"Movement mapping publish failed: {mqtt.error_string(result.rc)}", file=sys.stderr)
            return 1
        print(f"Published retained movement mapping to {arguments.movement_topic}")
    except (ValueError, json.JSONDecodeError) as error:
        print(f"Invalid movement mapping: {error}", file=sys.stderr)
        return 2

    sequence = 0
    print(f"Publishing to {arguments.host}:{arguments.port}/{arguments.topic}")
    try:
        while True:
            t = sequence * 0.3
            payload = {
                "sequence": sequence,
                "accel_x": round(math.sin(t) * 0.98, 3),
                "accel_y": round(math.cos(t) * 0.12, 3),
                "accel_z": round(9.81 + math.sin(t * 0.5) * 0.05, 3),
                "gyro_x": round(math.sin(t * 2) * 1.5, 2),
                "gyro_y": round(math.cos(t * 1.7) * 0.8, 2),
                "gyro_z": round(math.sin(t * 0.9) * 2.1, 2),
                "servo_angle": round(90 + 45 * math.sin(t * 0.4)),
                "servo_pwm": round(1500 + 500 * math.sin(t * 0.4)),
                "temperature_c": round(22.5 + (sequence % 10) * 0.2, 1),
                "humidity": round(55.0 + math.sin(t * 0.2) * 10, 1),
                "source": "python-test-server",
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
            result = client.publish(arguments.topic, json.dumps(payload), qos=0, retain=False)
            result.wait_for_publish()
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"Publish failed: {mqtt.error_string(result.rc)}", file=sys.stderr)
            else:
                print(f"[{sequence}] {payload}")
            sequence += 1
            time.sleep(arguments.interval)
    except KeyboardInterrupt:
        print("Stopping")
    finally:
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
