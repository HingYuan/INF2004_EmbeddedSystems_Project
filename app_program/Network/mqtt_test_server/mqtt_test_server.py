"""Publish a retained movement mapping to the Pico W.

This is a test publisher, not a broker. Run it against Mosquitto, EMQX,
or another MQTT broker that the Pico W can reach.
"""

from __future__ import annotations

import argparse
import json
import sys

import paho.mqtt.client as mqtt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="MQTT broker IPv4 address or hostname")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--movement-topic", default="robot/config/movement")
    parser.add_argument(
        "--movement-config",
        default='{"w":"MOVE_FORWARD","s":"MOVE_BACKWARD","a":"TURN_LEFT","d":"TURN_RIGHT","e":"STOP"}',
        help="JSON movement mapping to publish retained",
    )
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

    client.loop_stop()
    client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
