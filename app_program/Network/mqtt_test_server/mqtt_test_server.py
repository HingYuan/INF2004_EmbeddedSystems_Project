"""Publish a retained movement mapping to the Pico W.

This is a test publisher, not a broker. Run it against Mosquitto, EMQX,
or another MQTT broker that the Pico W can reach.
"""

from __future__ import annotations

import argparse
import json
import sys

import paho.mqtt.client as mqtt


DIRECTIONS = {
    "MOVE_FORWARD": 0,
    "MOVE_BACKWARD": 1,
    "TURN_LEFT": 2,
    "TURN_RIGHT": 3,
    "STOP": 4,
}


def encode_movement_mapping(mapping: dict) -> bytes:
    entries = []
    seen = set()
    for key, direction_name in mapping.items():
        if not isinstance(key, str) or len(key) != 1:
            raise ValueError("movement mapping keys must be single characters")
        key_code = ord(key)
        if key_code < 33 or key_code > 126 or key in ('"', "\\"):
            raise ValueError(f"unsupported movement mapping key: {key!r}")
        folded_key = key.upper()
        if folded_key in seen:
            raise ValueError(f"duplicate movement mapping key: {folded_key}")
        seen.add(folded_key)
        if direction_name not in DIRECTIONS:
            raise ValueError(f"invalid movement direction: {direction_name}")
        entries.append((DIRECTIONS[direction_name] << 7) | ord(folded_key))

    if not 1 <= len(entries) <= 66:
        raise ValueError("movement mapping must contain 1 to 66 entries")

    payload = bytearray(2 + (len(entries) * 10 + 7) // 8)
    payload[0] = 0x13
    payload[1] = len(entries)
    for index, entry in enumerate(entries):
        for bit in range(10):
            offset = index * 10 + bit
            if (entry >> bit) & 1:
                payload[2 + offset // 8] |= 1 << (offset % 8)
    return bytes(payload)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="MQTT broker IPv4 address or hostname")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--movement-topic", default="robot/config/movement")
    parser.add_argument(
        "--movement-config",
        default='{"a":"MOVE_FORWARD","z":"MOVE_BACKWARD","c":"TURN_LEFT","f":"TURN_RIGHT","b":"STOP"}',
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
        binary_mapping = encode_movement_mapping(movement_config)
        result = client.publish(
            arguments.movement_topic,
            binary_mapping,
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
