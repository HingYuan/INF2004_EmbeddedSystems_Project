"""Publish repeatable test messages for the Pico W MQTT dashboard.

This is a test publisher, not a broker. Run it against Mosquitto, EMQX,
or another MQTT broker that the Pico W can reach.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="MQTT broker IPv4 address or hostname")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--topic", default="dashboard/test")
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
    sequence = 0
    print(f"Publishing to {arguments.host}:{arguments.port}/{arguments.topic}")
    try:
        while True:
            payload = {
                "sequence": sequence,
                "temperature_c": round(22.5 + (sequence % 10) * 0.2, 1),
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
