"""Small anonymous MQTT broker for local dashboard testing."""

from __future__ import annotations

import argparse
import asyncio

from amqtt.broker import Broker


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Address to listen on")
    parser.add_argument("--port", type=int, default=1883)
    return parser.parse_args()


async def run_broker(host: str, port: int) -> None:
    config = {
        "listeners": {
            "default": {
                "type": "tcp",
                "bind": f"{host}:{port}",
            }
        },
        "sys_interval": 10,
        "auth": {
            "allow-anonymous": True,
        },
        "topic-check": {
            "enabled": False,
        },
    }
    broker = Broker(config)
    await broker.start()
    print(f"MQTT broker listening on {host}:{port}")
    try:
        await asyncio.Event().wait()
    finally:
        await broker.shutdown()


def main() -> None:
    arguments = parse_args()
    try:
        asyncio.run(run_broker(arguments.host, arguments.port))
    except KeyboardInterrupt:
        print("Broker stopped")


if __name__ == "__main__":
    main()
