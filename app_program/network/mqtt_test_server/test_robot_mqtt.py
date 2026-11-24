"""
Simple interactive command & telemetry test CLI for the Pico W robot.
Listens to robot/telemetry and robot/status, and allows typing L,R,T commands
to send to robot/command.
"""

import sys
import time
import threading
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"  # Or your PC's IP if connecting Pico W
PORT = 1883

SUB_TOPIC = "robot/telemetry"
STATUS_TOPIC = "robot/status"
PUB_TOPIC = "robot/command"

def on_connect(client, userdata, flags, rc, properties=None):
    print(f"[MQTT] Connected to broker (rc={rc})")
    client.subscribe(SUB_TOPIC)
    client.subscribe(STATUS_TOPIC)
    print(f"[MQTT] Subscribed to '{SUB_TOPIC}' and '{STATUS_TOPIC}'")

def on_message(client, userdata, msg):
    print(f"\n[TELEMETRY] {msg.topic}: {msg.payload.decode('utf-8', errors='ignore')}")
    print("Send command (L,R,T) > ", end="", flush=True)

def main():
    broker = sys.argv[1] if len(sys.argv) > 1 else BROKER
    print(f"Connecting to broker at {broker}:{PORT}...")
    
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(broker, PORT, 60)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    client.loop_start()

    print("\n=== Robot MQTT Controller ===")
    print("Format: <left_speed>,<right_speed>,<turn_ticks>")
    print("Example: '20,-20,0' (forward), '0,0,0' (stop), '-20,20,0' (backward)")
    print("Type 'exit' to quit.\n")

    try:
        while True:
            cmd = input("Send command (L,R,T) > ").strip()
            if cmd.lower() in ("exit", "quit"):
                break
            if cmd:
                client.publish(PUB_TOPIC, cmd)
                print(f"[SENT] -> {PUB_TOPIC}: {cmd}")
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
