# MQTT broker and dashboard test publisher

This folder contains:

- `mqtt_broker.py`: a small anonymous MQTT broker for local testing.
- `mqtt_test_server.py`: a publisher for retained barcode movement mappings.

## Install

From this directory:

```powershell
python -m pip install -r requirements.txt
```

## Start the broker

Start this first in one PowerShell window:

```powershell
python .\mqtt_broker.py
```

The broker listens anonymously on all network interfaces at TCP port `1883`.
Allow inbound TCP port `1883` through Windows Firewall so the Pico W can
connect. To bind to a different address or port:

```powershell
python .\mqtt_broker.py --host 0.0.0.0 --port 1883
```

Anonymous access is intended for local testing only. Do not expose this broker
to the public Internet.

## Start the test publisher

In a second PowerShell window, use the PC's LAN IPv4 address or hotspot IP
(for example, `192.168.137.1`):

```powershell
python .\mqtt_test_server.py `
  --host 192.168.137.1 `
  --port 1883
```

The publisher sends only the retained movement mapping to the Pico W. It does
not send synthetic IMU, servo, or other sensor data. The dashboard reads real
sensor telemetry from the Pico W's `/api/telemetry` endpoint.

### Custom barcode movement mapping

Map barcode keys to navigation actions with `--movement-config`:

```powershell
python .\mqtt_test_server.py `
  --host 127.0.0.1 `
  --movement-config '{"Z":"MOVE_FORWARD","B":"STOP","C":"TURN_LEFT","D":"TURN_RIGHT"}'
```

Supported actions are:

- `MOVE_FORWARD`
- `MOVE_BACKWARD`
- `TURN_LEFT`
- `TURN_RIGHT`
- `STOP`

Mapping keys are normalized to uppercase by the firmware. The mapping is
published retained on `robot/config/movement`, so the Pico receives the latest
mapping after reconnecting.

Other publisher options:

```text
--movement-topic TOPIC   Mapping topic (default: robot/config/movement)
--username USER          Optional broker username
--password PASSWORD      Optional broker password
```

## Topics

The default firmware topics are defined in `config/mqtt_config.h`:

| Topic | Direction | Purpose |
|---|---|---|
| `robot/command` | Broker to Pico | Remote movement commands |
| `robot/config/movement` | Broker to Pico | Retained barcode-to-action mapping |
| `robot/telemetry` | Pico to broker | Firmware telemetry |
| `/api/telemetry` | Pico to dashboard | Live sensor data from the robot's core-0 tasks |

The web dashboard exposes firmware telemetry at `/api/telemetry` and active
barcode mappings at `/api/mqtt`. Firmware ultrasonic telemetry includes
`obs_dist`, `obs_dist_valid`, `obs_status`, `closest_cm`, `closest_deg`, and `obs_width`.

The Pico W must be able to reach the PC's hotspot or LAN IPv4 address.
