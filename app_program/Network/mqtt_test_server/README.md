# MQTT dashboard test publisher

This folder contains both a small MQTT broker and a publisher that sends sample JSON messages for the Pico W dashboard.

## Install

From this directory:

```powershell
python -m pip install -r requirements.txt
```

## Run the broker

Start this first in one PowerShell window:

```powershell
python .\mqtt_broker.py
```

It listens anonymously on all network interfaces at TCP port `1883`. Use
`--port` to choose another port. Allow inbound TCP port `1883` through
Windows Firewall so the Pico W can connect.

## Run the test publisher

In a second PowerShell window, use the PC's LAN IPv4 address (or hotspot IP such as `192.168.137.1`). Use the same
broker address and topic configured in `config/mqtt_config.h`:

```powershell
python .\mqtt_test_server.py --host 192.168.137.1 --topic sensors/imu
```

Optional settings:

```powershell
python .\mqtt_test_server.py `
  --host 192.168.137.1 `
  --port 1883 `
  --topic sensors/imu `
  --interval 1 `
  --username myuser `
  --password mypassword
```

The payload publishes sensor data formatted for the web dashboard:
- **IMU:** `accel_x`, `accel_y`, `accel_z`, `gyro_x`, `gyro_y`, `gyro_z`
- **Servo:** `servo_angle`, `servo_pwm`
- **Sensors:** `temperature_c`, `humidity`
- **Metadata:** `sequence`, `source`, `timestamp`

The firmware subscribes to `sensors/#` by default in `config/mqtt_config.h`, so `sensors/imu` (or any sub-topic under `sensors/`) is parsed and rendered in real time.

The Pico W must be able to reach the PC's hotspot or LAN IPv4 address. The broker allows
anonymous connections for local testing only; do not expose it to the public
Internet.
