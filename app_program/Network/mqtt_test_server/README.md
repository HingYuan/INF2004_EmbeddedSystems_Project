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

In a second PowerShell window, use the PC's LAN IPv4 address. Use the same
broker address and topic configured in `config/mqtt_config.h`:

```powershell
python .\mqtt_test_server.py --host 192.168.1.10 --topic dashboard/test
```

Optional settings:

```powershell
python .\mqtt_test_server.py `
  --host 192.168.1.10 `
  --port 1883 `
  --topic dashboard/test `
  --interval 2 `
  --username myuser `
  --password mypassword
```

The payload contains `sequence`, `temperature_c`, `source`, and `timestamp` fields. The firmware subscribes to `dashboard/#` by default in the example configuration, so `dashboard/test` is included.

The Pico W must be able to reach the PC's LAN IPv4 address. The broker allows
anonymous connections for local testing only; do not expose it to the public
Internet.
