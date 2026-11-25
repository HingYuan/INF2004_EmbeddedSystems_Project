# Testing — Team CSC03

All test records for **Team CSC03** (autonomous RP2040 / Pico W robotic rover running micro T-Kernel 3.0 SMP) live in this folder. A test only counts as done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide and CSC03 ownership / build instructions
├── network/            Buddy 1 (2500290) — Wi-Fi, Binary MQTT v1, HTTP Web Server & Dashboard
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, build logs, host verification logs
├── motor/            Buddy 2 (2500728) — Motion Control (PWM, Encoders, PID, Odometry)
├── ir/            Buddy 3 (2501325) — IR Line Following, Code 39 Barcode & Navigator FSM
├── imu/            Buddy 4 (2500253) — IMU Terrain, Hump & Tilt/Lift Safety (LSM303DLHC)
├── ultrasonic/            Buddy 5 (2502888) — Ultrasonic Profiling, Width Estimation & Bypass
├── integration/           cross-subsystem IPC, pin ownership, and Core 0 / Core 1 tests (shared)
└── system/                end-to-end FR1–FR12 & NFR1–NFR12 mission verification (shared)
```

## Three levels of test

| Level | ID format | Tests what | Owner | Where |
|---|---|---|---|---|
| Unit | `S1-U01`..`S5-U08` | one subsystem on its own | subsystem owner (Buddies 1–5) | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`..`INT-09` | cross-subsystem IPC, shared flags, pin & kernel-object contracts (`IF-01`..`IF-09`) | owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`..`SYS-10` | complete rover against FR1–FR12 and NFR1–NFR12 | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped.

## Test methods in CSC03

| Method | Meaning | Where it runs |
|---|---|---|
| **static / host** | Compile-time `ROBOT_TASKS_STATIC_ASSERT` checks (38 total), ARM GCC `-Wall -Wextra` multi-profile builds, ELF symbol/size checks, and host Python verification of static tables/codecs/geometry | Host PC (automated, reproducible without hardware) |
| **auto (on-target)** | Built-in `#ifndef NDEBUG` boot-time self-checks (`mqtt_client_codec_check`, `motor_right_enc_self_check`, `imu_detector_self_check`, `bypass_helpers_width_check`, `bypass_helpers_self_check`) compiled into `CONSOLE=uart` images | RP2040 / Pico W at startup |
| **manual** | Physical bench, instrument, track, or live Wi-Fi/MQTT/dashboard test observed on the assembled rover | Assembled Pico W car + test track / LAN |

## How to build and run tests

### 1. Debug image with built-in `#ifndef NDEBUG` boot self-checks (`CONSOLE=uart`)

`build_make/pico_rp2040.mk` omits `-DNDEBUG` when `CONSOLE=uart`, enabling all five firmware boot self-checks (`mqtt_client_codec_check`, `motor_right_enc_self_check`, `imu_detector_self_check`, `bypass_helpers_width_check`, `bypass_helpers_self_check`):

```sh
cd build_make
mingw32-make -j8 SMP=1 CONSOLE=uart
# Output: mtk3pico_smp1_uart.elf / mtk3pico_smp1_uart.uf2
```

> **Pin note:** `CONSOLE=uart` uses UART0 on GP0/GP1, which shares pins with the LSM303DLHC IMU (`I2C0` on GP0/GP1). Use `CONSOLE=uart` for boot self-checks and early serial diagnostics, and `CONSOLE=usb_cdc` for full integrated rover runs with the IMU connected.

### 2. Full production rover image (`CONSOLE=usb_cdc` + Wi-Fi + HTTP Web Server + Binary MQTT v1)

Copy `config/wifi_credentials.example.h` → `config/wifi_credentials.h` and `config/mqtt_config.example.h` → `config/mqtt_config.h` with your local SSID/broker values, then build:

```sh
cd build_make
mingw32-make -j8 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WEB_SERVER=1 MQTT=1
# Output: mtk3pico_smp1_usb_cdc_wifi_dns.elf / mtk3pico_smp1_usb_cdc_wifi_dns.uf2
```

### 3. Optional standalone bench tests

- **500 mm Motor Odometry Ruler Test**: Set `#define MOTOR_STAGE2_PAD_TEST (1u)` in `app_program/motor/motor_task.c`, rebuild, flash, place rover at a 0 mm tape mark, and compare physical travel against `[DISTANCE TEST COMPLETE]` serial output. Reset to `(0u)` afterwards.
- **Local MQTT Broker & Mapping Publisher** (in `app_program/network/mqtt_test_server/`):
  ```sh
  python mqtt_broker.py --host 0.0.0.0 --port 1883
  python mqtt_test_server.py --host <BROKER_IP> --port 1883
  ```

### 4. Save console output and record results

Save logs to `logs/` named `YYYY-MM-DD_<what>.txt`:

```sh
# macOS
cat /dev/cu.usbmodem1101 | tee ../testing/subsystem1/logs/2026-11-25_unit.txt
# Linux / WSL
cat /dev/ttyACM0 | tee ../testing/subsystem1/logs/2026-11-25_unit.txt
# Windows PuTTY: Session -> Logging -> All session output
```

Append a new row to the **Results** table with date, test ID, exact commit (`git rev-parse --short HEAD`), tester, `PASS`/`FAIL`, and the saved log file (or unlisted YouTube link in `videos/README.md` for physical rover demos).

## Rules

- **Never edit or delete an old result.** Append a new row for every re-run; history is the evidence.
- **Never fabricate hardware results.** Only record `PASS` for tests actually executed on the recorded commit. Unrun physical/bench tests stay in the *Pending on-hardware verification* section until executed on hardware.
- **Always record the commit.** Commit code changes before running hardware tests so results map to an exact revision.
- **A FAIL needs a note** stating what failed and, later, which commit fixed it.
- Before merging into `main`, re-run the affected subsystem's unit tests. Before demo or hand-in, run the complete unit, integration, and system suite.
