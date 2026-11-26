# Subsystem 4 — test log

**Subsystem:** IMU Motion, Hump & Tilt/Lift Terrain Monitoring — LSM303DLHC (Buddy 4)  ·  **Student ID:** `2500253` (Maxwell Menendez)  ·  **Code:** `app_program/imu/imu_task.{c,h}`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | IMU safety detector self-check (`imu_detector_self_check()`), Rajan `atan2` accuracy, 12-bit sign extension, hump height formula & 32-bit timer wraparound | FR5, FR12, NFR7, Tribunal 1 | static / host + auto (on-target) | 1. Host Python verifies Rajan `atan2` max error (`< 0.0045 rad` over `0..359 deg`), 12-bit accel sign extension (`-2048..2047`), hump height formula (`max(68*sin_roll, 60*sin_pitch)`), and `0xFFFF0000 + 80000` wraparound timer; ARM GCC verifies `imu_confirm_size == 9`, `imu_runtime_size == 2`, `imu_msg_size == 6`, and compiled `#ifndef NDEBUG` self-check string in `mtk3pico_smp1_uart.elf`. 2. Boot `SMP=1 CONSOLE=uart` on Pico W to run `imu_detector_self_check()`. | Host math & compile-time asserts pass (`atan2` max err `0.00376 rad`, hump @10 deg `11.87 mm`); on-target boot passes without `[IMU] Detector self-check failed`. |
| S4-U02 | `I2C0` (`"iica"`) pin reroute to `GP0` (SDA) / `GP1` (SCL), LSM303DLHC accel (`0x19` / fallback `0x18`) & mag (`0x1E`) register setup | FR5, FR12 | manual | 1. Build with `CONSOLE=usb_cdc` (avoiding UART0 clash on `GP0/GP1`), wire LSM303DLHC to Robo Pico Grove 1 (`GP0/GP1`), and boot rover on flat ground. 2. Observe serial output for init handshake. | Console prints `[IMU STATUS] CONNECTED!`; `CTRL1_A=0x57` (100 Hz), `CTRL4_A=0x88` (BDU, ±2g HR), `CRA_M=0x14`, `CRB_M=0x20`, `MR_M=0x00` succeed without retry loop. |
| S4-U03 | 50-sample stationary flat-ground calibration (`0.8..1.2 g`, `< 20 deg` tilt) & disturbance retry | FR5, FR12 | manual | 1. Tilt or shake rover briefly during startup calibration to confirm rejection and reset. 2. Place rover still on flat surface and wait ~1 s for 50 consecutive valid samples. | While disturbed, prints `[IMU] Calibration incomplete; keep flat, retrying`; on flat surface, completes and prints `[IMU] Calibrated! Offsets -> P:<p> R:<r> (samples:50)` and enters `IMU_STATE_MONITORING`. |
| S4-U04 | 3-sample median + EMA (`alpha=0.22`) tilt filtering, magnetometer compass heading (`0..359 deg`) & 500 ms telemetry cadence | FR5, FR10, NFR10, `IF-05` | manual | 1. Tap chassis gently while flat to confirm single-sample spike rejection via `imu_median3_f32()`. 2. Rotate rover horizontally through 0°, 90°, 180°, 270° and read `/api/telemetry` (`heading`, `heading_dir`) and `[IMU] Pitch:<p> Roll:<r> H:<h>mm` every 500 ms. | Single mechanical tap does not trigger false hump/tilt; heading updates smoothly within `0..359` degrees (`N/NE/E/SE/S/SW/W/NW`); pose updates every `500 ms`. |
| S4-U05 | Hump detection (`>= 5.5 deg` for `80 ms`), peak height tracking (`0..68 mm`) & flat return notification (`IMU_MSG_NO_HUMP`) | FR5, NFR7, `IF-05` | manual | 1. Drive rover slowly over a calibrated track hump (`~15..30 mm` high). 2. Record `[IMU HUMP]` console timestamps and compare logged `Peak:<h>mm` and dashboard `height_mm` against physical hump height. | Emits `IMU_MSG_HUMP_DETECTED` and prints `[IMU HUMP] Detected! Tilt P:<p> R:<r>` within `<= 100 ms` of sustained `>= 5.5 deg` tilt; upon returning flat for `80 ms`, emits `IMU_MSG_NO_HUMP` and prints `[IMU HUMP] Returned to flat. Peak:<h>mm`. |
| S4-U06 | Unsafe tilt (`> 25 deg` for `200 ms`), inversion (`z * baseline_z < -0.2 * baseline_z^2`), free-fall/shock (`< 0.25 g` or `> 2.0 g` for `80 ms`) & 500 ms upright recovery | FR12, NFR7, NFR12, `IF-05`, `IF-09` | manual | 1. Tilt rover `> 30 deg` (and hold `> 200 ms`), flip inverted, and perform quick vertical lift (`> 2.0 g` or `< 0.25 g` for `> 80 ms`). 2. Verify immediate `ROBOT_ESTOP_REQUEST` + `ROBOT_FAULT_IMU_TILT` + `IMU_MSG_LIFTED_OVERTURNED`. 3. Return rover upright and flat for `> 500 ms`. | Prints `[IMU] LIFTED/OVERTURNED -> STOP` with `Cause=tilt|inverted|low-g|high-g`; asserts `ROBOT_ESTOP_REQUEST` and `ROBOT_FAULT_IMU_TILT`; after `500 ms` upright, clears `ROBOT_FAULT_IMU_TILT` and prints `[IMU] Upright -> Monitoring resumed` (note: E-stop remains latched by Navigator `MODE_SAFE_STOP`). |
| S4-U07 | Sustained I2C bus disconnect / read failure (`>= 500 ms`) failsafe E-stop latch (no fabricated flat data) | FR12, NFR12, Tribunal 2 #12, `IF-09` | manual | 1. While rover is in `IMU_STATE_MONITORING`, unplug Grove 1 I2C cable (`GP0/GP1`) for `> 600 ms`. 2. Observe serial output and fault flags. 3. Reconnect cable and verify latched stop behavior. | After `500 ms` (`IMU_READ_FAULT_USEC`) of failed reads, sets `ROBOT_ESTOP_REQUEST | ROBOT_FAULT_IMU_READ` and prints `[IMU] SENSOR READ FAULT -> STOP`; on reconnect clears `ROBOT_FAULT_IMU_READ` and prints `[IMU] Reads recovered; stop stays latched`. |

*Method:* **static / host** = compile-time `ROBOT_TASKS_STATIC_ASSERT` + ARM GCC build + host Python verification; **auto (on-target)** = `#ifndef NDEBUG` boot self-check (`CONSOLE=uart`); **manual** = observed on live LSM303DLHC hardware.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S4-U01 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_host_imu_math_and_build_verify.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | Rajan `atan2` max error `0.00376 rad`, 12-bit sign extension, hump height @10 deg `11.87 mm`, wraparound timer, and all 3 IMU static asserts passed; `[IMU] Detector self-check failed` verified compiled in `mtk3pico_smp1_uart.elf`. On-target boot run pending Pico W flash. |

### Pending on-hardware verification

The following hardware tests require the LSM303DLHC connected to Robo Pico Grove 1 (`GP0/GP1`, `CONSOLE=usb_cdc`):
- **S4-U01** (on-target boot check via `mtk3pico_smp1_uart.uf2`).
- **S4-U02**, **S4-U03**, **S4-U04**, **S4-U05**, **S4-U06**, **S4-U07**: Capture USB-CDC serial logs and physical hump/tilt measurements into `logs/`.

### Known Tribunal 2 open items (tracked on `main` @ `2f19d6b`)
- Tribunal 2 #12 (note: `main` @ `2f19d6b` ALREADY removed silent `(0, 0, 1.0g)` fabrication and enforces explicit I2C `bool` failure + `500 ms` `ROBOT_FAULT_IMU_READ` + `ROBOT_ESTOP_REQUEST`, whereas tightening read-fault timeout from `500 ms` to `80 ms` and adding frozen-register stuck detection remain optional follow-up enhancements) and #13 (`nav_handle_hump` ALREADY guards `MSG_IMU_UPDATE == p_msg->type`; optional `150 ms` Navigator freshness timeout remains open).
