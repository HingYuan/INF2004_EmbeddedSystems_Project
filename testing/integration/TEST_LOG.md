# Integration — test log

Tests that two or more **Team CSC03** subsystems work together across their micro T-Kernel 3.0 IPC objects, shared spinlock-protected structures, and RP2040 dual-core (Core 0 / Core 1) boundaries. Both subsystem owners agree on each test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## CSC03 interface map (`main` @ `2f19d6b`)

| ID | From → To | Mechanism | Kernel / shared object | Payload / meaning |
|---|---|---|---|---|
| `IF-01` | S3 (`ir_task_barcode_task`) → S3 (`ir_task_decoder_task`) | RTOS message buffer | `gh_barcode_mbf` (20 × 4 B) | `ir_task_bar_record_t` (1-bit level + 26-bit duration in 100 us units) |
| `IF-02` | S3 (`ir_task_sensor_task`, `ir_task_decoder_task`) → S3 (`navigator_task`) | RTOS message buffer | `gh_nav_mbf` (64 × 34 B) | `MSG_IR_UPDATE` (`ir_state`), `MSG_BARCODE_DECODED` (`decoded_char`) |
| `IF-03` | S3 (`navigator_task`) & S5 (`bypass_helpers`) → S2 (`motor_task`) | RTOS message buffer + API | `gh_motor_mbf` (10 × 4 B) | `motor_task_cmd_t` (`±100%` speeds + ticks), `motor_task_move_*`/`turn_*`/`stop()` |
| `IF-04` | S2 (`motor_task`) → S3 (`navigator_task`) | RTOS message buffer + accessors | `gh_nav_mbf`, `motor_task_get_*` | `MSG_MOTOR_STOPPED`, `MSG_MOTOR_MOVE_DONE`, encoder ticks & mm/cm odometry |
| `IF-05` | S4 (`imu_task`) → S3 (`navigator_task`) | RTOS message buffer + lock-protected setters | `gh_nav_mbf`, `navigator_task_set_imu_*` | `MSG_IMU_UPDATE` (`IMU_MSG_HUMP_DETECTED`, `IMU_MSG_NO_HUMP`, `IMU_MSG_LIFTED_OVERTURNED`), pose/height |
| `IF-06` | S5 (`ultrasonic_task`) ↔ S3 (`navigator_task`) | Event flag + RTOS message buffer | `gh_ranger_flg` (7 bits), `gh_nav_mbf` | `SONAR_SCAN_REQUEST`, `SONAR_PROBE_*`, `SONAR_TRACK_*`, `SONAR_RESUME_FLAG`; `MSG_OBSTACLE_*`, `MSG_SONAR_*`, `MSG_TRACKING_OBSTACLE` |
| `IF-07` | S1 (`mqtt_client`, Core 1) → S3 (`navigator_task`, Core 0) | RTOS message buffer (`TMO_POL`) + lock-protected map | `gh_nav_mbf`, `g_movement_map` | `MSG_WIFI_BARCODE_MAPPING` (`dir << 7 | ascii`), `MSG_REMOTE_CMD` (`motor_task_cmd_t`) |
| `IF-08` | S3/S4/S5/S2 (Core 0) → S1 (`mqtt_client`, `web_server`, Core 1) | Spinlock-protected shared struct | `g_telemetry_snapshot` (32 B, `ISpinLock`) | `navigator_task_read_telem()` feeding 2 Hz `0x11` MQTT & HTTP `/api/telemetry` |
| `IF-09` | S1, S2, S3, S4, S5 ↔ Shared safety bus | Spinlock + ARM `dmb` atomic bitmask | `g_robot_flags` (13 bits: `0..12`) | `ROBOT_FAULT_*`, `ROBOT_STATUS_NO_ECHO`, `ROBOT_STOP_ACK_REQUEST`, `ROBOT_ENCODER_DEGRADED`, `ROBOT_ESTOP_REQUEST` |

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | S1 (Core 1 MQTT `0x13`) → S3 (Core 0 Navigator) → S2 (Motor) boot mapping & barcode route execution | FR2, FR3, FR4, `IF-01`, `IF-02`, `IF-03`, `IF-07` | manual | 1. Boot rover in `MODE_WIFI_SEND_COMMS` (confirm motors stopped). 2. Run `mqtt_test_server.py` with 5 mappings (`a:MOVE_FORWARD, z:MOVE_BACKWARD, c:TURN_LEFT, f:TURN_RIGHT, b:STOP`). 3. Pass Code 39 `*C*` over `GP6`, then drive over a `B_B` junction. | Navigator logs 5 `[nav] mapping received` lines, enters `MODE_LINE_FOLLOWING`, decodes `'C'` (`0x148`), stores `DIR_LEFT`, advances 900 ms at `B_B`, and executes left pivot cleanly via `gh_motor_mbf`. |
| INT-02 | S3 (IR line tracking) ↔ S2 (Motor closed-loop steering & heading-lock PID) sustained straight + curve tracking | FR1, NFR1, NFR2, `IF-02`, `IF-03` | manual | 1. Place rover on closed-loop oval/curved black tape track with straightaways and curves. 2. Run continuously for 2 minutes while logging `[dist]` and `[Diagnostics] CPU Load`. | Rover tracks straightaways at `16%` with PID heading lock, transitions smoothly to `10/30` base and `5/40` ramp on curves, stays on line with 0 buffer stalls, and keeps Core 0 CPU load bounded. |
| INT-03 | S4 (IMU) → S3 (Navigator) → S2 (Motor) hump speed boost (`30%`), pitch-balanced climb & return to line following | FR5, NFR7, `IF-05`, `IF-03` | manual | 1. Drive rover in `MODE_LINE_FOLLOWING` onto a track hump (`>= 5.5 deg` tilt for `>= 80 ms`). 2. Observe `MODE_GOING_OVER_HUMP` (`mode=3`) entry, motor speed boost (`NAV_FORWARD_SPEED_HUMP = 30`), pitch compensation, and flat exit. | `imu_task` sends `IMU_MSG_HUMP_DETECTED` within `<= 100 ms`; Navigator switches to `mode=3` and commands `30%` drive modulated by pitch (`10..30%`); on `IMU_MSG_NO_HUMP`, Navigator restores `mode=2` (`16%`) and peak height appears in telemetry. |
| INT-04 | S4 (IMU lift/tilt/I2C fault) → `g_robot_flags` → S2 (Motor E-stop) & S3 (Navigator `MODE_SAFE_STOP`) | FR12, NFR7, NFR12, `IF-05`, `IF-09` | manual | 1. While rover drives along the track, lift rover abruptly or tilt `> 30 deg` for `> 200 ms`. 2. Observe E-stop latency across S4, S2, S3, and S1 dashboard. | `imu_task` asserts `ROBOT_ESTOP_REQUEST | ROBOT_FAULT_IMU_TILT` and sends `IMU_MSG_LIFTED_OVERTURNED`; `motor_task` cuts PWM to `0%` within `<= 20 ms` (`state=6`); Navigator logs `[nav] STOP: robot lifted/overturned` and latches `MODE_SAFE_STOP` (`mode=10`). |
| INT-05 | Shared 13-bit `g_robot_flags` bit uniqueness, spinlock/DMB atomicity & IPC struct ABI sizes across all 5 subsystems | NFR12, Tribunal 1, Tribunal 2 #7, `IF-09` | static / host | 1. Host Python audits `ROBOT_*` flag shifts `0..12` in `robot_tasks.h` for zero overlap. 2. ARM GCC build enforces all cross-subsystem packed IPC struct `ROBOT_TASKS_STATIC_ASSERT` sizes (`motor_task_cmd_t` 4 B, `ir_task_bar_record_t` 4 B, `navigator_task_msg_t` 34 B, `navigator_task_telemetry_t` 32 B, `navigator_task_obs_profile_t` 33 B, `navigator_task_bypass_ctx_t` 68 B). | 13/13 unique bits (`0..12`), `0` overlaps, guarded by `ISpinLock` + `dmb`; all IPC struct static assertions pass in Debug and Production builds. |
| INT-06 | S5 (Ultrasonic) ↔ S3 (Navigator) ↔ S2 (Motor) obstacle stop, `MSG_MOTOR_STOPPED` handshake, dual scan, rectangular bypass & S3 IR line reacquisition | FR6, FR7, FR8, FR9, NFR5, NFR6, `IF-03`, `IF-04`, `IF-06`, `IF-09` | manual | 1. Place a `15 cm` wide box on the black tape path. 2. Let rover drive into range (`<= 20 cm`), stop, wait for `[motors] Stop confirmed` (`MSG_MOTOR_STOPPED`), complete Scan 1, reverse 20 mm (`BYPASS_SCAN_BACK`), complete Scan 2, execute rectangular detour around box, and re-enter line via 2 consecutive black IR samples (`BYPASS_FIND_LINE`). | Full handshake completes without deadlock: `SONAR_SCAN_REQUEST` fires only after `MSG_MOTOR_STOPPED`; `MSG_MOTOR_MOVE_DONE` advances each leg; `SONAR_RESUME_FLAG` re-centers servo (`1695 us`); rover aligns back onto tape (`[nav] Reentry line detected`) and resumes `MODE_LINE_FOLLOWING`. |
| INT-07 | Core 0 (`navigator_task`, `imu_task`, `ultrasonic_task`, `motor_task`) → Core 1 (`mqtt_client`, `web_server`) torn-read-free telemetry snapshot & Core 0 isolation during 30 s Wi-Fi drop | FR10, NFR8, NFR9, NFR10, `IF-08` | manual | 1. Drive rover autonomously on track while polling `GET /api/telemetry` at 1 Hz and recording `0x11` MQTT frames at 2 Hz. 2. Disconnect Wi-Fi AP for 30 s mid-run and observe Core 0 line following & obstacle safety. 3. Restore AP and confirm telemetry resumes. | Zero corrupted/torn JSON or binary telemetry fields (`seq` strictly monotonic); during 30 s Wi-Fi drop, Core 0 IR/motor/IMU/sonar tasks keep running uninterrupted (`ROBOT_FAULT_WIFI | ROBOT_FAULT_MQTT` set); telemetry & dashboard auto-recover within `<= 10 s` of AP return. |
| INT-08 | Full RP2040 GPIO pin ownership audit across S1–S5 (`CONSOLE=usb_cdc` production profile vs `CONSOLE=uart` debug profile) | Hardware safety, `IF-01`..`IF-09` | static / host | Run host Python pin-conflict audit verifying every hardware pin defined in `motor_task.h`, `ir_task.h`, `imu_task.h`, `ultrasonic_task.h`, and `pico_rp2040.mk` (`CYW43_*`). | 20 claimed pins, **0 pin conflicts** under production `CONSOLE=usb_cdc` (`GP0/GP1` documented as shared with UART0 only in `CONSOLE=uart`). |
| INT-09 | micro T-Kernel 3.0 kernel object budget audit vs `config/config.h` limits across `app_main.c`, `robot_tasks.c`, `cyw43_utk_kernel.c`, and USB-CDC | System reliability, RTOS config | static / host | Audit total tasks, message buffers, event flags, semaphores, mutexes, and fixed memory pools created at startup against `CNF_MAX_*` limits in `config/config.h`. | All allocations fit within `config/config.h`: Tasks `14/32`, MBF `4/4` (100% allocated — note: adding any new MBF requires raising `CNF_MAX_MBFID`), FLG `2/16`, SEM `1/16`, MTX `1/4`, MPF `1/4`. |

*Method:* **static / host** = compile-time `ROBOT_TASKS_STATIC_ASSERT` + ARM GCC build + host Python audit; **manual** = observed across wired multi-subsystem Pico W rover.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | INT-05 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_ipc_and_pin_audit.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | 13/13 unique `g_robot_flags` bits (`0..12`, `ISpinLock` + `dmb`) and all cross-subsystem packed IPC struct static assertions verified. |
| 2026-11-25 | INT-08 | `2f19d6b` | Automated (host Python) | PASS | `logs/2026-11-25_ipc_and_pin_audit.txt` | 20 claimed RP2040 GPIO pins audited; 0 conflicts under `CONSOLE=usb_cdc` (`GP0/GP1` shared with UART0 only in `CONSOLE=uart`). |
| 2026-11-25 | INT-09 | `2f19d6b` | Automated (host Python) | PASS | `logs/2026-11-25_ipc_and_pin_audit.txt` | Kernel object budget verified: Tasks 14/32, MBF 4/4 (100% full), FLG 2/16, SEM 1/16, MTX 1/4, MPF 1/4. |

### Pending on-hardware verification

The following multi-subsystem hardware integration tests require the assembled Pico W rover on the test track:
- **INT-01**, **INT-02**, **INT-03**, **INT-04**, **INT-06**, **INT-07**: Run jointly by the paired subsystem owners on the assembled rover, saving USB-CDC logs, broker captures, and video links into `logs/` and `videos/README.md`.
