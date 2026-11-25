# Subsystem 5 — test log

**Subsystem:** Ultrasonic Scanning, Obstacle Profiling, Width Estimation & Rectangular Bypass (Buddy 5)  ·  **Student ID:** `2502888` (Ng Chi Siang)  ·  **Code:** `app_program/ultrasonic/{ultrasonic_task,bypass_helpers}.{c,h}`, `app_program/ultrasonic/ultrasonic_servo_main.{c,h}`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Pan-servo 50 Hz PWM pulse math (`1345..2045 us`, center `1695 us`), HC-SR04 echo math (`58 us/cm`), swept-radius & bypass compile-time assertions | FR6, FR7, FR8, Tribunal 1 | static / host | 1. Build both Debug & Production ELFs enforcing all 8 ultrasonic/bypass `ROBOT_TASKS_STATIC_ASSERT` checks (`sonar_confirm_bits`, `sonar_runtime_size == 2`, `bypass_turn_angles`, `bypass_pivot_clear`, `bypass_depth_clear`, `bypass_pivot_pos`, `bypass_swept_radius` `144 >= 136`, `bypass_beam_half < 450`). 2. Verify host Python servo PWM (`125 MHz / 125 / 20000 = 50.0 Hz`, `-45/0/+45 deg` -> `1345/1695/2045 us`, clamped) and HC-SR04 `1160 us -> 20 cm`. | All 8 compile-time asserts pass with 0 `-Wall -Wextra` warnings; servo `50.0 Hz` and pulse clamping exact; HC-SR04 rounding exact. |
| S5-U02 | Width estimation unit self-check (`bypass_helpers_width_check()`): 1-scan/2-scan merge, conflict detection, broad-span anti-inflation regression (12 cm box), ray classification & 3-sample spread check | FR7, FR10 | auto (on-target) | Build `SMP=1 CONSOLE=uart` (symbol `bypass_helpers_width_check` verified linked at `0x10012468`) and boot Pico W so `navigator_task` runs `bypass_helpers_width_check()` (`bypass_helpers.c:288-365`). | Returns `true`; boot proceeds without `[bypass] Self-check failed` and without asserting `ROBOT_ESTOP_REQUEST`. |
| S5-U03 | Bypass side-offset & pivot helper self-check (`bypass_helpers_self_check()`): left/right unknown, bounded (`90 + 100 = 190 mm`), occluded & signed pivot speeds | FR8 | auto (on-target) | Build `SMP=1 CONSOLE=uart` (symbol `bypass_helpers_self_check` verified linked at `0x1001290c`) and boot Pico W so `navigator_task` runs `bypass_helpers_self_check()` (`bypass_helpers.c:452-504`). | Returns `true`; zero-sign pivot returns `raw == 0`; `±1` pivots set `(-side)*25` on both wheels with zero ticks; no `[bypass] Self-check failed`. |
| S5-U04 | HC-SR04 ranging accuracy (`GP5` TRIG / `GP4` ECHO), `< 2 cm` `SONAR_TOO_CLOSE`, `>= 26 ms` `SONAR_NO_ECHO` & stuck-echo `SONAR_SENSOR_FAULT` | FR6, NFR5 | manual | 1. Place flat cardboard target at 10 cm, 20 cm, 40 cm, and 100 cm in front of HC-SR04; compare serial/telemetry `obs_dist` to ruler. 2. Place target `< 2 cm` away. 3. Aim sensor into open space (`> 4 m`). 4. Pull `GP4` ECHO high (`3.3 V` via resistor) to simulate stuck-high hardware fault. | Measured distances match ruler within `±2 cm`; `< 2 cm` reports `SONAR_TOO_CLOSE` (`front_cm=1`); open space reports `SONAR_NO_ECHO` (`ROBOT_STATUS_NO_ECHO` set); stuck ECHO triggers `SONAR_SENSOR_FAULT` (`ROBOT_FAULT_SONAR` + `ROBOT_ESTOP_REQUEST`). |
| S5-U05 | 2-sample forward obstacle confirmation at `<= 20 cm` (`SONAR_OBSTACLE_TRIG_CM`, `SONAR_CONFIRM_COUNT = 2`) & E-stop handoff | FR6, NFR5, `IF-06`, `IF-09` | manual | 1. Wave a brief single-ping target (`< 60 ms`) at 15 cm to verify 1-sample glitch rejection. 2. Hold an obstacle steady at `<= 20 cm` for `>= 120 ms` (2 pings at 60 ms period). | Single transient ping resets `confirmations = 0` without stopping; 2 consecutive `<= 20 cm` pings assert `ROBOT_ESTOP_REQUEST`, log `[sonar] Obstacle at <d> cm`, and send `MSG_OBSTACLE_DETECTED`. |
| S5-U06 | Stationary 7-ray coarse sweep (`-45..+45 deg` in `15 deg` steps), 2-pass fine edge bisection & 2-scan width estimation after `20 mm` reverse | FR7, FR10, `IF-06` | manual | 1. Place a measured `12 cm`–`20 cm` flat-faced box centered 18–20 cm in front of rover. 2. Trigger obstacle detection and capture `[sonar] coarse`, `[sonar] fine`, `[width] Scan front=...`, and `[width] Dashboard: <w> cm (estimated)` logs across Scan 1 and Scan 2 (after `BYPASS_SCAN_BACK_MM = 20 mm`). | Servo sweeps cleanly `-45..+45 deg` and re-centers to `1695 us`; coarse valid `>= 5/7`; `[width] Dashboard: <w> cm (estimated)` matches physical box width within `±3 cm` and updates dashboard `obs_width`. |
| S5-U07 | Rectangular obstacle bypass execution (side selection, `90 deg` pivots, `BYPASS_WIDTH_CHECK` & `BYPASS_RETURN_CHECK` 2-confirm front probes, `±45 deg` side tracking, line search) | FR8, FR9, NFR5, NFR6, `IF-03`, `IF-06` | manual | 1. Place a box obstacle on the black line track with clearance on at least one side. 2. Let rover detect, dual-scan, select side (`[bypass] Dynamic offsets L=... R=...`), pivot out, shift out, align parallel, verify width clear, advance with `±45 deg` side tracking, turn in, verify return clear, and search (`BYPASS_FIND_LINE`) until 2 consecutive black IR readings reacquire line. | Zero physical contact with obstacle; logs complete phase progression through `[bypass] Line found; reentry side=..., align=...` and returns cleanly to `MODE_LINE_FOLLOWING`. |
| S5-U08 | Bypass safety abort on insufficient pivot room (`front_cm + 10 < 17 cm`), exploration distance limit (`> 1500 mm` side / `> 3000 mm` fwd), stale tracking (`> 1000 ms`), or hardware fault | FR8, NFR5, `IF-09` | manual | 1. Place obstacle `< 7 cm` from sensor (`front_cm + 10 < 12 + 5`) and trigger assess. 2. Separately, block `BYPASS_FIND_LINE` on a blank white area with no tape until search distance expires. | Both scenarios cleanly invoke `bypass_helpers_abort()`, assert `ROBOT_ESTOP_REQUEST`, stop motors (`0, 0`), log `[bypass] Fault: <reason>`, and latch `MODE_SAFE_STOP`. |

*Method:* **static / host** = compile-time `ROBOT_TASKS_STATIC_ASSERT` + ARM GCC build + host Python verification; **auto (on-target)** = `#ifndef NDEBUG` boot self-check (`CONSOLE=uart`); **manual** = observed on live HC-SR04/servo hardware and track.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S5-U01 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_host_bypass_and_sonar_verify.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | Servo 50.0 Hz PWM (`1345/1695/2045 us`), HC-SR04 `1160 us = 20 cm`, swept-radius `144 >= 136`, and all 8 ultrasonic/bypass static asserts passed with 0 `-Wall -Wextra` warnings. |

### Pending on-hardware verification

The following on-target self-checks and physical bench/track tests require the Pico W rover:
- **S5-U02**, **S5-U03**: Flash `mtk3pico_smp1_uart.uf2` (symbols `bypass_helpers_width_check` at `0x10012468` and `bypass_helpers_self_check` at `0x1001290c` verified linked in ELF) and capture boot console log.
- **S5-U04**, **S5-U05**, **S5-U06**, **S5-U07**, **S5-U08**: Run with wired HC-SR04 (`GP4/GP5`) and pan-servo (`GP15`) on track obstacles, saving serial logs and video links into `logs/` / `videos/README.md`.

### Known Tribunal 2 open items (tracked on `main` @ `2f19d6b`)
- Tribunal 2 #10 (`bypass_helpers_probe_result()` currently treats `SONAR_NO_ECHO` as clear rather than unknown) and #11 (separating temporary obstacle-scan stop ownership from independent IMU/motor E-stop latches during `BYPASS_WAIT_RELEASE`) are accepted design enhancements not yet patched on `2f19d6b`; add regression tests (`S5-U09+`) when merged.
