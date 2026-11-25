# Subsystem 2 — test log

**Subsystem:** Motion Control — Dual-Wheel PWM, Quadrature Encoders, Heading-Lock PID & Odometry (Buddy 2)  ·  **Student ID:** `2500728` (Nnurul Akeell Bin NurulAmin)  ·  **Code:** `app_program/motor/motor_task.{c,h}`, `app_program/motor/motors_main.{c,h}`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Right encoder Channel A → Channel B automatic failover at `>= 8` tick drift with offset continuity & clean reset | NFR2, Tribunal 2 #6 | static / host + auto (on-target) | 1. Host Python mirrors `motor_right_enc_ticks()` state machine against all 9 assertions in `motor_right_enc_self_check()`. 2. Build `SMP=1 CONSOLE=uart` and boot Pico W so `motor_init_hardware()` executes `motor_right_enc_self_check()` before enabling Timer Alarm 0 IRQ. | Failover latches `b_using_b = 1` at `ticks_a=12, ticks_b=21` (`out=20`), tracks reverse to `0` and `-60`, resets cleanly to `0`. On target, no `[motors] Encoder self-check failed` message. |
| S2-U02 | `motor_task_clamp()` `[-100, 100]` speed bounds, 20 kHz hardware PWM math (`MOTOR_PWM_WRAP = 6249`) & compile-time struct assertions | FR11, Tribunal 1, Tribunal 2 #7 | static / host | 1. Build both Debug & Production ELFs enforcing `motor_cmd_size` (4 B), `motor_runtime_size` (4 B), `motor_enc_isr_size` (2 B), `motor_stall_bits` (`75 <= 127`). 2. Verify PWM frequency `125 MHz / 6250 = 20 kHz`, duty calculation for `-128..127` clamped to `-100..100`, zero signed wraparound. | `20000 Hz` PWM, `duty(0%)=0`, `duty(25%)=1562`, `duty(100%)=6250`, no int8 wraparound, 0 `-Wall -Wextra` warnings. |
| S2-U03 | 20 kHz H-bridge PWM output & polarity on `GP8/GP9` (Left) and `GP10/GP11` (Right) with soft-start ramp (`18%` start, `+2%`/20 ms) | FR4, FR11, NFR2 | manual | 1. Lift rover wheels off ground and command forward `(+25, -25)`, reverse `(-25, +25)`, and stop `(0, 0)`. 2. Inspect `GP8..GP11` on oscilloscope/logic analyzer and verify wheel rotation direction. | `20 kHz ± 1%` PWM on active half-bridge pin with complementary pin held low (0% duty); soft-start ramps `18% -> 25%` smoothly (`MOTOR_STATE_ACCEL -> MOTOR_STATE_CRUISE`); `[motors] state=1`, `state=2`, `state=4`, `state=0`. |
| S2-U04 | 4000 Hz Timer Alarm 0 quadrature encoder ISR (`GP16/17` Left, `GP2/3` Right) & 2-sample debounce filter | NFR2 | manual | 1. Spin left and right wheels by hand 1 full revolution (`~20` optical slots per motor shaft × gear ratio) forward and backward while logging `[dist]` serial line (every 500 ms). 2. Verify tick signs and absence of bounce chatter under 50%–100% motor load. | Left (`L`) and right (`R`) ticks increment monotonically in forward direction and decrement in reverse; no runaway spike noise under motor load; right channel B failover logs `[motors] Right encoder A missing; using B` if GP2 disconnected. |
| S2-U05 | Straight heading-lock PID (`Kp=1/2`, `Kd=3`, `Ki=1/50`, `±8%` trim) & single-encoder degraded 20% crawl fallback | FR4, NFR2, Tribunal 2 #3 | manual | 1. Drive rover straight on flat floor for 1 m and observe `err = run_l - run_r` and PWM trim on `[dist]` log. 2. Disconnect one wheel encoder (or hold 1 wheel sensor blocked) while driving >3 cm (`> 90` tick delta). | Straight run tracks within `<= 3 cm` lateral drift with bounded `±8%` trim; when `abs(diff) > 90` with one side at `0`, `ROBOT_ENCODER_DEGRADED` and `MOTOR_FAULT_BIT_ENCODER` assert and PWM caps to `20%` (`MOTOR_ENC_CRAWL_PCT`). |
| S2-U06 | Bounded translation & 90° pivot odometry accuracy (`0.33 mm/tick`, `MOTOR_STAGE2_PAD_TEST` 500 mm ruler test) | FR4, FR8, `IF-03`, `IF-04` | static / host + manual | 1. Host Python verifies `749 * 33 / 100 = 247 mm`, `500 mm = 1515 ticks`, `90 deg` pivot (`wheelbase=100 mm`) = `237 ticks`, stall cutoff `75 * 20 ms = 1500 ms`. 2. Set `MOTOR_STAGE2_PAD_TEST = 1u`, flash, run 500 mm ruler test on track surface, and measure actual distance with ruler. | Host math verified exact; physical rover stops near 500 mm mark (`±15 mm`), prints `[DISTANCE TEST COMPLETE]` with `Reported: 50 cm`, and emits `MSG_MOTOR_MOVE_DONE` (`[motors] Move completed`) after 3 zero-tick ticks (`60 ms`). |
| S2-U07 | Dual-wheel mechanical stall detection (`75` periods = `1.5 s` at `>= 20%` PWM) & fault recovery on stop command | FR11, NFR7 | manual | 1. Command rover at `>= 25%` PWM while physically immobilizing both wheels for `>= 1.6 s`. 2. Confirm PWM drops to 0% and `MOTOR_STATE_FAULT` latches (`MOTOR_FAULT_BIT_STALL`). 3. Send zero-speed stop `(0, 0)` command. | Both motor outputs cut to 0% after `1.5 s ± 0.1 s`; `[motors] state=5` (`MOTOR_STATE_FAULT`) with `ROBOT_FAULT_STALL` set; zero-speed command clears `MOTOR_FAULT_BIT_STALL` and returns to `MOTOR_STATE_IDLE` (`state=0`). |
| S2-U08 | E-stop immediate PWM cutoff (`<= 20 ms` cycle), 3-tick settle (`60 ms`) & `MSG_MOTOR_STOPPED` acknowledgement | FR11, NFR7, Tribunal 2 #7, `IF-04`, `IF-09` | manual | 1. While rover drives in `MOTOR_STATE_CRUISE`, assert `ROBOT_ESTOP_REQUEST` + `ROBOT_STOP_ACK_REQUEST` (via obstacle or IMU trigger). 2. Measure PWM cutoff latency and confirm `[motors] Stop confirmed`. 3. Clear `ROBOT_ESTOP_REQUEST` and verify return to `MOTOR_STATE_IDLE`. | PWM drops to 0% on the next 20 ms tick (`MOTOR_STATE_ESTOP`, `state=6`, `MOTOR_FAULT_BIT_ESTOP` set); after 3 zero-tick periods (`60 ms`), `MSG_MOTOR_STOPPED` sent and `[motors] Stop confirmed` logged; clearing request transitions cleanly to `state=0`. |

*Method:* **static / host** = compile-time `ROBOT_TASKS_STATIC_ASSERT` + ARM GCC build + host Python verification; **auto (on-target)** = `#ifndef NDEBUG` boot self-check (`CONSOLE=uart`); **manual** = observed on bench / ruler / oscilloscope.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S2-U01 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_host_motor_math_and_build_verify.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | Right-encoder A→B failover algorithm verified across all 9 vectors; `[motors] Encoder self-check failed` string verified compiled in `mtk3pico_smp1_uart.elf`. On-target boot run pending Pico W flash. |
| 2026-11-25 | S2-U02 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_host_motor_math_and_build_verify.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | 20 kHz PWM (`6249` wrap), `[-100, 100]` clamping, and all 4 motor `ROBOT_TASKS_STATIC_ASSERT` checks passed with 0 `-Wall -Wextra` warnings. |
| 2026-11-25 | S2-U06 | `2f19d6b` | Automated (host Python math check) | PASS | `logs/2026-11-25_host_motor_math_and_build_verify.txt` | Host odometry/pivot/stall arithmetic verified (`500 mm = 1515 ticks`, `90 deg = 237 ticks`, stall = `1500 ms`). Physical 500 mm `MOTOR_STAGE2_PAD_TEST` ruler run pending hardware. |

### Pending on-hardware verification

The following physical bench/track tests require the assembled Pico W rover:
- **S2-U01** (on-target boot check via `mtk3pico_smp1_uart.uf2`).
- **S2-U03**, **S2-U04**, **S2-U05**, **S2-U06** (physical 500 mm ruler test), **S2-U07**, **S2-U08**: Capture serial output and ruler/scope evidence into `logs/`.

### Known Tribunal 2 open items (tracked on `main` @ `2f19d6b`)
- Tribunal 2 #1 (`motor_task_move_forward/backward/turn_*` currently queue `ticks=0` when `dist_mm <= 0` instead of returning `E_PAR`), #2 (`motor_check_stalled` latches `MOTOR_STATE_FAULT` without asserting `ROBOT_ESTOP_REQUEST` to Navigator), #3 (`motor_start_new_motion` clears `ROBOT_ENCODER_DEGRADED` from IDLE while `MOTOR_FAULT_BIT_ENCODER` remains latched), and #5 (startup `tk_ext_tsk()` on PWM/GPIO/ISR setup error does not assert `ROBOT_ESTOP_REQUEST`) are accepted design items not yet patched on `2f19d6b`; add regression tests (`S2-U09+`) when implemented.
