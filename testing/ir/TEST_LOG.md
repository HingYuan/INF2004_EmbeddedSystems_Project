# Subsystem 3 — test log

**Subsystem:** IR Line Following, Code 39 Barcode Decoding & Central Navigator FSM (Buddy 3)  ·  **Student ID:** `2501325` (Chan Hing Yeung)  ·  **Code:** `app_program/ir/ir_task.{c,h}`, `app_program/util/code39.{c,h}`, `app_program/navigator/navigator_task.{c,h}`, `app_program/ir/ir_encoders_main.{c,h}`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 (`ISO/IEC 16388`) 44-entry lookup table completeness, 3-of-9 wide-element parity & unknown-pattern fallback (`'?'`) | FR2, NFR3 | static / host | 1. Parse `g_code39_dict` in `app_program/util/code39.c` via host Python verification. 2. Verify all 44 symbols (`A–Z`, `0–9`, `-`, `.`, ` `, `$`, `/`, `+`, `%`, `*`), 9-bit bounds, exact popcount of 3 wide elements per symbol, 0 duplicate patterns/characters, and `code39_entry_size == 2` compile-time assert. | All 44/44 ISO/IEC 16388 entries unique and valid with exact 3-of-9 parity; invalid pattern returns `'?'`; `sizeof(code39_entry_t) == 2`. |
| S3-U02 | Two-sensor IR line classifier (`ir_classify_line()`) 4-state truth table at fixed `IR_ADC_THRESHOLD_MV = 500 mV` | FR1 | static / host | Verify `ir_classify_line(left_mv, right_mv)` boundary voltages `0, 499, 500, 3300 mV` across all 4 black/white combinations (`>= 500 mV` = black). | `(499,499)->IR_STATE_W_W`, `(499,500)->IR_STATE_W_B`, `(500,499)->IR_STATE_B_W`, `(500,500)->IR_STATE_B_B`. |
| S3-U03 | 10-bar Code 39 adaptive mean thresholding (`ir_decode_bar_window()`) & start/stop asterisk (`'*'`) / unknown (`'?'`) filtering | FR2, NFR3, `IF-01`, `IF-02` | static / host | Feed synthetic 10-bar duration sequences (leading space + 9 alternating narrow/wide bars, 1:3 ratio) for `'A'`, `'B'`, `'C'`, `'F'`, `'Z'`, `'*'`, and corrupted `0x000` through the exact `ir_decode_bar_window()` mean-threshold algorithm. | `'A'`, `'B'`, `'C'`, `'F'`, `'Z'` decode accurately and forward `MSG_BARCODE_DECODED`; `'*'` and `'?'` decode and print but are NOT forwarded to Navigator. |
| S3-U04 | Barcode RX inter-bar timeout reset (`IR_BARCODE_RX_TIMEOUT_MSEC = 500 ms`) on `GP6` (`ir_task_barcode_task` → `ir_task_decoder_task`) | FR2, NFR3, `IF-01` | manual | 1. Flash rover and pass partial (<10) black/white bars over `GP6` barcode IR sensor, then pause `> 500 ms`. 2. Pass a complete valid Code 39 character (e.g. `*A*`) over `GP6`. | Partial burst resets `decoder.bar_count = 0` on `E_TMOUT` after 500 ms; subsequent valid barcode decodes cleanly (`[decoder] Matched pattern 0x109 to character: A`) without frame desync. |
| S3-U05 | Live ADC line sensing (`GP27` ADC1 right, `GP28` ADC2 center) on black tape vs white track at 100 Hz | FR1, NFR1 | manual | 1. Place rover on white surface (`W_W`), right sensor on black (`W_B`), center/left on black (`B_W`), and both on black junction bar (`B_B`). 2. Observe Navigator telemetry `ir` state (`0..3`) and motor differential steering (`16%` straight, `10/30` base turn, `5/40` ramp after 50 ticks). | All 4 states transition cleanly with zero false junction triggers; curved drift ramps smoothly from `10/30` to `5/40` after `NAV_TURNING_RAMPUP_LIMIT = 50` consecutive off-center samples. |
| S3-U06 | Navigator boot gate (`MODE_WIFI_SEND_COMMS`) → 5 barcode mappings → `MODE_LINE_FOLLOWING` & command retention | FR2, FR3, NFR4, `IF-07` | manual | 1. Boot rover and confirm motors stay at `(0, 0)` in `MODE_WIFI_SEND_COMMS` (`mode=1`). 2. Publish 5-entry mapping via `mqtt_test_server.py`. 3. Scan barcode(s) matching mapped characters and verify route retention until junction execution. | Console prints `[nav] mapping received: <C> -> <dir>` 5 times, then switches to `MODE_LINE_FOLLOWING` (`mode=2`); decoded barcode direction stored in `ctx.instructions` and retained unchanged until junction turn. |
| S3-U07 | Junction advance (`250 ms` pre + `900 ms` forward + `30 ms` post) and line-crossing pivot execution (`23%` pivot, `50 ms` settle/stop) | FR3, FR4, `IF-03` | manual | 1. Load route instructions for `DIR_LEFT`, `DIR_RIGHT`, `DIR_BACK` (2 crossings), and `DIR_FRONT`. 2. Drive rover into a `B_B` junction for each command and observe turn accuracy and return to `MODE_LINE_FOLLOWING`. | On `IR_STATE_B_B`, rover stops, advances 900 ms to align axle, pivots at `±23%` until required `W_W -> B_W/W_B` crossing count reaches 0, pauses 50 ms, increments `instruction_idx`, and resumes line following. |
| S3-U08 | Packed IPC & Navigator struct compile-time size assertions (`nav_state_size`, `nav_msg_size`, `telemetry_size`, `ir_bar_record_size`, `ir_runtime_size`) | Tribunal 1, NFR12 | static / host | Compile both Debug and Production firmware ELFs and verify all 15 IR/Code 39/Navigator `ROBOT_TASKS_STATIC_ASSERT` checks pass with 0 `-Wall -Wextra` warnings. | `sizeof(ir_task_runtime_t)==1`, `sizeof(ir_task_bar_record_t)==4`, `sizeof(code39_entry_t)==2`, `sizeof(navigator_task_state_t)==10`, `sizeof(navigator_task_msg_t)==34`, `sizeof(navigator_task_telemetry_t)==32`, 0 warnings. |

*Method:* **static / host** = compile-time `ROBOT_TASKS_STATIC_ASSERT` + ARM GCC build + host Python verification; **manual** = observed on physical IR track / serial / dashboard.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S3-U01 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_host_code39_and_ir_verify.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | All 44/44 ISO/IEC 16388 Code 39 entries verified unique with exact 3-of-9 parity; `code39_entry_size == 2` asserted. |
| 2026-11-25 | S3-U02 | `2f19d6b` | Automated (host Python) | PASS | `logs/2026-11-25_host_code39_and_ir_verify.txt` | `ir_classify_line()` 500 mV boundary truth table verified across all 4 states (`W_W`, `W_B`, `B_W`, `B_B`). |
| 2026-11-25 | S3-U03 | `2f19d6b` | Automated (host Python) | PASS | `logs/2026-11-25_host_code39_and_ir_verify.txt` | 10-bar adaptive mean thresholding verified for `A, B, C, F, Z, *` and invalid `0x000 -> '?'`. |
| 2026-11-25 | S3-U08 | `2f19d6b` | Automated (ARM GCC) | PASS | `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | All IR, Code 39, and Navigator compile-time `ROBOT_TASKS_STATIC_ASSERT` checks passed across Debug & Production ELFs with 0 `-Wall -Wextra` warnings. |

### Pending on-hardware verification

The following physical sensor/track tests require the assembled Pico W rover:
- **S3-U04**, **S3-U05**, **S3-U06**, **S3-U07**: Run on physical black-tape track with printed Code 39 barcodes (`>= 20` consecutive trials for NFR3 95% barcode reliability) and save serial logs + video links.

### Known Tribunal 2 open items (tracked on `main` @ `2f19d6b`)
- Tribunal 2 #8 (asserting `ROBOT_ESTOP_REQUEST` if `ir_task_sensor_task` fails ADC open/read rather than exiting loop silently) and #9 (single bounded reverse-and-rescan on `'?'` barcode read) are accepted design items not yet patched on `2f19d6b`; add regression tests (`S3-U09+`) when implemented.
