# Subsystem 1 — test log

**Subsystem:** Wi-Fi, Binary MQTT v1 Codec, HTTP Web Server & Dashboard (Buddy 1)  ·  **Student ID:** `2500290` (Bryan Tan Chee Kai)  ·  **Code:** `app_program/network/` (`mqtt_client.{c,h}`, `web_server.{c,h}`, `index_html.{c,h}`, `mqtt_test_server/`), `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Binary MQTT v1 `0x12` motor command decode & `[-100, 100]` speed bounds rejection | FR10, Tribunal 2 #15, `IF-07` | auto (on-target) | Build `SMP=1 CONSOLE=uart` and boot Pico W so `navigator_task` runs `mqtt_client_codec_check()` (`mqtt_client.c:311-360`): valid `{0x12, 100, -100, 255, 7}` decodes cleanly; out-of-range `101` and truncated 4-byte frame are rejected. | Boot completes without `[bypass] Self-check failed` and without asserting `ROBOT_ESTOP_REQUEST`. |
| S1-U02 | Binary MQTT v1 `0x13` barcode mapping codec (1..66 entries, 10-bit packing, case folding, duplicate/quote/padding rejection) | FR10, Tribunal 2 #18, `IF-07` | static / host + auto (on-target) | 1. Host Python check runs `encode_movement_mapping()` from `mqtt_test_server.py` on single entry (`'a'->TURN_LEFT`), 5-entry default, 66-entry max (`85 B`), and 5 invalid inputs (`{}`, duplicate `'a'/'A'`, `'"'`, `'\\'`, 2-char key). 2. On-target `mqtt_client_codec_check()` verifies `{0x13, 0x01, 0x41, 0x01}` decodes `'A'->DIR_LEFT`, `'B'->DIR_INVALID`, and rejects non-zero trailing pad bit `0x80`. | Single entry matches `13014101` bit-for-bit with C test vector; 5-entry = 9 B; max 66-entry = 85 B (`MQTT_PAYLOAD_MAX`); all invalid inputs rejected. |
| S1-U03 | Binary MQTT v1 `0x11` 35-byte telemetry bit-stream layout & sign-extension round-trip | FR10, NFR10, Tribunal 1, `IF-08` | static / host + auto (on-target) | 1. Host Python audit parses `mqtt_client_encode_telem()` field widths (`8 + 252 + 13 + 2 + 5 = 280` bits = 35 B, `flags` end at bit `273`). 2. On-target `mqtt_check_telem_frame()` encodes extreme values (`0xFFFFFFFF`, `INT32_MIN`, `INT32_MAX`, `-100/+100`, `-45 deg`, `-1800/+1800 ddeg`, all 13 flags) and checks every field plus zero padding mask `0xF8`. | Exact 35-byte wire layout (`MQTT_TELEM_FLAGS_BIT_END == 273`); zero non-zero padding bits; `INT32_MIN` round-trips cleanly. |
| S1-U04 | Embedded dashboard HTML flash asset integrity (`index_html.c`) & compile-time static assertions | FR10, BARR-C 3.6.b, Tribunal 1 | static / host | 1. Build full Wi-Fi/Web/MQTT ELF enforcing `index_html_len_match`, `web_state_size` (8 B), `web_html_len_fit`, `web_buf_limits`, `web_hdr_lengths`, `mqtt_runtime_size` (3 B), `mqtt_map_size` (49 B). 2. Run host Python verification on `index_html.c`. | Exact length `44339` bytes (`INDEX_HTML_BODY_LEN`), pure 7-bit ASCII source, FNV-1a 64-bit hash `0x3474edf60d899abb`, polls `/api/telemetry` & `/api/mqtt` every `1000 ms`, 0 compiler warnings. |
| S1-U05 | Embedded HTTP server routes (`/`, `/api/telemetry`, `/api/mqtt`, `/favicon.ico`) & single-client backpressure | FR10, NFR10, `IF-08` | manual | 1. Flash full Wi-Fi/Web/MQTT image and wait for `[web] listening on port 8080`. 2. Request `GET /`, `GET /api/telemetry`, `GET /api/mqtt`, and `GET /favicon.ico` from a LAN host via browser/curl. 3. Open a second concurrent connection during active HTML transfer. | `/` returns 200 OK with 44,339 B HTML; `/api/telemetry` & `/api/mqtt` return valid `application/json` with `Cache-Control: no-store`; `/favicon.ico` returns `404 Not Found`; concurrent second client aborted cleanly (`ERR_ABRT`) without crashing server. |
| S1-U06 | Live binary MQTT telemetry (2 Hz), retained status heartbeat (0.2 Hz) & offline LWT will (`0x14`) | FR10, NFR10, Tribunal 2 #20 | manual | 1. Run `mqtt_broker.py` on LAN host and connect Pico W. 2. Subscribe to `robot/telemetry` and `robot/status`. 3. Observe frame intervals for 30 s, then power-cut Pico W and wait >30 s keepalive. | `[mqtt] connected to <BROKER_IP>`; 35-byte `0x11` frames arrive every `500 ms`; 10-byte `0x14` status (`online=1`) arrives every `5000 ms`; broker publishes retained 10-byte offline will (`0x14`, `online=0`) on unexpected disconnect. |
| S1-U07 | Core 1 Wi-Fi auto-reconnect (5 s backoff), DHCP restart & MQTT recovery without reboot | NFR9, `IF-09` | manual | 1. With rover connected to AP & broker, disable AP/hotspot for 15 s. 2. Re-enable AP and time recovery on serial console (`app_main_wifi_status_task` + `mqtt_client_poll`). | `[wifi] link DOWN` and `ROBOT_FAULT_WIFI | ROBOT_FAULT_MQTT` set; within `<= 10 s` after AP returns, `[wifi] link UP`, new DHCP `[wifi] ip=...`, and `[mqtt] connected` resume automatically with fault bits cleared. |
| S1-U08 | Malformed MQTT payload & full Navigator queue non-blocking drop safety (`TMO_POL`) | Tribunal 2 #15, #18, #19, `IF-07` | manual | 1. Publish malformed `0x12` frame (speed `120` or length `3`) on `robot/command`. 2. Publish malformed `0x13` frame (bad padding or duplicate key) on `robot/config/movement`. 3. Confirm existing `g_movement_map` and Core 1 polling remain intact. | Console prints `[mqtt] invalid binary command ignored` / `[mqtt] invalid binary mapping ignored`; prior valid mapping preserved; Core 1 `cyw43_utk_task` never blocks (`TMO_POL`). |

*Method:* **static / host** = compile-time `ROBOT_TASKS_STATIC_ASSERT` + ARM GCC build + host Python verification; **auto (on-target)** = `#ifndef NDEBUG` boot self-check (`CONSOLE=uart`); **manual** = observed on live Pico W / LAN.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S1-U02 | `2f19d6b` | Automated (host Python) | PASS | `logs/2026-11-25_host_protocol_and_html_verify.txt` | `encode_movement_mapping()` verified: `'a'->TURN_LEFT` = `13014101` (matches C vector), 5-entry = 9 B, 66-entry max = 85 B, 5 invalid inputs rejected. On-target C check pending Pico W flash. |
| 2026-11-25 | S1-U03 | `2f19d6b` | Automated (host Python) | PASS | `logs/2026-11-25_host_protocol_and_html_verify.txt` | `mqtt_client_encode_telem()` bit-stream verified: 8 + 252 + 13 + 2 + 5 = 280 bits = 35 bytes (`flags` end bit 273). On-target C check pending Pico W flash. |
| 2026-11-25 | S1-U04 | `2f19d6b` | Automated (ARM GCC + host Python) | PASS | `logs/2026-11-25_host_protocol_and_html_verify.txt`, `../system/logs/2026-11-25_multi_profile_build_and_static_audit.txt` | `index_html.c` verified 44,339 B, pure 7-bit ASCII, FNV64 `0x3474edf60d899abb`, 0 `-Wall -Wextra` warnings, all 7 network `ROBOT_TASKS_STATIC_ASSERT` checks passed. |

### Pending on-hardware verification

The following tests require flashing the Pico W and observing live serial/network traffic (no results fabricated):
- **S1-U01** (plus on-target portion of **S1-U02**, **S1-U03**): Flash `mtk3pico_smp1_uart.uf2` (symbol `mqtt_client_codec_check` verified linked at `0x1001154c`) and capture clean boot console log.
- **S1-U05**, **S1-U06**, **S1-U07**, **S1-U08**: Flash `mtk3pico_smp1_usb_cdc_wifi_dns.uf2`, connect to LAN AP + `mqtt_broker.py`, capture serial + HTTP/MQTT client logs into `logs/`.

### Known Tribunal 2 open items (tracked on `main` @ `2f19d6b`)
- Tribunal 2 #14 (500 ms remote-command dead-man watchdog), #16 (`MODE_WIFI_SEND_COMMS` 20 s boot timeout), and #17 (exponential backoff on `mqtt_publish()` errors) are accepted design enhancements not yet implemented on `2f19d6b`; add dedicated regression tests (`S1-U09+`) when merged.
