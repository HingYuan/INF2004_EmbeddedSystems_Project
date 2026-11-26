# Team and subsystems

**Team:** CSC03 — INF2004 Autonomous Robotic Car  
**Implementation snapshot:** branch `csbit`, commit `6ffa02b8fd5de68f1c459450215ecedfb295025d`, reviewed 25 November 2026.  
**Repository:** https://github.com/HingYuan/INF2004_EmbeddedSystems_Project

This file records the implemented subsystem ownership, pins, peripherals, task priorities, interfaces and kernel-object allocation. Executable source and configuration take precedence over old design descriptions and stale code comments. The four supplied review/tribunal documents provide the member-to-buddy assignments and historical context; proposed changes in those documents are not automatically treated as implemented.

S1–S5 below mean Buddy 1–Buddy 5. They are ownership labels, not literal `subsystem1/`–`subsystem5/` directories. Paths are relative to this repository. The recorded build profile is `SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WEB_SERVER=1 MQTT=1`, with static-IP and UDP/TCP test options disabled. This identifies the local build configuration, not the firmware currently flashed on a board.

## 1. Members and subsystems

| # | Subsystem and responsibility | Student ID | GitHub username | Code | Tests and diagnostic support present |
|---|---|---|---|---|---|
| 1 | Network: Wi-Fi integration, MQTT commands and telemetry, HTTP dashboard | 2500290 | `Bryannyp` | `app_program/Network/`; integration with `lib/libwifi/sysdepend/pico_rp2040/` | `mqtt_codec_self_check()` in `Network/mqtt_client.c` in debug builds; `Network/mqtt_test_server/mqtt_test_server.py` publishes the current binary barcode mapping. | `testing/network/` directory. |
| 2 | Motor control: motor PWM, wheel encoder acquisition, odometry, bounded movement and stop acknowledgements | 2500728 | `akeell2002` | `app_program/Motor/` | `right_encoder_self_check()` in `Motor_task.c` in debug builds. | `testing/motor/` directory. |
| 3 | IR: analogue line sensing, digital barcode acquisition and Code 39 decoding; navigation and line-reentry integration | 2501325 | `HingYuan` | `app_program/IR/`; shared `app_program/Navigator/`; `app_program/Util/code39.h` | `ir_sensor_zeroing()` is diagnostic code, but its startup entry is commented out. | `testing/ir/` directory. |
| 4 | IMU: accelerometer/magnetometer acquisition, terrain and hump monitoring, tilt/lift detection and telemetry | 2500253 | `maxwellmenendez` | `app_program/IMU/IMU_task.c` and `IMU_task.h` | `imu_detector_self_check()` in `IMU_task.c` in debug builds. `IMU_backup.c` contains commented-out backup code. |`testing/imu/` directory. |
| 5 | Ultrasonic sensing and servo scanning: obstacle detection, profiling, width estimation and bypass coordination | 2502888 | `ChiSiang123` | `app_program/Ultrasonic/`; obstacle-related sections of shared `app_program/Navigator/` | `bypass_self_check()` and `obstacle_width_self_check()` in `Ultrasonic/Bypass_helpers.c`, called by Navigator in debug builds. | `testing/ultrasonic/` directory. |

Student IDs and buddy roles follow the supplied Week 6, Week 8 and pre-Tribunal records. Usernames are supported by the repository's contributor history. The test column describes code that exists; it does not assert that these checks or hardware tests passed during this review. The current USB build adds `-DNDEBUG`, so the debug self-checks are excluded from that build.

Shared integration files are `app_program/app_main.c`, `app_program/robot_tasks.c/.h`, `app_program/Navigator/Navigator_task.c/.h`, `app_program/Util/`, build/configuration files and this register. Navigator combines S3 route/junction/reentry logic with S5 obstacle/bypass logic and S1/S2/S4 interfaces; it is not a sixth student's subsystem. Changes to these shared contracts require coordination among the affected owners.

## 2. Pins

The table records the active application assignments. “Unassigned” means no active application use was found; it is not a claim about external wiring. Startup reuse is identified explicitly below.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | Reserved — Pico W radio/platform | CYW43439 radio control/data/clock pins; not available to student peripherals. |
| GP0 | S4 at runtime; platform during startup | IMU I2C0 SDA, selected by `imu_route_pins()`. Initially UART0 TX. |
| GP1 | S4 at runtime; platform during startup | IMU I2C0 SCL, selected by `imu_route_pins()`. Initially UART0 RX. |
| GP2 | S2 | Right encoder channel A, `MOTOR_RIGHT_ENC_A`. |
| GP3 | S2 | Right encoder channel B, `MOTOR_RIGHT_ENC_B`; backup channel used by right-encoder failover. |
| GP4 | S5 | Ultrasonic ECHO input, `SONAR_ECHO_PIN`. |
| GP5 | S5 | Ultrasonic TRIG output, `SONAR_TRIG_PIN`. |
| GP6 | S3 | Digital barcode input. `IR_LEFT_PIN` is passed to `ir_barcode_task`; the macro name does not mean the active left line-sensor input. |
| GP7 | Unassigned | No active application use. |
| GP8 | S2 at runtime; I2C0 during startup | Left motor output A, PWM slice 4 channel A. |
| GP9 | S2 at runtime; I2C0 during startup | Left motor output B, PWM slice 4 channel B. |
| GP10 | S2 | Right motor output A, PWM slice 5 channel A. |
| GP11 | S2 | Right motor output B, PWM slice 5 channel B. |
| GP12 | Unassigned | No active application use. |
| GP13 | Unassigned | No active application use. |
| GP14 | Unassigned | No active IMU interrupt input; the implemented IMU task polls I2C. |
| GP15 | S5 | Scanning servo, PWM slice 7 channel B, configured for 50 Hz. |
| GP16 | S2 at runtime; platform during startup | Left encoder channel A, `MOTOR_LEFT_ENC_A`. Also `BOARD_LED_PIN` in BSP startup; see the note below. |
| GP17 | S2 | Left encoder channel B is configured as an input; left odometry uses channel A rather than enabling left-channel-B failover. |
| GP18 | Unassigned | No active application use. |
| GP19 | Unassigned | The template's example heartbeat assignment does not apply. |
| GP20 | Unassigned | No active application use. |
| GP21 | Unassigned | No active application use. |
| GP22 | Unassigned | No active application use. |
| GP26 | Unassigned application input | ADC0-capable; BSP configures it as an analogue input, but the active line task does not sample channel 0. |
| GP27 | S3 | Right analogue line sensor: `tk_srea_dev(adc_dd, 1, ...)`, ADC1. |
| GP28 | S3 | Left analogue line sensor: `tk_srea_dev(adc_dd, 2, ...)`, ADC2. |

**Startup and comment discrepancies:**

- `config/config_tm.h` enables the UART0 mirror by default even with USB CDC. BSP startup selects UART on GP0/GP1; the IMU later changes those pins to I2C0. They cannot simultaneously provide external UART0 and IMU I2C signals. The template's `CRASH_REPORT=1` condition is not the allocation rule used by this source tree.
- BSP and the I2C0 driver initially select GP8/GP9 for I2C0. `motor_task()` subsequently selects PWM on those pins, and `imu_task()` selects GP0/GP1 for I2C0. The runtime motor and IMU assignments must be distinguished from this startup configuration.
- BSP startup configures `BOARD_LED_PIN`, currently GP16, as an output. `motor_task()` later changes GP16 to an encoder input. The example blink task is not started, but the startup output configuration remains in the code.
- The IR error strings mention GP26/GP27, but the actual ADC channel arguments are 1 and 2, corresponding to GP27/GP28. The IMU header comment that ultrasonic also uses GP0/GP1 is stale: the active ultrasonic definitions use GP5/GP4.

Primary evidence: `app_program/IR/IR_task.c/.h`, `app_program/Motor/Motor_task.c/.h`, `app_program/Ultrasonic/Ultrasonic_task.h`, `app_program/IMU/IMU_task.c/.h`, `kernel/sysdepend/pico_rp2040/hw_setting.c`, `include/sys/sysdepend/pico_rp2040/sysdef.h`, and the RP2040 ADC/I2C drivers under `device/`.

## 3. Peripherals

Hardware access follows the actual task and driver arrangement below. Other subsystems send motor commands, Navigator events or sonar requests instead of writing another subsystem's actuator registers. The ADC and I2C drivers execute through the calling task and their interrupt handlers; no separate ADC or I2C service task is created here.

| Peripheral | Owner subsystem | Owning task or execution context | Pins and use |
|---|---|---|---|
| UART0 | Platform console/serial driver | T-Monitor callers and UART driver interrupt handler; no application UART task | Initially GP0 TX / GP1 RX; these pins are later reassigned to S4 I2C0. `sera` is registered at startup. |
| UART1 | Unassigned | None | No active application use and no UART1 registration in the board startup path. |
| I2C0 | S4 through platform I2C driver | `imu_task()` uses `iica`; driver interrupt handler completes transfers | Runtime GP0 SDA / GP1 SCL. Accelerometer address 0x19, with 0x18 attempted as fallback; magnetometer 0x1E. |
| I2C1 | Unassigned | None | No active application use and no I2C1 registration in the board startup path. |
| SPI0 | Unassigned | None | No active application assignment. |
| SPI1 | Unassigned | None | No active application assignment. Pico W radio transport uses PIO SPI rather than assigning either of these hardware SPI blocks to a student task. |
| ADC | S3 through platform ADC driver | `ir_sensor_task()` uses `adca`; ADC interrupt handler completes conversions | ADC1/GP27 and ADC2/GP28. The barcode task uses digital GP6 and does not share the ADC. |
| PWM slice 4, channels A/B | S2 | `motor_task()` | GP8/GP9; divider 1, wrap 6249, giving 20 kHz at the configured 125 MHz system clock. |
| PWM slice 5, channels A/B | S2 | `motor_task()` | GP10/GP11; same motor PWM configuration. |
| PWM slice 7, channel B | S5 | `ultrasonic_task()` | GP15; divider 125, wrap 19999, giving 50 Hz at 125 MHz. Slice 7 timing is owned by S5 even though GP14/channel A is unused. |
| Hardware Timer Alarm 0 | S2 | `encoder_alarm_isr()`, installed by `motor_task()` | 250 microsecond re-arm interval, nominally 4 kHz; samples encoder GPIOs. This is not a kernel alarm-handler object. |
| Ultrasonic GPIO and pulse timing | S5 | `ultrasonic_task()` | GP5 TRIG / GP4 ECHO; task-level polling of echo timing using the hardware timer. |
| Wi-Fi radio, PIO/DMA transport, lwIP, MQTT and HTTP | Platform Wi-Fi service with S1 application modules | `cyw43_utk_task()` and callbacks/poll functions executed within that service | GP23/24/25/29; service pinned to physical core 1 in SMP builds. |
| USB controller and CDC console | Platform | `usb_service_task()` and USB interrupt handling | USB D+/D−; physical core 0 in SMP builds. |
| Free-running hardware timer reads | Shared read access | `time_us()` callers, including sensors, Navigator and diagnostics | No GPIO claim; reading the timer does not allocate another alarm. |

`config/config_device.h` enables serial, ADC and I2C support. `kernel/sysdepend/pico_rp2040/devinit.c` registers only unit 0 of each: `sera`, `adca` and `iica`.

### Wi-Fi and MQTT execution

S1 does not create a separate application `network_task`. The platform starts `cyw43_utk_task()` via `cyw43_utk_start()`. Its service loop calls `mqtt_dashboard_poll()` when MQTT is enabled and `web_server_poll()` when both networking and the web server are enabled. These application functions live under `app_program/Network/`; the template's suggested `mqtt_utk.c` heartbeat arrangement is not the implemented application path.

`wifi_status_task()` only reads `cyw43_utk_get_status()` snapshots and prints status. The robot tasks communicate with the network through Navigator messages, locked telemetry and shared flag APIs. MQTT telemetry is an explicitly encoded 35-byte binary payload; commands are 5 bytes, status is 10 bytes, and mapping packets are `2 + ceil(10N/8)` bytes for 1–66 entries. HTTP dashboard endpoints use JSON. The packed C structures are not the MQTT wire format.

## 4. Task priorities

Smaller numbers mean higher priority. The template's priority bands 12–21 and statement that priorities 1–4 are reserved do not describe this implementation. The actual task definitions are:

| Subsystem or service | Task | Priority | Physical core with SMP enabled | Startup condition |
|---|---|---|---|---|
| S1 status reporting | `wifi_status_task` | 8 | Core 1, `TP_PRC2` | `TM_WIFI_CYW43` enabled; created by `usermain()` |
| S2 | `motor_task` | 2 | Core 0, `TP_PRC1` | Active in `robot_tasks_start()` |
| S3 | `ir_sensor_task` | 2 | Core 0, `TP_PRC1` | Active |
| S3 | `ir_barcode_task` | 2 | Core 0, `TP_PRC1` | Active |
| S3 | `ir_barcode_decoder_task` | 2 | Core 0, `TP_PRC1` | Active |
| S4 | `imu_task` | 2 | Core 0, `TP_PRC1` | Active; its startup entry is not commented out |
| S5 | `ultrasonic_task` | 2 | Core 0, `TP_PRC1` | Active |
| Shared navigation | `navigator_task` | 1 | Core 0, `TP_PRC1` | Active |
| Shared diagnostics | `diagnostic_task` | 10 | Core 0, `TP_PRC1` | Active |
| Retained example pipeline | `producer_task` | 5 | Core 0, `TP_PRC1` | Still started by `usermain()` |
| Retained example pipeline | `consumer_task` | 6 | Core 0, `TP_PRC1` | Still started by `usermain()` |
| Platform Wi-Fi service | `cyw43_utk_task` | 4 | Core 1, `TP_PRC2` | Wi-Fi enabled |
| Platform USB console | `usb_service_task` | 3 | Core 0, `TP_PRC1` | T-Monitor and USB CDC enabled |
| Platform initial task | `init_task_main` calling `usermain` | 1 | No explicit affinity in its task attributes | Created during kernel startup; sleeps after application startup |

The eight entries in `robot_tasks_start()` each receive a 4096-byte stack. The retained producer, consumer and Wi-Fi status tasks also use 4096-byte stacks. The radio service uses 6144 bytes. There are no started monitor, blink or IR-zeroing tasks. In a supported single-core configuration the SMP affinity attributes are omitted.

Task priority and affinity are taken from `robot_tasks.c`, the task descriptors in `app_main.c`, `cyw43_utk_kernel.c`, `usb/usb_console.c` and `include/sys/inittask.h`, rather than descriptive comments in older documents.

## 5. Interfaces between subsystems

`robot_tasks_start()` creates the three robot message buffers and the sonar event flag centrally; their IDs are exported through `robot_tasks.h`. Their consumers and payload definitions are listed below. “Owners” identifies maintenance responsibility, not an inferred personal sign-off.

| ID | From → To | Mechanism | Object ID or API | Data and meaning | Owners |
|---|---|---|---|---|---|
| IF-01 | S3 barcode acquisition → S3 decoder | FIFO message buffer | `barcode_mbfid` | `BAR_RECORD` from `IR_task.h`: 4 bytes, state 1 bit and duration 26 bits in 100 microsecond units; capacity 20 records. Acquisition sends with `TMO_FEVR`; decoder receives with a 500 ms timeout. | S3; shared startup |
| IF-02 | S3 line sensing → Navigator | Shared FIFO message buffer | `navigator_mbfid` | `NAVIGATOR_MESSAGE` with `MSG_IR_UPDATE` and the classified black/white line state; sender uses `TMO_FEVR`. | S3; shared navigation |
| IF-03 | S3 barcode decoder → Navigator | Shared FIFO message buffer | `navigator_mbfid` | `MSG_BARCODE_DECODED`, decoded ASCII character. The current decoder suppresses `'*'` and `'?'`; sender uses `TMO_FEVR`. | S3; shared navigation |
| IF-04 | S5 ultrasonic → Navigator | Shared FIFO message buffer | `navigator_mbfid` | `MSG_OBSTACLE_DETECTED`, `MSG_OBSTACLE_PROFILED`, `MSG_SONAR_PROBED`, `MSG_TRACKING_OBSTACLE` and `MSG_SONAR_DISTANCE`. Payloads preserve distance/status/validity; probe/tracking payloads include angle and sample timestamp. Control replies use the 100 ms send helper; live distance/tracking samples use `TMO_POL`. | S5; shared navigation |
| IF-05 | Navigator → S5 ultrasonic | Event flag | `ranger_flgid` | `SONAR_RESUME_FLAG`, `SONAR_SCAN_REQUEST`, front/left/right probe flags and left/right tracking flags, defined only in `Ultrasonic_task.h`. Ultrasonic waits with `TWF_ORW` and `TWF_BITCLR`. | S5; shared navigation/startup |
| IF-06 | Navigator and bypass helpers → S2 Motor | FIFO message buffer plus bounded-motion API state | `motor_mbfid`; `moveForward`, `moveBackward`, `turnLeft`, `turnRight`, `stop` | `MOTOR_COMMAND` is 4 bytes: two signed 8-bit speeds and two independent unsigned 8-bit wheel tick targets. Capacity 10 commands. Bounded-motion helpers also store full-width targets inside `Motor_task.c` before sending, so their operation is not solely contained in the 4-byte packet. Helpers use `TMO_POL`; several direct Navigator sends use `TMO_FEVR`. | S2; shared navigation |
| IF-07 | S2 Motor → Navigator | Shared FIFO message buffer | `navigator_mbfid` | `MSG_MOTOR_STOPPED` and `MSG_MOTOR_MOVE_DONE`, sent non-blockingly after the relevant settling checks. There is no command identifier in these messages and no `MSG_MOTOR_FAULT` in the current enum. | S2; shared navigation |
| IF-08 | S4 IMU → Navigator | Shared FIFO message buffer | `navigator_mbfid` | `MSG_IMU_UPDATE` with `IMU_TELEMETRY_MSG`, a 6-byte packed payload containing event, pitch, roll, peak acceleration and height. Sends use `TMO_POL`. | S4; shared navigation |
| IF-09 | S1 network → Navigator | Shared FIFO message buffer | `navigator_mbfid` | `MSG_REMOTE_CMD` carries `MOTOR_COMMAND`; `MSG_WIFI_BARCODE_MAPPING` carries 7-bit ASCII plus 3-bit direction. MQTT callbacks validate/decode binary input and send with `TMO_POL`; a full queue can drop the message. | S1; shared navigation |
| IF-10 | Navigator and S4 → S1 telemetry/dashboard; shared readers | Shared data with a common lock | `telemetry_snapshot`, `telemetry_read`, `telemetry_imu_event`, `telemetry_heading`; `robot_data_lock/unlock` | `TELEMETRY_SNAPSHOT` is 32 bytes. Navigator stores navigation/odometry/sonar fields; IMU stores its telemetry fields. Writers and snapshot readers use the common lock. This is not a kernel mutex object. | S1, S4; shared navigation/startup |
| IF-11 | S2 Motor → Navigator and bypass helpers | Shared counters and getter functions | `enc_l_ticks`, `enc_r_ticks`, `motor_get_*` | Signed encoder counters, distance and speed. The Motor-owned ISR updates the encoder counters; Motor exposes odometry getters. Consumers do not configure the encoder GPIOs or Timer Alarm 0. | S2; shared navigation/S5 |
| IF-12 | Motor, IMU, sonar, Navigator and network → shared status/stop consumers | Aligned shared word and lock-protected updates | `robot_flags_get`, `robot_flags_update`, `robot_flag_assign` | Thirteen defined bits include motor/sensor/network faults, no-echo status, encoder-degraded state, stop-ack request and E-stop request. `motor_estop`, `motor_stop_ack_requested`, `motor_fault_word` and `sonar_fault` are views/macros over this word, not separate event-flag objects. | All affected owners; shared startup |
| IF-13 | Platform Wi-Fi service → S1 status task | Published status snapshot | `cyw43_utk_get_status` | Connection/address/network diagnostics read by `wifi_status_task`; the status task does not own the radio stack. | S1; platform Wi-Fi service |

All Navigator producers use the same **64-message** buffer and the same **34-byte** `NAVIGATOR_MESSAGE` envelope. `Navigator_task.h` contains compile-time size assertions. The shared application lock uses an interrupt-preserving spinlock with SMP, and interrupt masking without SMP; it is not counted as `tk_cre_mtx()` or `tk_cre_flg()` usage.

The implemented stop/scan exchange is ultrasonic E-stop request and obstacle event → Navigator stop-ack request → Motor `MSG_MOTOR_STOPPED` → Navigator `SONAR_SCAN_REQUEST` → ultrasonic profile reply. A request flag or profile reply is an interface event, not proof that physical stopping or clearance has been tested.

The normal startup mode is `MODE_WIFI_SEND_COMMS`, not immediate line following. Navigator also calls `clear_navigator_mbf()` after its initial two-second delay. These are actual source behaviours, even where older design notes describe a different startup sequence.

Integration records: [testing/integration/TEST_LOG.md](testing/integration/TEST_LOG.md).
System records: [testing/system/TEST_LOG.md](testing/system/TEST_LOG.md).


## 6. Kernel object budget

Limits below come from `config/config.h`. Counts assume successful normal startup with the recorded USB/Wi-Fi build profile, ADC and I2C devices opened by the active IR/IMU tasks, and no extra diagnostic tasks or device opens. They are a source-derived allocation count, not a live kernel measurement.

Central application IPC is counted under **Shared**, including the objects created by `robot_tasks_start()` and the retained producer/consumer example in `usermain()`. The **Platform** column includes kernel/device support and the USB/radio service tasks. Each object is counted once.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | Shared | Platform | Total | Remaining |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Task | 32 | 1 | 1 | 3 | 1 | 1 | 4 | 3 | 14 | 18 |
| Semaphore | 16 | 0 | 0 | 0 | 0 | 0 | 1 | 5 | 6 | 10 |
| Event flag | 16 | 0 | 0 | 0 | 0 | 0 | 2 | 5 | 7 | 9 |
| Mailbox | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 |
| Mutex | 4 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 3 |
| Message buffer | 4 | 0 | 0 | 0 | 0 | 0 | 4 | 0 | 4 | 0 |
| Fixed memory pool | 4 | 0 | 0 | 0 | 0 | 0 | 1 | 0 | 1 | 3 |
| Variable memory pool | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 |
| Cyclic handler | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 |
| Alarm handler | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 |

Allocation details:

- **Tasks:** S1 is `wifi_status_task`; S2/S3/S4/S5 account for six sensor/motor tasks. Shared comprises Navigator, diagnostics, producer and consumer. Platform comprises the initial task, USB service and Wi-Fi service. MQTT and HTTP do not add separate tasks.
- **Semaphores:** the example creates one credit semaphore. Registration of `adca`, `iica` and `sera` creates three semaphore-backed `FastLock` instances through `msdi_def_dev()`. Opening `adca` and `iica` creates two device-manager abort semaphores. Each additional successful device open needs another semaphore.
- **Event flags:** application allocation is `ranger_flgid` plus the example's `flgid`. Platform allocation is one device-manager `FastMLock`, one ADC completion flag, one I2C0 completion flag and two UART0 transmit/receive flags.
- **Mutex and fixed pool:** `usermain()` still creates the example's `mtxid` and `mpfid`; its producer/consumer tasks still run. These are not robot subsystem IPC.
- **Message buffers:** the robot creates `barcode_mbfid`, `navigator_mbfid` and `motor_mbfid`; the retained example creates `mbfid`. All four configured slots are occupied. The device-manager event buffer is disabled by `CNF_DEVT_MBFSZ0 = -1` and does not consume a fifth slot.
- **No cyclic/alarm objects:** task delays, the hardware microsecond counter and Motor's hardware Timer Alarm 0 are not `tk_cre_cyc()` or `tk_cre_alm()` allocations.

Robot message-buffer backing storage includes a 4-byte record-length header and word-rounded payload per entry:

| Buffer | Payload bytes | Capacity | Bytes per stored entry | Static backing storage |
|---|---:|---:|---:|---:|
| `barcode_mbfid` | 4 | 20 | 8 | 160 bytes |
| `navigator_mbfid` | 34 | 64 | 40 | 2560 bytes |
| `motor_mbfid` | 4 | 10 | 8 | 80 bytes |
| **Robot buffers total** | | | | **2800 bytes** |

The example's separate buffer is 56 bytes, making total application message-buffer backing storage **2856 bytes**, excluding kernel control blocks. The 2288-byte robot-buffer figure in the earlier compression comment predates the obstacle-width extension; it is not the current allocation.

Budget evidence: `app_program/app_main.c`, `app_program/robot_tasks.c`, `config/config.h`, `kernel/sysdepend/pico_rp2040/devinit.c`, `kernel/tkernel/device.c`, `kernel/tkernel/deviceio.c`, `device/common/drvif/msdrvif.c`, `lib/libtk/fastlock.c`, `lib/libtk/fastmlock.c`, and the RP2040 ADC/I2C/serial drivers.
