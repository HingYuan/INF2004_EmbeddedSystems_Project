# Project Table of Abbreviations and Acronyms (BARR-C:2018 Rule 1.5.b)

This table documents approved project-wide abbreviations and acronyms in
addition to BARR-C:2018 Appendix A. Every abbreviation used in identifiers or
comments must appear either in BARR-C Appendix A or in this table.

| Abbreviation | Full Meaning | Context |
|---|---|---|
| `adc` | Analog-to-Digital Converter | RP2040 ADC peripheral (`ir_task`) |
| `ang` | Angle | Servo/sonar angles in degrees |
| `assprc` | Assigned Processor Bitmask | micro T-Kernel SMP task affinity |
| `avg` | Average | Odometry and bar duration calculations |
| `b_` | Boolean Variable Prefix | BARR-C Rule 7.1.m |
| `bdu` | Block Data Update | LSM303DLHC `CTRL_REG4_A` register bit |
| `buf` | Buffer | Fixed static byte/word arrays |
| `cfg` | Configuration | Task/peripheral setup descriptors |
| `ch` | Channel / Character | ADC/PWM channel or ASCII byte |
| `chk` | Check | Debug self-check validation helper |
| `clr` | Clearance / Clear | Bypass clearance flags and bit masks |
| `cm` | Centimetre(s) | Distance unit |
| `cmd` | Command | Motor and MQTT command payloads |
| `cnt` | Count | Integer event/iteration counter |
| `ctx` | Context | Task runtime context structure |
| `cur` | Current | Active state or sample |
| `ddeg` | Tenths of a Degree (`0.1 deg`) | Fixed-point tilt telemetry |
| `deg` | Degree(s) | Angular unit |
| `dev` | Device | micro T-Kernel device driver handle |
| `diag` | Diagnostic | USB bring-up and CPU load diagnostics |
| `dir` | Direction | Navigation route direction enum |
| `dist` | Distance | Linear travel or sonar range |
| `dmm` | Tenths of a Millimetre (`0.1 mm`) | Fixed-point hump height telemetry |
| `dur` | Duration | Elapsed pulse or confirmation window |
| `enc` | Encoder | Optical wheel quadrature encoder |
| `err` | Error | micro T-Kernel `ER` or lwIP `err_t` |
| `est` | Estimated | Computed obstacle width |
| `estop` | Emergency Stop | Latched safety motor stop |
| `evt` | Event | IMU or Navigator event enum |
| `exinf` | Extended Information | micro T-Kernel task entry parameter |
| `f32` | 32-bit IEEE-754 Float (`float32_t`) | IMU single-precision helper suffix |
| `filt` | Filtered | Median/EMA smoothed signal |
| `flg` | Event Flag | micro T-Kernel event flag object |
| `fmt` | Format | String formatting helper |
| `fsm` | Finite State Machine | Control state machine |
| `g_` | Global Variable Prefix | BARR-C Rule 7.1.j |
| `gpio` | General-Purpose Input/Output | RP2040 GPIO peripheral |
| `gw` | Gateway | IPv4 network gateway |
| `h_` | Handle Variable Prefix | BARR-C Rule 7.1.n (RTOS `ID` handles) |
| `hdr` | Header | HTTP or MQTT frame header |
| `hist` | History | Rolling median sample buffer |
| `hw` | Hardware | Physical peripheral fault or init |
| `i2c` | Inter-Integrated Circuit | RP2040 I2C0 bus for LSM303DLHC |
| `idx` | Index | Zero-based array or route index |
| `imu` | Inertial Measurement Unit | LSM303DLHC accel + magnetometer |
| `init` | Initialize | Startup configuration routine |
| `inst` | Instruction | Route turn instruction |
| `integ` | Integral | Heading-lock PID integral term |
| `ir` | Infrared | Line-following and barcode IR sensors |
| `irq` | Interrupt Request | Hardware interrupt line |
| `isr` | Interrupt Service Routine | Hardware interrupt handler (`*_isr`) |
| `lat` | Lateral | Sideways X projection in cm |
| `len` | Length | Byte or string length |
| `lvl` | Level | Signal or diagnostic level |
| `mag` | Magnitude / Magnetometer | PWM speed magnitude or LSM303 mag |
| `max` | Maximum | Upper bound |
| `mbf` | Message Buffer | micro T-Kernel IPC message queue |
| `med` | Median | 3-sample median filter output |
| `mg` | Milli-g (`0.001 g`) | Accelerometer sensitivity unit |
| `min` | Minimum | Lower bound |
| `mm` | Millimetre(s) | Linear distance unit |
| `mmps` | Millimetres per Second | Linear vehicle speed (avoids `_s`) |
| `mon` | Monitor | System monitor task |
| `mot` | Motor | DC drive motor subsystem |
| `mpf` | Fixed-Size Memory Pool | micro T-Kernel IPC memory pool |
| `mrad` | Milliradian(s) | Fixed-point angle unit |
| `msec` | Millisecond(s) | BARR-C Appendix A time unit |
| `msg` | Message | IPC message payload |
| `mtx` | Mutex | micro T-Kernel mutual exclusion lock |
| `mv` | Millivolt(s) | Scaled ADC sensor voltage |
| `nav` | Navigator | Central robot decision task |
| `netif` | Network Interface | lwIP `struct netif` |
| `num` | Number | Count or pin index |
| `obs` | Obstacle | Detected obstacle profile/distance |
| `off` | Offset | Bit or lateral offset |
| `p_` | Pointer Variable Prefix | BARR-C Rule 7.1.k |
| `pad` | Padding | Bit-stream or body clearance margin |
| `pcb` | Protocol Control Block | lwIP `struct tcp_pcb` |
| `pct` | Percent (`%`) | PWM duty or CPU load percentage |
| `pend` | Pending | Deferred state flag |
| `periph` | Peripheral | Hardware subsystem task |
| `pid` | Proportional-Integral-Derivative | Straight-line wheel heading lock |
| `pkt` | Packet | Network or MQTT frame |
| `pos` | Position | Buffer write cursor or servo angle |
| `prc` | Processor | RP2040 core (1 = core 0, 2 = core 1) |
| `prev` | Previous | Prior loop/sample value |
| `pri` | Priority | RTOS task or IRQ priority |
| `prof` | Profile | Multi-ray obstacle sonar profile |
| `ptn` | Pattern | Event flag bitmask |
| `pub` | Publish | MQTT publish message |
| `pwm` | Pulse-Width Modulation | Motor and servo hardware PWM |
| `rad` | Radian(s) | Angular unit |
| `rec` | Record | Barcode or demo IPC record |
| `ref` | Reference | Baseline comparison value |
| `reg` | Register | Hardware or I2C register address |
| `rem` | Remaining | Unprocessed remainder |
| `req` | Request | Pending flag or HTTP request |
| `resp` | Response | HTTP API response buffer |
| `ret` | Return | Single-exit return variable |
| `rot` | Rotation(s) | Junction line-crossing counter |
| `rt` | Runtime | Packed runtime task state |
| `rx` | Receive | I2C/TCP receive buffer |
| `sem` | Semaphore | micro T-Kernel counting semaphore |
| `seq` | Sequence | Monotonic telemetry counter |
| `spd` | Speed | Motor PWM percentage |
| `sq` | Squared | Value multiplied by itself |
| `st` | Status | Measurement status enum |
| `stacd` | Start Code | micro T-Kernel task entry parameter |
| `str` | String | NUL-terminated ASCII string |
| `sz` | Size | Byte count |
| `tbl` | Table | Constant lookup table |
| `telem` | Telemetry | Snapshot / MQTT telemetry data |
| `tgt` | Target | Commanded setpoint |
| `thresh` | Threshold | Detection cutoff constant |
| `tk` | T-Kernel / Ticks | micro T-Kernel API prefix or ticks |
| `tmo` | Timeout | RTOS or movement timeout |
| `tol` | Tolerance | Allowed comparison band |
| `trig` | Trigger | HC-SR04 trigger pin or IRQ flag |
| `tx` | Transmit | I2C/TCP transmit buffer |
| `u16` | Unsigned 16-bit Integer | `uint16_t` |
| `u32` | Unsigned 32-bit Integer | `uint32_t` |
| `us` | Ultrasonic | Standalone ultrasonic sketch prefix |
| `usec` | Microsecond(s) | BARR-C Appendix A time unit |
| `val` | Value | Generic scalar value |
| `vel` | Velocity | Wheel speed difference |
| `wr` | Write | TCP write result |
