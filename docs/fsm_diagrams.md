# Firmware Finite State Machine Diagrams (BARR-C:2018 Rule 2.2.f)

This version-controlled document maintains the state machine and workflow diagrams referenced by [navigator_task.h](../../app_program/navigator/navigator_task.h), [navigator_task.c](../../app_program/navigator/navigator_task.c), [imu_task.c](../../app_program/imu/imu_task.c), [motor_task.c](../../app_program/motor/motor_task.c), and [ultrasonic_task.c](../../app_program/ultrasonic/ultrasonic_task.c).

---

## 1. Central Navigator FSM (`navigator_task`)

```mermaid
stateDiagram-v2
    [*] --> MODE_LINE_FOLLOWING
    MODE_LINE_FOLLOWING --> MODE_BARCODE_SCANNING : "MSG_BARCODE_DECODED"
    MODE_BARCODE_SCANNING --> MODE_WIFI_SEND_COMMS : "Route / MQTT command"
    MODE_WIFI_SEND_COMMS --> MODE_LINE_FOLLOWING : "Command applied"

    MODE_LINE_FOLLOWING --> MODE_DECISION_MAKING_ADVANCE : "IR_STATE_B_B junction"
    MODE_DECISION_MAKING_ADVANCE --> MODE_DECISION_MAKING_TURN : "Pre-advance complete"
    MODE_DECISION_MAKING_TURN --> MODE_LINE_FOLLOWING : "Line re-acquired"

    MODE_LINE_FOLLOWING --> MODE_GOING_OVER_HUMP : "IMU_MSG_HUMP_DETECTED"
    MODE_GOING_OVER_HUMP --> MODE_LINE_FOLLOWING : "IMU_MSG_NO_HUMP"

    MODE_LINE_FOLLOWING --> MODE_OBSTACLE_ASSESS : "MSG_OBSTACLE_DETECTED"
    MODE_OBSTACLE_ASSESS --> MODE_WALL_HUGGING : "Two-scan width planned"
    MODE_WALL_HUGGING --> MODE_REENTRY_ADVANCE : "Tape detected on return"
    MODE_REENTRY_ADVANCE --> MODE_REENTRY_PIVOT : "Centered over tape"
    MODE_REENTRY_PIVOT --> MODE_LINE_FOLLOWING : "Aligned with track"

    MODE_LINE_FOLLOWING --> MODE_SAFE_STOP : "IMU_MSG_LIFTED_OVERTURNED or HW fault"
    MODE_OBSTACLE_ASSESS --> MODE_SAFE_STOP : "Bypass abort / fault"
    MODE_WALL_HUGGING --> MODE_SAFE_STOP : "Bypass abort / fault"
    MODE_SAFE_STOP --> MODE_LINE_FOLLOWING : "Upright recovery confirmed"
```

---

## 2. Rectangular Obstacle Bypass Sub-FSM (`navigator_task` + `bypass_helpers`)

```mermaid
stateDiagram-v2
    [*] --> BYPASS_IDLE
    BYPASS_IDLE --> BYPASS_SCAN_BACK : "First 7-ray scan stored (reverse 20 mm)"
    BYPASS_SCAN_BACK --> BYPASS_SCAN_SECOND : "MSG_MOTOR_MOVE_DONE"
    BYPASS_SCAN_SECOND --> BYPASS_SCAN_RETURN : "Second scan stored (advance 20 mm)"
    BYPASS_SCAN_RETURN --> BYPASS_WAIT_RELEASE : "Side & shift planned"
    BYPASS_WAIT_RELEASE --> BYPASS_TURN_OUT : "E-stop released"
    BYPASS_TURN_OUT --> BYPASS_SHIFT_OUT : "90 deg outward pivot done"
    BYPASS_SHIFT_OUT --> BYPASS_TURN_PARALLEL : "Lateral shift done"
    BYPASS_TURN_PARALLEL --> BYPASS_WIDTH_CHECK : "Parallel alignment done"
    BYPASS_WIDTH_CHECK --> BYPASS_ADVANCE : "Front probe clear (x2)"
    BYPASS_WIDTH_CHECK --> BYPASS_WIDTH_TURN_OUT : "Obstacle still ahead"
    BYPASS_WIDTH_TURN_OUT --> BYPASS_WIDTH_EXTEND : "Outward pivot done"
    BYPASS_WIDTH_EXTEND --> BYPASS_WIDTH_ALIGN : "Extra 150 mm shift done"
    BYPASS_WIDTH_ALIGN --> BYPASS_WIDTH_CHECK : "Re-aligned parallel"
    BYPASS_ADVANCE --> BYPASS_TURN_IN : "Side sonar clear past depth"
    BYPASS_TURN_IN --> BYPASS_RETURN_CHECK : "90 deg inward pivot done"
    BYPASS_RETURN_CHECK --> BYPASS_FIND_LINE : "Return path clear"
    BYPASS_RETURN_CHECK --> BYPASS_DEPTH_TURN_BACK : "Obstacle blocks return"
    BYPASS_DEPTH_TURN_BACK --> BYPASS_ADVANCE : "Re-aligned parallel"
    BYPASS_FIND_LINE --> BYPASS_LINE_FOUND : "Black tape confirmed"
    BYPASS_LINE_FOUND --> BYPASS_IDLE : "Re-entry handoff"
    BYPASS_WAIT_RELEASE --> BYPASS_ABORT : "Timeout / fault"
    BYPASS_WIDTH_CHECK --> BYPASS_ABORT : "Max side limit / fault"
    BYPASS_ADVANCE --> BYPASS_ABORT : "Max forward limit / fault"
```

---

## 3. IMU Calibration, Hump & Lift/Overturn Safety FSM (`imu_task`)

```mermaid
stateDiagram-v2
    [*] --> IMU_STATE_INITIALIZE
    IMU_STATE_INITIALIZE --> IMU_STATE_CALIBRATING : "LSM303DLHC I2C configured"
    IMU_STATE_CALIBRATING --> IMU_STATE_MONITORING : "50 level stationary samples averaged"
    IMU_STATE_CALIBRATING --> IMU_STATE_INITIALIZE : "Read fault / timeout"
    IMU_STATE_MONITORING --> IMU_STATE_LIFTED_OVERTURNED : "Confirmed tilt, inversion, low-g, or high-g"
    IMU_STATE_LIFTED_OVERTURNED --> IMU_STATE_MONITORING : "500 ms upright quasi-static recovery"
```

---

## 4. Closed-Loop Motor Control FSM (`motor_task`)

```mermaid
stateDiagram-v2
    [*] --> MOTOR_STATE_IDLE
    MOTOR_STATE_IDLE --> MOTOR_STATE_ACCEL : "Non-zero cruise command"
    MOTOR_STATE_IDLE --> MOTOR_STATE_TURN : "Bounded tick command"
    MOTOR_STATE_ACCEL --> MOTOR_STATE_CRUISE : "Ramp reached target speed"
    MOTOR_STATE_ACCEL --> MOTOR_STATE_BRAKE : "Zero-speed stop command"
    MOTOR_STATE_CRUISE --> MOTOR_STATE_BRAKE : "Zero-speed stop command"
    MOTOR_STATE_CRUISE --> MOTOR_STATE_TURN : "Bounded tick command"
    MOTOR_STATE_TURN --> MOTOR_STATE_IDLE : "Goal ticks settled"
    MOTOR_STATE_BRAKE --> MOTOR_STATE_IDLE : "Zero PWM & encoders settled"
    MOTOR_STATE_ACCEL --> MOTOR_STATE_FAULT : "75-period wheel stall"
    MOTOR_STATE_CRUISE --> MOTOR_STATE_FAULT : "75-period wheel stall"
    MOTOR_STATE_TURN --> MOTOR_STATE_FAULT : "75-period wheel stall"
    MOTOR_STATE_FAULT --> MOTOR_STATE_IDLE : "Explicit zero-speed reset"
    MOTOR_STATE_IDLE --> MOTOR_STATE_ESTOP : "ROBOT_ESTOP_REQUEST"
    MOTOR_STATE_ACCEL --> MOTOR_STATE_ESTOP : "ROBOT_ESTOP_REQUEST"
    MOTOR_STATE_CRUISE --> MOTOR_STATE_ESTOP : "ROBOT_ESTOP_REQUEST"
    MOTOR_STATE_TURN --> MOTOR_STATE_ESTOP : "ROBOT_ESTOP_REQUEST"
    MOTOR_STATE_BRAKE --> MOTOR_STATE_ESTOP : "ROBOT_ESTOP_REQUEST"
    MOTOR_STATE_ESTOP --> MOTOR_STATE_IDLE : "ROBOT_ESTOP_REQUEST cleared"
```

---

## 5. Ultrasonic Detection, Profiling & Side-Tracking Workflow (`ultrasonic_task`)

```mermaid
flowchart TD
    A["Init GP5 TRIG, GP4 ECHO, GP15 50 Hz Servo PWM"] --> B["Center Ping Loop (60 ms)"]
    B --> C{"Distance <= 20 cm for 2 consecutive pings?"}
    C -- "No" --> B
    C -- "Yes" --> D["Assert ROBOT_ESTOP_REQUEST & send MSG_OBSTACLE_DETECTED"]
    D --> E["Wait for SONAR_SCAN_REQUEST event flag"]
    E --> F["Coarse 7-ray sweep (-45..+45 deg) + edge bisection"]
    F --> G["Send MSG_OBSTACLE_PROFILED to Navigator"]
    G --> H{"Wait on bypass event flags"}
    H -- "SONAR_SCAN_REQUEST" --> F
    H -- "SONAR_PROBE_FRONT / LEFT / RIGHT" --> I["3-sample stationary probe -> MSG_SONAR_PROBED"]
    I --> H
    H -- "SONAR_TRACK_LEFT / RIGHT" --> J["Side-tracking pings -> MSG_TRACKING_OBSTACLE"]
    J --> H
    H -- "SONAR_RESUME_FLAG" --> K["Re-center servo & clear confirmations"]
    K --> B
```
