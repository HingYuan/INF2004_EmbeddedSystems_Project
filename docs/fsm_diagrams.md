# Finite State Machine Diagrams (BARR-C:2018 Rule 2.2.f)

Version-controlled state machine specifications referenced by the firmware
modules under `app_program/`.

## 1. Navigator Top-Level FSM (`app_program/navigator/navigator_task.c`)

```mermaid
stateDiagram-v2
    [*] --> MODE_WIFI_SEND_COMMS
    MODE_WIFI_SEND_COMMS --> MODE_LINE_FOLLOWING : 5 mappings stored
    MODE_LINE_FOLLOWING --> MODE_DECISION_MAKING_ADVANCE : IR_STATE_B_B or reentry hit
    MODE_DECISION_MAKING_ADVANCE --> MODE_DECISION_MAKING_TURN : Advance 900 ms done
    MODE_DECISION_MAKING_TURN --> MODE_LINE_FOLLOWING : Turn rotations <= 0 or DIR_FRONT
    MODE_LINE_FOLLOWING --> MODE_GOING_OVER_HUMP : IMU_MSG_HUMP_DETECTED
    MODE_GOING_OVER_HUMP --> MODE_LINE_FOLLOWING : IMU_MSG_NO_HUMP
    MODE_LINE_FOLLOWING --> MODE_OBSTACLE_ASSESS : MSG_OBSTACLE_DETECTED
    MODE_OBSTACLE_ASSESS --> MODE_WALL_HUGGING : 2 scans + restore complete
    MODE_WALL_HUGGING --> MODE_LINE_FOLLOWING : 2x black IR (reentry armed)
    MODE_OBSTACLE_ASSESS --> MODE_SAFE_STOP : Bypass fault or timeout
    MODE_WALL_HUGGING --> MODE_SAFE_STOP : Bypass fault or timeout
    MODE_LINE_FOLLOWING --> MODE_SAFE_STOP : IMU_MSG_LIFTED_OVERTURNED
```

## 2. Rectangular Obstacle Bypass FSM (`app_program/ultrasonic/bypass_helpers.c`)

```mermaid
stateDiagram-v2
    [*] --> BYPASS_IDLE
    BYPASS_IDLE --> BYPASS_WAIT_RELEASE : 1st scan valid
    BYPASS_WAIT_RELEASE --> BYPASS_SCAN_BACK : E-stop released
    BYPASS_SCAN_BACK --> BYPASS_SCAN_SECOND : Reverse 20 mm done
    BYPASS_SCAN_SECOND --> BYPASS_SCAN_RETURN : 2nd scan + side planned
    BYPASS_SCAN_RETURN --> BYPASS_TURN_OUT : Restore 20 mm done
    BYPASS_TURN_OUT --> BYPASS_SHIFT_OUT : Turn 90 deg done
    BYPASS_SHIFT_OUT --> BYPASS_TURN_PARALLEL : Side step done
    BYPASS_TURN_PARALLEL --> BYPASS_WIDTH_CHECK : Parallel turn done
    BYPASS_WIDTH_CHECK --> BYPASS_WIDTH_TURN_OUT : 2x probe result
    BYPASS_WIDTH_TURN_OUT --> BYPASS_WIDTH_EXTEND : Turn out done
    BYPASS_WIDTH_EXTEND --> BYPASS_WIDTH_ALIGN : Extend 70 or 150 mm done
    BYPASS_WIDTH_ALIGN --> BYPASS_WIDTH_CHECK : Still blocked
    BYPASS_WIDTH_ALIGN --> BYPASS_ADVANCE : Width clearance done
    BYPASS_ADVANCE --> BYPASS_TURN_IN : Forward >= front + pivot
    BYPASS_TURN_IN --> BYPASS_RETURN_CHECK : First inward turn
    BYPASS_RETURN_CHECK --> BYPASS_DEPTH_TURN_BACK : 2x probe result
    BYPASS_DEPTH_TURN_BACK --> BYPASS_ADVANCE : Step 70 or 150 mm
    BYPASS_TURN_IN --> BYPASS_FIND_LINE : Depth clearance done
    BYPASS_FIND_LINE --> BYPASS_LINE_FOUND : 2 consecutive black IR
    BYPASS_FIND_LINE --> BYPASS_ABORT : Search limit reached
```

## 3. Motor Controller FSM (`app_program/motor/motor_task.c`)

```mermaid
stateDiagram-v2
    [*] --> MOTOR_STATE_IDLE
    MOTOR_STATE_IDLE --> MOTOR_STATE_ACCEL : Continuous speed cmd
    MOTOR_STATE_IDLE --> MOTOR_STATE_TURN : Bounded tick cmd
    MOTOR_STATE_ACCEL --> MOTOR_STATE_CRUISE : Ramp reached target
    MOTOR_STATE_ACCEL --> MOTOR_STATE_TURN : Bounded cmd received
    MOTOR_STATE_ACCEL --> MOTOR_STATE_BRAKE : Zero speed cmd
    MOTOR_STATE_CRUISE --> MOTOR_STATE_TURN : Bounded cmd received
    MOTOR_STATE_CRUISE --> MOTOR_STATE_BRAKE : Zero speed cmd
    MOTOR_STATE_CRUISE --> MOTOR_STATE_FAULT : Dual-wheel stall
    MOTOR_STATE_TURN --> MOTOR_STATE_BRAKE : Tick target met or zero cmd
    MOTOR_STATE_TURN --> MOTOR_STATE_FAULT : Dual-wheel stall
    MOTOR_STATE_BRAKE --> MOTOR_STATE_IDLE : PWM ramped to 0
    MOTOR_STATE_BRAKE --> MOTOR_STATE_ACCEL : New non-zero speed cmd
    MOTOR_STATE_BRAKE --> MOTOR_STATE_TURN : New bounded cmd
    MOTOR_STATE_FAULT --> MOTOR_STATE_IDLE : Zero speed cmd clears stall
    MOTOR_STATE_IDLE --> MOTOR_STATE_ESTOP : ROBOT_ESTOP_REQUEST
    MOTOR_STATE_ACCEL --> MOTOR_STATE_ESTOP : ROBOT_ESTOP_REQUEST
    MOTOR_STATE_CRUISE --> MOTOR_STATE_ESTOP : ROBOT_ESTOP_REQUEST
    MOTOR_STATE_TURN --> MOTOR_STATE_ESTOP : ROBOT_ESTOP_REQUEST
    MOTOR_STATE_BRAKE --> MOTOR_STATE_ESTOP : ROBOT_ESTOP_REQUEST
    MOTOR_STATE_ESTOP --> MOTOR_STATE_IDLE : E-stop cleared
```

## 4. IMU Monitoring FSM (`app_program/imu/imu_task.c`)

```mermaid
stateDiagram-v2
    [*] --> IMU_STATE_INITIALIZE
    IMU_STATE_INITIALIZE --> IMU_STATE_CALIBRATING : I2C open & LSM303 configured
    IMU_STATE_CALIBRATING --> IMU_STATE_MONITORING : 50 consecutive flat samples
    IMU_STATE_MONITORING --> IMU_STATE_LIFTED_OVERTURNED : Tilt, invert, low-g, or high-g confirmed
    IMU_STATE_LIFTED_OVERTURNED --> IMU_STATE_MONITORING : Upright & flat for 500 ms
```
