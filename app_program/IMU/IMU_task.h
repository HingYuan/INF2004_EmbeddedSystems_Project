#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>
#include <tk/tkernel.h>

/* 
=========================================
|       IMU HARDWARE CONFIGURATION      |
=========================================
*/
// Configured for Cytron Robo Pico Grove 1 (GP0/GP1) on RP2040 I2C0.
// The uT-Kernel driver only registers I2C0 as "iica" and routes it to GP8/GP9
// at boot; imu_task re-routes it to the pins below.
// Valid I2C0 pins: SDA = GP0/4/8/12/16/20, SCL = GP1/5/9/13/17/21.
// NOTE: GP0/GP1 are shared with the ultrasonic sensor and the UART console,
// so build with CONSOLE=usb_cdc and do not run ultrasonic_task at the same time.
#define IMU_I2C_DEVNAME       "iica"
#define IMU_SDA_PIN           0
#define IMU_SCL_PIN           1

#define LSM303_ACCEL_ADDR     0x19
#define LSM303_MAG_ADDR       0x1E

/* Detection Thresholds */
#define OVERTURN_TILT_DEG       25.0f   // Confirm sustained filtered tilt or actual Z inversion.
#define LIFT_ACCEL_DEV_G        1.0f    // Acceleration deviation from calibrated gravity.
#define IMU_LOW_G_RATIO         0.25f   // Keep free-fall detection even when deviation >= baseline g.
#define IMU_STATIC_DEV_G        0.15f   // Trust tilt only near the calibrated gravity magnitude.
#define IMU_SAMPLE_PERIOD_MS      20   // Sensor runs at 100 Hz; consume at most one sample per 20 ms.
#define IMU_HUMP_CONFIRM_US    80000U
#define IMU_TILT_CONFIRM_US   200000U
#define IMU_LIFT_CONFIRM_US    80000U
#define IMU_RECOVERY_US       500000U
#define IMU_TELEMETRY_US      500000U
#define IMU_SAMPLE_GAP_US     100000U  // A long gap cannot count as continuous confirmation.
#define IMU_READ_FAULT_US     500000U  // Persistent failed reads stop with a separate diagnostic.
#define HUMP_TILT_THRESHOLD_DEG 5.5f    // Minimum filtered tilt (deg) to trigger hump detection (single-wheel ramp)
#define HUMP_TRACK_WIDTH_MM     68.0f   // Track width between left and right wheels (mm)
#define HUMP_CHASSIS_SCALE_MM   60.0f   // Effective chassis wheelbase length (mm)

/* Keep state numbers stable for Navigator/network telemetry. */
typedef enum {
    STATE_INITIALIZE,
    STATE_CALIBRATING,
    STATE_MONITORING,
    STATE_LIFTED_OVERTURNED
} IMU_FSM_STATE;

typedef enum {
    IMU_STOP_NONE,
    IMU_STOP_TILT,
    IMU_STOP_INVERTED,
    IMU_STOP_LOW_G,
    IMU_STOP_HIGH_G
} IMU_STOP_REASON;

/* Continuous elapsed-time confirmation, independent of task scheduling. */
typedef struct {
    UW since_us;
    UW last_us;
    BOOL active;
} IMU_CONFIRM;

/* 
=========================================
|          PUBLIC INTERFACES            |
=========================================
*/
// Task entry function for micro T-Kernel
IMPORT void imu_task(INT stacd, void *exinf);

#endif