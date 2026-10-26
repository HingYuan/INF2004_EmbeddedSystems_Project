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
#define SAFE_TILT_LIMIT_DEG   30.0f   // >30 deg triggers Tilt Warning
#define SHOCK_THRESHOLD_G     2.5f    // >2.5g total acceleration triggers E-Stop
#define HUMP_THRESHOLD_G      0.35f   // Z-axis deviation threshold for hump detection

/* 
=========================================
|       MESSAGE TYPE DEFINITIONS        |
=========================================
*/
typedef enum {
    IMU_MSG_NONE,
    IMU_MSG_TILT_WARNING,
    IMU_MSG_SHOCK_DETECTED,
    IMU_MSG_HUMP_INTEGRATING
} IMU_EVENT_TYPE;

typedef struct {
    IMU_EVENT_TYPE event;
    float pitch_deg;
    float roll_deg;
    float peak_g;
    float estimated_height_cm;
} IMU_TELEMETRY_MSG;

/* 
=========================================
|          PUBLIC INTERFACES            |
=========================================
*/
// Task entry function for micro T-Kernel
IMPORT void imu_task(INT stacd, void *exinf);

#endif