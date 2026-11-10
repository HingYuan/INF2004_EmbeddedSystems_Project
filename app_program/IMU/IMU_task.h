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
#define SAFE_TILT_LIMIT_DEG     30.0f   // >30 deg triggers Tilt Warning
#define SHOCK_THRESHOLD_G       2.5f    // >2.5g total acceleration triggers E-Stop
#define HUMP_THRESHOLD_G        0.35f   // Z-axis deviation threshold for hump detection
#define HUMP_TILT_THRESHOLD_DEG 7.0f    // Minimum tilt (deg) to trigger hump detection (single-wheel ramp)
#define HUMP_TRACK_WIDTH_MM     100.0f  // Track width between left and right wheels (mm)
#define HUMP_CHASSIS_SCALE_MM   150.0f  // Effective chassis wheelbase length (mm)

/* 
=========================================
|          PUBLIC INTERFACES            |
=========================================
*/
// Task entry function for micro T-Kernel
IMPORT void imu_task(INT stacd, void *exinf);

#endif