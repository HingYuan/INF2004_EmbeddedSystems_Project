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

/* Detection Thresholds */
#define OVERTURN_TILT_DEG       40.0f   // >40 deg pitch/roll or Z-axis inversion triggers immediate stop
#define LIFT_ACCEL_DEV_G        0.22f   // Sustained |total_g - baseline_g| deviation triggers lift stop
#define HUMP_TILT_THRESHOLD_DEG 5.5f    // Minimum tilt (deg) to trigger hump detection (single-wheel ramp)
#define HUMP_TRACK_WIDTH_MM     68.0f   // Track width between left and right wheels (mm)
#define HUMP_CHASSIS_SCALE_MM   60.0f   // Effective chassis wheelbase length (mm)

/* 
=========================================
|          PUBLIC INTERFACES            |
=========================================
*/
// Task entry function for micro T-Kernel
IMPORT void imu_task(INT stacd, void *exinf);

#endif