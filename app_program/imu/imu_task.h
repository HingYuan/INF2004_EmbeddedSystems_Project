/**
 * @file imu_task.h
 * @brief LSM303DLHC accelerometer and magnetometer monitoring task interface.
 *
 * NOTE: Configured for Cytron Robo Pico Grove 1 (GP0/GP1) on RP2040 I2C0.
 * The uT-Kernel driver registers I2C0 as "iica" on GP8/GP9 at boot;
 * imu_task_run_task re-routes I2C0 to GP0/GP1. GP0/GP1 share UART0 console
 * pins, so build with CONSOLE=usb_cdc (ultrasonic uses GP4/GP5).
 *
 * Reference: STMicroelectronics LSM303DLHC Datasheet (DocID018771 Rev 2).
 *
 * BARR-C Deviation (Rules 5.2.a, 5.4.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel task entry signature mandates INT stacd; 50 Hz IMU
 *   tilt and hump thresholds retain bounded single-precision float32_t math.
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stddef.h>

#include <tk/tkernel.h>

#include "../robot_tasks.h"

#define IMU_I2C_DEVNAME                 "iica"
#define IMU_SDA_PIN                     (0u)
#define IMU_SCL_PIN                     (1u)

#define IMU_LSM303_ACCEL_ADDR           (0x19u)
#define IMU_LSM303_MAG_ADDR             (0x1Eu)

/* Detection thresholds and confirmation windows. */
#define IMU_OVERTURN_TILT_DEG           (25.0f)
#define IMU_LIFT_ACCEL_DEV_G            (1.0f)
#define IMU_LOW_G_RATIO                 (0.25f)
#define IMU_STATIC_DEV_G                (0.15f)
#define IMU_SAMPLE_PERIOD_MSEC          (20u)
#define IMU_HUMP_CONFIRM_USEC           (80000u)
#define IMU_TILT_CONFIRM_USEC           (200000u)
#define IMU_LIFT_CONFIRM_USEC           (80000u)
#define IMU_RECOVERY_USEC               (500000u)
#define IMU_TELEMETRY_USEC              (500000u)
#define IMU_SAMPLE_GAP_USEC             (100000u)
#define IMU_READ_FAULT_USEC             (500000u)
#define IMU_HUMP_TILT_THRESH_DEG        (5.5f)
#define IMU_HUMP_TRACK_WIDTH_MM         (68.0f)
#define IMU_HUMP_CHASSIS_SCALE_MM       (60.0f)

/* Keep state numbers stable for Navigator and network telemetry. */
typedef enum
{
    IMU_STATE_INITIALIZE        = 0,
    IMU_STATE_CALIBRATING       = 1,
    IMU_STATE_MONITORING        = 2,
    IMU_STATE_LIFTED_OVERTURNED = 3
} imu_task_state_t;

typedef enum
{
    IMU_STOP_NONE     = 0,
    IMU_STOP_TILT     = 1,
    IMU_STOP_INVERTED = 2,
    IMU_STOP_LOW_G    = 3,
    IMU_STOP_HIGH_G   = 4
} imu_task_stop_reason_t;

/**
 * @brief IMU acquisition, calibration, hump, and safety monitoring task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void imu_task_run_task(INT stacd, void * p_exinf);

#endif /* IMU_TASK_H */

/*** end of file ***/
