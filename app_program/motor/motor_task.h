/**
 * @file motor_task.h
 * @brief Dual-wheel PWM motor control, quadrature encoder, and odometry API.
 *
 * BARR-C Deviation (Rules 1.1.c, 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Packed 4-byte motor command union fits a single 32-bit RTOS
 *   message-buffer word; uT-Kernel APIs require ER and INT return/param types.
 */

#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../robot_tasks.h"

/* DC motor H-bridge PWM GPIO pins. */
#define MOTOR_LEFT_A                    (8u)
#define MOTOR_LEFT_B                    (9u)
#define MOTOR_RIGHT_A                   (10u)
#define MOTOR_RIGHT_B                   (11u)

/* Optical wheel encoder GPIO pins. */
#define MOTOR_LEFT_ENC_A                (16u)
#define MOTOR_LEFT_ENC_B                (17u)
#define MOTOR_RIGHT_ENC_A               (2u)
#define MOTOR_RIGHT_ENC_B               (3u)

/* Motor fault bit positions within the shared application flag word. */
#define MOTOR_FAULT_BIT_STALL           (1u << 0u)
#define MOTOR_FAULT_BIT_ENCODER         (1u << 1u)
#define MOTOR_FAULT_BIT_ESTOP           (1u << 2u)

/*
 * WARNING: Calibrated odometry constants (749 ticks = 247.5 mm on ruler,
 * giving 0.33 mm per tick). Recalibrate if wheel diameter or gears change.
 */
#define MOTOR_WHEEL_DIAMETER_MM         (60)
#define MOTOR_ENCODER_SLOTS             (20)
#define MOTOR_MM_PER_TICK_X100          (33)
#define MOTOR_MM_PER_TICK_X10           (3)
#define MOTOR_WHEELBASE_MM              (100)

typedef struct ROBOT_TASKS_PACKED
{
    int8_t  target_speed_left;
    int8_t  target_speed_right;
    uint8_t turn_ticks_left;
    uint8_t turn_ticks_right;
} motor_task_cmd_fields_t;

typedef union
{
    uint32_t                raw;
    motor_task_cmd_fields_t fields;
} motor_task_cmd_t;

typedef enum
{
    MOTOR_STATE_IDLE   = 0,
    MOTOR_STATE_ACCEL  = 1,
    MOTOR_STATE_CRUISE = 2,
    MOTOR_STATE_TURN   = 3,
    MOTOR_STATE_BRAKE  = 4,
    MOTOR_STATE_FAULT  = 5,
    MOTOR_STATE_ESTOP  = 6
} motor_task_state_t;

/**
 * @brief Clamp a signed 32-bit value into [low_limit, high_limit].
 * @param[in] value      Input value.
 * @param[in] low_limit  Minimum permitted value.
 * @param[in] high_limit Maximum permitted value.
 * @return Clamped integer value.
 */
int32_t motor_task_clamp(int32_t value, int32_t low_limit, int32_t high_limit);

/**
 * @brief Return the motor stall, encoder, and E-stop fault bitmask.
 * @return Bitmask of MOTOR_FAULT_BIT_* flags currently asserted.
 */
uint32_t motor_task_get_fault_word(void);

/**
 * @brief Check whether an emergency stop request is active.
 * @return True if ROBOT_ESTOP_REQUEST is set, false otherwise.
 */
bool motor_task_is_estop(void);

/**
 * @brief Check whether Navigator requested an E-stop acknowledgement.
 * @return True if ROBOT_STOP_ACK_REQUEST is set, false otherwise.
 */
bool motor_task_is_stop_ack_req(void);

/**
 * @brief Check whether closed-loop encoder operation is degraded.
 * @return True if ROBOT_ENCODER_DEGRADED is set, false otherwise.
 */
bool motor_task_is_enc_degraded(void);

/**
 * @brief Read lifetime signed left encoder tick counter.
 * @return Left encoder ticks.
 */
int32_t motor_task_get_enc_left(void);

/**
 * @brief Read lifetime signed right encoder tick counter.
 * @return Right encoder ticks.
 */
int32_t motor_task_get_enc_right(void);

/**
 * @brief Read count of overwritten motor command messages.
 * @return Message buffer drop counter.
 */
uint32_t motor_task_get_drop_count(void);

/**
 * @brief Compute left wheel odometry distance in millimetres.
 * @return Left wheel distance in mm.
 */
int32_t motor_task_get_left_dist_mm(void);

/**
 * @brief Compute right wheel odometry distance in millimetres.
 * @return Right wheel distance in mm.
 */
int32_t motor_task_get_right_dist_mm(void);

/**
 * @brief Compute average vehicle odometry distance in millimetres.
 * @return Average distance in mm.
 */
int32_t motor_task_get_avg_dist_mm(void);

/**
 * @brief Compute rounded average vehicle odometry distance in centimetres.
 * @return Average distance in cm.
 */
int32_t motor_task_get_avg_dist_cm(void);

/**
 * @brief Read latest estimated linear vehicle speed in mm per second.
 * @return Estimated speed in mm/second.
 */
int32_t motor_task_get_speed(void);

/**
 * @brief Queue a bounded straight forward translation command.
 * @param[in] dist_mm Target forward distance in millimetres.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_move_forward(int32_t dist_mm);

/**
 * @brief Queue a bounded straight backward translation command.
 * @param[in] dist_mm Target backward distance in millimetres.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_move_backward(int32_t dist_mm);

/**
 * @brief Queue a bounded in-place left pivot command.
 * @param[in] angle_deg Pivot angle in degrees.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_turn_left(int32_t angle_deg);

/**
 * @brief Queue a bounded in-place right pivot command.
 * @param[in] angle_deg Pivot angle in degrees.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_turn_right(int32_t angle_deg);

/**
 * @brief Queue an immediate zero-speed stop command.
 * @return E_OK on success, or negative uT-Kernel error code.
 */
ER motor_task_stop(void);

/**
 * @brief Main 50 Hz closed-loop motor control and fault monitoring task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void motor_task_run_task(INT stacd, void * p_exinf);

#endif /* MOTOR_TASK_H */

/*** end of file ***/
