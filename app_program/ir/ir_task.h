/**
 * @file ir_task.h
 * @brief Line-following IR sensor and Code 39 barcode reader task interfaces.
 *
 * BARR-C Deviation (Rules 1.1.c, 5.2.a, 5.6.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Packed 32-bit bar record conserves message-buffer SRAM using
 *   unsigned bit-fields (Rule 5.3.a); uT-Kernel task signatures require INT.
 */

#ifndef IR_TASK_H
#define IR_TASK_H

#include <stdint.h>

#include <tk/tkernel.h>

#include "../robot_tasks.h"

#define IR_LEFT_PIN                     (6u)
#define IR_CENTER_PIN                   (28u)
#define IR_RIGHT_PIN                    (27u)
#define IR_ADC_THRESHOLD_MV             (500u)

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t b_is_high : 1;
    uint32_t duration  : 26;
} ir_task_bar_record_t;

/**
 * @brief Optional sensor calibration task printing live IR GPIO and ADC data.
 * @param[in] stacd   Packed GPIO pin indices (bits 16..23 left, 8..15 right).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_zeroing_task(INT stacd, void * p_exinf);

/**
 * @brief Sample center and right line-following ADC channels at 100 Hz.
 * @param[in] stacd   Packed GPIO pin indices (bits 8..15 left, 0..7 right).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_sensor_task(INT stacd, void * p_exinf);

/**
 * @brief Monitor barcode IR GPIO transitions and push timed bar records.
 * @param[in] stacd   GPIO pin number of the barcode IR receiver.
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_barcode_task(INT stacd, void * p_exinf);

/**
 * @brief Decode queued Code 39 bar records and forward characters to Navigator.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_decoder_task(INT stacd, void * p_exinf);

#endif /* IR_TASK_H */

/*** end of file ***/
