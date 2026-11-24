/**
 * @file demo_tasks.h
 * @brief Console-independent LED liveness and USB bring-up indicator task.
 */

#ifndef DEMO_TASKS_H
#define DEMO_TASKS_H

#include <tk/tkernel.h>

/**
 * @brief Blink the board LED at 2 Hz when USB is up, or blink a status code.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void demo_tasks_blink_task (INT stacd, void * p_exinf);

#endif /* DEMO_TASKS_H */

/*** end of file ***/
