/**
 * @file free_time_diagnostic.h
 * @brief Background CPU idle-time load measurement task interface.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel task entry signature mandates INT stacd parameter.
 */

#ifndef FREE_TIME_DIAGNOSTIC_H
#define FREE_TIME_DIAGNOSTIC_H

#include <tk/tkernel.h>

/**
 * @brief Measure background idle iterations per second and print CPU load.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void free_time_diagnostic_run_task(INT stacd, void * p_exinf);

#endif /* FREE_TIME_DIAGNOSTIC_H */

/*** end of file ***/
