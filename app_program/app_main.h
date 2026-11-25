/**
 * @file app_main.h
 * @brief Application entry point declaration for micro T-Kernel 3.0.
 *
 * BARR-C Deviation (Rules 5.2.a, 6.1.g, 6.1.h, 6.1.i):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Symbol name usermain and return type INT are mandated by the
 *   micro T-Kernel 3.0 startup ABI (kernel/inittask/inittask.c).
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stddef.h>

#include <tk/tkernel.h>

/**
 * @brief Application entry point invoked by the micro T-Kernel init task.
 * @return Zero on normal exit (never returns), or 1 on startup failure.
 */
INT usermain(void);

#endif /* APP_MAIN_H */

/*** end of file ***/
