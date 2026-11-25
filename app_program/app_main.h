/**
 * @file app_main.h
 * @brief Application entry point declaration for micro T-Kernel 3.0.
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <tk/tkernel.h>

/**
 * @brief Application entry point invoked by the micro T-Kernel init task.
 *
 * BARR-C Deviation (Rules 6.1.g, 6.1.i):
 *   Approver: Project Team (INF2004).
 *   Rationale: The symbol name usermain is fixed by micro T-Kernel 3.0
 *   (kernel/inittask/inittask.c).
 *
 * @return Zero on normal exit (never returns), or 1 on startup failure.
 */
INT usermain (void);

#endif /* APP_MAIN_H */

/*** end of file ***/
