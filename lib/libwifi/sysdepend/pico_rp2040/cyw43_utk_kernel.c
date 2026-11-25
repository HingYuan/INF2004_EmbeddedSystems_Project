/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <tk/tkernel.h>

IMPORT void cyw43_utk_task(INT stacd, void *exinf);

/* Keep radio polling on physical RP2040 core 1.  USB CDC owns physical core 0
 * at priority 3; the network service is independent and remains above the
 * qualification task on its assigned core. */
#define CYW43_TASK_PRIORITY  4
#define CYW43_TASK_STACK     (6 * 1024)

EXPORT ER cyw43_utk_start(void)
{
    T_CTSK ctsk = {0};
    ID task;

#if TK_SUPPORT_SMP
    /*
     * Pin to processor 2 (physical RP2040 core 1): the radio owns PIO, DMA,
     * and its peripheral ownership path on that core.
     */
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
#else
    ctsk.tskatr = TA_HLNG | TA_RNG1;
#endif
	ctsk.task = (FP)cyw43_utk_task;
    ctsk.itskpri = CYW43_TASK_PRIORITY;
    ctsk.stksz = CYW43_TASK_STACK;
#if TK_SUPPORT_SMP
    ctsk.assprc = TP_PRC2;
#endif
#if USE_OBJECT_NAME
    ctsk.dsname = (UB*)"cyw43";
#endif

    task = tk_cre_tsk(&ctsk);
    if(task < E_OK) return task;
    return tk_sta_tsk(task, 0);
}
