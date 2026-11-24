/**
 * @file free_time_diagnostic.c
 * @brief Background CPU idle-time load measurement task implementation.
 *
 * WARNING: FREE_TIME_MAX_IDLE_COUNTS is empirically calibrated for RP2040 at
 * 125 MHz with -O2 optimization. Changing system clock or compiler flags
 * invalidates the idle percentage baseline.
 */

#include "free_time_diagnostic.h"

#include <stdint.h>

#include <tm/tmonitor.h>

#include "../robot_tasks.h"

#define FREE_TIME_MAX_IDLE_COUNTS       (7343000u)
#define FREE_TIME_WINDOW_USEC           (1000000u)
#define FREE_TIME_MAX_PERCENT           (100u)

/**
 * @brief Measure background idle iterations per second and print CPU load.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void free_time_diagnostic_task (INT stacd, void * p_exinf)
{
    uint32_t start_usec = robot_tasks_time_usec();
    uint32_t idle_cnt   = 0u;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        uint32_t now_usec = 0u;

        idle_cnt++;
        now_usec = robot_tasks_time_usec();

        if ((now_usec - start_usec) >= FREE_TIME_WINDOW_USEC)
        {
            uint32_t idle_pct =
                (idle_cnt * FREE_TIME_MAX_PERCENT) / FREE_TIME_MAX_IDLE_COUNTS;
            uint32_t cpu_load = 0u;

            if (idle_pct > FREE_TIME_MAX_PERCENT)
            {
                idle_pct = FREE_TIME_MAX_PERCENT;
            }

            cpu_load = FREE_TIME_MAX_PERCENT - idle_pct;

            /* Cast required by uT-Kernel tm_printf(const UB *, ...) API. */
            tm_printf((UB *)"\n[Diagnostics] CPU Load: %u%% (Raw: %u)\n",
                      (UINT)cpu_load,
                      (UINT)idle_cnt);

            idle_cnt   = 0u;
            start_usec = now_usec;
        }
    }
}

/*** end of file ***/
