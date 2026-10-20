#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "free_time_baseline.h"

volatile UW calib_counts = 0;
volatile BOOL calib_running = TRUE;

// The Counter: Runs at the lowest priority, counting endlessly
EXPORT void calib_counter_task(INT stacd, void *exinf) 
{
    (void)stacd;
    (void)exinf;

    while(calib_running) {
        calib_counts++;
    }
    
    // Exit task cleanly when stopped
    tk_ext_tsk();
}

// The Timer: Runs at a high priority, sleeps for 1s, then prints
EXPORT void calib_timer_task(INT stacd, void *exinf) 
{
    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"\n[Calibrating] Counting for exactly 1 second...\n");

    // Sleep for exactly 1000 ticks (1 second)
    tk_dly_tsk(1000);

    // Stop the counter task
    calib_running = FALSE;

    // Print the final calibrated number
    tm_printf((UB *)"\n=== CALIBRATION COMPLETE ===\n");
    tm_printf((UB *)"MAX_IDLE_COUNTS_PER_SEC %u\n", calib_counts);
    tm_printf((UB *)"============================\n\n");

    // Exit task cleanly
    tk_ext_tsk();
}