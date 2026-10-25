#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../robot_tasks.h"

#define MAX_IDLE_COUNTS_PER_SEC   7343000

volatile UW idle_counts = 0;

EXPORT void diagnostic_task(INT stacd, void *exinf) 
{
    (void)stacd;
    (void)exinf;

    UW start_time = time_us();
    UW counts = 0;

    while(1) {
        counts++;
        
        // Read the hardware timer
        UW current_time = time_us();
        
        // Check if 1,000,000 microseconds (1 second) have elapsed
        if (current_time - start_time >= 1000000) {
            
            // Calculate CPU Load
            INT idle_pct = (counts * 100) / MAX_IDLE_COUNTS_PER_SEC;
            if (idle_pct > 100) idle_pct = 100; 
            INT cpu_load = 100 - idle_pct;
            
            tm_printf((UB *)"\n[Diagnostics] CPU Load: %d%% (Raw: %u)\n", cpu_load, counts);
            
            // Reset the counter and timer for the next second
            counts = 0;
            start_time = current_time;
        }
    }
}