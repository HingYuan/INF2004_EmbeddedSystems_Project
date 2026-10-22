#include <tk/tkernel.h>

// Hardware Base Addresses
#define XOSC_BASE           0x40024000
#define CLOCKS_BASE         0x40008000
#define WATCHDOG_TICK       0x4005802C

// Force this function to run from SRAM so we don't crash the Flash memory!
__attribute__((noinline, section(".data"))) void init_sys_clock_125mhz(void)
{
    // 1. Enable Crystal (XOSC - 12MHz)
    *(volatile UW *)(XOSC_BASE + 0x00) = 0xAA0;
    *(volatile UW *)(XOSC_BASE + 0x0C) = 47;
    *(volatile UW *)(XOSC_BASE + 0x00) = (0xAA0 | (0xFAB << 12));
    while (!(*(volatile UW *)(XOSC_BASE + 0x04) & (1u << 31)));

    // 2. Set Reference Clock (CLK_REF) to XOSC (12MHz) to fix timer accuracy
    *(volatile UW *)(CLOCKS_BASE + 0x30) = 0x2; 

    // 3. Fix the hardware timer tick to exactly 1 microsecond (12MHz / 12 = 1MHz)
    *(volatile UW *)(WATCHDOG_TICK) = (12 | (1 << 9)); 

    // 4. Configure System PLL for 125 MHz
    *(volatile UW *)(PLL_SYS_BASE + 0x08) = 1;
    *(volatile UW *)(PLL_SYS_BASE + 0x04) = (1 << 8) | 125;
    *(volatile UW *)(PLL_SYS_BASE + 0x08) &= ~(1u << 5 | 1u << 0);
    while (!(*(volatile UW *)(PLL_SYS_BASE + 0x00) & (1u << 31)));

    *(volatile UW *)(PLL_SYS_BASE + 0x0C) = (6 << 16) | (2 << 12);
    *(volatile UW *)(PLL_SYS_BASE + 0x08) &= ~(1u << 3);

    // 5. Switch System Clock (CLK_SYS) to the 125 MHz PLL safely from RAM
    *(volatile UW *)(CLOCKS_BASE + 0x3C) = 0x0; 
    while (!(*(volatile UW *)(CLOCKS_BASE + 0x40) & (1u << 0))); 
    *(volatile UW *)(CLOCKS_BASE + 0x3C) = 0x2; 
    while (!(*(volatile UW *)(CLOCKS_BASE + 0x40) & (1u << 2))); 
}