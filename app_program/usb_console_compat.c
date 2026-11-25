/**
 * @file usb_console_compat.c
 * @brief Console-state accessor implementations for USB-CDC and UART builds.
 *
 * BARR-C Deviation (Rules 4.3.b, 5.2.a, 7.1.j):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Legacy driver lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c
 *   exports link symbols tm_usb_state, tm_usb_diag_level, tm_usb_pending, and
 *   tm_usb_dropped using uT-Kernel UW/INT types without a companion .h file.
 */

#include "usb_console_compat.h"

#include <tk/tkernel.h>

#define USB_UART_ENUMERATED_STATE       (3u)
#define USB_UART_DIAG_READY_LEVEL       (8)

#if TM_CONSOLE_USB_CDC
extern volatile UW tm_usb_dropped;

extern UW tm_usb_state(void);
extern INT tm_usb_diag_level(void);
extern UW tm_usb_pending(void);
#endif

/**
 * @brief Read current USB-CDC link enumeration state.
 * @return Value >= 2 once enumerated (constant 3 on UART builds).
 */
uint32_t usb_console_compat_state (void)
{
    uint32_t state_val = USB_UART_ENUMERATED_STATE;

#if TM_CONSOLE_USB_CDC
    /* Range-safe: UW is an unsigned 32-bit integer on RP2040. */
    state_val = (uint32_t)tm_usb_state();
#endif

    return state_val;
}

/**
 * @brief Read USB-CDC bring-up diagnostic level (0..8).
 * @return Current bring-up stage (constant 8 on UART builds).
 */
int32_t usb_console_compat_diag_lvl (void)
{
    int32_t diag_lvl = USB_UART_DIAG_READY_LEVEL;

#if TM_CONSOLE_USB_CDC
    /* Range-safe: tm_usb_diag_level returns 0..8, fitting int32_t. */
    diag_lvl = (int32_t)tm_usb_diag_level();
#endif

    return diag_lvl;
}

/**
 * @brief Read number of bytes currently queued in the console ring.
 * @return Queued byte count (0 on UART builds).
 */
uint32_t usb_console_compat_pending (void)
{
    uint32_t pending_cnt = 0u;

#if TM_CONSOLE_USB_CDC
    /* Range-safe: UW is an unsigned 32-bit integer on RP2040. */
    pending_cnt = (uint32_t)tm_usb_pending();
#endif

    return pending_cnt;
}

/**
 * @brief Read number of console bytes lost to ring overrun.
 * @return Dropped byte count (0 on UART builds).
 */
uint32_t usb_console_compat_dropped (void)
{
    uint32_t dropped_cnt = 0u;

#if TM_CONSOLE_USB_CDC
    /* Range-safe: UW is an unsigned 32-bit integer on RP2040. */
    dropped_cnt = (uint32_t)tm_usb_dropped;
#endif

    return dropped_cnt;
}

/*** end of file ***/
