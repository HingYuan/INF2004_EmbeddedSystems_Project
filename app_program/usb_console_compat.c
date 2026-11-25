/**
 * @file usb_console_compat.c
 * @brief Console-state accessor implementations for USB-CDC and UART builds.
 */

#include "usb_console_compat.h"

#include <tk/tkernel.h>

#if TM_CONSOLE_USB_CDC
/* External symbols exported by lib/libtm/sysdepend/pico_rp2040/usb. */
extern UW          tm_usb_state (void);
extern INT         tm_usb_diag_level (void);
extern UW          tm_usb_pending (void);
extern volatile UW tm_usb_dropped;
#else
#define USB_UART_ENUMERATED_STATE       (3u)
#define USB_UART_DIAG_READY_LEVEL       (8)
#endif

/**
 * @brief Read current USB-CDC link enumeration state.
 * @return Value >= 2 once enumerated (constant 3 on UART builds).
 */
uint32_t usb_console_compat_state (void)
{
    uint32_t state_val = 0u;

#if TM_CONSOLE_USB_CDC
    /* Range-safe: UW is an unsigned 32-bit integer on RP2040. */
    state_val = (uint32_t)tm_usb_state();
#else
    state_val = USB_UART_ENUMERATED_STATE;
#endif

    return state_val;
}

/**
 * @brief Read USB-CDC bring-up diagnostic level (0..8).
 * @return Current bring-up stage (constant 8 on UART builds).
 */
int32_t usb_console_compat_diag_lvl (void)
{
    int32_t diag_lvl = 0;

#if TM_CONSOLE_USB_CDC
    /* Range-safe: tm_usb_diag_level returns 0..8. */
    diag_lvl = (int32_t)tm_usb_diag_level();
#else
    diag_lvl = USB_UART_DIAG_READY_LEVEL;
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
#else
    pending_cnt = 0u;
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
#else
    dropped_cnt = 0u;
#endif

    return dropped_cnt;
}

/*** end of file ***/
