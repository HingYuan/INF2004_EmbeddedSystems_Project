/**
 * @file usb_console_compat.h
 * @brief Console-state accessors with UART-build fallbacks.
 *
 * The USB-CDC console exports live enumeration and drop counters; the UART
 * build has no ring buffer and returns constant "up, nothing pending" values.
 */

#ifndef USB_CONSOLE_COMPAT_H
#define USB_CONSOLE_COMPAT_H

#include <stdint.h>

/**
 * @brief Read current USB-CDC link enumeration state.
 * @return Value >= 2 once enumerated (constant 3 on UART builds).
 */
uint32_t usb_console_compat_state (void);

/**
 * @brief Read USB-CDC bring-up diagnostic level (0..8).
 * @return Current bring-up stage (constant 8 on UART builds).
 */
int32_t usb_console_compat_diag_lvl (void);

/**
 * @brief Read number of bytes currently queued in the console ring.
 * @return Queued byte count (0 on UART builds).
 */
uint32_t usb_console_compat_pending (void);

/**
 * @brief Read number of console bytes lost to ring overrun.
 * @return Dropped byte count (0 on UART builds).
 */
uint32_t usb_console_compat_dropped (void);

#endif /* USB_CONSOLE_COMPAT_H */

/*** end of file ***/
