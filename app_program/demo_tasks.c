/**
 * @file demo_tasks.c
 * @brief Console-independent LED liveness task implementation.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel task entry signature mandates INT stacd parameter.
 */

#include "demo_tasks.h"

#include <stdint.h>

#include <bsp/libbsp.h>

#include "usb_console_compat.h"

#define DEMO_LED_READY_LEVEL            (8)
#define DEMO_LED_GROUP_VALUE            (5)
#define DEMO_LED_READY_HALF_MSEC        (250u)
#define DEMO_LED_LONG_ON_MSEC           (600u)
#define DEMO_LED_LONG_OFF_MSEC          (300u)
#define DEMO_LED_SHORT_ON_MSEC          (150u)
#define DEMO_LED_SHORT_OFF_MSEC         (250u)
#define DEMO_LED_GAP_MSEC               (2000u)
#define DEMO_LED_DIAG_CP_ID             (8)

static void demo_tasks_blink_code(int32_t code_val);

/**
 * @brief Blink the board LED at 2 Hz when USB is up, or blink a status code.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void demo_tasks_blink_task (INT stacd, void * p_exinf)
{
    /* Range-safe: intentionally unused RTOS task entry parameters. */
    (void)stacd;
    /* Range-safe: intentionally unused RTOS extended info pointer. */
    (void)p_exinf;

#if BOARD_DIAG_GPIO_SCAN
    knl_diag_pulse(DEMO_LED_DIAG_CP_ID);
#endif

    for (;;)
    {
        int32_t diag_lvl = usb_console_compat_diag_lvl();

        if (diag_lvl >= DEMO_LED_READY_LEVEL)
        {
            /* Range-safe: LED GPIO output return is non-critical. */
            (void)gpio_set_val(BOARD_LED_PIN, 1u);
            /* Range-safe: 250 ms half-period LED blink delay. */
            (void)tk_dly_tsk(DEMO_LED_READY_HALF_MSEC);
            /* Range-safe: clear LED GPIO output. */
            (void)gpio_set_val(BOARD_LED_PIN, 0u);
            /* Range-safe: 250 ms half-period LED blink delay. */
            (void)tk_dly_tsk(DEMO_LED_READY_HALF_MSEC);
        }
        else
        {
            demo_tasks_blink_code(diag_lvl);
        }
    }
}

/**
 * @brief Emit a long/short LED blink sequence encoding USB bring-up stage 0..7.
 * @param[in] code_val Non-negative diagnostic stage code.
 */
static void demo_tasks_blink_code (int32_t code_val)
{
    int32_t long_cnt  = code_val / DEMO_LED_GROUP_VALUE;
    int32_t short_cnt = code_val % DEMO_LED_GROUP_VALUE;

    for (int32_t idx = 0; idx < long_cnt; idx++)
    {
        /* Range-safe: set LED GPIO high for long pulse. */
        (void)gpio_set_val(BOARD_LED_PIN, 1u);
        /* Range-safe: 600 ms long-pulse on duration. */
        (void)tk_dly_tsk(DEMO_LED_LONG_ON_MSEC);
        /* Range-safe: set LED GPIO low after long pulse. */
        (void)gpio_set_val(BOARD_LED_PIN, 0u);
        /* Range-safe: 300 ms inter-pulse gap. */
        (void)tk_dly_tsk(DEMO_LED_LONG_OFF_MSEC);
    }

    for (int32_t idx = 0; idx < short_cnt; idx++)
    {
        /* Range-safe: set LED GPIO high for short pulse. */
        (void)gpio_set_val(BOARD_LED_PIN, 1u);
        /* Range-safe: 150 ms short-pulse on duration. */
        (void)tk_dly_tsk(DEMO_LED_SHORT_ON_MSEC);
        /* Range-safe: set LED GPIO low after short pulse. */
        (void)gpio_set_val(BOARD_LED_PIN, 0u);
        /* Range-safe: 250 ms inter-pulse gap. */
        (void)tk_dly_tsk(DEMO_LED_SHORT_OFF_MSEC);
    }

    /* Range-safe: ensure LED rests low between diagnostic code bursts. */
    (void)gpio_set_val(BOARD_LED_PIN, 0u);
    /* Range-safe: 2000 ms inter-frame pause. */
    (void)tk_dly_tsk(DEMO_LED_GAP_MSEC);
}

/*** end of file ***/
