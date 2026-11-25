/** @file test_report.c
 *
 * @brief Power-on automated test suite runner and serial result formatter.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#include "test_report.h"

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "imu/imu_test.h"
#include "integration_test.h"
#include "ir/ir_task.h"
#include "ir/ir_test.h"
#include "motor/motor_task.h"
#include "motor/motor_test.h"
#include "navigator/navigator_task.h"
#include "network/network_test.h"
#include "robot_tasks.h"
#include "ultrasonic/ultrasonic_test.h"
#include "usb_console_compat.h"

#define TEST_SUITE_DRAIN_MSEC           (10u)
#define TEST_USB_WAIT_LOOPS             (50u)
#define TEST_USB_WAIT_STEP_MSEC         (100u)
#define TEST_USB_READY_STATE            (2u)
#define TEST_USB_SETTLE_MSEC            (300u)

static uint32_t g_total_count = 0u;
static uint32_t g_pass_count  = 0u;
static uint32_t g_fail_count  = 0u;

static void test_report_reset_ipc(void);

/**
 * @brief Record and print a single `[TEST] <ID> PASS|FAIL` check outcome.
 * @param[in] p_test_id   Test case identifier string.
 * @param[in] b_is_passed True if check passed, false if check failed.
 */
void test_report_record (const char * p_test_id, bool b_is_passed)
{
    const char * p_verdict = b_is_passed ? "PASS" : "FAIL";

    g_total_count++;

    if (b_is_passed)
    {
        g_pass_count++;
    }
    else
    {
        g_fail_count++;
    }

    /* Cast required by uT-Kernel tm_printf(const UB *, ...) signature. */
    tm_printf((UB *)"[TEST] %s %s\n", p_test_id, p_verdict);
}

/**
 * @brief Return total number of TEST_CHECK assertions executed so far.
 * @return Total check count.
 */
uint32_t test_report_get_total (void)
{
    return g_total_count;
}

/**
 * @brief Return number of TEST_CHECK assertions that failed so far.
 * @return Failed check count.
 */
uint32_t test_report_get_failed (void)
{
    return g_fail_count;
}

/**
 * @brief Execute every subsystem unit suite and integration suite in order.
 * @return True if all checks passed (0 failures), false otherwise.
 */
bool test_report_run_all (void)
{
    g_total_count = 0u;
    g_pass_count  = 0u;
    g_fail_count  = 0u;

    tm_printf((UB *)"\n=== INF2004 Automated Test Suite ===\n");

    motor_test_run();
    (void)tk_dly_tsk(TEST_SUITE_DRAIN_MSEC);

    ir_test_run();
    (void)tk_dly_tsk(TEST_SUITE_DRAIN_MSEC);

    ultrasonic_test_run();
    (void)tk_dly_tsk(TEST_SUITE_DRAIN_MSEC);

    imu_test_run();
    (void)tk_dly_tsk(TEST_SUITE_DRAIN_MSEC);

    network_test_run();
    (void)tk_dly_tsk(TEST_SUITE_DRAIN_MSEC);

    integration_test_run();
    (void)tk_dly_tsk(TEST_SUITE_DRAIN_MSEC);

    tm_printf((UB *)"[TEST] SUMMARY total=%u passed=%u failed=%u\n\n",
              (UINT)g_total_count,
              (UINT)g_pass_count,
              (UINT)g_fail_count);

    return (0u == g_fail_count);
}

/**
 * @brief Wait for console readiness, run all tests, and restore clean IPC.
 */
void test_report_boot_run (void)
{
    for (uint32_t idx = 0u;
         (idx < TEST_USB_WAIT_LOOPS) &&
         (usb_console_compat_state() < TEST_USB_READY_STATE);
         idx++)
    {
        (void)tk_dly_tsk(TEST_USB_WAIT_STEP_MSEC);
    }

    (void)tk_dly_tsk(TEST_USB_SETTLE_MSEC);
    (void)test_report_run_all();
    test_report_reset_ipc();
}

/**
 * @brief Drain shared RTOS queues and clear flags/telemetry after self-tests.
 */
static void test_report_reset_ipc (void)
{
    ir_task_bar_record_t bar_rec = { 0u, 0u };
    navigator_task_msg_t nav_msg = { 0 };
    motor_task_cmd_t     mot_cmd = { 0u };

    while (tk_rcv_mbf(robot_tasks_get_barcode_mbf(),
                      &bar_rec,
                      TMO_POL) >= E_OK)
    {
        /* Drain residual barcode test records. */
    }

    while (tk_rcv_mbf(robot_tasks_get_nav_mbf(), &nav_msg, TMO_POL) >= E_OK)
    {
        /* Drain residual Navigator test messages. */
    }

    while (tk_rcv_mbf(robot_tasks_get_motor_mbf(), &mot_cmd, TMO_POL) >= E_OK)
    {
        /* Drain residual motor command test messages. */
    }

    (void)tk_clr_flg(robot_tasks_get_ranger_flg(), 0u);
    robot_tasks_flags_update(0u, 0xFFFFFFFFu);
    navigator_task_set_sonar_telem(false, 0u, 0u);
    navigator_task_set_imu_pose(0, 0, 0, true, 0u);
    navigator_task_set_imu_height(0);
    navigator_task_set_imu_event(0u);
}

/*** end of file ***/
