/** @file test_report.h
 *
 * @brief Power-on automated test reporting macro and suite runner interface.
 *
 * BARR-C Deviation (Rules 6.3.a, 6.3.b):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: Course testing specification mandates the TEST_CHECK(ID, EXPR)
 *   macro signature across subsystem test suites; all macro parameters and the
 *   expansion body are fully parenthesized and evaluated once.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Evaluate one test assertion and print `[TEST] <ID> PASS|FAIL`.
 * @param[in] P_ID Immutable test case identifier (e.g. "S1-U01", "INT-01").
 * @param[in] EXPR Boolean condition expected to evaluate to true.
 */
#define TEST_CHECK(P_ID, EXPR) \
    (test_report_record((P_ID), (bool)(EXPR)))

/**
 * @brief Record and print a single `[TEST] <ID> PASS|FAIL` check outcome.
 * @param[in] p_test_id   Test case identifier string.
 * @param[in] b_is_passed True if check passed, false if check failed.
 */
void test_report_record(const char * p_test_id, bool b_is_passed);

/**
 * @brief Return total number of TEST_CHECK assertions executed so far.
 * @return Total check count.
 */
uint32_t test_report_get_total(void);

/**
 * @brief Return number of TEST_CHECK assertions that failed so far.
 * @return Failed check count.
 */
uint32_t test_report_get_failed(void);

/**
 * @brief Execute every subsystem unit suite and integration suite in order.
 * @return True if all checks passed (0 failures), false otherwise.
 */
bool test_report_run_all(void);

/**
 * @brief Wait for console readiness, run all tests, and restore clean IPC.
 */
void test_report_boot_run(void);

#endif /* TEST_REPORT_H */

/*** end of file ***/
