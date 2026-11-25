/**
 * @file sonar_servo_main.h
 * @brief Public entry declaration for standalone HC-SR04 and servo sweep test.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: ISO C hosted/SDK entry signature requires standard return type
 *   int for main(void).
 */

#ifndef SONAR_SERVO_MAIN_H
#define SONAR_SERVO_MAIN_H

/**
 * @brief Standalone Pico SDK entry point sweeping servo and printing range.
 * @return Zero on exit (never returns).
 */
int main(void);

#endif /* SONAR_SERVO_MAIN_H */

/*** end of file ***/
