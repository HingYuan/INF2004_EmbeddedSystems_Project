/**
 * @file motors_main.h
 * @brief Public entry declaration for standalone motor and encoder test sketch.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: ISO C hosted/SDK entry signature requires standard return type
 *   int for main(void).
 */

#ifndef MOTORS_MAIN_H
#define MOTORS_MAIN_H

/**
 * @brief Standalone Pico SDK entry point testing button-driven motors.
 * @return Zero on exit (never returns).
 */
int main(void);

#endif /* MOTORS_MAIN_H */

/*** end of file ***/
