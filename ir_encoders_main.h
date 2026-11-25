/**
 * @file ir_encoders_main.h
 * @brief Public entry declaration for standalone IR GPIO interrupt monitor.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: ISO C hosted/SDK entry signature requires standard return type
 *   int for main(void).
 */

#ifndef IR_ENCODERS_MAIN_H
#define IR_ENCODERS_MAIN_H

/**
 * @brief Standalone Pico SDK entry point monitoring three IR GPIO interrupts.
 * @return Zero on exit (never returns).
 */
int main(void);

#endif /* IR_ENCODERS_MAIN_H */

/*** end of file ***/
