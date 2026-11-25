/** @file module.h
 *
 * @brief Concise summary of the module public interface (BARR-C Appendix B).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#ifndef MODULE_H
#define MODULE_H

#include <stdbool.h>
#include <stdint.h>

#define MODULE_EXAMPLE_LIMIT            (100u)

typedef enum
{
    MODULE_STATUS_OK    = 0,
    MODULE_STATUS_ERROR = 1
} module_status_t;

/**
 * @brief Initialize module internal state.
 * @return MODULE_STATUS_OK on success, or MODULE_STATUS_ERROR on failure.
 */
module_status_t module_init(void);

#endif /* MODULE_H */

/*** end of file ***/
