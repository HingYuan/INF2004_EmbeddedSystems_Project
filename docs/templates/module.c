/** @file module.c
 *
 * @brief Implementation of the module interface (BARR-C Appendix C).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 INF2004 Project Team. All rights reserved.
 */

#include "module.h"

#include <stddef.h>

#define MODULE_PRIVATE_SCALE            (10u)

typedef struct
{
    uint32_t count;
    bool     b_is_ready;
} module_state_t;

static module_state_t g_module_state = { 0u, false };

static bool module_validate_limit(uint32_t value);

/**
 * @brief Initialize module internal state.
 * @return MODULE_STATUS_OK on success, or MODULE_STATUS_ERROR on failure.
 */
module_status_t module_init (void)
{
    module_status_t status = MODULE_STATUS_ERROR;

    if (module_validate_limit(MODULE_PRIVATE_SCALE))
    {
        g_module_state.count      = 0u;
        g_module_state.b_is_ready = true;
        status                    = MODULE_STATUS_OK;
    }

    return status;
}

/**
 * @brief Check whether a configuration value is within MODULE_EXAMPLE_LIMIT.
 * @param[in] value Unsigned value to check.
 * @return True if value <= MODULE_EXAMPLE_LIMIT, false otherwise.
 */
static bool module_validate_limit (uint32_t value)
{
    return (value <= MODULE_EXAMPLE_LIMIT);
}

/*** end of file ***/
