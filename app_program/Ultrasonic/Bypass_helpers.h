#ifndef BYPASS_HELPERS_H
#define BYPASS_HELPERS_H

#include "../Navigator/Navigator_task.h"

// Navigator keeps the phase, context and tuning values; these helpers execute each step.
IMPORT void bypass_plan_width(BYPASS_CONTEXT *bypass, const OBSTACLE_PROFILE *second);
#ifndef NDEBUG
IMPORT BOOL bypass_self_check(void);
#endif
IMPORT BOOL bypass_hardware_fault(void);
IMPORT void bypass_abort(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, const char *reason);
IMPORT void bypass_begin(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT left, INT right, UW amount);
IMPORT void bypass_pivot(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                        BYPASS_PHASE phase, INT direction);
IMPORT void bypass_track_side(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command);
IMPORT void bypass_probe(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command, BYPASS_PHASE phase);
IMPORT void bypass_follow_next(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command);
IMPORT BOOL bypass_complete_leg(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command);
IMPORT void bypass_probe_result(BYPASS_CONTEXT *bypass, MOTOR_COMMAND *command,
                               const SONAR_PROBE_RESULT *probe);

#endif
