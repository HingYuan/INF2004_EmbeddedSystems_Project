#ifndef BYPASS_HELPERS_H
#define BYPASS_HELPERS_H

#include "../Navigator/Navigator_task.h"

/* Rectangular front-face estimate, using each scan's own measured standoff. */
#define WIDTH_DEPTH_TOLERANCE_CM 2U
#define WIDTH_REFINE_PASSES      3U
#define WIDTH_REPEAT_TOLERANCE_MM 20U

typedef enum { WIDTH_RAY_UNCERTAIN, WIDTH_RAY_FACE, WIDTH_RAY_CLEAR } WIDTH_RAY_CLASS;
IMPORT WIDTH_RAY_CLASS obstacle_width_classify(UW front_cm, W angle, UW cm, SONAR_STATUS status);
IMPORT UW obstacle_width_offset_mm(UW front_cm, UW angle);
IMPORT OBSTACLE_WIDTH obstacle_width_estimate(const OBSTACLE_PROFILE *first,
                                             const OBSTACLE_PROFILE *second);
#ifndef NDEBUG
IMPORT BOOL obstacle_width_self_check(void);
#endif

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
