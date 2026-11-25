/**
 * @file ultrasonic_task.c
 * @brief HC-SR04 ultrasonic ranger, pan-servo profiling, and side tracking.
 *
 * NOTE: See docs/fsm_diagrams.md for the Ultrasonic scan/tracking workflow.
 * Reference: HC-SR04 Datasheet (10 usec trigger pulse, 58 usec per cm round
 * trip echo time, nominal 2..400 cm range).
 * WARNING: SONAR_SERVO_CENTER_USEC (1695 usec) is empirically calibrated to
 * the mounted servo horn alignment.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel task, message-buffer, event-flag, and BSP PWM APIs
 *   require native RTOS typedefs (INT, UINT, UW, ER, TMO, UB).
 */

#include "ultrasonic_task.h"

#include <stdbool.h>
#include <stdint.h>

#include <bsp/libbsp.h>
#include <tm/tmonitor.h>

#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"
#include "bypass_helpers.h"

/* Servo 50 Hz PWM settings (125 MHz / 125 = 1 MHz -> 20000 ticks = 20 ms). */
#define SONAR_SERVO_CLKDIV              (125u)
#define SONAR_SERVO_PWM_WRAP            (19999u)
#define SONAR_SERVO_CENTER_USEC         (1695u)
#define SONAR_SERVO_SPAN_USEC           (350)
#define SONAR_SERVO_SETTLE_MSEC         (150u)
#define SONAR_ANGLE_LIMIT_DEG           (45)
#define SONAR_ANGLE_STEP_DEG            (15)

/* Ultrasonic echo timing and filtering constants. */
#define SONAR_TRIG_LOW_USEC             (2u)
#define SONAR_TRIG_HIGH_USEC            (10u)
#define SONAR_TIMEOUT_USEC              (26000u)
#define SONAR_ECHO_END_USEC             (60000u)
#define SONAR_USEC_PER_CM               (58u)
#define SONAR_HALF_USEC_PER_CM          (29u)
#define SONAR_MIN_DIST_CM               (2u)
#define SONAR_MAX_DIST_CM               (400u)
#define SONAR_TOO_CLOSE_CM              (1u)
#define SONAR_PING_DELAY_MSEC           (60u)
#define SONAR_CONFIRM_COUNT             (2u)
#define SONAR_SAMPLES_PER_ANGLE         (3u)
#define SONAR_MIN_PAIR_SAMPLES          (2u)
#define SONAR_SAMPLE_LOW_IDX            (0u)
#define SONAR_SAMPLE_MID_IDX            (1u)
#define SONAR_SAMPLE_HIGH_IDX           (2u)
#define SONAR_PAIR_DIVISOR_U32          (2u)
#define SONAR_PAIR_DIVISOR_I32          (2)
#define SONAR_DEFAULT_SPREAD_CM         (15u)
#define SONAR_SEND_TIMEOUT_MSEC         (100)

/* Coarse and fine obstacle profiling constants. */
#define SONAR_COARSE_COUNT              (SONAR_PROFILE_RAYS)
#define SONAR_CENTER_RAY_IDX            (3u)
#define SONAR_RIGHT_OUTER_IDX           (0u)
#define SONAR_LEFT_OUTER_IDX            (6u)
#define SONAR_FIRST_SIDE_STEP           (1)
#define SONAR_HALF_SIDE_RAYS            (3)
#define SONAR_SIDE_LEFT                 (1)
#define SONAR_SIDE_RIGHT                (-1)
#define SONAR_FINE_PASSES               (2u)
#define SONAR_EDGE_DEPTH_GAP_CM         (10u)
#define SONAR_MRAD_PER_DEG_X1000        (17453)
#define SONAR_TRIG_SCALE                (1000)
#define SONAR_TRIG_SCALE_U32            (1000u)
#define SONAR_SIN_CUBE_DIVISOR          (6000000)
#define SONAR_COS_SQ_DIVISOR            (2000)

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t confirmations    : 2;
    uint32_t b_is_tracking    : 1;
    uint32_t tracking_ang_raw : 7;
} ultrasonic_task_runtime_t;

ROBOT_TASKS_STATIC_ASSERT(sonar_confirm_bits,
                          (SONAR_CONFIRM_COUNT <= 3u));
ROBOT_TASKS_STATIC_ASSERT(sonar_runtime_size,
                          (2u == sizeof(ultrasonic_task_runtime_t)));

static const int32_t g_coarse_angles[SONAR_COARSE_COUNT] =
{
    -45, -30, -15, 0, 15, 30, 45
};

static bool sonar_init_hardware(void);
static bool sonar_handle_hw_fault(ultrasonic_task_status_t status);
static bool sonar_handle_obstacle_event(ultrasonic_task_runtime_t * p_rt,
                                        uint32_t front_cm);
static bool sonar_trigger_pulse(void);
static ultrasonic_task_status_t sonar_measure_echo_cm(uint32_t * p_dist_cm);
static ultrasonic_task_status_t sonar_read_distance_cm(uint32_t * p_dist_cm);
static uint32_t sonar_servo_pulse_usec(int32_t angle_deg);
static void sonar_sort_three(uint32_t * p_readings);
static ultrasonic_task_status_t sonar_eval_three_samples(
    uint32_t * p_readings,
    uint32_t valid_cnt,
    uint32_t no_echo_cnt,
    bool b_is_width_mode,
    uint32_t * p_dist_cm);
static ultrasonic_task_status_t sonar_scan_samples(int32_t angle_deg,
                                                   uint32_t * p_dist_cm,
                                                   bool b_is_width_mode);
static ultrasonic_task_status_t sonar_scan_angle(int32_t angle_deg,
                                                 uint32_t * p_dist_cm);
static void sonar_update_closest(navigator_task_obs_profile_t * p_profile,
                                 int32_t angle_deg,
                                 uint32_t dist_cm);
static int32_t sonar_lateral_cm(int32_t angle_deg, uint32_t dist_cm);
static uint32_t sonar_forward_depth_cm(int32_t angle_deg, uint32_t dist_cm);
static bool sonar_same_obs_depth(int32_t angle_deg,
                                 uint32_t dist_cm,
                                 uint32_t center_cm);
static bool sonar_refine_edge_step(int32_t * p_near_ang,
                                   int32_t * p_far_ang,
                                   uint32_t center_cm,
                                   navigator_task_obs_profile_t * p_profile);
static bool sonar_refine_edge(int32_t near_ang,
                              int32_t far_ang,
                              uint32_t center_cm,
                              int32_t * p_edge_ang,
                              uint32_t * p_edge_dist,
                              navigator_task_obs_profile_t * p_profile);
static bool sonar_bisect_width_edge(uint32_t front_cm,
                                    int32_t side_sign,
                                    uint32_t near_ang,
                                    uint32_t far_ang,
                                    uint32_t * p_low_mm,
                                    uint32_t * p_high_mm);
static bool sonar_measure_width_edge(
    const navigator_task_obs_profile_t * p_profile,
    int32_t side_sign,
    uint32_t * p_low_mm,
    uint32_t * p_high_mm);
static void sonar_measure_width(navigator_task_obs_profile_t * p_profile);
static bool sonar_sample_coarse_ray(navigator_task_obs_profile_t * p_profile,
                                    uint32_t idx,
                                    uint32_t * p_valid_cnt);
static bool sonar_run_coarse_scan(navigator_task_obs_profile_t * p_profile,
                                  uint32_t * p_valid_cnt);
static void sonar_find_edges(navigator_task_obs_profile_t * p_profile,
                             uint32_t * p_left_cm,
                             uint32_t * p_right_cm);
static void sonar_compute_offsets(navigator_task_obs_profile_t * p_profile,
                                  uint32_t left_cm,
                                  uint32_t right_cm);
static void sonar_log_scan_summary(
    const navigator_task_obs_profile_t * p_profile,
    uint32_t valid_cnt);
static bool sonar_scan_obstacle(navigator_task_obs_profile_t * p_profile);
static bool sonar_send_to_nav(const navigator_task_msg_t * p_msg);
static void sonar_send_live_dist(uint32_t dist_cm,
                                 bool b_is_valid,
                                 ultrasonic_task_status_t status);
static bool sonar_handle_probe(UINT flag_bits);
static bool sonar_handle_track_tick(const ultrasonic_task_runtime_t * p_rt);
static bool sonar_handle_track_req(ultrasonic_task_runtime_t * p_rt,
                                   UINT flag_bits);
static bool sonar_dispatch_bypass_flg(ultrasonic_task_runtime_t * p_rt,
                                      UINT pattern,
                                      bool * pb_is_in_bypass);
static bool sonar_run_bypass_loop(ultrasonic_task_runtime_t * p_rt);

/**
 * @brief Check whether a latched sonar or servo hardware fault is active.
 * @return True if either sonar or servo fault flag is set.
 */
bool ultrasonic_task_has_fault (void)
{
    const uint32_t fault_mask = ROBOT_FAULT_SONAR | ROBOT_FAULT_SERVO;

    return (0u != (robot_tasks_flags_get() & fault_mask));
}

/**
 * @brief Main ultrasonic obstacle detection, profiling, and tracking task.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ultrasonic_task_run_task (INT stacd, void * p_exinf)
{
    ultrasonic_task_runtime_t sonar        = { 0u, 0u, 0u };
    bool                      b_is_running = sonar_init_hardware();

    /* Explicitly ignore unused RTOS task entry start code. */
    (void)stacd;
    /* Explicitly ignore unused RTOS extended info pointer. */
    (void)p_exinf;

    while (b_is_running)
    {
        uint32_t                 front_cm = 0u;
        ultrasonic_task_status_t status   = sonar_read_distance_cm(&front_cm);

        if (sonar_handle_hw_fault(status))
        {
            b_is_running = false;
        }
        else
        {
            bool b_is_valid =
                ((SONAR_OK == status) || (SONAR_TOO_CLOSE == status));

            if (SONAR_TOO_CLOSE == status)
            {
                front_cm = SONAR_TOO_CLOSE_CM;
            }

            sonar_send_live_dist(front_cm, b_is_valid, status);

            if ((b_is_valid) && (front_cm <= SONAR_OBSTACLE_TRIG_CM))
            {
                sonar.confirmations++;
            }
            else
            {
                sonar.confirmations = 0u;
            }

            if (sonar.confirmations < SONAR_CONFIRM_COUNT)
            {
                /* Ignore sleep return code between forward ultrasonic pings. */
                (void)tk_dly_tsk(SONAR_PING_DELAY_MSEC);
            }
            else
            {
                b_is_running = sonar_handle_obstacle_event(&sonar, front_cm);
            }
        }
    }
}

/**
 * @brief Handle confirmed obstacle detection, profiling, and bypass loop.
 * @param[in,out] p_rt     Sonar runtime state.
 * @param[in]     front_cm Confirmed obstacle distance in cm.
 * @return True if monitoring should resume normally, false on fatal fault.
 */
static bool sonar_handle_obstacle_event (ultrasonic_task_runtime_t * p_rt,
                                         uint32_t front_cm)
{
    navigator_task_msg_t nav_msg = { 0 };
    UINT                 pattern = 0u;
    bool                 b_is_ok = false;

    robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    /* Store MSG_OBSTACLE_DETECTED enum in 4-bit unsigned message field. */
    nav_msg.type             = (uint32_t)MSG_OBSTACLE_DETECTED;
    /* Range-safe: front_cm <= 20 fits in uint16_t. */
    nav_msg.data.obs_dist_cm = (uint16_t)front_cm;

    /* Cast format literal and front distance to RTOS UB* and UINT. */
    tm_printf((UB *)"[sonar] Obstacle at %u cm\n", (UINT)front_cm);

    if ((sonar_send_to_nav(&nav_msg)) &&
        (tk_wai_flg(robot_tasks_get_ranger_flg(),
                    SONAR_SCAN_REQUEST,
                    TWF_ORW | TWF_BITCLR,
                    &pattern,
                    TMO_FEVR) >= E_OK))
    {
        navigator_task_obs_profile_t profile  = { 0u };
        navigator_task_msg_t         prof_msg = { 0 };

        /* Profile validity is stored inside profile.b_is_profile_valid. */
        (void)sonar_scan_obstacle(&profile);
        /* Store MSG_OBSTACLE_PROFILED enum in 4-bit unsigned message field. */
        prof_msg.type             = (uint32_t)MSG_OBSTACLE_PROFILED;
        prof_msg.data.obs_profile = profile;

        if ((sonar_send_to_nav(&prof_msg)) && (sonar_run_bypass_loop(p_rt)))
        {
            p_rt->confirmations = 0u;
            /* Ignore delay return value before resuming forward pings. */
            (void)tk_dly_tsk(SONAR_PING_DELAY_MSEC);
            b_is_ok             = true;
        }
    }

    return b_is_ok;
}

/**
 * @brief Initialize ultrasonic GPIO pins and 50 Hz pan-servo PWM output.
 * @return True on success, false if GPIO or PWM setup failed.
 */
static bool sonar_init_hardware (void)
{
    bool b_is_ok = true;

    if ((gpio_set_pin(SONAR_TRIG_PIN, GPIO_MODE_OUT) < E_OK) ||
        (gpio_set_pin(SONAR_ECHO_PIN, GPIO_MODE_IN) < E_OK))
    {
        /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
        (void)sonar_handle_hw_fault(SONAR_SENSOR_FAULT);
        /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
        tm_printf((UB *)"[sonar] GPIO setup failed\n");
        b_is_ok = false;
    }

    if (b_is_ok)
    {
        /* Drive trigger pin low at boot; pin setup verified above. */
        (void)gpio_set_val(SONAR_TRIG_PIN, 0u);

        /* Cast 50 Hz servo wrap and center pulse constants to BSP UW type. */
        if ((pwm_set_divider(SONAR_SERVO_PIN, SONAR_SERVO_CLKDIV, 0u) < E_OK) ||
            (pwm_set_wrap(SONAR_SERVO_PIN, (UW)SONAR_SERVO_PWM_WRAP) < E_OK) ||
            (pwm_set_pin(SONAR_SERVO_PIN) < E_OK) ||
            (pwm_set_cc(SONAR_SERVO_PIN, (UW)SONAR_SERVO_CENTER_USEC) < E_OK) ||
            (pwm_set_enabled(SONAR_SERVO_PIN, TRUE) < E_OK))
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SERVO_FAULT);
            /* Cast ASCII literal to uT-Kernel UB pointer for console output. */
            tm_printf((UB *)"[sonar] Servo setup failed\n");
            b_is_ok = false;
        }
        else
        {
            /* Allow pan servo horn to settle at 0 deg center position. */
            (void)tk_dly_tsk(SONAR_SERVO_SETTLE_MSEC);
            /* Cast format literal and trigger distance to RTOS UB* and UINT. */
            tm_printf((UB *)"[sonar] Forward monitoring at %u cm\n",
                      (UINT)SONAR_OBSTACLE_TRIG_CM);
        }
    }

    return b_is_ok;
}

/**
 * @brief Latch sonar or servo hardware fault and assert E-stop if fatal.
 * @param[in] status Ultrasonic measurement status.
 * @return True if status was a fatal hardware fault, false otherwise.
 */
static bool sonar_handle_hw_fault (ultrasonic_task_status_t status)
{
    bool b_is_fault = ((SONAR_SENSOR_FAULT == status) ||
                       (SONAR_SERVO_FAULT == status));

    if (b_is_fault)
    {
        uint32_t fault_bit = (SONAR_SERVO_FAULT == status)
                             ? ROBOT_FAULT_SERVO
                             : ROBOT_FAULT_SONAR;

        robot_tasks_flags_update(fault_bit, 0u);
        robot_tasks_flag_assign(ROBOT_ESTOP_REQUEST, true);
    }

    return b_is_fault;
}

/**
 * @brief Emit a 10 usec trigger pulse on the HC-SR04 TRIG pin.
 * @return True if echo line was idle and GPIO writes succeeded, false on fault.
 */
static bool sonar_trigger_pulse (void)
{
    bool b_is_ok = (0u == gpio_get_val(SONAR_ECHO_PIN));

    if (b_is_ok)
    {
        b_is_ok = (gpio_set_val(SONAR_TRIG_PIN, 0u) >= E_OK);
    }

    if (b_is_ok)
    {
        robot_tasks_delay_usec(SONAR_TRIG_LOW_USEC);
        b_is_ok = (gpio_set_val(SONAR_TRIG_PIN, 1u) >= E_OK);
    }

    if (b_is_ok)
    {
        robot_tasks_delay_usec(SONAR_TRIG_HIGH_USEC);
        b_is_ok = (gpio_set_val(SONAR_TRIG_PIN, 0u) >= E_OK);
    }

    return b_is_ok;
}

/**
 * @brief Time the HC-SR04 echo high pulse and convert it to centimetres.
 * @param[out] p_dist_cm Measured distance in cm (0 on non-OK status).
 * @return Ultrasonic measurement status.
 */
static ultrasonic_task_status_t sonar_measure_echo_cm (uint32_t * p_dist_cm)
{
    uint32_t                 start_usec = robot_tasks_time_usec();
    ultrasonic_task_status_t status     = SONAR_OK;

    while ((0u == gpio_get_val(SONAR_ECHO_PIN)) && (SONAR_OK == status))
    {
        if ((robot_tasks_time_usec() - start_usec) >= SONAR_TIMEOUT_USEC)
        {
            status = SONAR_NO_ECHO;
        }
    }

    if (SONAR_OK == status)
    {
        start_usec = robot_tasks_time_usec();

        while ((0u != gpio_get_val(SONAR_ECHO_PIN)) && (SONAR_OK == status))
        {
            if ((robot_tasks_time_usec() - start_usec) >= SONAR_ECHO_END_USEC)
            {
                status = SONAR_SENSOR_FAULT;
            }
        }
    }

    if (SONAR_OK == status)
    {
        uint32_t pulse_usec  = robot_tasks_time_usec() - start_usec;
        uint32_t measured_cm =
            (pulse_usec + SONAR_HALF_USEC_PER_CM) / SONAR_USEC_PER_CM;

        if (measured_cm < SONAR_MIN_DIST_CM)
        {
            status = SONAR_TOO_CLOSE;
        }
        else if (measured_cm > SONAR_MAX_DIST_CM)
        {
            status = SONAR_NO_ECHO;
        }
        else
        {
            *p_dist_cm = measured_cm;
        }
    }

    return status;
}

/**
 * @brief Trigger one HC-SR04 ping and measure echo pulse width in cm.
 * @param[out] p_dist_cm Measured distance in cm (0 on non-OK status).
 * @return Ultrasonic status classification.
 */
static ultrasonic_task_status_t sonar_read_distance_cm (uint32_t * p_dist_cm)
{
    ultrasonic_task_status_t status = SONAR_SENSOR_FAULT;

    *p_dist_cm = 0u;

    if (sonar_trigger_pulse())
    {
        status = sonar_measure_echo_cm(p_dist_cm);
    }

    return status;
}

/**
 * @brief Convert signed angle (-45..+45 deg, positive = left) to servo usec.
 * @param[in] angle_deg Requested servo angle in degrees.
 * @return Clamped PWM pulse width in microseconds.
 */
static uint32_t sonar_servo_pulse_usec (int32_t angle_deg)
{
    int32_t clamped_ang = angle_deg;
    int32_t pulse_usec  = 0;

    if (clamped_ang < (-SONAR_ANGLE_LIMIT_DEG))
    {
        clamped_ang = -SONAR_ANGLE_LIMIT_DEG;
    }
    else if (clamped_ang > SONAR_ANGLE_LIMIT_DEG)
    {
        clamped_ang = SONAR_ANGLE_LIMIT_DEG;
    }
    else
    {
        /* Angle already inside [-45, +45]. */
    }

    /* Convert unsigned center pulse constant to int32_t for signed offset. */
    pulse_usec = ((int32_t)SONAR_SERVO_CENTER_USEC) +
                 ((clamped_ang * SONAR_SERVO_SPAN_USEC) /
                  SONAR_ANGLE_LIMIT_DEG);

    /* Range-safe: pulse_usec lies in [1345, 2045] usec. */
    return (uint32_t)pulse_usec;
}

/**
 * @brief Sort three unsigned distance samples into non-decreasing order.
 * @param[in,out] p_readings Array of three distance samples in cm.
 */
static void sonar_sort_three (uint32_t * p_readings)
{
    for (uint32_t idx_a = 0u; idx_a < SONAR_SAMPLES_PER_ANGLE; idx_a++)
    {
        for (uint32_t idx_b = idx_a + 1u;
             idx_b < SONAR_SAMPLES_PER_ANGLE;
             idx_b++)
        {
            if (p_readings[idx_b] < p_readings[idx_a])
            {
                uint32_t swap_val = p_readings[idx_a];

                p_readings[idx_a] = p_readings[idx_b];
                p_readings[idx_b] = swap_val;
            }
        }
    }
}

/**
 * @brief Reduce up to 3 sonar ping samples into one stable distance reading.
 * @param[in,out] p_readings      Collected valid samples (sorted if cnt == 3).
 * @param[in]     valid_cnt       Number of valid samples collected (0..3).
 * @param[in]     no_echo_cnt     Number of SONAR_NO_ECHO pings observed.
 * @param[in]     b_is_width_mode True for stricter width-measurement spread.
 * @param[out]    p_dist_cm       Output stable distance in cm when SONAR_OK.
 * @return Ultrasonic measurement status.
 */
static ultrasonic_task_status_t sonar_eval_three_samples (
    uint32_t * p_readings,
    uint32_t valid_cnt,
    uint32_t no_echo_cnt,
    bool b_is_width_mode,
    uint32_t * p_dist_cm)
{
    ultrasonic_task_status_t ret_status = SONAR_UNSTABLE;

    if (SONAR_SAMPLES_PER_ANGLE == no_echo_cnt)
    {
        ret_status = SONAR_NO_ECHO;
    }
    else if (valid_cnt < SONAR_MIN_PAIR_SAMPLES)
    {
        ret_status = SONAR_UNSTABLE;
    }
    else if (SONAR_MIN_PAIR_SAMPLES == valid_cnt)
    {
        uint32_t first_cm  = p_readings[SONAR_SAMPLE_LOW_IDX];
        uint32_t second_cm = p_readings[SONAR_SAMPLE_MID_IDX];
        uint32_t gap_cm    = (first_cm > second_cm)
                             ? (first_cm - second_cm)
                             : (second_cm - first_cm);
        uint32_t max_gap   = b_is_width_mode
                             ? BYPASS_WIDTH_SAMPLE_SPREAD_CM
                             : SONAR_DEFAULT_SPREAD_CM;

        if (gap_cm <= max_gap)
        {
            *p_dist_cm = (first_cm + second_cm) / SONAR_PAIR_DIVISOR_U32;
            ret_status = SONAR_OK;
        }
    }
    else
    {
        sonar_sort_three(p_readings);

        if ((!b_is_width_mode) ||
            (bypass_helpers_samples_valid(p_readings[SONAR_SAMPLE_LOW_IDX],
                                          p_readings[SONAR_SAMPLE_MID_IDX],
                                          p_readings[SONAR_SAMPLE_HIGH_IDX])))
        {
            *p_dist_cm = p_readings[SONAR_SAMPLE_MID_IDX];
            ret_status = SONAR_OK;
        }
    }

    return ret_status;
}

/**
 * @brief Move servo to angle_deg and take 3 ultrasonic pings for stability.
 * @param[in]  angle_deg       Target pan angle (-45..+45 deg).
 * @param[out] p_dist_cm       Median or averaged stable distance in cm.
 * @param[in]  b_is_width_mode True to apply stricter width-measurement spread.
 * @return Ultrasonic measurement status.
 */
static ultrasonic_task_status_t sonar_scan_samples (int32_t angle_deg,
                                                    uint32_t * p_dist_cm,
                                                    bool b_is_width_mode)
{
    ultrasonic_task_status_t ret_status                        = SONAR_OK;
    uint32_t                 readings[SONAR_SAMPLES_PER_ANGLE] = { 0u, 0u, 0u };
    uint32_t                 valid_cnt                         = 0u;
    uint32_t                 no_echo_cnt                       = 0u;
    bool                     b_should_abort                    = false;

    *p_dist_cm = 0u;

    /* Cast clamped servo pulse width in microseconds to BSP PWM UW type. */
    if (pwm_set_cc(SONAR_SERVO_PIN,
                   (UW)sonar_servo_pulse_usec(angle_deg)) < E_OK)
    {
        ret_status     = SONAR_SERVO_FAULT;
        b_should_abort = true;
    }

    if (!b_should_abort)
    {
        /* Allow pan servo horn to settle before pinging at new angle. */
        (void)tk_dly_tsk(SONAR_SERVO_SETTLE_MSEC);

        for (uint32_t idx = 0u;
             (idx < SONAR_SAMPLES_PER_ANGLE) && (!b_should_abort);
             idx++)
        {
            uint32_t                 sample_cm = 0u;
            ultrasonic_task_status_t ping_st   = SONAR_INVALID;

            if (idx > 0u)
            {
                /* Space consecutive pings by 60 ms to clear room echoes. */
                (void)tk_dly_tsk(SONAR_PING_DELAY_MSEC);
            }

            ping_st = sonar_read_distance_cm(&sample_cm);

            if (SONAR_OK == ping_st)
            {
                readings[valid_cnt] = sample_cm;
                valid_cnt++;
            }
            else if (SONAR_NO_ECHO == ping_st)
            {
                no_echo_cnt++;
            }
            else
            {
                ret_status     = ping_st;
                b_should_abort = true;
            }
        }
    }

    if (!b_should_abort)
    {
        ret_status = sonar_eval_three_samples(readings,
                                              valid_cnt,
                                              no_echo_cnt,
                                              b_is_width_mode,
                                              p_dist_cm);
    }

    return ret_status;
}

/**
 * @brief Measure distance at angle_deg using standard clearance tolerance.
 * @param[in]  angle_deg Target servo angle (-45..+45 deg).
 * @param[out] p_dist_cm Stable distance in cm.
 * @return Ultrasonic measurement status.
 */
static ultrasonic_task_status_t sonar_scan_angle (int32_t angle_deg,
                                                  uint32_t * p_dist_cm)
{
    return sonar_scan_samples(angle_deg, p_dist_cm, false);
}

/**
 * @brief Update the closest observed obstacle point in a profile.
 * @param[in,out] p_profile Obstacle profile being populated.
 * @param[in]     angle_deg Signed ray angle in degrees.
 * @param[in]     dist_cm   Measured distance in cm.
 */
static void sonar_update_closest (navigator_task_obs_profile_t * p_profile,
                                  int32_t angle_deg,
                                  uint32_t dist_cm)
{
    if ((0u == p_profile->closest_cm) || (dist_cm < p_profile->closest_cm))
    {
        p_profile->closest_cm      = dist_cm;
        p_profile->closest_ang_raw =
            robot_tasks_pack_signed(angle_deg, NAV_ANGLE_BIT_WIDTH);
    }
}

/**
 * @brief Fixed-point sideways projection distance * sin(angle) in cm.
 * @param[in] angle_deg Signed angle in degrees (-45..+45).
 * @param[in] dist_cm   Ray distance in cm.
 * @return Signed lateral offset in cm (positive = left).
 */
static int32_t sonar_lateral_cm (int32_t angle_deg, uint32_t dist_cm)
{
    int32_t rad_mrad   = (angle_deg * SONAR_MRAD_PER_DEG_X1000) /
                         SONAR_TRIG_SCALE;
    int32_t rad_cube   = (rad_mrad * rad_mrad) * rad_mrad;
    int32_t sin_scaled = rad_mrad - (rad_cube / SONAR_SIN_CUBE_DIVISOR);

    /* Range-safe: dist_cm <= 400 fits within positive int32_t. */
    return (((int32_t)dist_cm) * sin_scaled) / SONAR_TRIG_SCALE;
}

/**
 * @brief Fixed-point forward depth projection distance * cos(angle) in cm.
 * @param[in] angle_deg Signed angle in degrees (-45..+45).
 * @param[in] dist_cm   Ray distance in cm.
 * @return Projected forward depth in cm.
 */
static uint32_t sonar_forward_depth_cm (int32_t angle_deg, uint32_t dist_cm)
{
    int32_t rad_mrad   = (angle_deg * SONAR_MRAD_PER_DEG_X1000) /
                         SONAR_TRIG_SCALE;
    int32_t cos_scaled = SONAR_TRIG_SCALE -
                         ((rad_mrad * rad_mrad) / SONAR_COS_SQ_DIVISOR);

    /* Range-safe: for |angle| <= 45 deg, cos_scaled is in [691, 1000]. */
    return (dist_cm * ((uint32_t)cos_scaled)) / SONAR_TRIG_SCALE_U32;
}

/**
 * @brief Check whether a ray depth belongs to the same obstacle front face.
 * @param[in] angle_deg Signed ray angle in degrees.
 * @param[in] dist_cm   Measured ray distance in cm.
 * @param[in] center_cm Center ray standoff distance in cm.
 * @return True if projected depth is within EDGE_DEPTH_GAP_CM of center_cm.
 */
static bool sonar_same_obs_depth (int32_t angle_deg,
                                  uint32_t dist_cm,
                                  uint32_t center_cm)
{
    return (sonar_forward_depth_cm(angle_deg, dist_cm) <=
            (center_cm + SONAR_EDGE_DEPTH_GAP_CM));
}

/**
 * @brief Execute one midpoint bisection step for obstacle boundary refinement.
 * @param[in,out] p_near_ang Inner angle hitting the obstacle.
 * @param[in,out] p_far_ang  Outer angle clearing the obstacle.
 * @param[in]     center_cm  Center ray standoff in cm.
 * @param[in,out] p_profile  Profile updated with closest observed ray.
 * @return True if step succeeded, false on degenerate bracket or fault.
 */
static bool sonar_refine_edge_step (int32_t * p_near_ang,
                                    int32_t * p_far_ang,
                                    uint32_t center_cm,
                                    navigator_task_obs_profile_t * p_profile)
{
    int32_t  mid_ang = ((*p_near_ang) + (*p_far_ang)) / SONAR_PAIR_DIVISOR_I32;
    uint32_t dist_cm = 0u;
    bool     b_is_ok = ((mid_ang != (*p_near_ang)) &&
                        (mid_ang != (*p_far_ang)));

    if (b_is_ok)
    {
        ultrasonic_task_status_t status = sonar_scan_angle(mid_ang, &dist_cm);

        if (sonar_handle_hw_fault(status))
        {
            b_is_ok = false;
        }
        else if (SONAR_NO_ECHO == status)
        {
            *p_far_ang = mid_ang;
        }
        else if (SONAR_OK != status)
        {
            b_is_ok = false;
        }
        else
        {
            bool b_is_same_depth =
                sonar_same_obs_depth(mid_ang, dist_cm, center_cm);

            sonar_update_closest(p_profile, mid_ang, dist_cm);
            /* Cast format literal, angle, and distance to UB*, INT, UINT. */
            tm_printf((UB *)"[sonar] fine %d: %u cm\n",
                      (INT)mid_ang,
                      (UINT)dist_cm);
            *p_near_ang = b_is_same_depth ? mid_ang : (*p_near_ang);
            *p_far_ang  = b_is_same_depth ? (*p_far_ang) : mid_ang;
        }
    }

    return b_is_ok;
}

/**
 * @brief Bisect an obstacle boundary angle with two midpoint pings.
 * @param[in]     near_ang    Inner angle hitting the obstacle.
 * @param[in]     far_ang     Outer angle clearing the obstacle.
 * @param[in]     center_cm   Center ray standoff in cm.
 * @param[out]    p_edge_ang  Refined outer boundary angle in degrees.
 * @param[out]    p_edge_dist Projected boundary distance in cm.
 * @param[in,out] p_profile   Profile updated with closest observed ray.
 * @return True if boundary refinement completed without fault.
 */
static bool sonar_refine_edge (int32_t near_ang,
                               int32_t far_ang,
                               uint32_t center_cm,
                               int32_t * p_edge_ang,
                               uint32_t * p_edge_dist,
                               navigator_task_obs_profile_t * p_profile)
{
    bool b_is_ok = true;

    for (uint32_t pass_idx = 0u;
         (pass_idx < SONAR_FINE_PASSES) && (b_is_ok);
         pass_idx++)
    {
        b_is_ok = sonar_refine_edge_step(&near_ang,
                                         &far_ang,
                                         center_cm,
                                         p_profile);
    }

    if (b_is_ok)
    {
        uint32_t cosine_val =
            sonar_forward_depth_cm(far_ang, SONAR_TRIG_SCALE_U32);

        *p_edge_ang  = far_ang;
        *p_edge_dist = ((center_cm * SONAR_TRIG_SCALE_U32) +
                        (cosine_val - 1u)) / cosine_val;
    }

    return b_is_ok;
}

/**
 * @brief Bisect one front-face width bracket and convert endpoints to mm.
 * @param[in]  front_cm  Center front standoff in cm.
 * @param[in]  side_sign +1 for left edge, -1 for right edge.
 * @param[in]  near_ang  Inner ray angle magnitude hitting the face.
 * @param[in]  far_ang   Outer ray angle magnitude clearing the face.
 * @param[out] p_low_mm  Lower lateral edge bound in mm.
 * @param[out] p_high_mm Upper lateral edge bound in mm.
 * @return True if final bracket is valid and within BYPASS_WIDTH_MAX_SPAN_MM.
 */
static bool sonar_bisect_width_edge (uint32_t front_cm,
                                     int32_t side_sign,
                                     uint32_t near_ang,
                                     uint32_t far_ang,
                                     uint32_t * p_low_mm,
                                     uint32_t * p_high_mm)
{
    bool b_is_ok    = true;
    bool b_is_done  = false;
    bool b_is_valid = false;

    for (uint32_t pass_idx = 0u;
         ((pass_idx < BYPASS_WIDTH_REFINE_PASSES) && (b_is_ok)) && (!b_is_done);
         pass_idx++)
    {
        uint32_t mid_ang = (near_ang + far_ang) / SONAR_PAIR_DIVISOR_U32;

        if (mid_ang == near_ang)
        {
            b_is_done = true;
        }
        else
        {
            uint32_t                 sample_cm  = 0u;
            /* Range-safe: mid_ang <= 45 fits positive int32_t. */
            int32_t                  signed_ang = side_sign *
                                                  ((int32_t)mid_ang);
            ultrasonic_task_status_t status     = sonar_scan_samples(signed_ang,
                                                                     &sample_cm,
                                                                     true);

            if (sonar_handle_hw_fault(status))
            {
                b_is_ok = false;
            }
            else
            {
                bypass_helpers_ray_class_t ray_cls =
                    bypass_helpers_classify_ray(front_cm,
                                                signed_ang,
                                                sample_cm,
                                                status);

                near_ang = (WIDTH_RAY_FACE == ray_cls) ? mid_ang : near_ang;
                far_ang  = (WIDTH_RAY_CLEAR == ray_cls) ? mid_ang : far_ang;
            }
        }
    }

    if (b_is_ok)
    {
        *p_low_mm  = bypass_helpers_width_offset_mm(front_cm, near_ang);
        *p_high_mm = bypass_helpers_width_offset_mm(front_cm, far_ang);
        b_is_valid = (((*p_high_mm) > (*p_low_mm)) &&
                      (((*p_high_mm) - (*p_low_mm)) <=
                       BYPASS_WIDTH_MAX_SPAN_MM));
    }

    return b_is_valid;
}

/**
 * @brief Bracket one front-face edge for independent dashboard width reporting.
 * @param[in]  p_profile Stationary coarse obstacle profile.
 * @param[in]  side_sign +1 for left edge, -1 for right edge.
 * @param[out] p_low_mm  Lower lateral edge bound in mm.
 * @param[out] p_high_mm Upper lateral edge bound in mm.
 * @return True if edge is bounded within WIDTH_MAX_EDGE_SPAN_MM.
 */
static bool sonar_measure_width_edge (
    const navigator_task_obs_profile_t * p_profile,
    int32_t side_sign,
    uint32_t * p_low_mm,
    uint32_t * p_high_mm)
{
    bool    b_is_valid = false;
    int32_t last_face  = 0;
    int32_t outer_ray  = 0;

    if ((p_profile->front_cm >= SONAR_MIN_DIST_CM) &&
        (p_profile->front_cm <= SONAR_MAX_DIST_CM))
    {
        for (int32_t ray_step = SONAR_FIRST_SIDE_STEP;
             ray_step <= SONAR_HALF_SIDE_RAYS;
             ray_step++)
        {
            /* Range-safe: 3 +/- (1..3) evaluates to coarse index 0..6. */
            uint32_t ray_idx =
                (uint32_t)(((int32_t)SONAR_CENTER_RAY_IDX) +
                           (side_sign * ray_step));
            int32_t  ray_ang = side_sign * (ray_step * SONAR_ANGLE_STEP_DEG);

            if (WIDTH_RAY_FACE ==
                bypass_helpers_classify_ray(
                    p_profile->front_cm,
                    ray_ang,
                    navigator_task_get_ray_cm(p_profile, ray_idx),
                    navigator_task_get_ray_status(p_profile, ray_idx)))
            {
                last_face = ray_step;
            }
        }

        for (int32_t ray_step = last_face + SONAR_FIRST_SIDE_STEP;
             (ray_step <= SONAR_HALF_SIDE_RAYS) && (0 == outer_ray);
             ray_step++)
        {
            /* Range-safe: 3 +/- (1..3) evaluates to coarse index 0..6. */
            uint32_t ray_idx =
                (uint32_t)(((int32_t)SONAR_CENTER_RAY_IDX) +
                           (side_sign * ray_step));
            int32_t  ray_ang = side_sign * (ray_step * SONAR_ANGLE_STEP_DEG);

            if (WIDTH_RAY_CLEAR ==
                bypass_helpers_classify_ray(
                    p_profile->front_cm,
                    ray_ang,
                    navigator_task_get_ray_cm(p_profile, ray_idx),
                    navigator_task_get_ray_status(p_profile, ray_idx)))
            {
                outer_ray = ray_step;
            }
        }

        if (0 != outer_ray)
        {
            /* Range-safe: last_face in [0, 2],SONAR_ANGLE_STEP_DEG is 15. */
            uint32_t near_ang = ((uint32_t)last_face) *
                                ((uint32_t)SONAR_ANGLE_STEP_DEG);
            /* Range-safe: outer_ray in [1, 3], SONAR_ANGLE_STEP_DEG is 15. */
            uint32_t far_ang  = ((uint32_t)outer_ray) *
                                ((uint32_t)SONAR_ANGLE_STEP_DEG);

            b_is_valid = sonar_bisect_width_edge(p_profile->front_cm,
                                                 side_sign,
                                                 near_ang,
                                                 far_ang,
                                                 p_low_mm,
                                                 p_high_mm);
        }
    }

    return b_is_valid;
}

/**
 * @brief Measure left and right front-face edge brackets for dashboard width.
 * @param[in,out] p_profile Obstacle profile receiving measured_width brackets.
 */
static void sonar_measure_width (navigator_task_obs_profile_t * p_profile)
{
    uint32_t low_mm  = 0u;
    uint32_t high_mm = 0u;

    if (sonar_measure_width_edge(p_profile, SONAR_SIDE_LEFT, &low_mm, &high_mm))
    {
        p_profile->measured_width.left_min_mm     = low_mm;
        p_profile->measured_width.left_max_mm     = high_mm;
        p_profile->measured_width.b_is_left_valid = 1u;
    }

    if ((!ultrasonic_task_has_fault()) &&
        (sonar_measure_width_edge(p_profile,
                                  SONAR_SIDE_RIGHT,
                                  &low_mm,
                                  &high_mm)))
    {
        p_profile->measured_width.right_min_mm     = low_mm;
        p_profile->measured_width.right_max_mm     = high_mm;
        p_profile->measured_width.b_is_right_valid = 1u;
    }
}

/**
 * @brief Acquire one coarse ray angle with single retry on SONAR_UNSTABLE.
 * @param[in,out] p_profile   Profile storing ray readings and closest point.
 * @param[in]     idx         Coarse ray index (0..SONAR_COARSE_COUNT-1).
 * @param[in,out] p_valid_cnt Running count of SONAR_OK rays observed.
 * @return True if no fatal hardware fault occurred.
 */
static bool sonar_sample_coarse_ray (navigator_task_obs_profile_t * p_profile,
                                     uint32_t idx,
                                     uint32_t * p_valid_cnt)
{
    uint32_t                 meas_cm = 0u;
    ultrasonic_task_status_t status  = sonar_scan_angle(g_coarse_angles[idx],
                                                        &meas_cm);
    bool                     b_is_ok = true;

    navigator_task_set_ray(p_profile, idx, meas_cm, status);

    if (sonar_handle_hw_fault(navigator_task_get_ray_status(p_profile, idx)))
    {
        b_is_ok = false;
    }

    if ((b_is_ok) &&
        (SONAR_UNSTABLE == navigator_task_get_ray_status(p_profile, idx)))
    {
        /* Delay 60 ms before retrying unstable coarse sonar ray. */
        (void)tk_dly_tsk(SONAR_PING_DELAY_MSEC);
        status = sonar_scan_angle(g_coarse_angles[idx], &meas_cm);
        navigator_task_set_ray(p_profile, idx, meas_cm, status);

        if (sonar_handle_hw_fault(
                navigator_task_get_ray_status(p_profile, idx)))
        {
            b_is_ok = false;
        }
    }

    if (b_is_ok)
    {
        ultrasonic_task_status_t ray_st =
            navigator_task_get_ray_status(p_profile, idx);
        uint32_t                 ray_cm =
            navigator_task_get_ray_cm(p_profile, idx);

        if (SONAR_OK == ray_st)
        {
            (*p_valid_cnt)++;
            sonar_update_closest(p_profile, g_coarse_angles[idx], ray_cm);
        }

        /* Cast format literal, ray angle, and distance to UB*, INT, UINT. */
        tm_printf((UB *)"[sonar] coarse %d: %u cm valid=%d\n",
                  (INT)g_coarse_angles[idx],
                  (UINT)ray_cm,
                  (SONAR_OK == ray_st) ? 1 : 0);
    }

    return b_is_ok;
}

/**
 * @brief Execute 7-ray coarse sweep (-45..+45 deg) with single unstable retry.
 * @param[in,out] p_profile   Profile storing ray readings and closest point.
 * @param[out]    p_valid_cnt Count of SONAR_OK rays observed.
 * @return True if sweep completed without hardware fault.
 */
static bool sonar_run_coarse_scan (navigator_task_obs_profile_t * p_profile,
                                   uint32_t * p_valid_cnt)
{
    bool b_is_ok = true;

    *p_valid_cnt = 0u;

    for (uint32_t idx = 0u; (idx < SONAR_COARSE_COUNT) && (b_is_ok); idx++)
    {
        b_is_ok = sonar_sample_coarse_ray(p_profile, idx, p_valid_cnt);
    }

    return b_is_ok;
}

/**
 * @brief Locate and refine right and left obstacle boundary transitions.
 * @param[in,out] p_profile  Obstacle profile with completed coarse rays.
 * @param[out]    p_left_cm  Projected distance to refined left edge in cm.
 * @param[out]    p_right_cm Projected distance to refined right edge in cm.
 */
static void sonar_find_edges (navigator_task_obs_profile_t * p_profile,
                              uint32_t * p_left_cm,
                              uint32_t * p_right_cm)
{
    uint32_t center_cm      =
        navigator_task_get_ray_cm(p_profile, SONAR_CENTER_RAY_IDX);
    bool     b_right_active = true;
    bool     b_left_active  = true;

    for (uint32_t idx = SONAR_CENTER_RAY_IDX;
         (idx > 0u) && (b_right_active);
         idx--)
    {
        ultrasonic_task_status_t cur_st  =
            navigator_task_get_ray_status(p_profile, idx);
        ultrasonic_task_status_t next_st =
            navigator_task_get_ray_status(p_profile, idx - 1u);
        uint32_t                 cur_cm  =
            navigator_task_get_ray_cm(p_profile, idx);
        uint32_t                 next_cm =
            navigator_task_get_ray_cm(p_profile, idx - 1u);

        if ((SONAR_OK != cur_st) ||
            ((SONAR_OK != next_st) && (SONAR_NO_ECHO != next_st)) ||
            (!sonar_same_obs_depth(g_coarse_angles[idx], cur_cm, center_cm)))
        {
            b_right_active = false;
        }
        else if ((SONAR_NO_ECHO == next_st) ||
                 (!sonar_same_obs_depth(g_coarse_angles[idx - 1u],
                                        next_cm,
                                        center_cm)))
        {
            int32_t refined_ang = 0;

            p_profile->b_is_right_edge_valid = sonar_refine_edge(
                g_coarse_angles[idx],
                g_coarse_angles[idx - 1u],
                center_cm,
                &refined_ang,
                p_right_cm,
                p_profile) ? 1u : 0u;
            p_profile->right_edge_deg_raw =
                robot_tasks_pack_signed(refined_ang, NAV_ANGLE_BIT_WIDTH);
            b_right_active = false;
        }
        else
        {
            /* Both adjacent right-side rays hit same front face; continue. */
        }
    }

    for (uint32_t idx = SONAR_CENTER_RAY_IDX;
         (idx < (SONAR_COARSE_COUNT - 1u)) && (b_left_active);
         idx++)
    {
        ultrasonic_task_status_t cur_st  =
            navigator_task_get_ray_status(p_profile, idx);
        ultrasonic_task_status_t next_st =
            navigator_task_get_ray_status(p_profile, idx + 1u);
        uint32_t                 cur_cm  =
            navigator_task_get_ray_cm(p_profile, idx);
        uint32_t                 next_cm =
            navigator_task_get_ray_cm(p_profile, idx + 1u);

        if ((SONAR_OK != cur_st) ||
            ((SONAR_OK != next_st) && (SONAR_NO_ECHO != next_st)) ||
            (!sonar_same_obs_depth(g_coarse_angles[idx], cur_cm, center_cm)))
        {
            b_left_active = false;
        }
        else if ((SONAR_NO_ECHO == next_st) ||
                 (!sonar_same_obs_depth(g_coarse_angles[idx + 1u],
                                        next_cm,
                                        center_cm)))
        {
            int32_t refined_ang = 0;

            p_profile->b_is_left_edge_valid = sonar_refine_edge(
                g_coarse_angles[idx],
                g_coarse_angles[idx + 1u],
                center_cm,
                &refined_ang,
                p_left_cm,
                p_profile) ? 1u : 0u;
            p_profile->left_edge_deg_raw =
                robot_tasks_pack_signed(refined_ang, NAV_ANGLE_BIT_WIDTH);
            b_left_active = false;
        }
        else
        {
            /* Both adjacent left-side rays hit same front face; continue. */
        }
    }
}

/**
 * @brief Compute left/right body offsets and apparent width from refined edges.
 * @param[in,out] p_profile Profile containing refined boundary angles.
 * @param[in]     left_cm   Projected distance to refined left edge in cm.
 * @param[in]     right_cm  Projected distance to refined right edge in cm.
 */
static void sonar_compute_offsets (navigator_task_obs_profile_t * p_profile,
                                   uint32_t left_cm,
                                   uint32_t right_cm)
{
    int32_t left_ang  = robot_tasks_sign_extend(p_profile->left_edge_deg_raw,
                                                NAV_ANGLE_BIT_WIDTH);
    int32_t right_ang = robot_tasks_sign_extend(p_profile->right_edge_deg_raw,
                                                NAV_ANGLE_BIT_WIDTH);

    if (0u != p_profile->b_is_left_edge_valid)
    {
        int32_t  lat_cm  = sonar_lateral_cm(left_ang, left_cm);
        /* Convert positive signed lateral cm to uint32_t; clamp negative 0. */
        uint32_t pos_lat = (lat_cm > 0) ? ((uint32_t)lat_cm) : 0u;

        p_profile->left_offset_cm     = pos_lat +
                                        SONAR_ROBOT_HALF_WIDTH_CM +
                                        SONAR_WALL_BODY_GAP_CM;
        p_profile->b_is_left_off_valid = 1u;
    }

    if (0u != p_profile->b_is_right_edge_valid)
    {
        int32_t  lat_cm  = sonar_lateral_cm(right_ang, right_cm);
        /* Convert negated negative right lateral offset to uint32_t. */
        uint32_t neg_lat = (lat_cm < 0) ? ((uint32_t)(-lat_cm)) : 0u;

        p_profile->right_offset_cm     = neg_lat +
                                         SONAR_ROBOT_HALF_WIDTH_CM +
                                         SONAR_WALL_BODY_GAP_CM;
        p_profile->b_is_right_off_valid = 1u;
    }

    if ((0u != p_profile->b_is_left_edge_valid) &&
        (0u != p_profile->b_is_right_edge_valid))
    {
        int32_t left_x  = sonar_lateral_cm(left_ang, left_cm);
        int32_t right_x = sonar_lateral_cm(right_ang, right_cm);

        if (left_x > right_x)
        {
            /* Range-safe: left_x > right_x guarantees positive difference. */
            p_profile->estimated_width_cm = (uint32_t)(left_x - right_x);
            p_profile->b_is_width_valid   = 1u;
        }
    }
}

/**
 * @brief Print summary diagnostics for a completed stationary obstacle scan.
 * @param[in] p_profile Populated obstacle profile structure.
 * @param[in] valid_cnt Number of valid coarse rays observed.
 */
static void sonar_log_scan_summary (
    const navigator_task_obs_profile_t * p_profile,
    uint32_t valid_cnt)
{
    /* Cast format literal and coarse valid counts to RTOS UB* and UINT. */
    tm_printf((UB *)"[sonar] Coarse valid: %u/%u\n",
              (UINT)valid_cnt,
              (UINT)SONAR_COARSE_COUNT);
    /* Cast format literal and validity bit-fields to RTOS UB* and UINT. */
    tm_printf((UB *)"[sonar] Clearance profile=%u Right=%u Left=%u\n",
              (UINT)p_profile->b_is_profile_valid,
              (UINT)p_profile->b_is_right_edge_valid,
              (UINT)p_profile->b_is_left_edge_valid);
    /* Cast format literal and width bracket fields to RTOS UB* and UINT. */
    tm_printf((UB *)"[width] Scan front=%u cm; left edge=%u [%u..%u] mm; "
                    "right edge=%u [%u..%u] mm\n",
              (UINT)p_profile->front_cm,
              (UINT)p_profile->measured_width.b_is_left_valid,
              (UINT)p_profile->measured_width.left_min_mm,
              (UINT)p_profile->measured_width.left_max_mm,
              (UINT)p_profile->measured_width.b_is_right_valid,
              (UINT)p_profile->measured_width.right_min_mm,
              (UINT)p_profile->measured_width.right_max_mm);
}

/**
 * @brief Run full stationary coarse/fine obstacle profile and width bracket.
 * @param[out] p_profile Populated obstacle profile structure.
 * @return True if profile is valid and servo returned to center cleanly.
 */
static bool sonar_scan_obstacle (navigator_task_obs_profile_t * p_profile)
{
    navigator_task_obs_profile_t empty_prof = { 0u };
    uint32_t                     valid_cnt  = 0u;
    bool                         b_is_valid = false;

    *p_profile = empty_prof;

    if (sonar_run_coarse_scan(p_profile, &valid_cnt))
    {
        ultrasonic_task_status_t center_st      =
            navigator_task_get_ray_status(p_profile, SONAR_CENTER_RAY_IDX);
        uint32_t                 center_cm      =
            navigator_task_get_ray_cm(p_profile, SONAR_CENTER_RAY_IDX);
        bool                     b_is_center_ok = (SONAR_OK == center_st);
        uint32_t                 left_cm        = 0u;
        uint32_t                 right_cm       = 0u;
        ER                       servo_err      = E_OK;

        p_profile->front_cm = b_is_center_ok ? center_cm : 0u;
        /* Fit 3-bit ultrasonic status enum into uint8_t telemetry argument. */
        navigator_task_set_sonar_telem(b_is_center_ok,
                                       (uint8_t)center_st,
                                       b_is_center_ok ? center_cm : 0u);

        p_profile->left_scan_cm      =
            navigator_task_get_ray_cm(p_profile, SONAR_LEFT_OUTER_IDX);
        p_profile->right_scan_cm     =
            navigator_task_get_ray_cm(p_profile, SONAR_RIGHT_OUTER_IDX);
        /* Store outer left ray status enum in 3-bit unsigned profile field. */
        p_profile->left_scan_status  =
            (uint32_t)navigator_task_get_ray_status(p_profile,
                                                    SONAR_LEFT_OUTER_IDX);
        /* Store outer right ray status enum in 3-bit unsigned profile field. */
        p_profile->right_scan_status =
            (uint32_t)navigator_task_get_ray_status(p_profile,
                                                    SONAR_RIGHT_OUTER_IDX);

        if (b_is_center_ok)
        {
            sonar_find_edges(p_profile, &left_cm, &right_cm);
        }

        sonar_compute_offsets(p_profile, left_cm, right_cm);

        if ((b_is_center_ok) && (!ultrasonic_task_has_fault()))
        {
            sonar_measure_width(p_profile);
        }

        /* Cast servo center pulse width constant to BSP PWM UW type. */
        servo_err = pwm_set_cc(SONAR_SERVO_PIN, (UW)SONAR_SERVO_CENTER_USEC);

        if (servo_err >= E_OK)
        {
            /* Allow pan servo horn to return to center after scan sweep. */
            (void)tk_dly_tsk(SONAR_SERVO_SETTLE_MSEC);
        }
        else
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SERVO_FAULT);
        }

        b_is_valid                      = ((b_is_center_ok) &&
                                           (!ultrasonic_task_has_fault()) &&
                                           (servo_err >= E_OK));
        p_profile->b_is_profile_valid   = b_is_valid ? 1u : 0u;
        p_profile->b_is_left_clr_valid  = 0u;
        p_profile->b_is_right_clr_valid = 0u;

        sonar_log_scan_summary(p_profile, valid_cnt);
    }

    return b_is_valid;
}

/**
 * @brief Send one event message to Navigator and update no-echo status bit.
 * @param[in] p_msg Navigator message payload.
 * @return True if sent within timeout, false on RTOS send failure.
 */
static bool sonar_send_to_nav (const navigator_task_msg_t * p_msg)
{
    navigator_task_msg_t msg_copy = *p_msg;
    ER                   ret_err  = E_OK;
    bool                 b_is_ok  = true;

    /* Compare 4-bit message type field against MSG_SONAR_DISTANCE enum. */
    if ((uint32_t)MSG_SONAR_DISTANCE == p_msg->type)
    {
        /* Compare 3-bit sonar status field against SONAR_NO_ECHO enum. */
        robot_tasks_flag_assign(
            ROBOT_STATUS_NO_ECHO,
            ((uint32_t)SONAR_NO_ECHO == p_msg->data.sonar_dist.status));
    }
    /* Compare 4-bit message type against probe and tracking enum values. */
    else if (((uint32_t)MSG_SONAR_PROBED == p_msg->type) ||
             ((uint32_t)MSG_TRACKING_OBSTACLE == p_msg->type))
    {
        /* Compare 3-bit probe status field against SONAR_NO_ECHO enum. */
        robot_tasks_flag_assign(
            ROBOT_STATUS_NO_ECHO,
            ((uint32_t)SONAR_NO_ECHO == p_msg->data.sonar_probe.status));
    }
    else
    {
        /* Other message types do not carry sonar status. */
    }

    /* Cast message size and 100 ms timeout to RTOS INT and TMO types. */
    ret_err = tk_snd_mbf(robot_tasks_get_nav_mbf(),
                         &msg_copy,
                         (INT)sizeof(msg_copy),
                         (TMO)SONAR_SEND_TIMEOUT_MSEC);

    if (ret_err < E_OK)
    {
        /* Cast ASCII format literal to uT-Kernel UB pointer. */
        tm_printf((UB *)"[sonar] Failed to send Navigator event: %d\n",
                  ret_err);
        b_is_ok = false;
    }

    return b_is_ok;
}

/**
 * @brief Publish live front sonar distance to telemetry and Navigator.
 * @param[in] dist_cm    Measured front distance in cm.
 * @param[in] b_is_valid True if distance reading is usable.
 * @param[in] status     Ultrasonic measurement status.
 */
static void sonar_send_live_dist (uint32_t dist_cm,
                                  bool b_is_valid,
                                  ultrasonic_task_status_t status)
{
    navigator_task_msg_t sonar_msg = { 0 };

    /* Store MSG_SONAR_DISTANCE enum in 4-bit unsigned message field. */
    sonar_msg.type                       = (uint32_t)MSG_SONAR_DISTANCE;
    /* Range-safe: dist_cm is bounded to 0..400 cm (9 bits). */
    sonar_msg.data.sonar_dist.dist_cm    = (uint16_t)dist_cm;
    sonar_msg.data.sonar_dist.b_is_valid = b_is_valid ? 1u : 0u;
    /* Store 3-bit ultrasonic status enum in unsigned bit-field. */
    sonar_msg.data.sonar_dist.status     = (uint32_t)status;

    robot_tasks_flag_assign(ROBOT_STATUS_NO_ECHO, (SONAR_NO_ECHO == status));
    /* Fit 3-bit ultrasonic status enum into uint8_t telemetry parameter. */
    navigator_task_set_sonar_telem(b_is_valid, (uint8_t)status, dist_cm);

    /* Non-blocking send; cast message struct size to RTOS INT parameter. */
    (void)tk_snd_mbf(robot_tasks_get_nav_mbf(),
                     &sonar_msg,
                     (INT)sizeof(sonar_msg),
                     TMO_POL);
}

/**
 * @brief Perform one stationary probe at front, left (+45), or right (-45).
 * @param[in] flag_bits Event flag pattern selecting probe direction.
 * @return True if probe message was delivered to Navigator.
 */
static bool sonar_handle_probe (UINT flag_bits)
{
    int32_t                  angle_deg = 0;
    uint32_t                 dist_cm   = 0u;
    ultrasonic_task_status_t status    = SONAR_INVALID;
    navigator_task_msg_t     nav_msg   = { 0 };

    if (0u != (flag_bits & SONAR_PROBE_LEFT_FLAG))
    {
        angle_deg = SONAR_ANGLE_LIMIT_DEG;
    }
    else if (0u != (flag_bits & SONAR_PROBE_RIGHT_FLAG))
    {
        angle_deg = -SONAR_ANGLE_LIMIT_DEG;
    }
    else
    {
        angle_deg = 0;
    }

    status = sonar_scan_angle(angle_deg, &dist_cm);

    /* Cast servo center pulse width constant to BSP PWM UW parameter. */
    if (pwm_set_cc(SONAR_SERVO_PIN, (UW)SONAR_SERVO_CENTER_USEC) < E_OK)
    {
        status = SONAR_SERVO_FAULT;
    }
    else
    {
        /* Allow pan servo horn to settle back at 0 deg after probe. */
        (void)tk_dly_tsk(SONAR_SERVO_SETTLE_MSEC);
    }

    /* Store MSG_SONAR_PROBED enum in 4-bit unsigned message field. */
    nav_msg.type = (uint32_t)MSG_SONAR_PROBED;
    /* Range-safe: 7-bit packed signed angle fits in uint8_t bit-field. */
    nav_msg.data.sonar_probe.angle_deg_raw   =
        (uint8_t)robot_tasks_pack_signed(angle_deg, NAV_ANGLE_BIT_WIDTH);
    /* Range-safe: valid dist_cm <= 400 fits in 9-bit uint16_t bit-field. */
    nav_msg.data.sonar_probe.dist_cm         =
        (SONAR_OK == status) ? ((uint16_t)dist_cm) : 0u;
    nav_msg.data.sonar_probe.b_is_valid      = (SONAR_OK == status) ? 1u : 0u;
    /* Store 3-bit ultrasonic status enum in unsigned bit-field. */
    nav_msg.data.sonar_probe.status          = (uint32_t)status;
    nav_msg.data.sonar_probe.sampled_at_usec = robot_tasks_time_usec();

    /* Fit 3-bit ultrasonic status enum into uint8_t telemetry parameter. */
    navigator_task_set_sonar_telem((0u != nav_msg.data.sonar_probe.b_is_valid),
                                   (uint8_t)status,
                                   dist_cm);
    /* Fault bit and E-stop are latched if status is a hardware fault. */
    (void)sonar_handle_hw_fault(status);

    /* Cast format literal, probe angle, and distance to UB*, INT, UINT. */
    tm_printf((UB *)"[sonar] probe %d: %u cm valid=%u\n",
              (INT)angle_deg,
              (UINT)nav_msg.data.sonar_probe.dist_cm,
              (UINT)nav_msg.data.sonar_probe.b_is_valid);

    return sonar_send_to_nav(&nav_msg);
}

/**
 * @brief Acquire and stream one side-tracking ultrasonic sample to Navigator.
 * @param[in] p_rt Sonar runtime state holding parked tracking angle.
 * @return True if no fatal hardware fault occurred.
 */
static bool sonar_handle_track_tick (const ultrasonic_task_runtime_t * p_rt)
{
    uint32_t                 dist_cm      = 0u;
    ultrasonic_task_status_t status       = sonar_read_distance_cm(&dist_cm);
    bool                     b_has_failed = sonar_handle_hw_fault(status);
    navigator_task_msg_t     nav_msg      = { 0 };

    /* Store MSG_TRACKING_OBSTACLE enum in 4-bit unsigned message field. */
    nav_msg.type                             = (uint32_t)MSG_TRACKING_OBSTACLE;
    /* Store 3-bit ultrasonic status enum in unsigned bit-field. */
    nav_msg.data.sonar_probe.status          = (uint32_t)status;
    nav_msg.data.sonar_probe.b_is_valid      = (SONAR_OK == status) ? 1u : 0u;
    /* Range-safe: dist_cm is bounded to 0..400 cm. */
    nav_msg.data.sonar_probe.dist_cm         = (uint16_t)dist_cm;
    /* Range-safe: tracking_ang_raw is already a 7-bit packed signed angle. */
    nav_msg.data.sonar_probe.angle_deg_raw   = (uint8_t)p_rt->tracking_ang_raw;
    nav_msg.data.sonar_probe.sampled_at_usec = robot_tasks_time_usec();

    robot_tasks_flag_assign(ROBOT_STATUS_NO_ECHO, (SONAR_NO_ECHO == status));
    /* Non-blocking send; cast message struct size to RTOS INT parameter. */
    (void)tk_snd_mbf(robot_tasks_get_nav_mbf(),
                     &nav_msg,
                     (INT)sizeof(nav_msg),
                     TMO_POL);

    return (!b_has_failed);
}

/**
 * @brief Park servo at left (+45) or right (-45) for continuous side tracking.
 * @param[in,out] p_rt      Sonar runtime state updated with tracking angle.
 * @param[in]     flag_bits Event flag pattern selecting left or right side.
 * @return True on success, false if servo PWM update failed.
 */
static bool sonar_handle_track_req (ultrasonic_task_runtime_t * p_rt,
                                    UINT flag_bits)
{
    int32_t target_ang = (0u != (flag_bits & SONAR_TRACK_LEFT_FLAG))
                         ? SONAR_ANGLE_LIMIT_DEG
                         : (-SONAR_ANGLE_LIMIT_DEG);
    int32_t cur_ang    = robot_tasks_sign_extend(p_rt->tracking_ang_raw,
                                                 NAV_ANGLE_BIT_WIDTH);
    bool    b_is_ok    = true;

    if ((0u == p_rt->b_is_tracking) || (target_ang != cur_ang))
    {
        /* Cast servo tracking pulse width in microseconds to BSP UW type. */
        if (pwm_set_cc(SONAR_SERVO_PIN,
                       (UW)sonar_servo_pulse_usec(target_ang)) < E_OK)
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SERVO_FAULT);
            b_is_ok = false;
        }
        else
        {
            /* Allow pan servo horn to settle at +/-45 deg tracking angle. */
            (void)tk_dly_tsk(SONAR_SERVO_SETTLE_MSEC);
        }
    }

    if (b_is_ok)
    {
        p_rt->tracking_ang_raw =
            robot_tasks_pack_signed(target_ang, NAV_ANGLE_BIT_WIDTH);
        p_rt->b_is_tracking    = 1u;
    }

    return b_is_ok;
}

/**
 * @brief Dispatch one event flag pattern received inside the bypass loop.
 * @param[in,out] p_rt            Sonar runtime state.
 * @param[in]     pattern         Event flag bitmask from tk_wai_flg.
 * @param[in,out] pb_is_in_bypass Set false when bypass ends or faults.
 * @return True if no fatal hardware or messaging fault occurred.
 */
static bool sonar_dispatch_bypass_flg (ultrasonic_task_runtime_t * p_rt,
                                       UINT pattern,
                                       bool * pb_is_in_bypass)
{
    const UINT probe_mask   =
        SONAR_PROBE_FRONT_FLAG |
        SONAR_PROBE_LEFT_FLAG |
        SONAR_PROBE_RIGHT_FLAG;
    const UINT track_mask   =
        SONAR_TRACK_LEFT_FLAG |
        SONAR_TRACK_RIGHT_FLAG;
    bool       b_is_task_ok = true;

    if (0u != (pattern & SONAR_RESUME_FLAG))
    {
        /* Cast servo center pulse width constant to BSP PWM UW type. */
        if (pwm_set_cc(SONAR_SERVO_PIN, (UW)SONAR_SERVO_CENTER_USEC) < E_OK)
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SERVO_FAULT);
            b_is_task_ok = false;
        }
        else
        {
            /* Allow pan servo horn to settle at 0 deg before resuming. */
            (void)tk_dly_tsk(SONAR_SERVO_SETTLE_MSEC);
        }

        *pb_is_in_bypass = false;
    }
    else if (0u != (pattern & SONAR_SCAN_REQUEST))
    {
        navigator_task_msg_t nav_msg = { 0 };

        p_rt->b_is_tracking = 0u;
        /* Store MSG_OBSTACLE_PROFILED enum in 4-bit unsigned message field. */
        nav_msg.type        = (uint32_t)MSG_OBSTACLE_PROFILED;
        /* Profile validity flag is stored in obs_profile.b_is_profile_valid. */
        (void)sonar_scan_obstacle(&nav_msg.data.obs_profile);

        if (!sonar_send_to_nav(&nav_msg))
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SENSOR_FAULT);
            b_is_task_ok     = false;
            *pb_is_in_bypass = false;
        }
    }
    else if (0u != (pattern & probe_mask))
    {
        p_rt->b_is_tracking = 0u;

        if (!sonar_handle_probe(pattern))
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SENSOR_FAULT);
            b_is_task_ok     = false;
            *pb_is_in_bypass = false;
        }
    }
    else if (0u != (pattern & track_mask))
    {
        if (!sonar_handle_track_req(p_rt, pattern))
        {
            b_is_task_ok     = false;
            *pb_is_in_bypass = false;
        }
    }
    else
    {
        /* No matching flag bits set; continue waiting. */
    }

    return b_is_task_ok;
}

/**
 * @brief Service Navigator scan, probe, tracking, and resume event requests.
 * @param[in,out] p_rt Sonar runtime state.
 * @return True if SONAR_RESUME_FLAG finished bypass cleanly, false on fault.
 */
static bool sonar_run_bypass_loop (ultrasonic_task_runtime_t * p_rt)
{
    const UINT request_mask   =
        SONAR_RESUME_FLAG |
        SONAR_SCAN_REQUEST |
        SONAR_TRACK_LEFT_FLAG |
        SONAR_TRACK_RIGHT_FLAG |
        SONAR_PROBE_FRONT_FLAG |
        SONAR_PROBE_LEFT_FLAG |
        SONAR_PROBE_RIGHT_FLAG;
    bool       b_is_in_bypass = true;
    bool       b_is_task_ok   = true;

    p_rt->b_is_tracking    = 0u;
    p_rt->tracking_ang_raw = 0u;

    while (b_is_in_bypass)
    {
        UINT pattern = 0u;
        /* Cast 60 ms tracking poll interval to RTOS TMO timeout parameter. */
        TMO  timeout = (0u != p_rt->b_is_tracking)
                       ? ((TMO)SONAR_PING_DELAY_MSEC)
                       : TMO_FEVR;
        ER   ret_err = tk_wai_flg(robot_tasks_get_ranger_flg(),
                                  request_mask,
                                  TWF_ORW | TWF_BITCLR,
                                  &pattern,
                                  timeout);

        if ((E_TMOUT == ret_err) && (0u != p_rt->b_is_tracking))
        {
            if (!sonar_handle_track_tick(p_rt))
            {
                b_is_task_ok   = false;
                b_is_in_bypass = false;
            }
        }
        else if (ret_err < E_OK)
        {
            /* Fault bit and E-stop are latched by sonar_handle_hw_fault(). */
            (void)sonar_handle_hw_fault(SONAR_SENSOR_FAULT);
            b_is_task_ok   = false;
            b_is_in_bypass = false;
        }
        else
        {
            b_is_task_ok = sonar_dispatch_bypass_flg(p_rt,
                                                     pattern,
                                                     &b_is_in_bypass);
        }
    }

    return b_is_task_ok;
}

/*** end of file ***/
