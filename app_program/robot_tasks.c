/**
 * @file robot_tasks.c
 * @brief Shared synchronization, hardware timer helpers, and robot startup.
 */

#include "robot_tasks.h"

#include <sys/machine.h>
#include <sys/sysdef.h>
#include <tk/syslib.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
#include <tm/tmonitor.h>

#include "imu/imu_task.h"
#include "ir/ir_task.h"
#include "motor/motor_task.h"
#include "navigator/navigator_task.h"
#include "ultrasonic/ultrasonic_task.h"
#include "util/free_time_diagnostic.h"

#define ROBOT_PERIPH_TASK_STACK         (4096u)
#define ROBOT_STARTUP_DELAY_MSEC        (1000u)
#define ROBOT_BARCODE_QUEUE_CAP         (20u)
#define ROBOT_NAV_QUEUE_CAP             (64u)
#define ROBOT_MOTOR_QUEUE_CAP           (10u)

#define ROBOT_BARCODE_MBF_WORDS         \
    (ROBOT_BARCODE_QUEUE_CAP * \
     (((sizeof(ir_task_bar_record_t) + sizeof(uint32_t) - 1u) / \
       sizeof(uint32_t)) + 1u))
#define ROBOT_NAV_MBF_WORDS             \
    (ROBOT_NAV_QUEUE_CAP * \
     (((sizeof(navigator_task_msg_t) + sizeof(uint32_t) - 1u) / \
       sizeof(uint32_t)) + 1u))
#define ROBOT_MOTOR_MBF_WORDS           \
    (ROBOT_MOTOR_QUEUE_CAP * \
     (((sizeof(motor_task_cmd_t) + sizeof(uint32_t) - 1u) / \
       sizeof(uint32_t)) + 1u))

static volatile uint32_t g_robot_flags   = 0u;
static ID                gh_barcode_mbf  = 0;
static ID                gh_nav_mbf      = 0;
static ID                gh_motor_mbf    = 0;
static ID                gh_ranger_flg   = 0;

#if TK_SUPPORT_SMP
static T_SPLOCK          g_robot_lock    = 0u;
#endif

/* Static RTOS backing pools: kernel objects require persistent storage. */
static uint32_t g_barcode_mbf_buf[ROBOT_BARCODE_MBF_WORDS] = { 0u };
static uint32_t g_nav_mbf_buf[ROBOT_NAV_MBF_WORDS]         = { 0u };
static uint32_t g_motor_mbf_buf[ROBOT_MOTOR_MBF_WORDS]     = { 0u };

static ER robot_tasks_create_ipc (void);
static ER robot_tasks_launch_one (const robot_tasks_cfg_t * p_cfg);

/**
 * @brief Acquire the common interrupt-preserving application data lock.
 * @return Saved interrupt state for robot_tasks_data_unlock().
 */
UINT robot_tasks_data_lock (void)
{
    UINT saved_state = 0u;

#if TK_SUPPORT_SMP
    (void)ISpinLock(&g_robot_lock, &saved_state);
#else
    DI(saved_state);
#endif

    return saved_state;
}

/**
 * @brief Release the common application data lock.
 * @param[in] saved_state Saved interrupt state from robot_tasks_data_lock().
 */
void robot_tasks_data_unlock (UINT saved_state)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_robot_lock, saved_state);
#else
    EI(saved_state);
#endif
}

/**
 * @brief Read the shared fault and control flag word with acquire barrier.
 * @return Current 32-bit application flag bitmask.
 */
uint32_t robot_tasks_flags_get (void)
{
    uint32_t flag_val = g_robot_flags;

    /*
     * BARR-C Deviation (Rule 1.1.c):
     *   Approver: Project Team (INF2004).
     *   Rationale: RP2040 SMP dual-core reads require a hardware DMB barrier
     *   after an aligned flag load to order subsequent shared memory reads.
     */
    __asm__ volatile ("dmb" ::: "memory");

    return flag_val;
}

/**
 * @brief Atomically set and clear bits in the shared application flag word.
 * @param[in] set_mask   Bits to set.
 * @param[in] clear_mask Bits to clear before applying set_mask.
 */
void robot_tasks_flags_update (uint32_t set_mask, uint32_t clear_mask)
{
    UINT saved_state = robot_tasks_data_lock();

    g_robot_flags = (g_robot_flags & (~clear_mask)) | set_mask;
    robot_tasks_data_unlock(saved_state);
}

/**
 * @brief Assign a single bitmask in the shared application flag word.
 * @param[in] mask      Flag bitmask to update.
 * @param[in] b_enabled True to set mask, false to clear mask.
 */
void robot_tasks_flag_assign (uint32_t mask, bool b_enabled)
{
    uint32_t set_mask   = b_enabled ? mask : 0u;
    uint32_t clear_mask = b_enabled ? 0u : mask;

    robot_tasks_flags_update(set_mask, clear_mask);
}

/**
 * @brief Return the RTOS message buffer handle for barcode bar records.
 * @return Message buffer ID handle.
 */
ID robot_tasks_get_barcode_mbf (void)
{
    return gh_barcode_mbf;
}

/**
 * @brief Return the RTOS message buffer handle for Navigator messages.
 * @return Message buffer ID handle.
 */
ID robot_tasks_get_nav_mbf (void)
{
    return gh_nav_mbf;
}

/**
 * @brief Return the RTOS message buffer handle for motor commands.
 * @return Message buffer ID handle.
 */
ID robot_tasks_get_motor_mbf (void)
{
    return gh_motor_mbf;
}

/**
 * @brief Return the RTOS event flag handle shared with ultrasonic task.
 * @return Event flag ID handle.
 */
ID robot_tasks_get_ranger_flg (void)
{
    return gh_ranger_flg;
}

/**
 * @brief Read the RP2040 free-running 1-usec hardware timer low word.
 * @return Current 32-bit microsecond counter value.
 */
uint32_t robot_tasks_time_usec (void)
{
    /* Range-safe: TIMER_TIMERAWL is a 32-bit unsigned hardware register. */
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/**
 * @brief Busy-wait for a bounded microsecond interval using hardware timer.
 * @param[in] delay_usec Microseconds to wait.
 */
void robot_tasks_delay_usec (uint32_t delay_usec)
{
    uint32_t start_usec = robot_tasks_time_usec();

    while ((robot_tasks_time_usec() - start_usec) < delay_usec)
    {
        /* Busy-wait until the requested microsecond interval elapses. */
    }
}

/**
 * @brief Create shared IPC objects and start all robot peripheral tasks.
 * @return E_OK on success, or negative uT-Kernel error code on failure.
 */
ER robot_tasks_start (void)
{
    const uint32_t packed_ir_raw =
        (((uint32_t)IR_CENTER_PIN) << 8u) | ((uint32_t)IR_RIGHT_PIN);
    const uint32_t packed_sonar_raw =
        (((uint32_t)SONAR_TRIG_PIN) << 16u) |
        (((uint32_t)SONAR_ECHO_PIN) << 8u);

    /* Range-safe: packed GPIO pin numbers fit in non-negative 32-bit INT. */
    const robot_tasks_cfg_t task_table[] =
    {
        {
            ultrasonic_task,
            2,
            (INT)packed_sonar_raw,
            TP_PRC1,
            "Ultrasonic"
        },
        {
            ir_task_sensor_task,
            2,
            (INT)packed_ir_raw,
            TP_PRC1,
            "IR line sensors"
        },
        {
            ir_task_barcode_task,
            2,
            (INT)IR_LEFT_PIN,
            TP_PRC1,
            "IR barcode sensor"
        },
        {
            ir_task_decoder_task,
            2,
            0,
            TP_PRC1,
            "IR barcode decoder"
        },
        {
            motor_task,
            2,
            0,
            TP_PRC1,
            "Motors"
        },
        {
            imu_task,
            2,
            0,
            TP_PRC1,
            "IMU"
        },
        {
            navigator_task,
            1,
            0,
            TP_PRC1,
            "Navigator"
        },
        {
            free_time_diagnostic_task,
            10,
            0,
            TP_PRC1,
            "Diagnostic task"
        }
    };
    const uint32_t task_count =
        (uint32_t)(sizeof(task_table) / sizeof(task_table[0]));
    ER ret_err = robot_tasks_create_ipc();

    if (E_OK == ret_err)
    {
        (void)tk_dly_tsk(ROBOT_STARTUP_DELAY_MSEC);

        for (uint32_t task_idx = 0u;
             (task_idx < task_count) && (E_OK == ret_err);
             task_idx++)
        {
            ret_err = robot_tasks_launch_one(&task_table[task_idx]);
        }
    }

    return ret_err;
}

/**
 * @brief Create the event flag and message buffers shared by robot tasks.
 * @return E_OK on success, or negative uT-Kernel error code on failure.
 */
static ER robot_tasks_create_ipc (void)
{
    ER     ret_err = E_OK;
    T_CFLG cflg    = { 0 };

    cflg.flgatr = TA_TFIFO;
    gh_ranger_flg = tk_cre_flg(&cflg);

    if (gh_ranger_flg <= E_OK)
    {
        /* Cast required by uT-Kernel tm_printf(const UB *, ...) signature. */
        tm_printf((UB *)"[init] create ranger event flag failed, er=%d\n",
                  gh_ranger_flg);
        ret_err = (ER)gh_ranger_flg;
    }

    if (E_OK == ret_err)
    {
        T_CMBF cmbf = { 0 };

        cmbf.mbfatr = TA_TFIFO;
        cmbf.maxmsz = (INT)sizeof(ir_task_bar_record_t);
        cmbf.bufsz  = (SZ)sizeof(g_barcode_mbf_buf);
        cmbf.bufptr = g_barcode_mbf_buf;
        gh_barcode_mbf = tk_cre_mbf(&cmbf);

        if (gh_barcode_mbf <= E_OK)
        {
            tm_printf((UB *)"[init] create barcode mbf failed, er=%d\n",
                      gh_barcode_mbf);
            ret_err = (ER)gh_barcode_mbf;
        }
    }

    if (E_OK == ret_err)
    {
        T_CMBF cmbf = { 0 };

        cmbf.mbfatr = TA_TFIFO;
        cmbf.maxmsz = (INT)sizeof(navigator_task_msg_t);
        cmbf.bufsz  = (SZ)sizeof(g_nav_mbf_buf);
        cmbf.bufptr = g_nav_mbf_buf;
        gh_nav_mbf = tk_cre_mbf(&cmbf);

        if (gh_nav_mbf <= E_OK)
        {
            tm_printf((UB *)"[init] create navigator mbf failed, er=%d\n",
                      gh_nav_mbf);
            ret_err = (ER)gh_nav_mbf;
        }
    }

    if (E_OK == ret_err)
    {
        T_CMBF cmbf = { 0 };

        cmbf.mbfatr = TA_TFIFO;
        cmbf.maxmsz = (INT)sizeof(motor_task_cmd_t);
        cmbf.bufsz  = (SZ)sizeof(g_motor_mbf_buf);
        cmbf.bufptr = g_motor_mbf_buf;
        gh_motor_mbf = tk_cre_mbf(&cmbf);

        if (gh_motor_mbf <= E_OK)
        {
            tm_printf((UB *)"[init] create motor mbf failed, er=%d\n",
                      gh_motor_mbf);
            ret_err = (ER)gh_motor_mbf;
        }
    }

    return ret_err;
}

/**
 * @brief Create and start a single uT-Kernel task from a descriptor.
 * @param[in] p_cfg Task configuration record.
 * @return E_OK on success, or negative uT-Kernel error code on failure.
 */
static ER robot_tasks_launch_one (const robot_tasks_cfg_t * p_cfg)
{
    ER     ret_err = E_OK;
    T_CTSK ctsk    = { 0 };
    ID     h_task  = 0;

    ctsk.tskatr = TA_HLNG | TA_RNG3;

#if TK_SUPPORT_SMP
    if (0u != p_cfg->assprc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc  = (UW)p_cfg->assprc;
    }
#endif

    ctsk.itskpri = p_cfg->priority;
    ctsk.stksz   = (SZ)ROBOT_PERIPH_TASK_STACK;
    ctsk.task    = p_cfg->fn_task;

    h_task = tk_cre_tsk(&ctsk);

    if (h_task <= E_OK)
    {
        tm_printf((UB *)"[init] create %s failed, er=%d\n",
                  (const UB *)p_cfg->p_name,
                  h_task);
        ret_err = (ER)h_task;
    }
    else
    {
        ret_err = tk_sta_tsk(h_task, p_cfg->stacd);

        if (ret_err < E_OK)
        {
            tm_printf((UB *)"[init] start %s failed, er=%d\n",
                      (const UB *)p_cfg->p_name,
                      ret_err);
            (void)tk_del_tsk(h_task);
        }
    }

    return ret_err;
}

/*** end of file ***/
