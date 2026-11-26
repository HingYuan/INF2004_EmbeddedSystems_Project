/**
 * @file robot_tasks.c
 * @brief Shared synchronization, bit-field helpers, and robot startup.
 *
 * BARR-C Deviation (Rules 1.1.c, 2.1.c.ii, 5.2.a, 5.4.a, 6.1.e, 6.1.i, 6.5.b):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: ARMv6-M DMB and IPSR inline assembly instructions provide SMP
 *   memory ordering and unhandled-IRQ vector identification on RP2040;
 *   Default_Handler overrides the weak vector symbol required by vector_tbl.c;
 *   serial startup diagnostics use uT-Kernel tm_printf(const UB *, ...).
 */

#include "robot_tasks.h"

#include <math.h>

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
#if defined(TM_TEST_ENABLE) && (0 != TM_TEST_ENABLE)
#include "test_report.h"
#endif

#define ROBOT_PERIPH_TASK_STACK         (4096u)
#define ROBOT_STARTUP_DELAY_MSEC        (1000u)
#define ROBOT_BARCODE_QUEUE_CAP         (20u)
#define ROBOT_NAV_QUEUE_CAP             (64u)
#define ROBOT_MOTOR_QUEUE_CAP           (10u)
#define ROBOT_NAV_TASK_PRIO             (1)
#define ROBOT_SENSOR_TASK_PRIO          (2)
#define ROBOT_DIAG_TASK_PRIO            (10)
#define ROBOT_BITS_PER_BYTE             (8u)
#define ROBOT_MAX_U32_BITS              (32u)
#define ROBOT_FULL_U32_MASK             (0xFFFFFFFFu)
#define ROBOT_TENTHS_SCALE_F32          (10.0f)
#define ROBOT_HALF_ROUND_F32            (0.5f)
#define ROBOT_ARMV6M_IRQ_BASE           (16u)
#define ROBOT_RP2040_IRQ_COUNT          (32u)

#define ROBOT_BARCODE_MBF_WORDS \
    (ROBOT_BARCODE_QUEUE_CAP * \
     (((sizeof(ir_task_bar_record_t) + sizeof(uint32_t) - 1u) / \
       sizeof(uint32_t)) + 1u))
#define ROBOT_NAV_MBF_WORDS \
    (ROBOT_NAV_QUEUE_CAP * \
     (((sizeof(navigator_task_msg_t) + sizeof(uint32_t) - 1u) / \
       sizeof(uint32_t)) + 1u))
#define ROBOT_MOTOR_MBF_WORDS \
    (ROBOT_MOTOR_QUEUE_CAP * \
     (((sizeof(motor_task_cmd_t) + sizeof(uint32_t) - 1u) / \
       sizeof(uint32_t)) + 1u))

#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
ROBOT_TASKS_STATIC_ASSERT(little_endian,
                          (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__));
#endif
ROBOT_TASKS_STATIC_ASSERT(float32_size, (4u == sizeof(float32_t)));

static volatile uint32_t g_robot_flags   = 0u;
static ID                gh_barcode_mbf  = 0;
static ID                gh_nav_mbf      = 0;
static ID                gh_motor_mbf    = 0;
static ID                gh_ranger_flg   = 0;

#if TK_SUPPORT_SMP
static volatile T_SPLOCK g_robot_lock    = 0u;
#endif

/* Static RTOS backing pools: kernel objects require persistent storage. */
static uint32_t          g_barcode_mbf_buf[ROBOT_BARCODE_MBF_WORDS] = { 0u };
static uint32_t          g_nav_mbf_buf[ROBOT_NAV_MBF_WORDS]         = { 0u };
static uint32_t          g_motor_mbf_buf[ROBOT_MOTOR_MBF_WORDS]     = { 0u };

static ER robot_tasks_create_ipc(void);
static ER robot_tasks_launch_one(const robot_tasks_cfg_t * p_cfg);
static void robot_tasks_unhandled_isr(void);

/**
 * @brief Vector-table fallback trampoline for unexpected external interrupts.
 */
void Default_Handler(void);

/**
 * @brief Acquire the common interrupt-preserving application data lock.
 * @return Saved interrupt state for robot_tasks_data_unlock().
 */
UINT robot_tasks_data_lock (void)
{
    UINT saved_state = 0u;

#if TK_SUPPORT_SMP
    /* Range-safe: discard ER return; g_robot_lock is always valid. */
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
    /* Range-safe: discard ER return; saved_state comes from data_lock(). */
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

    /* ARMv6-M DMB orders subsequent shared-memory reads after flag load. */
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
 * @param[in] mask         Flag bitmask to update.
 * @param[in] b_is_enabled True to set mask, false to clear mask.
 */
void robot_tasks_flag_assign (uint32_t mask, bool b_is_enabled)
{
    uint32_t set_mask   = b_is_enabled ? mask : 0u;
    uint32_t clear_mask = b_is_enabled ? 0u : mask;

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
    /* Range-safe: GPIO pin constants 0..28 widen lossless to uint32_t. */
    const uint32_t packed_ir_raw =
        (((uint32_t)IR_CENTER_PIN) << 8u) | ((uint32_t)IR_RIGHT_PIN);
    /* Range-safe: GPIO pin constants 0..28 shifted <= 16 bits fit uint32_t. */
    const uint32_t packed_sonar_raw =
        (((uint32_t)SONAR_TRIG_PIN) << 16u) |
        (((uint32_t)SONAR_ECHO_PIN) << 8u);
    /* Range-safe: packed GPIO pin bitmasks <= 0x1CFF00 fit positive INT. */
    /* Range-safe: 24-bit/12-bit/5-bit GPIO pin masks fit positive INT. */
    const robot_tasks_cfg_t task_table[] =
    {
        {
            ultrasonic_task_run_task,
            ROBOT_SENSOR_TASK_PRIO,
            (INT)packed_sonar_raw, /* 24-bit pin triplet fits positive INT */
            TP_PRC1,
            "Ultrasonic"
        },
        {
            ir_task_sensor_task,
            ROBOT_SENSOR_TASK_PRIO,
            (INT)packed_ir_raw, /* 12-bit ADC pin pair fits positive INT */
            TP_PRC1,
            "IR line sensors"
        },
        {
            ir_task_barcode_task,
            ROBOT_SENSOR_TASK_PRIO,
            (INT)IR_LEFT_PIN, /* GPIO pin 26 fits positive INT */
            TP_PRC1,
            "IR barcode sensor"
        },
        {
            ir_task_decoder_task,
            ROBOT_SENSOR_TASK_PRIO,
            0,
            TP_PRC1,
            "IR barcode decoder"
        },
        {
            motor_task_run_task,
            ROBOT_SENSOR_TASK_PRIO,
            0,
            TP_PRC1,
            "Motors"
        },
        {
            imu_task_run_task,
            ROBOT_SENSOR_TASK_PRIO,
            0,
            TP_PRC1,
            "IMU"
        },
        {
            navigator_task_run_task,
            ROBOT_NAV_TASK_PRIO,
            0,
            TP_PRC1,
            "Navigator"
        },
        {
            free_time_diagnostic_run_task,
            ROBOT_DIAG_TASK_PRIO,
            0,
            TP_PRC1,
            "Diagnostic task"
        }
    };
    /* Range-safe: static 8-element array count fits in uint32_t. */
    const uint32_t task_count =
        (uint32_t)(sizeof(task_table) / sizeof(task_table[0]));
    ER             ret_err    = robot_tasks_create_ipc();

    if (E_OK == ret_err)
    {
#if defined(TM_TEST_ENABLE) && (0 != TM_TEST_ENABLE)
        test_report_boot_run();
#endif
        /* Range-safe: startup delay return status is non-critical here. */
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
 * @brief Read a little-endian bit field from a byte buffer.
 * @param[in] p_bytes    Source byte array.
 * @param[in] bit_offset Starting bit index.
 * @param[in] bit_width  Field width in bits (1..32).
 * @return Extracted unsigned bit field value.
 */
uint32_t robot_tasks_bits_get (const uint8_t * p_bytes,
                               uint32_t bit_offset,
                               uint32_t bit_width)
{
    uint32_t result = 0u;

    for (uint32_t bit_idx = 0u; bit_idx < bit_width; bit_idx++)
    {
        uint32_t total_bit = bit_offset + bit_idx;
        /* Range-safe: uint8_t (0..255) widens lossless to uint32_t. */
        uint32_t byte_val  = (uint32_t)p_bytes[total_bit / ROBOT_BITS_PER_BYTE];
        uint32_t bit_val   =
            (byte_val >> (total_bit % ROBOT_BITS_PER_BYTE)) & 1u;

        result |= (bit_val << bit_idx);
    }

    return result;
}

/**
 * @brief Write a little-endian bit field into a byte buffer.
 * @param[in,out] p_bytes    Destination byte array.
 * @param[in]     bit_offset Starting bit index.
 * @param[in]     bit_width  Field width in bits (1..32).
 * @param[in]     bit_value  Unsigned value to store.
 */
void robot_tasks_bits_set (uint8_t * p_bytes,
                           uint32_t bit_offset,
                           uint32_t bit_width,
                           uint32_t bit_value)
{
    for (uint32_t bit_idx = 0u; bit_idx < bit_width; bit_idx++)
    {
        uint32_t  total_bit = bit_offset + bit_idx;
        uint32_t  bit_pos   = total_bit % ROBOT_BITS_PER_BYTE;
        uint8_t * p_byte    = &p_bytes[total_bit / ROBOT_BITS_PER_BYTE];
        /* Range-safe: uint8_t dereference widens lossless to uint32_t. */
        uint32_t  cleared   = ((uint32_t)(*p_byte)) & (~(1u << bit_pos));
        uint32_t  inserted  = ((bit_value >> bit_idx) & 1u) << bit_pos;

        /* Range-safe: cleared | inserted fits within 8 bits (0..255). */
        *p_byte = (uint8_t)(cleared | inserted);
    }
}

/**
 * @brief Sign-extend an N-bit two's-complement value to int32_t.
 * @param[in] raw_value Unsigned N-bit field value.
 * @param[in] bit_width Field width in bits (1..32).
 * @return Sign-extended 32-bit signed integer.
 */
int32_t robot_tasks_sign_extend (uint32_t raw_value, uint32_t bit_width)
{
    uint32_t sign_bit = 1u << (bit_width - 1u);
    uint32_t extended = (raw_value ^ sign_bit) - sign_bit;

    /* Range-safe: two's-complement bit pattern maps directly on ARMv6-M. */
    return (int32_t)extended;
}

/**
 * @brief Pack a signed integer into an unsigned N-bit two's-complement field.
 * @param[in] signed_val Signed value fitting within bit_width bits.
 * @param[in] bit_width  Field width in bits (1..32).
 * @return Masked unsigned bit pattern suitable for unsigned bit-fields.
 */
uint32_t robot_tasks_pack_signed (int32_t signed_val, uint32_t bit_width)
{
    uint32_t mask = (ROBOT_MAX_U32_BITS == bit_width)
                    ? ROBOT_FULL_U32_MASK
                    : ((1u << bit_width) - 1u);

    /* Range-safe: C99 modulo-2^32 unsigned conversion preserves low bits. */
    return (((uint32_t)signed_val) & mask);
}

/**
 * @brief Test whether an IEEE-754 float32_t is finite (not NaN or Inf).
 * @param[in] float_val Single-precision floating-point value to inspect.
 * @return True if finite, false if NaN or infinite.
 */
bool robot_tasks_is_finite_f32 (float32_t float_val)
{
    return (0 != isfinite(float_val));
}

/**
 * @brief Scale and clamp a float32_t into tenths fixed-point int16_t.
 * @param[in] float_val Input floating-point value.
 * @param[in] min_val   Lower clamp bound in tenths.
 * @param[in] max_val   Upper clamp bound in tenths.
 * @return Rounded and clamped fixed-point value.
 */
int16_t robot_tasks_scale_tenths (float32_t float_val,
                                  int16_t min_val,
                                  int16_t max_val)
{
    int16_t   result = min_val;
    float32_t scaled = float_val * ROBOT_TENTHS_SCALE_F32;
    /* Range-safe: int16_t (-32768..32767) converts exactly to float32_t. */
    float32_t min_f  = (float32_t)min_val;
    /* Range-safe: int16_t (-32768..32767) converts exactly to float32_t. */
    float32_t max_f  = (float32_t)max_val;

    if (0 == isfinite(scaled))
    {
        result = (scaled > 0.0f) ? max_val : min_val;
    }
    else if (scaled < min_f)
    {
        result = min_val;
    }
    else if (scaled >= max_f)
    {
        result = max_val;
    }
    else
    {
        float32_t rounded = (scaled < 0.0f)
                            ? (scaled - ROBOT_HALF_ROUND_F32)
                            : (scaled + ROBOT_HALF_ROUND_F32);

        if (0 != isfinite(rounded))
        {
            /* Range-safe: rounded is bounded within [min_val, max_val]. */
            result = (int16_t)rounded;
        }
    }

    return result;
}

/**
 * @brief Vector-table fallback trampoline for unexpected external interrupts.
 */
void Default_Handler (void)
{
    robot_tasks_unhandled_isr();
}

/**
 * @brief Create the event flag and message buffers shared by robot tasks.
 * @return E_OK on success, or negative uT-Kernel error code on failure.
 */
static ER robot_tasks_create_ipc (void)
{
    ER     ret_err = E_OK;
    T_CFLG cflg    = { 0 };

    cflg.flgatr   = TA_TFIFO;
    gh_ranger_flg = tk_cre_flg(&cflg);

    if (gh_ranger_flg <= E_OK)
    {
        /* Range-safe: ASCII literal bytes map identically to const UB *. */
        tm_printf((const UB *)"[init] create ranger event flag failed, er=%d\n",
                  gh_ranger_flg);
        /* Range-safe: negative ID holds a standard uT-Kernel ER error code. */
        ret_err = (ER)gh_ranger_flg;
    }

    if (E_OK == ret_err)
    {
        T_CMBF cmbf = { 0 };

        cmbf.mbfatr    = TA_TFIFO;
        /* Range-safe: struct size (4 bytes) fits positive 32-bit INT. */
        cmbf.maxmsz    = (INT)sizeof(ir_task_bar_record_t);
        /* Range-safe: static pool byte size fits positive 32-bit SZ. */
        cmbf.bufsz     = (SZ)sizeof(g_barcode_mbf_buf);
        cmbf.bufptr    = g_barcode_mbf_buf;
        gh_barcode_mbf = tk_cre_mbf(&cmbf);

        if (gh_barcode_mbf <= E_OK)
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[init] create barcode mbf failed, er=%d\n",
                      gh_barcode_mbf);
            /* Range-safe: negative ID holds a uT-Kernel ER error code. */
            ret_err = (ER)gh_barcode_mbf;
        }
    }

    if (E_OK == ret_err)
    {
        T_CMBF cmbf = { 0 };

        cmbf.mbfatr = TA_TFIFO;
        /* Range-safe: struct size (34 bytes) fits positive 32-bit INT. */
        cmbf.maxmsz = (INT)sizeof(navigator_task_msg_t);
        /* Range-safe: static pool byte size fits positive 32-bit SZ. */
        cmbf.bufsz  = (SZ)sizeof(g_nav_mbf_buf);
        cmbf.bufptr = g_nav_mbf_buf;
        gh_nav_mbf  = tk_cre_mbf(&cmbf);

        if (gh_nav_mbf <= E_OK)
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[init] create navigator mbf failed, er=%d\n",
                      gh_nav_mbf);
            /* Range-safe: negative ID holds a uT-Kernel ER error code. */
            ret_err = (ER)gh_nav_mbf;
        }
    }

    if (E_OK == ret_err)
    {
        T_CMBF cmbf = { 0 };

        cmbf.mbfatr  = TA_TFIFO;
        /* Range-safe: union size (4 bytes) fits positive 32-bit INT. */
        cmbf.maxmsz  = (INT)sizeof(motor_task_cmd_t);
        /* Range-safe: static pool byte size fits positive 32-bit SZ. */
        cmbf.bufsz   = (SZ)sizeof(g_motor_mbf_buf);
        cmbf.bufptr  = g_motor_mbf_buf;
        gh_motor_mbf = tk_cre_mbf(&cmbf);

        if (gh_motor_mbf <= E_OK)
        {
            /* Range-safe: ASCII literal bytes map identically to const UB *. */
            tm_printf((const UB *)"[init] create motor mbf failed, er=%d\n",
                      gh_motor_mbf);
            /* Range-safe: negative ID holds a uT-Kernel ER error code. */
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
        /* Range-safe: uint32_t processor mask maps 1-to-1 to 32-bit UW. */
        ctsk.assprc  = (UW)p_cfg->assprc;
    }
#endif

    ctsk.itskpri = p_cfg->priority;
    /* Range-safe: 4096-byte stack constant fits positive 32-bit SZ. */
    ctsk.stksz   = (SZ)ROBOT_PERIPH_TASK_STACK;
    ctsk.task    = p_cfg->p_fn_task;

    h_task = tk_cre_tsk(&ctsk);

    if (h_task <= E_OK)
    {
        /* Range-safe: ASCII strings map identically to const UB *. */
        tm_printf((const UB *)"[init] create %s failed, er=%d\n",
                  (const UB *)p_cfg->p_name,
                  h_task);
        /* Range-safe: negative ID holds a uT-Kernel ER error code. */
        ret_err = (ER)h_task;
    }
    else
    {
        ret_err = tk_sta_tsk(h_task, p_cfg->stacd);

        if (ret_err < E_OK)
        {
            /* Range-safe: ASCII strings map identically to const UB *. */
            tm_printf((const UB *)"[init] start %s failed, er=%d\n",
                      (const UB *)p_cfg->p_name,
                      ret_err);
            /* Range-safe: best-effort cleanup of unstarted task object. */
            (void)tk_del_tsk(h_task);
        }
    }

    return ret_err;
}

/**
 * @brief Stub ISR disabling unexpected NVIC interrupt sources (Rule 6.5.d).
 */
static void robot_tasks_unhandled_isr (void)
{
    uint32_t ipsr_val = 0u;

    __asm__ volatile ("mrs %0, ipsr" : "=r" (ipsr_val));

    if (ipsr_val >= ROBOT_ARMV6M_IRQ_BASE)
    {
        uint32_t irq_num = ipsr_val - ROBOT_ARMV6M_IRQ_BASE;

        if (irq_num < ROBOT_RP2040_IRQ_COUNT)
        {
            /* Range-safe: irq_num in 0..31 fits unsigned UINT parameter. */
            DisableInt((UINT)irq_num);
        }
    }

    g_robot_flags |= ROBOT_ESTOP_REQUEST;

    for (;;)
    {
        /* Halt safely after masking unexpected interrupt source. */
    }
}

/*** end of file ***/
