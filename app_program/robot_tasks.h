/**
 * @file robot_tasks.h
 * @brief Shared synchronization, bit-packing utilities, and task startup API.
 *
 * NOTE: Compression baseline preserves compact packed internal structs and
 * explicit little-endian bit streams for sonar rays, routes, and MQTT v1.
 * Packed struct layout is internal to this GCC firmware, never the MQTT wire
 * ABI; MQTT uses explicit bit-stream shifts and masks (see mqtt_client.h).
 *
 * BARR-C Deviation (Rules 1.1.a, 1.1.c, 2.1.c.ii, 5.2.a):
 *   Approver: Project Team (INF2004).
 *   Rationale:
 *   1. Rule 1.1.a / 1.1.c: Build requires -std=gnu11 for Pico SDK headers;
 *      packed internal structs use ROBOT_TASKS_PACKED to save ~7.8 KB RAM on
 *      RP2040; robot_tasks_flags_get() uses an ARMv6-M DMB memory barrier.
 *   2. Rule 2.1.c.ii: pico_rp2040.mk defines -DNDEBUG for CONSOLE=usb_cdc
 *      builds, so intentional operator serial telemetry remains active while
 *      debug-only checks and diagnostics are wrapped in #ifndef NDEBUG.
 *   3. Rule 5.2.a: uT-Kernel 3.0 API boundaries retain native RTOS typedefs
 *      (INT, UINT, ER, ID, SZ, TMO) to avoid incompatible-pointer diagnostics
 *      against kernel headers where INT is int and int32_t is long on ARM.
 */

#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#include <tk/typedef.h>

/* Localized compiler attribute for RAM-compressed internal structs (1.1.c). */
#define ROBOT_TASKS_PACKED              __attribute__((packed))

/* C99-portable compile-time assertion (Rules 1.1.a, 5.5.a, 5.5.b). */
#define ROBOT_TASKS_STATIC_ASSERT(name, expr) \
    typedef uint8_t robot_tasks_sa_##name[(expr) ? 1 : -1]

/* Verify RP2040 little-endian bit-field layout assumption (Rule 5.5.b). */
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
ROBOT_TASKS_STATIC_ASSERT(little_endian,
                          (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__));
#endif

/* Shared application fault and control flags (one aligned 32-bit word). */
#define ROBOT_FAULT_STALL               (1u << 0u)
#define ROBOT_FAULT_ENCODER             (1u << 1u)
#define ROBOT_FAULT_ESTOP               (1u << 2u)
#define ROBOT_FAULT_SONAR               (1u << 3u)
#define ROBOT_FAULT_SERVO               (1u << 4u)
#define ROBOT_FAULT_IMU_READ            (1u << 5u)
#define ROBOT_FAULT_IMU_TILT            (1u << 6u)
#define ROBOT_STATUS_NO_ECHO            (1u << 7u)
#define ROBOT_FAULT_WIFI                (1u << 8u)
#define ROBOT_FAULT_MQTT                (1u << 9u)
#define ROBOT_STOP_ACK_REQUEST          (1u << 10u)
#define ROBOT_ENCODER_DEGRADED          (1u << 11u)
#define ROBOT_ESTOP_REQUEST             (1u << 12u)
#define ROBOT_FLAG_BITS                 (13u)

/* Fixed-width IEEE-754 single-precision floating-point type (Rule 5.4.b.i). */
typedef float float32_t;

ROBOT_TASKS_STATIC_ASSERT(float32_size, (4u == sizeof(float32_t)));

typedef union
{
    float32_t f32_val;
    uint32_t  u32_bits;
} robot_tasks_f32_bits_t;

typedef void (*robot_tasks_fn_t)(INT stacd, void * p_exinf);

typedef struct
{
    robot_tasks_fn_t fn_task;
    INT              priority;
    INT              stacd;
    uint32_t         assprc;
    const char *     p_name;
} robot_tasks_cfg_t;

/**
 * @brief Acquire the common interrupt-preserving application data lock.
 * @return Saved interrupt state for robot_tasks_data_unlock().
 */
UINT robot_tasks_data_lock (void);

/**
 * @brief Release the common application data lock.
 * @param[in] saved_state Saved interrupt state from robot_tasks_data_lock().
 */
void robot_tasks_data_unlock (UINT saved_state);

/**
 * @brief Read the shared fault and control flag word with acquire barrier.
 * @return Current 32-bit application flag bitmask.
 */
uint32_t robot_tasks_flags_get (void);

/**
 * @brief Atomically set and clear bits in the shared application flag word.
 * @param[in] set_mask   Bits to set.
 * @param[in] clear_mask Bits to clear before applying set_mask.
 */
void robot_tasks_flags_update (uint32_t set_mask, uint32_t clear_mask);

/**
 * @brief Assign a single bitmask in the shared application flag word.
 * @param[in] mask      Flag bitmask to update.
 * @param[in] b_enabled True to set mask, false to clear mask.
 */
void robot_tasks_flag_assign (uint32_t mask, bool b_enabled);

/**
 * @brief Return the RTOS message buffer handle for barcode bar records.
 * @return Message buffer ID handle.
 */
ID robot_tasks_get_barcode_mbf (void);

/**
 * @brief Return the RTOS message buffer handle for Navigator messages.
 * @return Message buffer ID handle.
 */
ID robot_tasks_get_nav_mbf (void);

/**
 * @brief Return the RTOS message buffer handle for motor commands.
 * @return Message buffer ID handle.
 */
ID robot_tasks_get_motor_mbf (void);

/**
 * @brief Return the RTOS event flag handle shared with ultrasonic task.
 * @return Event flag ID handle.
 */
ID robot_tasks_get_ranger_flg (void);

/**
 * @brief Read the RP2040 free-running 1-usec hardware timer low word.
 * @return Current 32-bit microsecond counter value.
 */
uint32_t robot_tasks_time_usec (void);

/**
 * @brief Busy-wait for a bounded microsecond interval using hardware timer.
 * @param[in] delay_usec Microseconds to wait.
 */
void robot_tasks_delay_usec (uint32_t delay_usec);

/**
 * @brief Create shared IPC objects and start all robot peripheral tasks.
 * @return E_OK on success, or negative uT-Kernel error code on failure.
 */
ER robot_tasks_start (void);

/**
 * @brief Read a little-endian bit field from a byte buffer.
 * @param[in] p_bytes    Source byte array.
 * @param[in] bit_offset Starting bit index.
 * @param[in] bit_width  Field width in bits (1..32).
 * @return Extracted unsigned bit field value.
 */
static inline uint32_t robot_tasks_bits_get (const uint8_t * p_bytes,
                                             uint32_t bit_offset,
                                             uint32_t bit_width)
{
    uint32_t result = 0u;

    for (uint32_t bit_idx = 0u; bit_idx < bit_width; bit_idx++)
    {
        uint32_t total_bit = bit_offset + bit_idx;
        uint32_t byte_val  = (uint32_t)p_bytes[total_bit / 8u];
        uint32_t bit_val   = (byte_val >> (total_bit % 8u)) & 1u;

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
static inline void robot_tasks_bits_set (uint8_t * p_bytes,
                                         uint32_t bit_offset,
                                         uint32_t bit_width,
                                         uint32_t bit_value)
{
    for (uint32_t bit_idx = 0u; bit_idx < bit_width; bit_idx++)
    {
        uint32_t  total_bit = bit_offset + bit_idx;
        uint32_t  bit_pos   = total_bit % 8u;
        uint8_t * p_byte    = &p_bytes[total_bit / 8u];
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
static inline int32_t robot_tasks_sign_extend (uint32_t raw_value,
                                               uint32_t bit_width)
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
static inline uint32_t robot_tasks_pack_signed (int32_t signed_val,
                                                uint32_t bit_width)
{
    uint32_t mask = (32u == bit_width)
                    ? 0xFFFFFFFFu
                    : ((1u << bit_width) - 1u);

    /* Range-safe: C99 modulo-2^32 unsigned conversion preserves low bits. */
    return (((uint32_t)signed_val) & mask);
}

/**
 * @brief Test whether an IEEE-754 float32_t is finite (not NaN or Inf).
 * @param[in] float_val Single-precision floating-point value to inspect.
 * @return True if finite, false if NaN or infinite.
 */
static inline bool robot_tasks_is_finite_f32 (float32_t float_val)
{
    robot_tasks_f32_bits_t ieee_bits = { 0.0f };

    ieee_bits.f32_val = float_val;

    return (0x7F800000u != (ieee_bits.u32_bits & 0x7F800000u));
}

/**
 * @brief Convert a float32_t to clamped tenths fixed-point int16_t.
 * @param[in] float_val Input floating-point value.
 * @param[in] min_val   Lower clamp bound in tenths.
 * @param[in] max_val   Upper clamp bound in tenths.
 * @return Rounded and clamped fixed-point value.
 */
static inline int16_t robot_tasks_fixed10 (float32_t float_val,
                                           int16_t min_val,
                                           int16_t max_val)
{
    int16_t   result = min_val;
    float32_t scaled = float_val * 10.0f;

    if (!robot_tasks_is_finite_f32(scaled))
    {
        result = (scaled > 0.0f) ? max_val : min_val;
    }
    else if (scaled < (float32_t)min_val)
    {
        result = min_val;
    }
    else if (scaled >= (float32_t)max_val)
    {
        result = max_val;
    }
    else
    {
        float32_t rounded = (scaled < 0.0f)
                            ? (scaled - 0.5f)
                            : (scaled + 0.5f);

        /* Range-safe: scaled is bounded within [min_val, max_val). */
        result = (int16_t)rounded;
    }

    return result;
}

#endif /* ROBOT_TASKS_H */

/*** end of file ***/
