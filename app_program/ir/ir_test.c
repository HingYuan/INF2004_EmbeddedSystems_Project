/**
 * @file ir_test.c
 * @brief Subsystem 2 (IR Sensors, Code 39 Barcode & Navigator) unit tests.
 *
 * BARR-C Deviation (Rule 5.2.a):
 *   Approver: Maxwell Menendez (Project Manager, INF2004).
 *   Rationale: uT-Kernel message-buffer APIs require native ID and INT types.
 */

#include "ir_test.h"

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"
#include "../test_report.h"
#include "../util/code39.h"
#include "ir_task.h"

#define IR_TEST_ADC_REF_MV              (3300u)
#define IR_TEST_ADC_MAX_RAW             (4095u)
#define IR_TEST_BARS_PER_CHAR           (10u)
#define IR_TEST_FIRST_ELEM_IDX          (1u)
#define IR_TEST_CODE39_ELEMENTS         (9u)
#define IR_TEST_BAR_QUEUE_CAP           (20u)
#define IR_TEST_BAR_REC_BYTES           (4u)
#define IR_TEST_MAX_DURATION_26BIT      (0x03FFFFFFu)
#define IR_TEST_ALPHA_COUNT             (26u)
#define IR_TEST_SYM_COUNT               (18u)
#define IR_TEST_BIT_BUF_BYTES           (8u)
#define IR_TEST_TELEM_BYTES             (32u)

typedef struct
{
    uint16_t pattern;
    uint8_t  expected;
} ir_test_code39_pair_t;

static navigator_task_ir_state_t ir_test_eval_adc_pair(uint32_t raw_left,
                                                       uint32_t raw_right);
static uint8_t ir_test_decode_bars(const ir_task_bar_record_t * p_bars);
static bool ir_test_check_adc_threshold(void);
static bool ir_test_check_code39_alpha(void);
static bool ir_test_check_code39_digits_sym(void);
static bool ir_test_check_adaptive_window(void);
static bool ir_test_check_bar_mbf_fifo(void);
static bool ir_test_check_route_table(void);
static bool ir_test_check_bit_helpers(void);
static bool ir_test_check_telem_snapshot(void);

/**
 * @brief Execute all automated unit tests (`S2-U01`..`S2-U08`) for Subsystem 2.
 */
void ir_test_run (void)
{
    TEST_CHECK("S2-U01", ir_test_check_adc_threshold());
    TEST_CHECK("S2-U02", ir_test_check_code39_alpha());
    TEST_CHECK("S2-U03", ir_test_check_code39_digits_sym());
    TEST_CHECK("S2-U04", ir_test_check_adaptive_window());
    TEST_CHECK("S2-U05", ir_test_check_bar_mbf_fifo());
    TEST_CHECK("S2-U06", ir_test_check_route_table());
    TEST_CHECK("S2-U07", ir_test_check_bit_helpers());
    TEST_CHECK("S2-U08", ir_test_check_telem_snapshot());
}

/**
 * @brief Scale 12-bit ADC counts to millivolts and classify into 4 IR states.
 * @param[in] raw_left  Raw 12-bit left/center ADC sample (0..4095).
 * @param[in] raw_right Raw 12-bit right ADC sample (0..4095).
 * @return Classified two-sensor line state.
 */
static navigator_task_ir_state_t ir_test_eval_adc_pair (uint32_t raw_left,
                                                        uint32_t raw_right)
{
    /* Range-safe: scaled 12-bit ADC millivolt value lies in [0, 3300]. */
    uint16_t                  left_mv          =
        (uint16_t)((raw_left * IR_TEST_ADC_REF_MV) / IR_TEST_ADC_MAX_RAW);
    /* Range-safe: scaled right ADC millivolt value lies in [0, 3300]. */
    uint16_t                  right_mv         =
        (uint16_t)((raw_right * IR_TEST_ADC_REF_MV) / IR_TEST_ADC_MAX_RAW);
    bool                      b_is_left_black  =
        (left_mv >= IR_ADC_THRESHOLD_MV);
    bool                      b_is_right_black =
        (right_mv >= IR_ADC_THRESHOLD_MV);
    navigator_task_ir_state_t state            = IR_STATE_W_B;

    if ((b_is_left_black) && (b_is_right_black))
    {
        state = IR_STATE_B_B;
    }
    else if ((!b_is_left_black) && (!b_is_right_black))
    {
        state = IR_STATE_W_W;
    }
    else if (b_is_left_black)
    {
        state = IR_STATE_B_W;
    }
    else
    {
        state = IR_STATE_W_B;
    }

    return state;
}

/**
 * @brief Apply 9-element mean duration thresholding and Code 39 lookup.
 * @param[in] p_bars 10-element bar array (index 0 is leading quiet space).
 * @return Decoded ASCII character or '?'.
 */
static uint8_t ir_test_decode_bars (const ir_task_bar_record_t * p_bars)
{
    uint32_t sum_dur      = 0u;
    uint32_t avg_dur      = 0u;
    uint16_t pattern_bits = 0u;

    for (uint32_t idx = IR_TEST_FIRST_ELEM_IDX;
         idx < IR_TEST_BARS_PER_CHAR;
         idx++)
    {
        sum_dur += p_bars[idx].duration;
    }

    avg_dur = sum_dur / IR_TEST_CODE39_ELEMENTS;

    for (uint32_t idx = IR_TEST_FIRST_ELEM_IDX;
         idx < IR_TEST_BARS_PER_CHAR;
         idx++)
    {
        if (p_bars[idx].duration > avg_dur)
        {
            uint32_t shift_pos = (IR_TEST_BARS_PER_CHAR - 1u) - idx;

            /* Range-safe: shift_pos is 0..8, fitting in 9-bit uint16_t mask. */
            pattern_bits |= (uint16_t)(1u << shift_pos);
        }
    }

    return code39_decode_pattern(pattern_bits);
}

/**
 * @brief S2-U01: Verify 12-bit ADC scaling and 500 mV W/B state boundaries.
 * @return True if all 4 IR states and exact 499/500 mV edges classify cleanly.
 */
static bool ir_test_check_adc_threshold (void)
{
    /* 620 * 3300 / 4095 = 499 mV (W); 621 * 3300 / 4095 = 500 mV (B). */
    return ((500u == IR_ADC_THRESHOLD_MV) &&
            (IR_STATE_W_W == ir_test_eval_adc_pair(0u, 0u)) &&
            (IR_STATE_W_W == ir_test_eval_adc_pair(620u, 620u)) &&
            (IR_STATE_B_W == ir_test_eval_adc_pair(621u, 620u)) &&
            (IR_STATE_W_B == ir_test_eval_adc_pair(620u, 621u)) &&
            (IR_STATE_B_B == ir_test_eval_adc_pair(621u, 621u)) &&
            (IR_STATE_B_B == ir_test_eval_adc_pair(4095u, 4095u)));
}

/**
 * @brief S2-U02: Verify all 26 ISO/IEC 16388 Code 39 uppercase letters A-Z.
 * @return True if every letter pattern decodes to its exact ASCII byte.
 */
static bool ir_test_check_code39_alpha (void)
{
    /* Cast ASCII uppercase character literals to uint8_t table entries. */
    static const ir_test_code39_pair_t alpha_tbl[IR_TEST_ALPHA_COUNT] =
    {
        { 0x109u, (uint8_t)'A' }, { 0x049u, (uint8_t)'B' },
        { 0x148u, (uint8_t)'C' }, { 0x019u, (uint8_t)'D' },
        { 0x118u, (uint8_t)'E' }, { 0x058u, (uint8_t)'F' },
        { 0x00Du, (uint8_t)'G' }, { 0x10Cu, (uint8_t)'H' },
        { 0x04Cu, (uint8_t)'I' }, { 0x01Cu, (uint8_t)'J' },
        { 0x103u, (uint8_t)'K' }, { 0x043u, (uint8_t)'L' },
        { 0x142u, (uint8_t)'M' }, { 0x013u, (uint8_t)'N' },
        { 0x112u, (uint8_t)'O' }, { 0x052u, (uint8_t)'P' },
        { 0x007u, (uint8_t)'Q' }, { 0x106u, (uint8_t)'R' },
        { 0x046u, (uint8_t)'S' }, { 0x016u, (uint8_t)'T' },
        { 0x181u, (uint8_t)'U' }, { 0x0C1u, (uint8_t)'V' },
        { 0x1C0u, (uint8_t)'W' }, { 0x091u, (uint8_t)'X' },
        { 0x190u, (uint8_t)'Y' }, { 0x0D0u, (uint8_t)'Z' }
    };
    bool b_is_ok = true;

    for (uint32_t idx = 0u; (idx < IR_TEST_ALPHA_COUNT) && (b_is_ok); idx++)
    {
        b_is_ok = (alpha_tbl[idx].expected ==
                   code39_decode_pattern(alpha_tbl[idx].pattern));
    }

    return b_is_ok;
}

/**
 * @brief S2-U03: Verify Code 39 digits, symbols, '*' delimiter, and '?' guard.
 * @return True if all 18 non-letter entries and invalid patterns match.
 */
static bool ir_test_check_code39_digits_sym (void)
{
    /* Cast ASCII digit and symbol literals to uint8_t table entries. */
    static const ir_test_code39_pair_t sym_tbl[IR_TEST_SYM_COUNT] =
    {
        { 0x034u, (uint8_t)'0' }, { 0x121u, (uint8_t)'1' },
        { 0x061u, (uint8_t)'2' }, { 0x160u, (uint8_t)'3' },
        { 0x031u, (uint8_t)'4' }, { 0x130u, (uint8_t)'5' },
        { 0x070u, (uint8_t)'6' }, { 0x025u, (uint8_t)'7' },
        { 0x124u, (uint8_t)'8' }, { 0x064u, (uint8_t)'9' },
        { 0x085u, (uint8_t)'-' }, { 0x184u, (uint8_t)'.' },
        { 0x0C4u, (uint8_t)' ' }, { 0x0A8u, (uint8_t)'$' },
        { 0x0A2u, (uint8_t)'/' }, { 0x08Au, (uint8_t)'+' },
        { 0x02Au, (uint8_t)'%' }, { 0x094u, (uint8_t)'*' }
    };
    bool b_is_ok = true;

    for (uint32_t idx = 0u; (idx < IR_TEST_SYM_COUNT) && (b_is_ok); idx++)
    {
        b_is_ok = (sym_tbl[idx].expected ==
                   code39_decode_pattern(sym_tbl[idx].pattern));
    }

    /* Verify invalid 9-bit and >=10-bit patterns return '?' sentinel byte. */
    b_is_ok = ((b_is_ok) &&
               ((uint8_t)'?' == code39_decode_pattern(0x000u)) &&
               ((uint8_t)'?' == code39_decode_pattern(0x1FFu)) &&
               ((uint8_t)'?' == code39_decode_pattern(0x001u)) &&
               ((uint8_t)'?' == code39_decode_pattern(0x200u)));

    return b_is_ok;
}

/**
 * @brief S2-U04: Verify speed-invariant 10-bar window thresholding.
 * @return True if fast and slow barcode passes decode identically.
 */
static bool ir_test_check_adaptive_window (void)
{
    /* Pattern 0x058 = 'F' (0b001011000); bar[0] is arbitrarily long space. */
    const uint16_t       f_pat                            = 0x058u;
    const uint16_t       s_pat                            = 0x094u;
    ir_task_bar_record_t fast_bars[IR_TEST_BARS_PER_CHAR] = { { 0u, 9999u } };
    ir_task_bar_record_t slow_bars[IR_TEST_BARS_PER_CHAR] = { { 0u, 15u } };
    ir_task_bar_record_t star_bars[IR_TEST_BARS_PER_CHAR] = { { 0u, 500u } };
    ir_task_bar_record_t flat_bars[IR_TEST_BARS_PER_CHAR] = { { 0u, 100u } };

    for (uint32_t idx = IR_TEST_FIRST_ELEM_IDX;
         idx < IR_TEST_BARS_PER_CHAR;
         idx++)
    {
        uint32_t mask = 1u << ((IR_TEST_BARS_PER_CHAR - 1u) - idx);

        fast_bars[idx].b_is_high = idx & 1u;
        fast_bars[idx].duration  = (0u != (f_pat & mask)) ? 60u : 20u;

        slow_bars[idx].b_is_high = idx & 1u;
        slow_bars[idx].duration  = (0u != (f_pat & mask)) ? 450u : 150u;

        star_bars[idx].b_is_high = idx & 1u;
        star_bars[idx].duration  = (0u != (s_pat & mask)) ? 90u : 30u;

        flat_bars[idx].b_is_high = idx & 1u;
        flat_bars[idx].duration  = 50u;
    }

    /* Compare decoded ASCII uint8_t results against 'F', '*', and '?'. */
    return (((uint8_t)'F' == ir_test_decode_bars(fast_bars)) &&
            ((uint8_t)'F' == ir_test_decode_bars(slow_bars)) &&
            ((uint8_t)'*' == ir_test_decode_bars(star_bars)) &&
            ((uint8_t)'?' == ir_test_decode_bars(flat_bars)));
}

/**
 * @brief S2-U05: Verify packed bar record bitfields and 20-entry barcode FIFO.
 * @return True if all 20 records round-trip intact through gh_barcode_mbf.
 */
static bool ir_test_check_bar_mbf_fifo (void)
{
    ID                   h_mbf   = robot_tasks_get_barcode_mbf();
    ir_task_bar_record_t rec     = { 0u, 0u };
    bool                 b_is_ok =
        (IR_TEST_BAR_REC_BYTES == sizeof(ir_task_bar_record_t));

    while (tk_rcv_mbf(h_mbf, &rec, TMO_POL) >= E_OK)
    {
        /* Ensure queue starts empty. */
    }

    for (uint32_t idx = 0u; (idx < IR_TEST_BAR_QUEUE_CAP) && (b_is_ok); idx++)
    {
        ir_task_bar_record_t tx_rec = { 0u, 0u };

        tx_rec.b_is_high = idx & 1u;
        tx_rec.duration  = (idx == (IR_TEST_BAR_QUEUE_CAP - 1u))
                           ? IR_TEST_MAX_DURATION_26BIT
                           : ((idx + 1u) * 125u);
        /* Cast 4-byte bar record struct size to RTOS INT parameter. */
        b_is_ok = (E_OK == tk_snd_mbf(h_mbf,
                                      &tx_rec,
                                      (INT)sizeof(tx_rec),
                                      TMO_POL));
    }

    for (uint32_t idx = 0u; (idx < IR_TEST_BAR_QUEUE_CAP) && (b_is_ok); idx++)
    {
        ir_task_bar_record_t rx_rec  = { 0u, 0u };
        uint32_t             exp_dur = (idx == (IR_TEST_BAR_QUEUE_CAP - 1u))
                                       ? IR_TEST_MAX_DURATION_26BIT
                                       : ((idx + 1u) * 125u);
        INT                  rx_sz   = tk_rcv_mbf(h_mbf, &rx_rec, TMO_POL);

        /* Verify received INT byte count matches 4-byte bar record size. */
        b_is_ok = (((INT)sizeof(rx_rec) == rx_sz) &&
                   ((idx & 1u) == rx_rec.b_is_high) &&
                   (exp_dur == rx_rec.duration));
    }

    return b_is_ok;
}

/**
 * @brief S2-U06: Verify packed 3-bit Navigator route table and bounds guards.
 * @return True if all 5 route slots store/retrieve without neighbour bleed.
 */
static bool ir_test_check_route_table (void)
{
    static const navigator_task_dir_t dirs[NAV_NUMBER_OF_COMMANDS] =
    {
        DIR_FRONT,
        DIR_LEFT,
        DIR_RIGHT,
        DIR_BACK,
        DIR_STOP
    };
    navigator_task_route_t route   = { { 0u } };
    bool                   b_is_ok = true;

    for (uint32_t idx = 0u; idx < NAV_NUMBER_OF_COMMANDS; idx++)
    {
        navigator_task_set_route(&route, idx, dirs[idx]);
    }

    /* Out-of-range writes must be ignored safely. */
    navigator_task_set_route(&route, NAV_NUMBER_OF_COMMANDS, DIR_LEFT);
    navigator_task_set_route(&route, 99u, DIR_RIGHT);

    for (uint32_t idx = 0u; (idx < NAV_NUMBER_OF_COMMANDS) && (b_is_ok); idx++)
    {
        b_is_ok = (dirs[idx] == navigator_task_get_route(&route, idx));
    }

    /* Overwrite middle slot and verify neighbours remain untouched. */
    navigator_task_set_route(&route, 2u, DIR_INVALID);
    b_is_ok = ((b_is_ok) &&
               (DIR_LEFT == navigator_task_get_route(&route, 1u)) &&
               (DIR_INVALID == navigator_task_get_route(&route, 2u)) &&
               (DIR_BACK == navigator_task_get_route(&route, 3u)) &&
               (DIR_INVALID ==
                navigator_task_get_route(&route, NAV_NUMBER_OF_COMMANDS)));

    return b_is_ok;
}

/**
 * @brief S2-U07: Verify cross-byte bitfields and signed two's-complement pack.
 * @return True if 2-, 3-, 7-, 12-, and 32-bit signed/unsigned round-trips hold.
 */
static bool ir_test_check_bit_helpers (void)
{
    uint8_t                     buf[IR_TEST_BIT_BUF_BYTES] = { 0u };
    navigator_task_bypass_ctx_t bypass                     = { 0u };
    uint32_t                    ang_p                      =
        robot_tasks_pack_signed(-45, NAV_ANGLE_BIT_WIDTH);
    uint32_t                    tilt_p                     =
        robot_tasks_pack_signed(-1800, NAV_TILT_BIT_WIDTH);
    uint32_t                    rot_p                      =
        robot_tasks_pack_signed(2, 3u);
    bool                        b_is_ok                    = false;

    robot_tasks_bits_set(buf, 5u, 12u, 0xABCu);
    robot_tasks_bits_set(buf, 17u, 32u, 0xDEADBEEFu);

    navigator_task_set_side(&bypass, -1);
    b_is_ok = (-1 == navigator_task_get_side(&bypass));
    navigator_task_set_side(&bypass, 1);
    b_is_ok = ((b_is_ok) && (1 == navigator_task_get_side(&bypass)));
    navigator_task_set_side(&bypass, 0);

    return ((b_is_ok) &&
            (0 == navigator_task_get_side(&bypass)) &&
            (0xABCu == robot_tasks_bits_get(buf, 5u, 12u)) &&
            (0xDEADBEEFu == robot_tasks_bits_get(buf, 17u, 32u)) &&
            (-45 == robot_tasks_sign_extend(ang_p, NAV_ANGLE_BIT_WIDTH)) &&
            (-1800 == robot_tasks_sign_extend(tilt_p, NAV_TILT_BIT_WIDTH)) &&
            (2 == robot_tasks_sign_extend(rot_p, 3u)));
}

/**
 * @brief S2-U08: Verify spinlock-protected Navigator telemetry snapshot API.
 * @return True if snapshot size and concurrent setter updates read cleanly.
 */
static bool ir_test_check_telem_snapshot (void)
{
    navigator_task_telemetry_t snap = { 0u };

    /* Fit 3-bit SONAR_OK enum into uint8_t telemetry setter argument. */
    navigator_task_set_sonar_telem(true, (uint8_t)SONAR_OK, 142u);
    navigator_task_set_imu_pose(-125, 84, 270, true, 2u);
    navigator_task_set_imu_height(245);
    /* Fit 2-bit IMU_MSG_HUMP_DETECTED enum into uint8_t setter argument. */
    navigator_task_set_imu_event((uint8_t)IMU_MSG_HUMP_DETECTED);
    navigator_task_read_telem(&snap);

    /* Reset snapshot after reading. */
    navigator_task_set_sonar_telem(false, 0u, 0u);
    navigator_task_set_imu_pose(0, 0, 0, true, 0u);
    navigator_task_set_imu_height(0);
    navigator_task_set_imu_event(0u);

    /* Verify snapshot size and unsigned status/event bit-fields. */
    return ((IR_TEST_TELEM_BYTES == sizeof(navigator_task_telemetry_t)) &&
            (1u == snap.b_is_obs_dist_valid) &&
            ((uint32_t)SONAR_OK == snap.obs_status) &&
            (142u == snap.obs_dist_cm) &&
            (-125 ==
             robot_tasks_sign_extend(snap.pitch_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            (84 ==
             robot_tasks_sign_extend(snap.roll_ddeg_raw,
                                     NAV_TILT_BIT_WIDTH)) &&
            (270u == snap.heading_deg) &&
            (245u == snap.height_dmm) &&
            (2u == snap.imu_state) &&
            ((uint32_t)IMU_MSG_HUMP_DETECTED == snap.imu_event));
}

/*** end of file ***/
