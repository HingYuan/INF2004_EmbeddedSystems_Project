/**
 * @file ir_task.c
 * @brief Line-following IR sensor and Code 39 barcode decoder tasks.
 *
 * NOTE: Line-following ADC reads RP2040 ADC channel 1 (GP27, right sensor)
 * and channel 2 (GP28, left/center sensor) using a 12-bit (0..4095) range
 * referenced to 3300 mV.
 */

#include "ir_task.h"

#include <stdbool.h>
#include <stdint.h>

#include <bsp/libbsp.h>
#include <tk/device.h>
#include <tm/tmonitor.h>

#include "../navigator/navigator_task.h"
#include "../robot_tasks.h"
#include "../util/code39.h"

/* Includes leading whitespace interval before the 9 Code 39 bar elements. */
#define IR_BARS_PER_CHAR                (10u)
#define IR_CODE39_ELEMENTS              (9u)
#define IR_ZEROING_BARCODE_PIN          (7u)
#define IR_ADC_REF_MV                   (3300u)
#define IR_ADC_MAX_RAW                  (4095u)
#define IR_ADC_CH_BARCODE               (0)
#define IR_ADC_CH_RIGHT                 (1)
#define IR_ADC_CH_LEFT                  (2)
#define IR_ZEROING_PERIOD_MSEC          (20u)
#define IR_LINE_PERIOD_MSEC             (1u)
#define IR_BARCODE_PERIOD_MSEC          (10u)
#define IR_BAR_DURATION_DIV_USEC        (100u)
#define IR_ASCII_ASTERISK               ((uint8_t)'*')
#define IR_ASCII_UNKNOWN                ((uint8_t)'?')

typedef struct ROBOT_TASKS_PACKED
{
    uint32_t previous_state : 1;
    uint32_t current_state  : 1;
    uint32_t bar_count      : 4;
} ir_task_runtime_t;

ROBOT_TASKS_STATIC_ASSERT(ir_runtime_size, (1u == sizeof(ir_task_runtime_t)));

static navigator_task_ir_state_t ir_classify_line (uint16_t left_mv,
                                                   uint16_t right_mv);
static void ir_decode_bar_window (
    const ir_task_bar_record_t bar_arr[IR_BARS_PER_CHAR]);

/**
 * @brief Optional sensor calibration task printing live IR GPIO and ADC data.
 * @param[in] stacd   Packed GPIO pin indices (bits 16..23 left, 8..15 right).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_zeroing_task (INT stacd, void * p_exinf)
{
    uint32_t packed_pins = (uint32_t)stacd;
    uint32_t pin_left    = (packed_pins >> 16u) & 0xFFu;
    uint32_t pin_right   = (packed_pins >> 8u) & 0xFFu;
    ID       h_adc_dev   = 0;
    bool     b_ready     = true;

    (void)p_exinf;

    if (gpio_set_pin((UINT)pin_left, GPIO_MODE_IN) < E_OK)
    {
        tm_printf((UB *)"[ir] GP%u input setup failed\n", (UINT)pin_left);
        b_ready = false;
    }

    if (b_ready && (gpio_set_pin((UINT)pin_right, GPIO_MODE_IN) < E_OK))
    {
        tm_printf((UB *)"[ir] GP%u input setup failed\n", (UINT)pin_right);
        b_ready = false;
    }

    if (b_ready)
    {
        h_adc_dev = tk_opn_dev((const UB *)"adca", TD_READ);

        if (h_adc_dev < E_OK)
        {
            tm_printf((UB *)"[ir] opening ADC failed: %d\n", h_adc_dev);
            b_ready = false;
        }
    }

    while (b_ready)
    {
        UINT left_val    = gpio_get_val((UINT)pin_left);
        UINT right_val   = gpio_get_val((UINT)pin_right);
        UW   barcode_raw = 0u;
        SZ   actual_sz   = 0;
        ER   ret_err     = E_OK;

        ret_err = tk_srea_dev(h_adc_dev,
                              IR_ADC_CH_BARCODE,
                              &barcode_raw,
                              1,
                              &actual_sz);

        if (E_OK != ret_err)
        {
            tm_printf((UB *)"[ir] ADC read failed: %d\n", ret_err);
            b_ready = false;
        }
        else
        {
            uint32_t barcode_mv =
                (((uint32_t)barcode_raw) * IR_ADC_REF_MV) / IR_ADC_MAX_RAW;

            tm_printf((UB *)"[IR left] %u\t", left_val);
            tm_printf((UB *)"[IR right] %u\n", right_val);
            tm_printf((UB *)"[IR barcode] %u\n", (UINT)barcode_mv);
            (void)tk_dly_tsk(IR_ZEROING_PERIOD_MSEC);
        }
    }
}

/**
 * @brief Sample center and right line-following ADC channels at 100 Hz.
 * @param[in] stacd   Packed GPIO pin indices (unused; ADC channels fixed).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_sensor_task (INT stacd, void * p_exinf)
{
    ID   h_adc_dev = tk_opn_dev((const UB *)"adca", TD_READ);
    bool b_ready   = true;

    (void)stacd;
    (void)p_exinf;

    if (h_adc_dev < E_OK)
    {
        tm_printf((UB *)"[ir] opening ADC failed: %d\n", h_adc_dev);
        b_ready = false;
    }

    while (b_ready)
    {
        UW raw_right = 0u;
        UW raw_left  = 0u;
        SZ actual_sz = 0;
        ER err_right = E_OK;
        ER err_left  = E_OK;

        err_right = tk_srea_dev(h_adc_dev,
                                IR_ADC_CH_RIGHT,
                                &raw_right,
                                1,
                                &actual_sz);

        if (E_OK != err_right)
        {
            tm_printf((UB *)"[ir] GP27 (ADC ch1) read failed: %d\n",
                      err_right);
            b_ready = false;
        }

        if (b_ready)
        {
            err_left = tk_srea_dev(h_adc_dev,
                                   IR_ADC_CH_LEFT,
                                   &raw_left,
                                   1,
                                   &actual_sz);

            if (E_OK != err_left)
            {
                tm_printf((UB *)"[ir] GP28 (ADC ch2) read failed: %d\n",
                          err_left);
                b_ready = false;
            }
        }

        if (b_ready)
        {
            /* Range-safe: 12-bit ADC scaled to 0..3300 mV fits in uint16_t. */
            uint16_t right_mv = (uint16_t)((((uint32_t)raw_right) *
                                            IR_ADC_REF_MV) / IR_ADC_MAX_RAW);
            uint16_t left_mv  = (uint16_t)((((uint32_t)raw_left) *
                                            IR_ADC_REF_MV) / IR_ADC_MAX_RAW);
            navigator_task_msg_t nav_msg = { 0 };

            nav_msg.type          = (uint32_t)MSG_IR_UPDATE;
            nav_msg.data.ir_state = (uint8_t)ir_classify_line(left_mv,
                                                              right_mv);
            (void)tk_snd_mbf(robot_tasks_get_nav_mbf(),
                             (void *)&nav_msg,
                             (INT)sizeof(navigator_task_msg_t),
                             TMO_FEVR);
            (void)tk_dly_tsk(IR_LINE_PERIOD_MSEC);
        }
    }
}

/**
 * @brief Monitor barcode IR GPIO transitions and push timed bar records.
 * @param[in] stacd   GPIO pin number of the barcode IR receiver.
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_barcode_task (INT stacd, void * p_exinf)
{
    ir_task_runtime_t state     = { 0u, 0u, 0u };
    UINT              pin_num   = (UINT)stacd;
    uint32_t          prev_usec = 0u;
    bool              b_ready   = true;

    (void)p_exinf;

    if (gpio_set_pin(pin_num, GPIO_MODE_IN) < E_OK)
    {
        tm_printf((UB *)"[ir] GP%u input setup failed\n", pin_num);
        b_ready = false;
    }

    if (b_ready)
    {
        state.previous_state = (0u != gpio_get_val(pin_num)) ? 1u : 0u;
        prev_usec            = robot_tasks_time_usec();

        tm_printf((UB *)"[ir] Task started, waiting for bars...\n");
        tm_printf((UB *)"[ir] GP%u initial %s\n",
                  pin_num,
                  (0u != state.previous_state) ? "HIGH" : "LOW");
    }

    while (b_ready)
    {
        uint32_t now_usec = robot_tasks_time_usec();

        state.current_state = (0u != gpio_get_val(pin_num)) ? 1u : 0u;

        if (state.current_state != state.previous_state)
        {
            ir_task_bar_record_t bar_rec = { 0u, 0u };

            bar_rec.state    = state.previous_state;
            bar_rec.duration =
                (now_usec - prev_usec) / IR_BAR_DURATION_DIV_USEC;
            (void)tk_snd_mbf(robot_tasks_get_barcode_mbf(),
                             (void *)&bar_rec,
                             (INT)sizeof(ir_task_bar_record_t),
                             TMO_FEVR);

            state.previous_state = state.current_state;
            prev_usec            = now_usec;
        }

        (void)tk_dly_tsk(IR_BARCODE_PERIOD_MSEC);
    }
}

/**
 * @brief Decode queued Code 39 bar records and forward characters to Navigator.
 * @param[in] stacd   Task start code (unused).
 * @param[in] p_exinf Extended task information pointer (unused).
 */
void ir_task_decoder_task (INT stacd, void * p_exinf)
{
    ir_task_bar_record_t bar_arr[IR_BARS_PER_CHAR] = { { 0u, 0u } };
    ir_task_runtime_t    decoder                   = { 0u, 0u, 0u };

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        ir_task_bar_record_t bar_rec  = { 0u, 0u };
        INT                  msg_size = 0;

        msg_size = tk_rcv_mbf(robot_tasks_get_barcode_mbf(),
                              (void *)&bar_rec,
                              TMO_FEVR);

        if (E_TMOUT == msg_size)
        {
            decoder.bar_count = 0u;
        }
        else if ((INT)sizeof(ir_task_bar_record_t) == msg_size)
        {
            bar_arr[decoder.bar_count] = bar_rec;
            decoder.bar_count++;

            if (decoder.bar_count >= IR_BARS_PER_CHAR)
            {
                ir_decode_bar_window(bar_arr);
                decoder.bar_count -= IR_BARS_PER_CHAR;
            }
        }
        else
        {
            /* Ignore partial or error receive returns cleanly. */
        }

        (void)tk_dly_tsk(IR_BARCODE_PERIOD_MSEC);
    }
}

/**
 * @brief Classify left and right IR ADC millivolt readings into 4 line states.
 * @param[in] left_mv  Left/center IR reading in millivolts.
 * @param[in] right_mv Right IR reading in millivolts.
 * @return Classified two-sensor line state enum value.
 */
static navigator_task_ir_state_t ir_classify_line (uint16_t left_mv,
                                                   uint16_t right_mv)
{
    navigator_task_ir_state_t line_state = IR_STATE_W_B;
    bool b_left_black  = (left_mv >= IR_ADC_THRESHOLD_MV);
    bool b_right_black = (right_mv >= IR_ADC_THRESHOLD_MV);

    if (b_left_black && b_right_black)
    {
        line_state = IR_STATE_B_B;
    }
    else if ((!b_left_black) && (!b_right_black))
    {
        line_state = IR_STATE_W_W;
    }
    else if (b_left_black && (!b_right_black))
    {
        line_state = IR_STATE_B_W;
    }
    else
    {
        line_state = IR_STATE_W_B;
    }

    return line_state;
}

/**
 * @brief Threshold a 10-bar window into a 9-bit Code 39 character and notify.
 * @param[in] bar_arr Array of 10 bar records (index 0 is leading space).
 */
static void ir_decode_bar_window (
    const ir_task_bar_record_t bar_arr[IR_BARS_PER_CHAR])
{
    uint32_t sum_dur      = 0u;
    uint32_t avg_dur      = 0u;
    uint16_t pattern_bits = 0u;
    uint8_t  decoded_char = IR_ASCII_UNKNOWN;

    for (uint32_t bar_idx = 1u; bar_idx < IR_BARS_PER_CHAR; bar_idx++)
    {
        sum_dur += bar_arr[bar_idx].duration;
    }

    avg_dur = sum_dur / IR_CODE39_ELEMENTS;

    for (uint32_t bar_idx = 1u; bar_idx < IR_BARS_PER_CHAR; bar_idx++)
    {
        if (bar_arr[bar_idx].duration > avg_dur)
        {
            uint32_t shift_pos = (IR_BARS_PER_CHAR - 1u) - bar_idx;

            /* Range-safe: shift_pos is 0..8 so bit fits in uint16_t. */
            pattern_bits |= (uint16_t)(1u << shift_pos);
        }
    }

    decoded_char = code39_decode_pattern(pattern_bits);

    tm_printf((UB *)"\ndecoded char: %c\n", (INT)decoded_char);
    tm_printf((UB *)"[decoder] Matched pattern 0x%03X to character: %c\n",
              (UINT)pattern_bits,
              (INT)decoded_char);

    if ((IR_ASCII_ASTERISK != decoded_char) &&
        (IR_ASCII_UNKNOWN != decoded_char))
    {
        navigator_task_msg_t nav_msg = { 0 };

        nav_msg.type              = (uint32_t)MSG_BARCODE_DECODED;
        nav_msg.data.decoded_char = decoded_char;
        (void)tk_snd_mbf(robot_tasks_get_nav_mbf(),
                         (void *)&nav_msg,
                         (INT)sizeof(navigator_task_msg_t),
                         TMO_FEVR);
    }
}

/*** end of file ***/
