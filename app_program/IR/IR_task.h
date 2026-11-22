#ifndef IR_TASK_H
#define IR_TASK_H

#include <tk/tkernel.h>

#define IR_LEFT_PIN         6
#define IR_CENTER_PIN       28
#define IR_RIGHT_PIN        27

#define ADC_IR_THRESHOLD    500

typedef struct __attribute__((packed)) {
    UW state : 1;
    UW duration : 26; // 0..42949672 x 100 us, preserves the full wrapping timer interval.
} BAR_RECORD;
_Static_assert(sizeof(BAR_RECORD) == 4, "barcode record packing");

typedef struct __attribute__((packed)) {
    UW previous_state : 1;
    UW current_state : 1;
    UW bar_count : 4; // 0..10.
} IR_RUNTIME_STATE;

IMPORT void ir_sensor_zeroing(INT stacd, void *exinf);
IMPORT void ir_sensor_task(INT stacd, void *exinf);
IMPORT void ir_barcode_task(INT stacd, void *exinf);
IMPORT void ir_barcode_decoder_task(INT stacd, void *exinf);

#endif