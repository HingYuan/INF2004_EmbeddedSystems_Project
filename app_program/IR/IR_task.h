#ifndef IR_TASK_H
#define IR_TASK_H

#include <tk/tkernel.h>

#define IR_LEFT_PIN         6
#define IR_CENTER_PIN       28
#define IR_RIGHT_PIN        27

#define ADC_IR_THRESHOLD    500

typedef struct {
    INT state;
    UW duration;
} BAR_RECORD;

IMPORT void ir_sensor_zeroing(INT stacd, void *exinf);
IMPORT void ir_sensor_task(INT stacd, void *exinf);
IMPORT void ir_barcode_task(INT stacd, void *exinf);
IMPORT void ir_barcode_decoder_task(INT stacd, void *exinf);

#endif