#ifndef IR_TASK_H
#define IR_TASK_H

#include <tk/tkernel.h>

#define IR_LEFT_PIN         5
#define IR_CENTER_PIN      17
#define IR_RIGHT_PIN       26

typedef struct {
    INT state;
    UW duration;
} BAR_RECORD;

IMPORT void ir_sensor_task(INT pin_left, INT pin_right, INT side, void *exinf);
IMPORT void ir_barcode_task(INT pin, void *exinf);
IMPORT void ir_barcode_decoder_task(INT stacd, void *exinf);

#endif