#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <tk/tkernel.h>

IMPORT ID barcode_mbfid;
IMPORT ID navigator_mbfid;
IMPORT ID motor_mbfid;
IMPORT ID ranger_flgid;

typedef struct {
    int16_t  speed_left;       // current motor target %
    int16_t  speed_right;
    uint8_t  ir_state;         // last IR state
    uint8_t  movement_mode;    // current MOVEMENT_MODE
    int32_t  enc_l;            // encoder left ticks
    int32_t  enc_r;            // encoder right ticks
    uint32_t seq;              // update counter
} TELEMETRY_SNAPSHOT;

IMPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot;

IMPORT UW time_us(void);
IMPORT void delay_us(UW delay);
IMPORT ER robot_tasks_start(void);

#endif