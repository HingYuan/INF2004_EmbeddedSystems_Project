#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <tk/tkernel.h>

IMPORT ID barcode_mbfid;
IMPORT ID navigator_mbfid;
IMPORT ID ranger_flgid;

IMPORT UW time_us(void);
IMPORT void delay_us(UW delay);
IMPORT ER robot_tasks_start(void);

#endif