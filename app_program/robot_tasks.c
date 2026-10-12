#include <stdint.h>

#include <sys/machine.h>
#include <sys/sysdef.h>
#include <tk/tkernel.h>
#include <tk/device.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "robot_tasks.h"
#include "IR/IR_task.h"
#include "IMU/IMU_task.h"
#include "Motor/Motor_task.h"
#include "Navigator/Navigator_task.h"
#include "Ultrasonic/Ultrasonic_task.h"
// #include "Network/Network_task.h"
#include "Navigator/Navigator_task.h"

/* 
==========================
|     T KERNEL MISC      |
==========================    
*/

#define PERIPHERAL_TASK_STACK    4096

typedef void (*PERIPHERAL_TASK)(INT stacd, void *exinf);

typedef struct {
	PERIPHERAL_TASK task;
    INT priority;
	INT stacd;
	UW assprc;
	const char *name;
} TASK_START;

// Message buffers
EXPORT ID barcode_mbfid;
EXPORT ID navigator_mbfid;

// Flags
EXPORT ID ranger_flgid;

/* 
==========================
|          CODE          |
==========================    
*/

EXPORT UW time_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

EXPORT void delay_us(UW delay)
{
	UW start = time_us();

	while((UW)(time_us() - start) < delay) {
	}
}

LOCAL ER start_task(PERIPHERAL_TASK entry, INT priority, INT stacd, UW assprc, const char *name)
{
	T_CTSK ctsk = {0};
	ID tid;
	ER er;

	ctsk.tskatr = TA_HLNG | TA_RNG3;
	#if TK_SUPPORT_SMP
	if(assprc != 0) {
		ctsk.tskatr |= TA_ASSPRC;
		ctsk.assprc = assprc;
	}
	#else
	(void)assprc;
	#endif
	ctsk.itskpri = priority;
	ctsk.stksz = PERIPHERAL_TASK_STACK;
	ctsk.task = entry;
	tid = tk_cre_tsk(&ctsk);
	if(tid <= E_OK) {
		tm_printf((UB *)"[init] create %s failed, er=%d\n", (UB *)name, tid);
		return (ER)tid;
	}

	er = tk_sta_tsk(tid, stacd);
	if(er < E_OK) {
		tm_printf((UB *)"[init] start %s failed, er=%d\n", (UB *)name, er);
		(void)tk_del_tsk(tid);
	}
	return er;
}

EXPORT ER robot_tasks_start(void)
{
    tk_dly_tsk(1000);

	static const TASK_START tasks[] = {
		// { servo_task, 0, "servo" },
		// { ultrasonic_task, 0, "ultrasonic" },
		// { ir_sensor_task, IR_LEFT_PIN, "IR-left" },
		// { ir_sensor_task, IR_CENTER_PIN, "IR-center" },
		{ ir_barcode_task, 5, IR_LEFT_PIN, TP_PRC1, "IR-right" },
		{ ir_barcode_decoder_task, 5, 0, TP_PRC1, "IR-barcode-decoder" },
		{ motor_task, 9, 0, TP_PRC2, "motors" },
		// { imu_task, 0, "IMU" }
	};
    
    // Create flags
	T_CFLG cflg = {0};
	cflg.flgatr = TA_TFIFO;

	ranger_flgid = tk_cre_flg(&cflg);
	if(ranger_flgid <= E_OK) {
		tm_printf((UB *)"[init] create ranger event flag failed, er=%d\n",
			  ranger_flgid);
		return (ER)ranger_flgid;
	}

    // Create message buffers
    T_CMBF cmbf = {0};
	cmbf.mbfatr = TA_TFIFO; // First-In, First-Out queue
	cmbf.maxmsz = sizeof(BAR_RECORD); // Max size of one message
	cmbf.bufsz = sizeof(BAR_RECORD) * 18; // Capacity for 20 queued bars

	barcode_mbfid = tk_cre_mbf(&cmbf);
	if(barcode_mbfid <= E_OK) {
		tm_printf((UB *)"[init] create barcode mbf failed, er=%d\n", barcode_mbfid);
		return (ER)barcode_mbfid;
	}

    cmbf.maxmsz = sizeof(NAVIGATOR_MESSAGE);
	cmbf.bufsz = sizeof(NAVIGATOR_MESSAGE) * 20;

	navigator_mbfid = tk_cre_mbf(&cmbf);
	if(navigator_mbfid <= E_OK) {
		tm_printf((UB *)"[init] create navigator mbf failed, er=%d\n", navigator_mbfid);
		return (ER)navigator_mbfid;
	}

    UINT i;
	ER er;
	for(i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
		er = start_task(tasks[i].task, tasks[i].priority, tasks[i].stacd, tasks[i].assprc, tasks[i].name);
		if(er < E_OK) return er;
	}
	return E_OK;
}