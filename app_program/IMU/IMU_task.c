#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "IMU_task.h"
#include "../robot_tasks.h"



// LOCAL ER read_accel(ID iic, int16_t *x, int16_t *y, int16_t *z)
// {
// 	UB data[6];
// 	INT i;

// 	for(i = 0; i < 6; i++) {
// 		ER er = i2c_read_reg(iic, 0x19, (UW)(0xA8 + i), &data[i]);
// 		if(er < E_OK) return er;
// 	}
// 	*x = (int16_t)(((UINT)data[1] << 8) | data[0]) >> 4;
// 	*y = (int16_t)(((UINT)data[3] << 8) | data[2]) >> 4;
// 	*z = (int16_t)(((UINT)data[5] << 8) | data[4]) >> 4;
// 	return E_OK;
// }

// LOCAL ER read_magnetometer(ID iic, int16_t *x, int16_t *y, int16_t *z)
// {
// 	UB data[6];
// 	INT i;
// 	static const UB registers[6] = { 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

// 	for(i = 0; i < 6; i++) {
// 		ER er = i2c_read_reg(iic, 0x1E, registers[i], &data[i]);
// 		if(er < E_OK) return er;
// 	}
// 	*x = (int16_t)(((UINT)data[0] << 8) | data[1]);
// 	*z = (int16_t)(((UINT)data[2] << 8) | data[3]);
// 	*y = (int16_t)(((UINT)data[4] << 8) | data[5]);
// 	return E_OK;
// }

// EXPORT void imu_task(INT stacd, void *exinf)
// {
// 	ID iic;
// 	ER er;
// 	int16_t ax, ay, az;
// 	int16_t mx, my, mz;

// 	(void)stacd;
// 	(void)exinf;
// 	iic = tk_opn_dev((const UB *)"iica", TD_UPDATE);
// 	if(iic <= E_OK) {
// 		tm_printf((UB *)"[imu] cannot open iica, er=%d\n", iic);
// 		return;
// 	}

// 	er = i2c_write_reg(iic, 0x19, 0x20, 0x57);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x19, 0x23, 0x08);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x00, 0x14);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x01, 0x20);
// 	if(er >= E_OK) er = i2c_write_reg(iic, 0x1E, 0x02, 0x00);
// 	if(er < E_OK) {
// 		tm_printf((UB *)"[imu] sensor setup failed, er=%d\n", er);
// 		(void)tk_cls_dev(iic, 0);
// 		return;
// 	}
// 	tm_printf((UB *)"[imu] LSM303DLHC at accel=0x19 mag=0x1e; raw units\n");

// 	while(1) {
// 		er = read_accel(iic, &ax, &ay, &az);
// 		if(er >= E_OK) er = read_magnetometer(iic, &mx, &my, &mz);
// 		if(er >= E_OK) {
// 			tm_printf((UB *)"[imu] accel=%d,%d,%d mag=%d,%d,%d\n",
// 				  ax, ay, az, mx, my, mz);
// 		} else {
// 			tm_printf((UB *)"[imu] I2C read failed, er=%d\n", er);
// 		}
// 		tk_dly_tsk(250);
// 	}
// }