#ifndef _I2C_SOFT_DEMO_H_
#define _I2C_SOFT_DEMO_H_
#include "i2c_soft_driver.h"
#define KEY_I2C_SOFT_WRITE_DATA_EVT    0x0001
#define KEY_I2C_SOFT_READ_DATA_EVT     0x0002
#define KEY_I2C_SOFT_RX_DATA_EVT       0x0004

void I2c_Soft_Demo_Init(uint8 task_id);
uint16 I2c_Soft_ProcessEvent( uint8 task_id, uint16 events );
#endif

