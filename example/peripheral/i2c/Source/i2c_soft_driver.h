#ifndef _I2C_SOFT_DRIVER_H_
#define _I2C_SOFT_DRIVER_H_

#include "types.h"
#include "gpio.h"

#define READ_CMD        1

#define I2C_ACK         0
#define I2C_NAK         1

typedef enum
{
    IS_8BIT = 0,
    IS_16BIT
} i2c_reg_addrmode_enum_t;

typedef enum
{
    HW_I2C = 0,
    SW_I2C
} i2c_type_enum_t;

typedef struct
{
    GPIO_Pin_e scl;
    GPIO_Pin_e sda;
} i2c_drv_struct_t;


extern void i2c_init(i2c_drv_struct_t* p_i2c);
extern void i2c_deinit(i2c_drv_struct_t* p_i2c);
extern void i2c_driver_read(uint8 device_addr, uint16 reg_addr, uint8* buf, uint16 len, i2c_reg_addrmode_enum_t mode);
extern void i2c_driver_write(uint8 device_addr, uint16 reg_addr, uint8* buf, uint16 len, i2c_reg_addrmode_enum_t mode);

#endif



