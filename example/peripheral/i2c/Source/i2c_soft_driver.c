/**
****************************************************************************
    @file     sotfwave iic driver.c
    @brief    i2c driver source file
    @author   Eric Li
    @version  V1.0.0
    @date     2020/11/05
    @history
    @note
    detailed description:

******************************************************************************
    @attention

    <h2><center>&copy; COPYRIGHT 2020 PHY </center></h2>
*/


#include "gpio.h"
#include "error.h"
#include "log.h"
#include "i2c_soft_driver.h"
#include "types.h"
#include "clock.h"

#define PHY_I2C_PRINTF_ENABLE
#ifdef PHY_I2C_PRINTF_ENABLE
    #define I2C_PRINTF(fmt, args...)        dbg_printf(fmt, ##args)
#else
    #define I2C__PRINTF(fmt, args...)
#endif

#define I2C_DELAY()         __NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP(); \
    __NOP();__NOP();__NOP();__NOP();__NOP()

#if 1
#define I2C_SCL_SET_INPUT(scl_pin) \
    do { \
        hal_gpio_pin_init(scl_pin, GPIO_INPUT); \
        hal_gpio_pull_set(scl_pin, STRONG_PULL_UP); \
    } while (0)

#define I2C_SCL_SET_OUTPUT(scl_pin) \
    do{ \
        hal_gpio_cfg_analog_io(scl_pin,Bit_DISABLE); \
        hal_gpio_pin_init(scl_pin,GPIO_OUTPUT); \
        hal_gpio_fast_write(scl_pin,1); \
    }while(0)

#define I2C_SCL_OUTPUT_LOW(scl_pin)             hal_gpio_fast_write(scl_pin,0)
#define I2C_SCL_OUTPUT_HIGH(scl_pin)            hal_gpio_fast_write(scl_pin,1)
#define READ_SCL_INPUT_STATUES(scl_pin)         hal_gpio_read(scl_pin)


#define I2C_SDA_SET_INPUT(sda_pin)  \
    do { \
        hal_gpio_pin_init(sda_pin, GPIO_INPUT); \
        hal_gpio_pull_set(sda_pin, STRONG_PULL_UP); \
    } while (0)

#define I2C_SDA_SET_OUTPUT(sda_pin) \
    do{ \
        hal_gpio_cfg_analog_io(sda_pin,Bit_DISABLE); \
        hal_gpio_pin_init(sda_pin,GPIO_OUTPUT); \
        hal_gpio_fast_write(sda_pin,1); \
    }while(0)
#define I2C_SDA_OUTPUT_LOW(sda_pin)             hal_gpio_fast_write(sda_pin,0)
#define I2C_SDA_OUTPUT_HIGH(sda_pin)            hal_gpio_fast_write(sda_pin,1)
#define READ_SDA_INPUT_STATUES(sda_pin)         hal_gpio_read(sda_pin)


static i2c_drv_struct_t* g_p_i2c = NULL;

/*******************************************************************************
    Function Name  : i2c_start
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static void i2c_start(void)
{
    // Make sure both SDA and SCL high.
    I2C_SDA_OUTPUT_HIGH(g_p_i2c->sda);
    I2C_DELAY();
    I2C_SCL_OUTPUT_HIGH(g_p_i2c->scl);
    I2C_DELAY();
    // SDA: \__
    I2C_SDA_OUTPUT_LOW(g_p_i2c->sda);
    I2C_DELAY();
    // SCL: \__
    I2C_SCL_OUTPUT_LOW(g_p_i2c->scl);
    I2C_DELAY();
}
/*******************************************************************************
    Function Name  : i2c_stop
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static void i2c_stop(void)
{
    // Make sure SDA low.
    I2C_SDA_OUTPUT_LOW(g_p_i2c->sda);
    I2C_DELAY();
    // SCL: __/
    I2C_SCL_OUTPUT_HIGH(g_p_i2c->scl);
    I2C_DELAY();
    // SDA: __/
    I2C_SDA_OUTPUT_HIGH(g_p_i2c->sda);
    I2C_DELAY();
}
/*******************************************************************************
    Function Name  : i2c_read_byte
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static void i2c_read_byte(uint8* data, bool ack)
{
    uint8 i, byte_read = 0;
    // Before call the function, SCL should be low.
    // Make sure SDA is an input
    I2C_SDA_SET_INPUT(g_p_i2c->sda);

    // MSB first
    for (i = 0x80; i != 0; i >>= 1)
    {
        I2C_SCL_OUTPUT_HIGH(g_p_i2c->scl);
        I2C_DELAY();

        if (TRUE==READ_SDA_INPUT_STATUES(g_p_i2c->sda))
        {
            byte_read |= i;
        }

        I2C_SCL_OUTPUT_LOW(g_p_i2c->scl);
        I2C_DELAY();
    }

    // Make sure SDA is an output before we exit the function
    I2C_SDA_SET_OUTPUT(g_p_i2c->sda);
    *data = byte_read;

    // Send ACK bit

    // SDA high == NACK, SDA low == ACK
    if (ack==I2C_ACK)
    {
        I2C_SDA_OUTPUT_LOW(g_p_i2c->sda);
    }
    else
    {
        I2C_SDA_OUTPUT_HIGH(g_p_i2c->sda);
    }

    // Let SDA line settle for a moment
    I2C_DELAY();
    // Drive SCL high to start ACK/NACK bit transfer
    I2C_SCL_OUTPUT_HIGH(g_p_i2c->scl);
    I2C_DELAY();
    // Finish ACK/NACK bit clock cycle and give slave a moment to react
    I2C_SCL_OUTPUT_LOW(g_p_i2c->scl);
    I2C_DELAY();
}
/*******************************************************************************
    Function Name  : i2c_write_byte
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static bool i2c_write_byte(uint8 data)
{
    bool ret = I2C_NAK;
    uint16 i;

    // Before call the function, SCL should be low.

    // MSB first
    for (i = 0x80; i != 0; i >>= 1)
    {
        if (data & i)
        {
            I2C_SDA_OUTPUT_HIGH(g_p_i2c->sda);
        }
        else
        {
            I2C_SDA_OUTPUT_LOW(g_p_i2c->sda);
        }

        I2C_DELAY();
        I2C_SCL_OUTPUT_HIGH(g_p_i2c->scl);
        I2C_DELAY();
        I2C_SCL_OUTPUT_LOW(g_p_i2c->scl);
        I2C_DELAY();
    }

    // Configure SDA pin as input for receiving the ACK bit
    I2C_SDA_SET_INPUT(g_p_i2c->sda);
    I2C_SCL_OUTPUT_HIGH(g_p_i2c->scl);
    I2C_DELAY();
    #if 1
    {
        //  wait ack sign
        I2C_SCL_SET_INPUT(g_p_i2c->scl);

        for(i=0; i<50000; i++)
        {
            if (TRUE==READ_SCL_INPUT_STATUES(g_p_i2c->scl))
            {
                break;
            }
            else
            {
                I2C_DELAY();
            }
        }

        I2C_SCL_SET_OUTPUT(g_p_i2c->scl);
    }
    #endif

    //  raed ack
    if (TRUE==READ_SDA_INPUT_STATUES(g_p_i2c->sda))
        ret = I2C_ACK;

    // Finish ACK/NACK bit clock cycle and give slave a moment to release control
    // of the SDA line
    I2C_SCL_OUTPUT_LOW(g_p_i2c->scl);
    I2C_DELAY();
    // Configure SDA pin as output as other module functions expect that
    I2C_SDA_SET_OUTPUT(g_p_i2c->sda);
    return ret;
}
/*******************************************************************************
    Function Name  : i2c_init
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_init(i2c_drv_struct_t* p_i2c)
{
    if(p_i2c!=NULL)
    {
        g_p_i2c = p_i2c;
        I2C_SCL_SET_OUTPUT(g_p_i2c->scl);
        I2C_SDA_SET_OUTPUT(g_p_i2c->sda);
    }
}
/*******************************************************************************
    Function Name  : i2c_deinit
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_deinit(i2c_drv_struct_t* p_i2c)
{
    if(p_i2c!=NULL)
    {
        I2C_SCL_SET_INPUT(p_i2c->scl);
        I2C_SDA_SET_INPUT(p_i2c->sda);
        g_p_i2c = NULL;
    }
}


/*******************************************************************************
    Function Name  : i2c_driver_read
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_driver_read(uint8 device_addr, uint16 reg_addr, uint8* buf, uint16 len, i2c_reg_addrmode_enum_t mode)
{
    #if 1

    if(g_p_i2c!=NULL)
    {
        i2c_start();
        i2c_write_byte(device_addr);

        if(mode==IS_8BIT)
        {
            i2c_write_byte((uint8)reg_addr);
        }
        else
        {
            i2c_write_byte((uint8)(reg_addr>>8));
            i2c_write_byte((uint8)reg_addr);
        }

        i2c_start();
        i2c_write_byte(device_addr + READ_CMD);

        while (1)
        {
            if (len <= 1)
            {
                i2c_read_byte(buf, I2C_NAK);
                break;
            }
            else
            {
                i2c_read_byte(buf, I2C_ACK);
                len --;
            }

            buf ++;
            I2C_DELAY();
        }

        i2c_stop();
    }

    #else
    #endif
}
/*******************************************************************************
    Function Name  : i2c_driver_write
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_driver_write(uint8 device_addr, uint16 reg_addr, uint8* buf, uint16 len, i2c_reg_addrmode_enum_t mode)
{
    if(g_p_i2c!=NULL)
    {
        i2c_start();
        i2c_write_byte(device_addr);

        if(mode==IS_8BIT)
        {
            i2c_write_byte((uint8)reg_addr);
        }
        else
        {
            i2c_write_byte((uint8)(reg_addr>>8));
            i2c_write_byte((uint8)reg_addr);
        }

        while (len --)
        {
            i2c_write_byte(*buf ++);
        }

        i2c_stop();
    }
}
#endif


#if 0
#define I2C_SCL_SET_INPUT() \
    do { \
        hal_gpio_pin_init(PIN_SW_I2C_SCL, GPIO_INPUT); \
        hal_gpio_pull_set(PIN_SW_I2C_SCL, STRONG_PULL_UP); \
    } while (0)

#define I2C_SCL_SET_OUTPUT() \
    do{ \
        hal_gpio_cfg_analog_io(PIN_SW_I2C_SCL,Bit_DISABLE); \
        hal_gpio_pin_init(PIN_SW_I2C_SCL,GPIO_OUTPUT); \
        hal_gpio_fast_write(PIN_SW_I2C_SCL,1); \
    }while(0)

#define I2C_SCL_OUTPUT_LOW()            hal_gpio_fast_write(PIN_SW_I2C_SCL,0)
#define I2C_SCL_OUTPUT_HIGH()           hal_gpio_fast_write(PIN_SW_I2C_SCL,1)
#define READ_SCL_INPUT_STATUES()        hal_gpio_read(PIN_SW_I2C_SCL)


#define I2C_SDA_SET_INPUT() \
    do { \
        hal_gpio_pin_init(PIN_SW_I2C_SDA, GPIO_INPUT); \
        hal_gpio_pull_set(PIN_SW_I2C_SDA, STRONG_PULL_UP); \
    } while (0)

#define I2C_SDA_SET_OUTPUT() \
    do{ \
        hal_gpio_cfg_analog_io(PIN_SW_I2C_SDA,Bit_DISABLE); \
        hal_gpio_pin_init(PIN_SW_I2C_SDA,GPIO_OUTPUT); \
        hal_gpio_fast_write(PIN_SW_I2C_SDA,1); \
    }while(0)
#define I2C_SDA_OUTPUT_LOW()            hal_gpio_fast_write(PIN_SW_I2C_SDA,0)
#define I2C_SDA_OUTPUT_HIGH()           hal_gpio_fast_write(PIN_SW_I2C_SDA,1)
#define READ_SDA_INPUT_STATUES()        hal_gpio_read(PIN_SW_I2C_SDA)

/*******************************************************************************
    Function Name  : i2c_checkBusy
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
/*
    static uint8 i2c_checkBusy(void)
    {
    if(READ_SCL_INPUT_STATUES()==true){
        return 0;
    }else{
        return 1;
    }
    }
*/
/*******************************************************************************
    Function Name  : i2c_start
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static void i2c_start(void)
{
    // Make sure both SDA and SCL high.
    I2C_SDA_OUTPUT_HIGH();
    I2C_DELAY();
    I2C_SCL_OUTPUT_HIGH();
    I2C_DELAY();
    // SDA: \__
    I2C_SDA_OUTPUT_LOW();
    I2C_DELAY();
    // SCL: \__
    I2C_SCL_OUTPUT_LOW();
    I2C_DELAY();
}
/*******************************************************************************
    Function Name  : i2c_stop
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static void i2c_stop(void)
{
    // Make sure SDA low.
    I2C_SDA_OUTPUT_LOW();
    I2C_DELAY();
    // SCL: __/
    I2C_SCL_OUTPUT_HIGH();
    I2C_DELAY();
    // SDA: __/
    I2C_SDA_OUTPUT_HIGH();
    I2C_DELAY();
}
/*******************************************************************************
    Function Name  : i2c_read_byte
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static void i2c_read_byte(uint8* data, bool ack)
{
    uint8 i, byte_read = 0;
    // Before call the function, SCL should be low.
    // Make sure SDA is an input
    I2C_SDA_SET_INPUT();

    // MSB first
    for (i = 0x80; i != 0; i >>= 1)
    {
        I2C_SCL_OUTPUT_HIGH();
        I2C_DELAY();

        if (TRUE==READ_SDA_INPUT_STATUES())
        {
            byte_read |= i;
        }

        I2C_SCL_OUTPUT_LOW();
        I2C_DELAY();
    }

    // Make sure SDA is an output before we exit the function
    I2C_SDA_SET_OUTPUT();
    *data = byte_read;

    // Send ACK bit

    // SDA high == NACK, SDA low == ACK
    if (ack==I2C_ACK)
    {
        I2C_SDA_OUTPUT_LOW();
    }
    else
    {
        I2C_SDA_OUTPUT_HIGH();
    }

    // Let SDA line settle for a moment
    I2C_DELAY();
    // Drive SCL high to start ACK/NACK bit transfer
    I2C_SCL_OUTPUT_HIGH();
    I2C_DELAY();
    // Finish ACK/NACK bit clock cycle and give slave a moment to react
    I2C_SCL_OUTPUT_LOW();
    I2C_DELAY();
}
/*******************************************************************************
    Function Name  : i2c_write_byte
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
static bool i2c_write_byte(uint8 data)
{
    bool ret = I2C_NAK;
    uint8 i;

    // Before call the function, SCL should be low.

    // MSB first
    for (i = 0x80; i != 0; i >>= 1)
    {
        if (data & i)
        {
            I2C_SDA_OUTPUT_HIGH();
        }
        else
        {
            I2C_SDA_OUTPUT_LOW();
        }

        I2C_DELAY();
        I2C_SCL_OUTPUT_HIGH();
        I2C_DELAY();
        I2C_SCL_OUTPUT_LOW();
        I2C_DELAY();
    }

    // Configure SDA pin as input for receiving the ACK bit
    I2C_SDA_SET_INPUT();
    I2C_SCL_OUTPUT_HIGH();
    I2C_DELAY();
    #if 1
    {
        //  wait ack sign
        I2C_SCL_SET_INPUT();

        for(i=0; i<100; i++)
        {
            if (TRUE==READ_SCL_INPUT_STATUES())
            {
                break;
            }
            else
            {
                I2C_DELAY();
            }
        }

        I2C_SCL_SET_OUTPUT();
    }
    #endif

    //  raed ack
    if (TRUE==READ_SDA_INPUT_STATUES())
        ret = I2C_ACK;

    // Finish ACK/NACK bit clock cycle and give slave a moment to release control
    // of the SDA line
    I2C_SCL_OUTPUT_LOW();
    I2C_DELAY();
    // Configure SDA pin as output as other module functions expect that
    I2C_SDA_SET_OUTPUT();
    return ret;
}
/*******************************************************************************
    Function Name  : i2c_init
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_init(void)
{
    I2C_SCL_SET_OUTPUT();
    I2C_SDA_SET_OUTPUT();
}
/*******************************************************************************
    Function Name  : i2c_deinit
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_deinit(void)
{
    I2C_SCL_SET_INPUT();
    I2C_SDA_SET_INPUT();
}

/*******************************************************************************
    Function Name  : i2c_driver_read
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_driver_read(uint8 device_addr, uint16 reg_addr, uint8* buf, uint16 len, i2c_reg_addrmode_enum_t mode)
{
    i2c_start();
    i2c_write_byte(device_addr);

    if(mode==IS_8BIT)
    {
        i2c_write_byte((uint8)reg_addr);
    }
    else
    {
        i2c_write_byte((uint8)(reg_addr>>8));
        i2c_write_byte((uint8)reg_addr);
    }

    i2c_start();
    i2c_write_byte(device_addr + READ_CMD);

    while (1)
    {
        if (len <= 1)
        {
            i2c_read_byte(buf, I2C_NAK);
            break;
        }
        else
        {
            i2c_read_byte(buf, I2C_ACK);
            len --;
        }

        buf ++;
        I2C_DELAY();
    }

    i2c_stop();
}
/*******************************************************************************
    Function Name  : i2c_driver_write
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void i2c_driver_write(uint8 device_addr, uint16 reg_addr, uint8* buf, uint16 len, i2c_reg_addrmode_enum_t mode)
{
    i2c_start();
    i2c_write_byte(device_addr);

    if(mode==IS_8BIT)
    {
        i2c_write_byte((uint8)reg_addr);
    }
    else
    {
        i2c_write_byte((uint8)(reg_addr>>8));
        i2c_write_byte((uint8)reg_addr);
    }

    while (len --)
    {
        i2c_write_byte(*buf ++);
    }

    i2c_stop();
}
/*******************************************************************************
    Function Name  : add_i2c_interface
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void add_i2c_interface(uint8 addr, GPIO_Pin_e scl, GPIO_Pin_e sda)
{
}
/*******************************************************************************
    Function Name  : remove_i2c_interface
    Description    :
    Input          : None
    Output         : None
    Return         : None
*******************************************************************************/
void remove_i2c_interface(uint8 addr)
{
}


#endif


