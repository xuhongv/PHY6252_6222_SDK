#include "i2c_soft_demo.h"
#include "osal.h"
#include "log.h"

static uint8_t i2c_soft_taskid;
#define DATA_LEN  7
static uint8 i2c_tx_buf[DATA_LEN]= {0x50,0x51,0x52,0x53,0x54,0x55,0x56};
//uint8 i2c_tx_buf[DATA_LEN]={0,1,2,3,4,5,6};
static uint8 i2c_rx_buf[DATA_LEN]= {0};

#define I2C_MASTER_SDA P33
#define I2C_MASTER_CLK P34
#define slave_i2c_addr 0xa0
#define REG_ADDR       0

static i2c_drv_struct_t i2c_drv =
{
    .scl = I2C_MASTER_CLK,
    .sda = I2C_MASTER_SDA,
};

void I2c_Soft_Demo_Init(uint8 task_id)
{
    i2c_soft_taskid = task_id;
    LOG("i2c sfoft demo start...\n");
    hal_gpio_pin_init(I2C_MASTER_SDA,OEN);
    hal_gpio_pin_init(I2C_MASTER_CLK,OEN);
    hal_gpio_pull_set(I2C_MASTER_CLK,STRONG_PULL_UP);
    hal_gpio_pull_set(I2C_MASTER_SDA,STRONG_PULL_UP);
    hal_gpio_fast_write(I2C_MASTER_SDA,1);
    hal_gpio_fast_write(I2C_MASTER_CLK,1);
    i2c_init(&i2c_drv);
    osal_start_timerEx(i2c_soft_taskid, KEY_I2C_SOFT_WRITE_DATA_EVT, 10);
}

uint16 I2c_Soft_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != i2c_soft_taskid)
    {
        return 0;
    }

    if( events & KEY_I2C_SOFT_WRITE_DATA_EVT)
    {
        LOG("KEY_I2C_SOFT_WRITE_DATA_EVT\n");
        i2c_driver_write(slave_i2c_addr, REG_ADDR, i2c_tx_buf, DATA_LEN,IS_8BIT);
        LOG("soft i2c write data ok\n");
        osal_start_timerEx(i2c_soft_taskid,KEY_I2C_SOFT_READ_DATA_EVT, 1000);
        return (events ^ KEY_I2C_SOFT_WRITE_DATA_EVT);
    }

    if( events & KEY_I2C_SOFT_READ_DATA_EVT)
    {
        LOG("KEY_I2C_SOFT_READ_DATA_EVT\n");
        i2c_driver_read(slave_i2c_addr, REG_ADDR, i2c_rx_buf, DATA_LEN,IS_8BIT);
        LOG("I2C_RX_data=[");

        for(uint8 i=0; i<DATA_LEN; i++)
        {
            LOG("0x%x,",i2c_rx_buf[i]);
        }

        LOG("]\n");
        osal_start_timerEx(i2c_soft_taskid,KEY_I2C_SOFT_WRITE_DATA_EVT, 10);
        return (events ^ KEY_I2C_SOFT_READ_DATA_EVT);
    }

    return 0;
}

