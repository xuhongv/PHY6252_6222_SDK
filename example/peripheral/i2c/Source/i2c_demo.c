/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       gpio_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "i2c_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "key.h"

#include "flash.h"
#include "i2c.h"


#include "pwrmgr.h"
#include "error.h"


#define I2C_MASTER_SDA P33
#define I2C_MASTER_CLK P34
#define slave_i2c_addr 0x50
#define REG_ADDR 0

static void* master_pi2c;
static uint8_t i2c_TaskID;


#define I2C_MASTER_SEND_DATA_LEN 7
uint8 I2C_TX_data[I2C_MASTER_SEND_DATA_LEN]= {0x50,0x51,0x52,0x53,0x54,0x55,0x56};
uint8 I2C_RX_data[I2C_MASTER_SEND_DATA_LEN]= {0};

static int I2CWrite(void* pi2c,  uint8* data,uint8 len,uint8 slave_addr,uint8 Addr)
{
    hal_i2c_addr_update(pi2c,slave_addr);           //发送24c02设备地址--写
    HAL_ENTER_CRITICAL_SECTION();
    hal_i2c_send(pi2c, (uint8*)&Addr, 1);            //发送24c02设备地址--写
    hal_i2c_send(pi2c, data, len);
    HAL_EXIT_CRITICAL_SECTION();
    return hal_i2c_wait_tx_completed(pi2c);
}

static int I2CRead(void* pi2c,  uint8* data,uint8 len,uint8 slave_addr,uint8 Addr)
{
    int ret ;
    ret=hal_i2c_read(pi2c,slave_addr,Addr,data,len);

    if(ret==PPlus_SUCCESS)
    {
        LOG("i2c read sucess \n");
    }
    else
    {
        LOG("i2c read error:%d\n",ret);
    }

    return ret;
}

void I2c_Demo_Init(uint8 task_id)
{
    i2c_TaskID = task_id;
    LOG("i2c demo start...\n");
    hal_gpio_pin_init(I2C_MASTER_SDA,IE);
    hal_gpio_pin_init(I2C_MASTER_CLK,IE);
    hal_gpio_pull_set(I2C_MASTER_SDA,STRONG_PULL_UP);
    hal_gpio_pull_set(I2C_MASTER_CLK,STRONG_PULL_UP);
    hal_i2c_pin_init(I2C_0, I2C_MASTER_SDA, I2C_MASTER_CLK);
    master_pi2c=hal_i2c_init(I2C_0,I2C_CLOCK_400K);

    if(master_pi2c==NULL)
    {
        LOG("I2C master init fail\n");
    }
    else
    {
        LOG("I2C master init OK\n");
    }

    osal_start_timerEx(i2c_TaskID, KEY_I2C_WRITE_DATA_EVT, 10);
}

uint16 I2c_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != i2c_TaskID)
    {
        return 0;
    }

    if( events & KEY_I2C_WRITE_DATA_EVT)
    {
        int ret;
        ret=I2CWrite(master_pi2c,I2C_TX_data,I2C_MASTER_SEND_DATA_LEN,slave_i2c_addr,REG_ADDR);

        if(ret==PPlus_SUCCESS)
        {
            LOG("I2C  Master  send data   ok\n");
            osal_start_timerEx(i2c_TaskID, KEY_I2C_READ_DATA_EVT, 10);
        }

        return (events ^ KEY_I2C_WRITE_DATA_EVT);
    }

    if( events & KEY_I2C_READ_DATA_EVT)
    {
        uint8_t ret;
        ret=I2CRead(master_pi2c,I2C_RX_data,I2C_MASTER_SEND_DATA_LEN,slave_i2c_addr,REG_ADDR);

        if(ret==PPlus_SUCCESS)
        {
            LOG("I2C_RX_data=[");

            for(uint8 i=0; i<I2C_MASTER_SEND_DATA_LEN; i++)
            {
                LOG("0x%x,",I2C_RX_data[i]);
            }

            LOG("]\n");
            osal_start_timerEx(i2c_TaskID, KEY_I2C_WRITE_DATA_EVT, 1000);
        }

        return (events ^ KEY_I2C_READ_DATA_EVT);
    }

    return 0;
}

/*********************************************************************
*********************************************************************/
