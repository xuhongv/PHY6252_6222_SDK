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
#include "i2c_dma_demo.h"
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
#include "dma.h"

#define I2C_MASTER_SDA P33
#define I2C_MASTER_CLK P34
#define slave_i2c_addr 0x50
#define REG_ADDR 0

static void* master_pi2c;
static uint8_t i2c_TaskID;
static uint8_t i2c_mode=I2C_MODE_TX;

#define DATA_LEN  7
static uint8 i2c_tx_buf[DATA_LEN]= {0x50,0x51,0x52,0x53,0x54,0x55,0x56};
//uint8 i2c_tx_buf[DATA_LEN]={0,1,2,3,4,5,6};
static uint8 i2c_rx_buf[DATA_LEN]= {0};

__ATTR_SECTION_SRAM__ static void dma_chx_cb(DMA_CH_t ch)
{
    if(ch==DMA_CH_0)
    {
        LOG("\ndma ch0 done!!!\n");

        if(i2c_mode==I2C_MODE_TX)
        {
            osal_start_timerEx(i2c_TaskID, KEY_I2C_DMA_READ_DATA_EVT, 1000);
        }
        else if(i2c_mode==I2C_MODE_RX)
        {
            osal_start_timerEx(i2c_TaskID, KEY_I2C_DMA_RX_DATA_EVT, 5);
        }
    }
}

uint8_t i2c_dma_tx_config(i2c_dev_t i2cx,uint8_t* tx_buf,uint16_t tx_len,DMA_CH_t dma_ch,uint8_t dma_int)
{
    HAL_DMA_t ch_cfg;
    DMA_CH_CFG_t cfg;
    ch_cfg.dma_channel = dma_ch;
    ch_cfg.evt_handler = dma_chx_cb;
    hal_dma_init_channel(ch_cfg);
    AP_I2C_TypeDef* I2cx = NULL;
    I2cx = (i2cx == I2C_0) ? AP_I2C0 : AP_I2C1;
    I2cx->IC_DMA_CR &= 0x00;
    cfg.transf_size = tx_len;
    cfg.sinc = DMA_INC_INC;
    cfg.src_tr_width = DMA_WIDTH_BYTE;
    cfg.src_msize = DMA_BSIZE_1;
    cfg.src_addr = (uint32_t)tx_buf;
    cfg.dinc = DMA_INC_NCHG;
    cfg.dst_tr_width = DMA_WIDTH_BYTE;
    cfg.dst_msize = DMA_BSIZE_1;
    cfg.dst_addr =(uint32_t)&(I2cx->IC_DATA_CMD);
    cfg.enable_int = dma_int;
    I2cx->IC_DMA_CR |= 0x02;
    I2cx->IC_DMA_TDLR = 16;
    uint8_t retval = hal_dma_config_channel(dma_ch,&cfg);
    LOG("dam tx config ret is :%d\n",retval);
    return retval;
}

uint8_t i2c_dma_rx_config(i2c_dev_t i2cx,uint8_t* rx_buf,uint16_t rx_len,DMA_CH_t dma_ch,uint8_t dma_int)
{
    HAL_DMA_t ch_cfg;
    DMA_CH_CFG_t cfg;
    ch_cfg.dma_channel = dma_ch;
    ch_cfg.evt_handler = dma_chx_cb;
    hal_dma_init_channel(ch_cfg);
    AP_I2C_TypeDef* I2cx = NULL;
    I2cx = (i2cx == I2C_0) ? AP_I2C0 : AP_I2C1;
    I2cx->IC_DMA_CR  &= 0x02;
    cfg.transf_size = rx_len;
    cfg.sinc = DMA_INC_NCHG;
    cfg.src_tr_width = DMA_WIDTH_BYTE;
    cfg.src_msize = DMA_BSIZE_1;
    cfg.src_addr = (uint32_t)&(I2cx->IC_DATA_CMD);
    cfg.dinc = DMA_INC_INC;
    cfg.dst_tr_width = DMA_WIDTH_BYTE;
    cfg.dst_msize = DMA_BSIZE_1;
    cfg.dst_addr =(uint32_t)rx_buf;
    cfg.enable_int = dma_int;
    I2cx->IC_DMA_CR |= 0x01;
    I2cx->IC_DMA_TDLR = 0;
    uint8_t retval = hal_dma_config_channel(dma_ch,&cfg);
    LOG("dam rx config ret is :%d\n",retval);
    return retval;
}

void I2c_Dma_Demo_Init(uint8 task_id)
{
    i2c_TaskID = task_id;
    LOG("i2c demo start...\n");
    hal_dma_deinit();
    hal_dma_init();
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
        osal_start_timerEx(i2c_TaskID, KEY_I2C_DMA_WRITE_DATA_EVT, 10);
    }
}

uint16 I2c_Dma_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != i2c_TaskID)
    {
        return 0;
    }

    if( events & KEY_I2C_DMA_WRITE_DATA_EVT)
    {
        LOG("KEY_I2C_WRITE_DATA_EVT\n");

        if(i2c_dma_tx_config(I2C_0,i2c_tx_buf,sizeof(i2c_tx_buf),DMA_CH_0,TRUE)==PPlus_SUCCESS)
        {
            i2c_mode=I2C_MODE_TX;
            hal_i2c_addr_update(master_pi2c,slave_i2c_addr);    //发送24c02设备地址--写
            uint8_t reg_addr=REG_ADDR;
            hal_i2c_send(master_pi2c, (uint8*)&reg_addr,1);           //发送24c02设备地址--写
            hal_dma_stop_channel(DMA_CH_0);
            uint8_t ret=hal_dma_start_channel(DMA_CH_0);
            LOG("tx start ret is %d\n",ret);
        }

        return (events ^ KEY_I2C_DMA_WRITE_DATA_EVT);
    }

    if( events & KEY_I2C_DMA_READ_DATA_EVT)
    {
        LOG("KEY_I2C_READ_DATA_EVT\n");

        if(i2c_dma_rx_config(I2C_0,i2c_rx_buf,sizeof(i2c_rx_buf),DMA_CH_0,TRUE)==PPlus_SUCCESS)
        {
            i2c_mode=I2C_MODE_RX;
            hal_i2c_addr_update(master_pi2c,slave_i2c_addr);    //发送24c02设备地址--写
            uint8_t reg_addr=REG_ADDR;
            hal_i2c_send(master_pi2c, (uint8*)&reg_addr,1);           //发送24c02设备地址--写
            extern void hal_master_send_read_cmd(void* pi2c, uint8_t len);
            hal_master_send_read_cmd(master_pi2c,DATA_LEN);
            hal_dma_stop_channel(DMA_CH_0);
            uint8_t ret=hal_dma_start_channel(DMA_CH_0);
            LOG("rx start ret is %d\n",ret);
        }

        return (events ^ KEY_I2C_DMA_READ_DATA_EVT);
    }

    if( events & KEY_I2C_DMA_RX_DATA_EVT)
    {
        LOG("I2C_RX_data=[");

        for(uint8 i=0; i<DATA_LEN; i++)
        {
            LOG("0x%x,",i2c_rx_buf[i]);
        }

        LOG("]\n");
        osal_start_timerEx(i2c_TaskID, KEY_I2C_DMA_WRITE_DATA_EVT, 10);
        return (events ^ KEY_I2C_DMA_RX_DATA_EVT);
    }

    return 0;
}

/*********************************************************************
*********************************************************************/
