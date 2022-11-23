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
    Filename:       phy_plus_phy.c
    Revised:
    Revision:

    Description:    This file contains the phyplus phy sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "log.h"
#include "timer.h"
#include "phy_plus_phy.h"
#include "ll.h"
#include "ll_hw_drv.h"
#include "clock.h"
#include "gpio.h"
#include "flash.h"
#include "rf_phy_nrf.h"

/*********************************************************************
    MACROS
*/
#define PHYPLUS_SET_SYNCWORD(x)                 PHY_REG_WT(0x4003004c,(x))
#define PHYPLUS_SET_CRC_SEED(x)                 subWriteReg(0x40030048,23,0,(x))
#define PHYPLUS_SET_WHITEN_SEED(x)              subWriteReg(0x40030048,31,24,(x))

/*********************************************************************
    CONSTANTS
*/
#define PHYPLUS_RFPHY_TX_ONLY                   (0x00)
#define PHYPLUS_RFPHY_RX_ONLY                   (0x01)
#define PHYPLUS_RFPHY_TRX_ONLY                  (0x02)
#define PHYPLUS_RFPHY_RX_TXACK                  (0x03)
#define PHYPLUS_RFPHY_TX_PENDING                (0x10)
#define PHYPLUS_RFPHY_RX_PENDING                (0x11)
#define PHYPLUS_RFPHY_TRX_PENDING               (0x12)
#define PHYPLUS_RFPHY_RX_TXACK_PENDING          (0x13)

#define PHYPLUS_RFPHY_IDLE                      (0xFF)

#define RFPHY_STATUS_SET_PENDING(x)             (0x10 |(x))
#define RFPHY_STATUS_CLR_PENDING(x)             (0x0F &(x))

#define PHYPLUS_AUTOACK_ENABLE                  (1)
#define PHYPLUS_AUTOACK_DISABLE                 (0)
#define LL_HW_MODE_STX                          (0x00)
#define LL_HW_MODE_SRX                          (0x01)
#define LL_HW_MODE_TRX                          (0x02)


#define PHYPLUS_HW_SCAN_DELAY                   (80)
#define PHYPLUS_HW_BB_DELAY                     (90)
#define PHYPLUS_HW_AFE_DELAY                    ( 8)
#define PHYPLUS_HW_PLL_DELAY                    (60)

#define DEFAULT_CRC_SEED                        (0x555555)

#define DEFAULT_WHITEN_SEED                     (0x37)
#define WHITEN_SEED_CH37                        (0x53)
#define WHITEN_SEED_CH38                        (0x33)
#define WHITEN_SEED_CH39                        (0x73)
#define DEFAULT_WHITEN_SEED                     (0x37)
#define WHITEN_OFF                              (0x00)

#define BLE_ADV_CHN37                           (02)
#define BLE_ADV_CHN38                           (26)
#define BLE_ADV_CHN39                           (80)


#define DEFAULT_SYNCWORD                        (0x8e89bed6)

#define PHYPLUS_PKT_FMT_1M                      (0x01)
#define PHYPLUS_PKT_FMT_2M                      (0x02)
#define PHYPLUS_PKT_FMT_500K                    (0x05)
#define PHYPLUS_PKT_FMT_100K                    (0x06)

#define PHYPLUS_HW_MAX_RX_TO                    (20000)
#define PHYPLUS_HW_MIN_SCAN_TIME_US             (2000)



#define RFPHY_TX_PENDING_RETRY_DLY              (2)  //Ms
#define RFPHY_RX_PENDING_RETRY_DLY              (2)  //Ms


extern uint8 ll_hw_get_tr_mode(void);
extern volatile uint32 llWaitingIrq;
/*********************************************************************
    TYPE Define
*/
typedef struct pktCfg_s
{
    uint8_t     pktFmt;
    uint8_t     pduLen;
    uint8_t     wtSeed;
    uint8_t     crcFmt;
    uint32_t    crcSeed;
    uint32_t    syncWord;
    uint8_t*    p_txBuf;
    uint8_t*    p_rxBuf;
} pktCfg_t;

typedef struct phyCtx_s
{
    uint8_t     Status;
    uint32_t    txIntv;
    uint32_t    txDuration;
    uint32_t    rxIntv;
    uint32_t    rxDuration;
    uint8_t     rfChn;
    uint32_t    rxOnlyTO;
    uint16_t    rxAckTO;
    uint32_t    rxScanT0;
    uint8_t     reTxCnt;
    uint8_t     reTxMax;
    uint8_t     txAck;
    uint16_t    reTxDly;
    uint8_t     enAutoAck;
} phyCtx_t;

typedef struct phySch_s
{
    uint32_t    txMargin;
    uint32_t    rxMargin;
} phySch_t;

typedef struct phyDebug_s
{
    uint32_t   rx_data_cnt;
    uint32_t   rx_data_ign;
    uint32_t   rx_crc_err;
    uint32_t   rx_txack_cnt;
    uint32_t   tx_data_cnt;
    uint32_t   tx_ack_cnt;
    uint32_t   tx_retry_cnt;
    uint32_t   rx_txack_t0;
    uint32_t   rx_txack_t1;
} phyDeubg_t;

/*********************************************************************
    LOCAL VARIABLES
*/
uint8 PhyPlusPhy_TaskID; // Task ID for internal task/event processing
//volatile uint32 phyWaitingIrq = FALSE;
uint32 PHY_ISR_entry_time = 0;

__align(4) uint8_t  phyBufRx[256];
__align(4) uint8_t  phyBufTx[256];
//static uint8_t s_pubAddr[6];
uint8_t adv_buffer[32];


uint16 phyFoff=0;
uint8  phyCarrSens=0;
uint8  phyRssi=0;

static pktCfg_t s_pktCfg;
static phyCtx_t s_phy;
static phyDeubg_t s_phyDbg;
static phySch_t s_phySch;
/*********************************************************************
    LOCAL FUNCTIONS
*/
extern void PLUSPHY_IRQHandler(void);
uint32 BLE_IRQHandler_Restore = NULL;
void PhyPlusPhy_Set_BLE_IRQHandler(void)
{
    if(BLE_IRQHandler_Restore)
    {
        JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   BLE_IRQHandler_Restore;
        ll_hw_set_crc_fmt(LL_HW_CRC_BLE_FMT,LL_HW_CRC_BLE_FMT);
        subWriteReg(0x40030040, 4, 4, 0);
        subWriteReg(0x40030008,8,8,1);
    }
}

static uint32  read_ble_remainder_time(void)
{
    uint32 currentCount;
    uint32 g_tim1_pass = read_current_fine_time();
    currentCount = AP_TIM1->CurrentCount;

    if((currentCount < 6) || NVIC_GetPendingIRQ(TIM1_IRQn))
        return 0;
    else
        return (currentCount >> 2);
}

static uint8 phy_allow_tx(void)
{
    uint32 advTime, margin;
    uint32 remainTime;
    uint8 ret = FALSE;
    uint16 pktLen = phyBufTx[1]+2+3;//pdulen + header + crc

    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        pktLen = (nrfTxBuf.pldLen + 9);

    if(s_pktCfg.pktFmt==PHYPLUS_PKT_FMT_1M)
    {
        pktLen = pktLen<<3;
    }
    else
    {
        pktLen = pktLen<<2;
    }

    // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION( );

    // read global config to get advTime and margin
    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        advTime = (pktLen+s_phy.reTxDly)*s_phy.reTxMax; //
    else
        advTime = pktLen+200; //

    margin = s_phySch.txMargin;
    // remain time before trigger LL HW
    remainTime = read_ble_remainder_time();

    if ((remainTime > advTime + margin)
            && !llWaitingIrq)
        ret = TRUE;

    HAL_EXIT_CRITICAL_SECTION();

    if(ret==FALSE)
    {
        //LOG("[DIS TX] s%d r%d a%d m%d\n",llWaitingIrq,remainTime,advTime,margin);
    }

    return ret;
}

static uint8 phy_allow_rx(uint32_t* scanTimeAllow)
{
    uint32 scanTime, margin;
    uint32 remainTime;
    uint8 ret = FALSE;
    uint32_t t0=read_current_fine_time();
    // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION( );
    // read global config to get advTime and margin
    margin = s_phySch.rxMargin;
    // remain time before trigger LL HW
    remainTime = read_ble_remainder_time();
    remainTime = remainTime>margin ?
                 remainTime - margin : 0;
    scanTime = (TIME_DELTA(t0, s_phy.rxScanT0)<s_phy.rxOnlyTO) ?
               s_phy.rxOnlyTO - TIME_DELTA(t0, s_phy.rxScanT0) :
               0;
    scanTime = remainTime>scanTime ? scanTime : remainTime;
    scanTime = scanTime<PHYPLUS_HW_MIN_SCAN_TIME_US ? 0 : scanTime;
    scanTime  = scanTime>0xffff ? 0xffff:scanTime;// max scan time out is 16bit

    if ((scanTime) && !llWaitingIrq)
        ret = TRUE;

    *scanTimeAllow = scanTime;
    HAL_EXIT_CRITICAL_SECTION();

    if(ret==FALSE)
    {
        //LOG("[DIS RX] s%d r%d a%d m%d\n",llWaitingIrq,remainTime,scanTime,margin);
    }

    return ret;
}

static uint8_t phy_rx_data_check(void)
{
    uint8_t ret=PPlus_SUCCESS;

    if(s_phy.Status==PHYPLUS_RFPHY_RX_ONLY || s_phy.Status==PHYPLUS_RFPHY_RX_TXACK)
    {
        ret = nrf_rxdata_check(phyBufRx);

        //process data
        if(ret==PPlus_SUCCESS)
        {
            osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DATA_PROCESS_EVT);
            s_phyDbg.rx_data_cnt++;
        }
        else
        {
            s_phyDbg.rx_data_ign++;
        }
    }
    //process txAck
    else if(s_phy.Status==PHYPLUS_RFPHY_TRX_ONLY)
    {
        //process data check
        ret = nrf_txack_check(phyBufRx);//check from the addr[0]

        if(ret==PPlus_SUCCESS)
        {
            s_phy.txAck=1;
            s_phyDbg.rx_txack_cnt++;
        }

        return ret;
    }

    return ret;
}

void phy_set_channel(uint8 rfChnIdx)
{
    if(g_rfPhyFreqOffSet>=0)
        PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
    else
        PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
}

void phy_hw_go(void)
{
    //20190115 ZQ recorded ll re-trigger
    if(llWaitingIrq==TRUE)
    {
        LOG("[PHY TRIG ERR]\n");
    }

    *(volatile uint32_t*)(LL_HW_BASE+ 0x14) = LL_HW_IRQ_MASK;   //clr  irq status
    *(volatile uint32_t*)(LL_HW_BASE+ 0x0c) = 0x0001;           //mask irq :only use mode done
    *(volatile uint32_t*)(LL_HW_BASE+ 0x00) = 0x0001;           //trig
    uint8_t rfChnIdx = PHY_REG_RD(0x400300b4)&0xff;

    if(rfChnIdx<2)
    {
        rfChnIdx=2;
    }
    else if(rfChnIdx>80)
    {
        rfChnIdx=80;
    }

    if(s_pktCfg.pktFmt==PKT_FMT_BLE2M)
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0_2Mbps,g_rfPhyTpCal1_2Mbps,(rfChnIdx-2)>>1));
    else
        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0,g_rfPhyTpCal1,(rfChnIdx-2)>>1));

    //change to diff demod for nrf2.4G
    subWriteReg(0x40030004,14,8,60);
    subWriteReg(0x40030008,8,8,0);
}


void phy_hw_stop(void)
{
    uint8_t cnt=0;
    ll_hw_set_rx_timeout(33);//will trigger ll_hw_irq=RTO

    while(llWaitingIrq)
    {
        WaitRTCCount(3);
        cnt++;

        if(cnt>10)
        {
            LOG("[PHY STOP ERR]\n");
            break;
        }
    };
}

void phy_hw_set_srx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_srx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_set_stx(void)
{
    ll_hw_set_stx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_set_trx(uint16 rxTimeOutUs)
{
    ll_hw_set_rx_timeout(rxTimeOutUs);
    ll_hw_set_trx();
    ll_hw_set_trx_settle(   PHYPLUS_HW_BB_DELAY,         // set BB delay
                            PHYPLUS_HW_AFE_DELAY,
                            PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
}

void phy_hw_timing_setting(void)
{
    ll_hw_set_tx_rx_release (10,     1);
    ll_hw_set_rx_tx_interval(       60);        //T_IFS=150us for BLE 1M
    ll_hw_set_tx_rx_interval(       10);        //T_IFS=150us for BLE 1M, 20220510 for NRF T_IFS 130us,set to 10
    ll_hw_set_trx_settle    (57, 8, 52);        //TxBB,RxAFE,PL
}

void phy_hw_pktFmt_Config(pktCfg_t cfg)
{
    //baseband cfg
    rf_phy_bb_cfg(cfg.pktFmt);

    //pktfmt
    if(cfg.crcFmt==LL_HW_CRC_NULL)
    {
        //fix length mode ,no hw crc gen/check
        ll_hw_set_pplus_pktfmt(cfg.pduLen);
        ll_hw_ign_rfifo(LL_HW_IGN_NONE);
    }
    else
    {
        //crc
        ll_hw_set_crc_fmt(cfg.crcFmt,cfg.crcFmt);
        PHYPLUS_SET_CRC_SEED(cfg.crcSeed);
        ll_hw_ign_rfifo(LL_HW_IGN_CRC);
    }

    //whiten
    PHYPLUS_SET_WHITEN_SEED(cfg.wtSeed);
    //syncword
    PHYPLUS_SET_SYNCWORD(cfg.syncWord);
}

uint8_t phy_rf_tx(void)
{
    if(phy_allow_tx()==FALSE)
    {
        return PPlus_ERR_BUSY;
    }
    else
    {
        JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&PLUSPHY_IRQHandler;
    }

    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    phy_set_channel(s_phy.rfChn);

    if(s_phy.Status==PHYPLUS_RFPHY_TRX_ONLY)
        phy_hw_set_trx(s_phy.rxAckTO);
    else
        phy_hw_set_stx();

    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();

    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
    {
        set_max_length(s_pktCfg.pduLen);
        ll_hw_write_tfifo(s_pktCfg.p_txBuf,s_pktCfg.pduLen);
    }
    else
    {
        set_max_length(0xff);
        //need updata phyBufTx
        ll_hw_write_tfifo(s_pktCfg.p_txBuf,s_pktCfg.p_rxBuf[1]+2);
    }

    phy_hw_go();
    llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
    return PPlus_SUCCESS;
}
uint8_t phy_rf_rx(void)
{
    uint32_t scanTime;

    if(phy_allow_rx(&scanTime)==FALSE)
    {
        return PPlus_ERR_BUSY;
    }
    else
    {
        JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&PLUSPHY_IRQHandler;
    }

    phy_hw_stop();
    HAL_ENTER_CRITICAL_SECTION();
    phy_hw_pktFmt_Config(s_pktCfg);
    phy_hw_timing_setting();
    phy_set_channel(s_phy.rfChn);
    phy_hw_set_srx((0xffff&scanTime));
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();

    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        set_max_length(s_pktCfg.pduLen);
    else
        set_max_length(0xff);

    phy_hw_go();
    llWaitingIrq=TRUE;
    HAL_EXIT_CRITICAL_SECTION();
    return PPlus_SUCCESS;
}
void phy_rx_data_process(void)
{
    uint8_t pduLen=0;

    if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
    {
        pduLen = s_pktCfg.pduLen;
        LOG("-------------------------\n");
        LOG("[PHY RX] [-%03ddbm %4dKHz %02d CH] ",phyRssi,phyFoff-512,s_phy.rfChn);
        LOG("PCF: L%2x pid %2x no_ack %2x ",nrfRxBuf.pldLen,nrfRxBuf.pid,nrfRxBuf.noAckBit);
        LOG("ADDR:");
        my_dump_byte(&nrfRxBuf.addr[0], 5);
        LOG("PDU:");
        my_dump_byte(&nrfRxBuf.pdu[0], nrfRxBuf.pldLen);
    }
    else
    {
        pduLen=phyBufRx[1];
        {
            LOG("-------------------------\n");
            LOG("[PHY RX] [-%03ddbm %4dKHz %02d CH] ",phyRssi,phyFoff-512,s_phy.rfChn);

            for(uint8_t i=0; i<pduLen; i++)
                LOG("%02x ",phyBufRx[i]);

            LOG("\n");
        }
    }
}

void phy_tx_buf_updata(uint8_t* adva,uint8_t* txHead,uint8_t* txPayload,uint8_t dlen)
{
    osal_memcpy(&(phyBufTx[0]),&(txHead[0]),2);          //copy tx header
    osal_memcpy(&(phyBufTx[2]),&(adva[0]),6);              //copy AdvA
    osal_memcpy(&(phyBufTx[8]),&(txPayload[0]),dlen);      //copy payload
    LOG("\n-----------------------------------------------\n");
    LOG("PHY BUF Tx Dump\n");

    for(uint8_t i=0; i<phyBufTx[1]+2; i++)
        LOG("%02x ",phyBufTx[i]);

    LOG("\n-----------------------------------------------\n");
}


void phy_rf_process_recv(void)
{
    if(s_phy.Status == PHYPLUS_RFPHY_IDLE)
        return;

    HAL_ENTER_CRITICAL_SECTION();
    s_phy.Status = RFPHY_STATUS_CLR_PENDING(s_phy.Status);
    uint8_t ret= phy_rf_rx();
    HAL_EXIT_CRITICAL_SECTION();

    if(ret==PPlus_ERR_BUSY)
    {
        s_phy.Status = RFPHY_STATUS_SET_PENDING(s_phy.Status);
        osal_start_timerEx(PhyPlusPhy_TaskID,PPP_RX_PENDING_PROCESS_EVT,RFPHY_RX_PENDING_RETRY_DLY);
    }
}

void phy_rf_process_tsmt(void)
{
    if(s_phy.Status == PHYPLUS_RFPHY_IDLE)
        return;

    HAL_ENTER_CRITICAL_SECTION();
    s_phy.Status = RFPHY_STATUS_CLR_PENDING(s_phy.Status);
    uint8_t ret= phy_rf_tx();
    HAL_EXIT_CRITICAL_SECTION();

    if(ret==PPlus_ERR_BUSY)
    {
        s_phy.Status = RFPHY_STATUS_SET_PENDING(s_phy.Status);
        osal_start_timerEx(PhyPlusPhy_TaskID,PPP_TX_PENDING_PROCESS_EVT,RFPHY_TX_PENDING_RETRY_DLY);
    }
}

void phy_rf_schedule(void)
{
    uint32_t t0=read_current_fine_time();

    if(s_phy.Status == PHYPLUS_RFPHY_TX_ONLY)
    {
        s_phy.Status = PHYPLUS_RFPHY_IDLE;
        osal_set_event(PhyPlusPhy_TaskID,PPP_TX_DONE_EVT);
    }
    else if(    s_phy.Status == PHYPLUS_RFPHY_RX_ONLY)
    {
        if(TIME_DELTA(t0, s_phy.rxScanT0)<s_phy.rxOnlyTO)
        {
            phy_rf_process_recv();
        }
        else
        {
            s_phy.Status = PHYPLUS_RFPHY_IDLE;
            osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DONE_EVT);
        }
    }
    else if(s_phy.Status == PHYPLUS_RFPHY_RX_TXACK)
    {
        s_phy.Status = PHYPLUS_RFPHY_RX_ONLY;
    }
    else if(s_phy.Status == PHYPLUS_RFPHY_TRX_ONLY)
    {
        if(s_phy.txAck || s_phy.reTxCnt==s_phy.reTxMax)
        {
            s_phy.Status = PHYPLUS_RFPHY_IDLE;
            osal_set_event(PhyPlusPhy_TaskID,PPP_TRX_DONE_EVT);
        }
        else
        {
            if(s_phy.reTxDly>s_phy.rxAckTO)
                WaitUs(s_phy.reTxDly-s_phy.rxAckTO);

            s_phy.reTxCnt++;
            s_phyDbg.tx_retry_cnt++;
            phy_rf_process_tsmt();
        }
    }
}
/*******************************************************************************
    @fn          PLUSPHY_IRQHandler

    @brief      Interrupt Request Handler for Link Layer

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None
*/
void PLUSPHY_IRQHandler(void)
{
    uint8         mode;
    uint32_t      irq_status;
    uint32_t      T2, delay;
    PHY_ISR_entry_time = read_current_fine_time();
    irq_status = ll_hw_get_irq_status();

    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        PhyPlusPhy_Set_BLE_IRQHandler();
        return;
    }

    llWaitingIrq = FALSE;
    HAL_ENTER_CRITICAL_SECTION();
    mode = ll_hw_get_tr_mode();

    // ===================   mode TRX process 1
    if (mode == LL_HW_MODE_STX  &&
            (s_phy.Status == PHYPLUS_RFPHY_TX_ONLY)
       )
    {
        //osal_set_event(PhyPlusPhy_TaskID,PPP_TX_DONE_EVT);
    }
    else if(mode == LL_HW_MODE_SRX  &&
            (s_phy.Status == PHYPLUS_RFPHY_RX_ONLY)
           )
    {
        rf_phy_get_pktFoot(&phyRssi,&phyFoff,&phyCarrSens);

        if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        {
            if(0==(irq_status & LIRQ_RTO))
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo_pplus(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                //do crc check
                uint32_t t0,t1;
                uint8_t dLen;

                if(nrf_pkt_crc_check(phyBufRx))
                {
                    if(s_phy.enAutoAck == PHYPLUS_AUTOACK_ENABLE)
                    {
                        ll_hw_set_stx();
                        t0=read_current_fine_time();
                        T2 = TIME_DELTA(t0,PHY_ISR_entry_time);
                        delay = 127-T2;
                        ll_hw_set_trx_settle(   delay,         // set BB delay
                                                PHYPLUS_HW_AFE_DELAY,
                                                PHYPLUS_HW_PLL_DELAY);        //RxAFE,PLL
                        phy_hw_go();
                        nrfAckBuf.pid = GET_NRF_PKT_PID(phyBufRx);
                        nrf_pkt_enc(&nrfAckBuf,phyBufTx,&dLen);
                        ll_hw_rst_tfifo();
                        set_max_length(dLen);
                        ll_hw_write_tfifo(s_pktCfg.p_txBuf,dLen);
                        t1=read_current_fine_time();
                        llWaitingIrq=TRUE;
                        s_phy.Status = PHYPLUS_RFPHY_RX_TXACK;
                        s_phyDbg.rx_txack_t0 = TIME_DELTA(t0,PHY_ISR_entry_time);
                        s_phyDbg.rx_txack_t1 = TIME_DELTA(t1,t0);
                        s_phyDbg.tx_ack_cnt++;
                    }

                    phy_rx_data_check();
                }
                else
                    s_phyDbg.rx_crc_err++;
            }
        }
        else
        {
            if(irq_status & LIRQ_COK)
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }

        //osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DONE_EVT);
    }
    else if(mode == LL_HW_MODE_TRX  &&
            (s_phy.Status == PHYPLUS_RFPHY_TRX_ONLY)
           )
    {
        rf_phy_get_pktFoot(&phyRssi,&phyFoff,&phyCarrSens);
        s_phyDbg.tx_data_cnt++;

        if(s_pktCfg.crcFmt==LL_HW_CRC_NULL)
        {
            if(0==(irq_status & LIRQ_RTO))
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo_pplus(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }
        else
        {
            if(irq_status & LIRQ_COK)
            {
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;
                ll_hw_read_rfifo(s_pktCfg.p_rxBuf, &pktLen,&pktFoot0,&pktFoot1);
                rf_phy_get_pktFoot_fromPkt(pktFoot0,pktFoot1,
                                           &phyRssi,&phyFoff,&phyCarrSens);
                phy_rx_data_check();
            }
        }

        //osal_set_event(PhyPlusPhy_TaskID,PPP_TRX_DONE_EVT);
    }

    // post ISR process
    if(llWaitingIrq!=TRUE)
    {
        ll_hw_clr_irq();
        PhyPlusPhy_Set_BLE_IRQHandler();
    }

    HAL_EXIT_CRITICAL_SECTION();
    phy_rf_schedule();
}

/*********************************************************************
    @fn      PhyPlusPhy_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void PhyPlusPhy_Init(uint8 task_id)
{
    PhyPlusPhy_TaskID = task_id;
    //set phy irq handeler
    //JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&PLUSPHY_IRQHandler;
    BLE_IRQHandler_Restore = JUMP_FUNCTION(V4_IRQ_HANDLER) ;

    for(int i=0; i<32; i++)
        adv_buffer[i]=i;

    uint8_t nrf_addr[5] = {0xc7,0x34,0x89,0x71};//NRF ADDR Format  //{0xc7,0x34,0x89,0x71,0xd7}
    uint8_t nrf_addrLen = NRF_ADDR_LEN_4BYTE;
    uint8_t nrf_pudLen = 32;
    uint8_t nrf_mode    = NRF_MODE_ENHANCE_SHOCKBURST;//NRF_MODE_ENHANCE_SHOCKBURST
    uint8_t nrf_crcByte = CRC_ITU_16_LEN_2BYTE;//CRC_ITU_16_LEN_2BYTE
    //phy pktfmt config
    s_phy.enAutoAck     =   PHYPLUS_AUTOACK_ENABLE; // for NRF_MODE_ENHANCE_SHOCKBURST need enable autoack
    s_phy.Status        =   PHYPLUS_RFPHY_IDLE;
    s_phySch.txMargin   =   1500;//us
    s_phySch.rxMargin   =   2000;//us
    s_phy.rxOnlyTO      =   1000*1000;//us
    s_phy.rfChn         =   34;//26;//

    if(s_phy.enAutoAck == PHYPLUS_AUTOACK_DISABLE)
    {
        s_phy.rxAckTO       =   0;//us, Set to 0 switch to STX instead of TRX mode for send data
        s_phy.reTxMax       =   0;//tx retry count Max, Set to 0 to turn off Auto-ReTx
        s_phy.reTxDly       =   0;//auto retry delay
    }
    else
    {
        s_phy.rxAckTO       =   500;//us, Set to 0 switch to STX instead of TRX mode for send data
        s_phy.reTxMax       =   5;//tx retry count Max
        s_phy.reTxDly       =   500;//auto retry delay
    }

    s_pktCfg.pktFmt     =   PHYPLUS_PKT_FMT_1M;
    s_pktCfg.pduLen     =   nrf_pudLen+6;// (1+2+2+1);
    s_pktCfg.crcFmt     =   LL_HW_CRC_NULL;//LL_HW_CRC_BLE_FMT;LL_HW_CRC_NULL
    s_pktCfg.crcSeed    =   DEFAULT_CRC_SEED;
    s_pktCfg.wtSeed     =   WHITEN_OFF;//WHITEN_SEED_CH37;//DEFAULT_WHITEN_SEED;
    s_pktCfg.syncWord   =   nrf_pkt_init(nrf_addr, nrf_addrLen,nrf_pudLen,nrf_crcByte,nrf_mode);
    s_pktCfg.p_txBuf    =   &(phyBufTx[4]);
    s_pktCfg.p_rxBuf    =   &(phyBufRx[4]);
    LOG("[NRF_CFG] :v%08x  Mode %d  pdu len %d CRCB %d addrLen %d Addr:",
        nrfConfig.version,nrfConfig.mode,nrfTxBuf.pldLen,nrfConfig.crcByte,nrfConfig.addrLen);
    my_dump_byte(nrfTxBuf.addr,nrfConfig.addrLen);
    hal_gpio_pin_init(P11, GPIO_INPUT);
    hal_gpio_pull_set(P11, STRONG_PULL_UP);
//    if(0==hal_gpio_read(P11))
//    {
//        LOG("START PPP_PERIODIC_TX_EVT\n");
//        VOID osal_start_timerEx(PhyPlusPhy_TaskID, PPP_PERIODIC_TX_EVT, 1000);
//    }
//    else
//    {
//        LOG("START PPP_PERIODIC_RX_EVT\n");
//        VOID osal_start_timerEx(PhyPlusPhy_TaskID, PPP_PERIODIC_RX_EVT, 100);
//    }
    LOG("[PHY] init done %d rfchn%d AutoAck %d SW[%8x] CRC[%d %8x] WT[%2x]\n"\
        ,s_phy.Status,s_phy.rfChn,s_phy.enAutoAck,s_pktCfg.syncWord,s_pktCfg.crcFmt, s_pktCfg.crcSeed,s_pktCfg.wtSeed);
}
static void show_phy_debug_info(void)
{
    LOG("[PHY DBG]st %02x [TX]dat %d ack %d rty %d [ackT]%d %d [RX]dat %d ack%d crc %d ign %d \n"
        ,s_phy.Status,s_phyDbg.tx_data_cnt,s_phyDbg.tx_ack_cnt,s_phyDbg.tx_retry_cnt
        ,s_phyDbg.rx_txack_t0,s_phyDbg.rx_txack_t1
        ,s_phyDbg.rx_data_cnt,s_phyDbg.rx_txack_cnt,s_phyDbg.rx_crc_err,s_phyDbg.rx_data_ign);
}

static void process_rx_done_evt(void)
{
    LOG("rx done evt\n");

    if(s_phy.rxOnlyTO==RFPHY_RX_SCAN_ALLWAYS_ON)
    {
        osal_set_event(PhyPlusPhy_TaskID,PPP_PERIODIC_RX_EVT);
    }

    show_phy_debug_info();
}

static void process_tx_done_evt(void)
{
    LOG("tx done evt\n");
}
static void process_trx_done_evt(void)
{
    if(s_phy.txAck)
    {
        //adv_buffer[0]+=1;
        LOG("[TX OK]\n");
    }
    else
    {
        LOG("[TX Fail]\n");
        //osal_start_timerEx(PhyPlusPhy_TaskID,PPP_PERIODIC_TX_EVT,10);
    }

    LOG("trx done evt reTry %d reMax %d\n",s_phy.reTxCnt,s_phy.reTxMax);
    show_phy_debug_info();
}


uint8_t phy_rf_send_data(uint8_t* din, uint8_t dLen)
{
    if(PHYPLUS_RFPHY_IDLE !=s_phy.Status)
        return PPlus_ERR_BUSY;

    if(s_phy.rxAckTO==0)
        s_phy.Status = PHYPLUS_RFPHY_TX_ONLY;
    else
        s_phy.Status = PHYPLUS_RFPHY_TRX_ONLY;

    s_phy.reTxCnt = 0;
    s_phy.txAck   = 0;
    nrf_pkt_gen(din, dLen, phyBufTx);
    phy_rf_process_tsmt();
    return PPlus_SUCCESS;
}

uint8_t phy_rf_stop_rx(void)
{
    if( PHYPLUS_RFPHY_RX_ONLY ==s_phy.Status ||
            PHYPLUS_RFPHY_RX_PENDING ==s_phy.Status)
    {
        s_phy.rxOnlyTO=0;

        if(PHYPLUS_RFPHY_RX_ONLY ==s_phy.Status)
            phy_hw_stop();
        else
        {
            s_phy.Status = PHYPLUS_RFPHY_IDLE;
            osal_set_event(PhyPlusPhy_TaskID,PPP_RX_DONE_EVT);
        }

        return PPlus_SUCCESS;
    }
    else
        return PPlus_ERR_INVALID_STATE;
}

uint8_t phy_rf_start_rx(uint32 rxTimeOut)
{
    if(PHYPLUS_RFPHY_IDLE !=s_phy.Status)
        return PPlus_ERR_BUSY;

    s_phy.rxOnlyTO = rxTimeOut;
    s_phy.Status = PHYPLUS_RFPHY_RX_ONLY;
    s_phy.rxScanT0 = read_current_fine_time();
    phy_rf_process_recv();
    return PPlus_SUCCESS;
}

uint8_t phy_rf_get_current_status(void)
{
    return s_phy.Status;
}

/*********************************************************************
    @fn      PhyPlusPhy_ProcessEvent

    @brief   Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 PhyPlusPhy_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id;

    if (events & PPP_PERIODIC_TX_EVT)
    {
        if(s_phy.Status==PHYPLUS_RFPHY_IDLE)
        {
            phy_rf_send_data(adv_buffer, 30);
            osal_start_timerEx(PhyPlusPhy_TaskID,PPP_PERIODIC_TX_EVT,500);
        }
        else
        {
            LOG("SKIP TX_EVT Current Stats %d\n",s_phy.Status);
            osal_start_timerEx(PhyPlusPhy_TaskID,PPP_PERIODIC_TX_EVT,20);
        }

        return(events ^ PPP_PERIODIC_TX_EVT);
    }

    if (events & PPP_PERIODIC_RX_EVT)
    {
        if(s_phy.Status==PHYPLUS_RFPHY_IDLE)
        {
            if(s_phy.rxOnlyTO == RFPHY_RX_SCAN_ALLWAYS_ON )
                phy_rf_start_rx(RFPHY_RX_SCAN_ALLWAYS_ON);
        }
        else
        {
            LOG("SKIP RX_EVT Current Stats %d\n",s_phy.Status);
            osal_start_timerEx(PhyPlusPhy_TaskID,PPP_PERIODIC_RX_EVT,20);
        }

        return(events ^ PPP_PERIODIC_RX_EVT);
    }

    if(events & PPP_RX_PENDING_PROCESS_EVT)
    {
        phy_rf_process_recv();
        return(events ^ PPP_RX_PENDING_PROCESS_EVT);
    }

    if(events & PPP_TX_PENDING_PROCESS_EVT)
    {
        phy_rf_process_tsmt();
        return(events ^ PPP_TX_PENDING_PROCESS_EVT);
    }

    if(events & PPP_RX_DATA_PROCESS_EVT)
    {
        phy_rx_data_process();
        return(events ^ PPP_RX_DATA_PROCESS_EVT);
    }

    if(events & PPP_TX_DONE_EVT)
    {
        process_tx_done_evt();
        return(events ^ PPP_TX_DONE_EVT);
    }

    if(events & PPP_RX_DONE_EVT)
    {
        process_rx_done_evt();
        return(events ^ PPP_RX_DONE_EVT);
    }

    if(events & PPP_TRX_DONE_EVT)
    {
        process_trx_done_evt();
        return(events ^ PPP_TRX_DONE_EVT);
    }

    return 0;
}

/*********************************************************************
*********************************************************************/
