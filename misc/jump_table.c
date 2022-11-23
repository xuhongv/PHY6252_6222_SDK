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
    Filename:       jump_table.c
    Revised:
    Revision:

    Description:    Jump table that holds function pointers and veriables used in ROM code.


**************************************************************************************************/

/*******************************************************************************
    INCLUDES
*/
#include "jump_function.h"
#include "global_config.h"
#include "OSAL_Tasks.h"
#include "rf_phy_driver.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include "log.h"


#define DEF_HARD_FAULT_LOG_DEBUG                (1)

//#define DEF_HARD_FAULT_FS_ENABLE 1


/*******************************************************************************
    MACROS
*/


/*
    HF FS Recorder Description
    HF_NUM_FSID: [0xFE00], save the total number of hard fault records
    HF_REC_FSID: [0xFE01 -- 0xFE00+MAX_FS_HARD_FAULT_REC_NUM]
            only save the latest HF_REC,when HF_SUM_FSID > MAX_FS_HARD_FAULT_REC_NUM
            , the earliset recorder will be replaced
    HF_REC Total Size:
    MAX_FS_HARD_FAULT_REC_NUM *(FSID_HARD_FAULT_SIZE + 1+ FS_Header*2)

*/


#if( DEF_HARD_FAULT_FS_ENABLE)
    #include "fs.h"
#endif

#define DEF_HARD_FAULT_DUMP_SP_LEN          256
#define MAX_FS_HARD_FAULT_REC_NUM           16  // should be 2**n

#define FSID_HARD_FAULT_BASE                0xFE00
#define FSID_HARD_FAULT_NUM                 (FSID_HARD_FAULT_BASE)

#define FSID_HARD_FAULT_CURRENT(x)          (FSID_HARD_FAULT_BASE+1+((x)&(MAX_FS_HARD_FAULT_REC_NUM-1) ) )

#define FSID_REC_EXTRA_BUF_LEN              4  // for hf rec , 2 for osal event 1 for osal sys tick, 1 for rtc tick 

#define FSID_HARD_FAULT_SIZE                ((17+1+FSID_REC_EXTRA_BUF_LEN)*4+DEF_HARD_FAULT_DUMP_SP_LEN)
/*
    17 for cpu reg,
    1 for ICSR,
*/

#if(DEF_HARD_FAULT_LOG_DEBUG)

    #define HF_DBG_LOG_PINOUT                   P9
    #define HF_DBG_LOG                          log_printf
    #define HF_DBG_LOG_baudrate                 115200
    #define HF_DBG_DUMMY_IO                     P27
    #define HF_DBG_LOG_INIT                     {rom_uart_init(HF_DBG_LOG_baudrate,HF_DBG_LOG_PINOUT,HF_DBG_DUMMY_IO,NULL);}

#else

    #define HF_DBG_LOG(...)
    #define HF_DBG_LOG_INIT

#endif


void (*trap_c_callback)(void);
extern void log_printf(const char* format, ...);
extern uint32_t rtc_get_counter(void);
extern uint32 osal_sys_tick;
extern int rom_uart_init(int baud, gpio_pin_e tx_pin, gpio_pin_e rx_pin, comm_cb_t cb);


#ifdef DEF_HARD_FAULT_FS_ENABLE

uint8 _hard_fault_fs_write( uint16_t id, uint16_t len, uint8_t* pBuf)
{
    AP_WDT_FEED;
    int ret = PPlus_SUCCESS;
    uint32_t free_size = 0;
    hal_fs_get_free_size(&free_size);

    if( free_size < len+32)
    {
        if(hal_fs_get_garbage_size(NULL) > len+32)
        {
            uint32_t t0 = rtc_get_counter();
            hal_fs_garbage_collect();
            HF_DBG_LOG("DO HF_FS Garbage Collect Cost %d (rtcTick) \n",rtc_get_counter()-t0);
        }
        else
        {
            HF_DBG_LOG("hf_wr fail\n");
            return NV_OPER_FAILED;
        }
    }

    ret = hal_fs_item_write(id,  pBuf,len);

    if(ret !=0)
    {
        HF_DBG_LOG("hf_wr_ret:%d\n",ret);
        return NV_OPER_FAILED;
    }

    //LOG("Success\n");
    return SUCCESS;
}

void _hard_fault_record_process(uint32_t* stk,uint32_t* buf,uint16_t len)
{
    uint32_t rec[FSID_HARD_FAULT_SIZE/4];
    int i;
    fs_set_psr_protection(0);

    for(i=0; i<len; i++)
        rec[i]=buf[i];

    for(i=0; i<17; i++)
        rec[len+i]=stk[i];

    rec[len+17]=(*(volatile uint32_t*)0xE000ED04);
    uint32 sp = stk[0]&0x1ffffffc;

    for(i=0; i<DEF_HARD_FAULT_DUMP_SP_LEN; i=i+4)
    {
        if(sp+i>0x1ffffffc)
            break;

        rec[len+18+i/4]=read_reg(sp+i);
    }

    uint8_t rec_num;
    uint16_t rec_id;
    int ret=hal_fs_item_read(FSID_HARD_FAULT_NUM,&rec_num,1,NULL);

    if(ret==PPlus_ERR_FS_NOT_FIND_ID)
    {
        rec_num=0;
        rec_id=FSID_HARD_FAULT_CURRENT(rec_num);
    }
    else if(ret==PPlus_SUCCESS)
    {
        rec_num=rec_num+1;
        rec_id=FSID_HARD_FAULT_CURRENT(rec_num);
    }
    else
    {
        HF_DBG_LOG("\n[HF REC]fs err:%d\n",ret);
        return;
    }

    HF_DBG_LOG("\n[HF REC]save:num %d id %04x\n",rec_num,rec_id);
    _hard_fault_fs_write(FSID_HARD_FAULT_NUM,1,&rec_num);
    _hard_fault_fs_write(rec_id,FSID_HARD_FAULT_SIZE,(uint8_t*)rec);
}
void _hard_fault_record_load(void)
{
    uint8_t rec_num;
    uint32_t rec[FSID_HARD_FAULT_SIZE/4];
    int i,j;
    #if(DEBUG_INFO==0)
    /*
        use rom log when debug info turn off
        otherwise, use system log_printf
    */
    HF_DBG_LOG_INIT;
    #endif
    int ret=hal_fs_item_read(FSID_HARD_FAULT_NUM,&rec_num,1,NULL);

    if(ret==PPlus_SUCCESS)
    {
        log_printf("\n[HF REC] Found recorder Total Num %d\n",rec_num+1);
    }
    else
    {
        log_printf("\n[HF REC] No recorder Found\n");
        return;
    }

    for(i=0; i<MAX_FS_HARD_FAULT_REC_NUM; i++)
    {
        AP_WDT_FEED;
        ret=hal_fs_item_read(FSID_HARD_FAULT_CURRENT(i),(uint8_t*)rec,FSID_HARD_FAULT_SIZE,NULL);

        if(ret==SUCCESS)
        {
            HF_DBG_LOG("\n[HF REC] Load ID = [%04x]================\n",FSID_HARD_FAULT_CURRENT(i));
            uint32_t* stk = &rec[FSID_REC_EXTRA_BUF_LEN];
            HF_DBG_LOG("R0-R3        = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[9], stk[10], stk[11], stk[12]);
            HF_DBG_LOG("R4-R7        = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[1], stk[2], stk[3], stk[4]);
            HF_DBG_LOG("R8-R11       = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[5], stk[6], stk[7], stk[8]);
            HF_DBG_LOG("R12,SP,LR,PC = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[13], stk[0], stk[14], stk[15]);
            HF_DBG_LOG("PSR  = 0x%08x  ", stk[16]);
            HF_DBG_LOG("ICSR = 0x%08x\n", stk[17]);
            HF_DBG_LOG("[OSAL]idx %d Func 0x%08x systick %08x rtc %08x\n ",rec[0],rec[1],rec[2],rec[3]);
            HF_DBG_LOG("-----------dump stack--------------\n");

            for(j=0; j<DEF_HARD_FAULT_DUMP_SP_LEN; j=j+4)
            {
                if(stk[0]+j>0x1ffffffc)
                    break;

                if(0==(j&0x0f))
                    HF_DBG_LOG("\n[%08X]",stk[0]+j);

                HF_DBG_LOG("%08x ",stk[18+j/4]);
            }

            HF_DBG_LOG("\n");
        }
        else
            break;
    }
}

#endif


#if   defined ( __CC_ARM )

void _hard_fault(uint32_t* arg)
{
    uint32_t* stk = (uint32_t*)((uint32_t)arg);
    uint8 task_cnt = *(uint8*)JUMP_FUNCTION(TASK_COUNT);
    pTaskEventHandlerFn pFunc=NULL;
    uint8_t* p_activeTaskID = (uint8_t*)0x1fff08b4;//activeTaskID
    uint32_t buf[FSID_REC_EXTRA_BUF_LEN];
    HF_DBG_LOG_INIT;
    HF_DBG_LOG("\n[Hard fault handler]\n");
    HF_DBG_LOG("R0-R3        = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[9], stk[10], stk[11], stk[12]);
    HF_DBG_LOG("R4-R7        = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[1], stk[2], stk[3], stk[4]);
    HF_DBG_LOG("R8-R11       = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[5], stk[6], stk[7], stk[8]);
    HF_DBG_LOG("R12,SP,LR,PC = 0x%08x 0x%08x 0x%08x 0x%08x\n", stk[13], stk[0], stk[14], stk[15]);
    HF_DBG_LOG("PSR  = 0x%08x  ", stk[16]);
    HF_DBG_LOG("ICSR = 0x%08x\n", *(volatile uint32_t*)0xE000ED04);

    if(p_activeTaskID[0]<task_cnt)
        pFunc = ((pTaskEventHandlerFn*)(JUMP_FUNCTION(TASKS_ARRAY)))[p_activeTaskID[0]];

    buf[0]=p_activeTaskID[0];
    buf[1]=(uint32_t)pFunc;
    buf[2]=osal_sys_tick;
    buf[3]=rtc_get_counter();
    HF_DBG_LOG("[OSAL]idx %d Func 0x%08x systick %08x rtc %08x\n ",buf[0],buf[1],buf[2],buf[3]);
    (void) buf;
    HF_DBG_LOG("-----------dump stack--------------\n");
    uint32 sp = stk[0]&0x1ffffffc;

    for(int i=0; i<DEF_HARD_FAULT_DUMP_SP_LEN; i=i+4)
    {
        if(sp+i>0x1ffffffc)
            break;

        if(0==(i&0x0f))
            HF_DBG_LOG("\n[%08X]",sp+i);

        HF_DBG_LOG("%08x ",read_reg(sp+i));
    }

    #if( DEF_HARD_FAULT_FS_ENABLE)
    _hard_fault_record_process(stk,buf,FSID_REC_EXTRA_BUF_LEN);
    #endif

    if (trap_c_callback)
    {
        trap_c_callback();
    }

    while (1);
}
// *INDENT-OFF*
__asm void hard_fault(void)
{
    PRESERVE8
    IMPORT  _hard_fault
    ldr     r0, = 0x1FFF0800 /*store in global config 0x1fff0400 0x1fff0800*/
    subs    r0, r0, #72
    mov     r1, sp
    str     r1, [r0]
    adds    r0, #4
    stmia   r0!, {r4 - r7}
    mov     r4, r8
    mov     r5, r9
    mov     r6, r10
    mov     r7, r11
    stmia   r0!, {r4 - r7}
    pop     {r4 - r5} /* pop rom Hardfault stack*/
    pop     {r4 - r7} /* pop exception entry R0-R1*/
    stmia   r0!, {r4 - r7}
    pop     {r4 - r7}/* pop exception entry R12 LR PC xPSR*/
    stmia   r0!, {r4 - r7}
    subs    r0, r0, #68
    ldr     r1, = _hard_fault
    ldr     r2, = 0x1FFF1830 /*mov sp to rom initial_sp*/
    mov     sp, r2
    bx      r1
    ALIGN   4
}

#elif defined ( __GNUC__ )
static void hard_fault(void)
{
    while (1) {
        ;
    }

}


#endif
// *INDENT-ON*

/*******************************************************************************
    CONSTANTS
*/
// jump table, this table save the function entry which will be called by ROM code
// item 1 - 4 for OSAL task entry
// item 224 - 255 for ISR(Interrupt Service Routine) entry
// others are reserved by ROM code
const uint32_t* const jump_table_base[256] __attribute__((section("jump_table_mem_area"))) =
{
    (const uint32_t*)0,                         // 0. write Log
    (const uint32_t*)osalInitTasks,             // 1. init entry of app
    (const uint32_t*)tasksArr,                  // 2. task list
    (const uint32_t*)& tasksCnt,                // 3. task count
    (const uint32_t*)& tasksEvents,             // 4. task events
    0, 0, 0, 0, 0,                              // 5 - 9, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 10 - 19, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 20 - 29, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0,                     // <30 - - 37>
    0, 0,
    0, 0, 0, 0, 0, 0, //40 - 45
    0, 0, 0, 0,                                 //46 - 49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 50 - 59, reserved for rom patch
    0,   // < 60 -
    0,
    0,
    0,
    0, 0, 0, 0, 0, 0,                           //  -69>, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 70 -79, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 80 - 89, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,               // 90 - 99, reserved for rom patch
    (const uint32_t*)hal_pwrmgr_sleep_process,         // <100 -
    (const uint32_t*)hal_pwrmgr_wakeup_process,
    (const uint32_t*)rf_phy_ini,
    0,
    0,
    0,
    0, 0, 0, 0,                       // - 109, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 110 -119, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 120 -129, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 130 -139, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 140 -149, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 150 -159, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 160 -169, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 170 -179, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 180 -189, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 190 -199, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 200 - 209, reserved for rom patch
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,     // 210 - 219, reserved for rom patch
    (const uint32_t*)hard_fault, 0, 0, 0, 0, 0, 0, 0,           // 220 - 227
    0, 0,       // 228 - 229
    0, 0, 0, 0, 0,  // 230 - 234
    (const uint32_t*)hal_UART0_IRQHandler,      // 235 uart irq handler
    0, 0, 0, 0, 0,    // 236 - 240
    0, 0, 0, 0, 0, 0, 0, 0, 0,     // 241 - 249, for ISR entry
    0, 0, 0, 0, 0, 0                  // 250 - 255, for ISR entry
};



/*******************************************************************************
    Prototypes
*/


/*******************************************************************************
    LOCAL VARIABLES
*/


/*********************************************************************
    EXTERNAL VARIABLES
*/
uint32 global_config[SOFT_PARAMETER_NUM] __attribute__((section("global_config_area")));




