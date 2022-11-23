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
#include "rom_sym_def.h"
#include "timer.h"
#include "error.h"
#include "clock.h"
#include "pwrmgr.h"
#include "jump_function.h"

#ifndef TIM2_IRQHANDLER1_ENABLE
    #define TIM2_IRQHANDLER1_ENABLE 0
#endif

#if (TIM2_IRQHANDLER1_ENABLE==TRUE)
#define JUMP_TIM2_IRQHANDLER1_FUNCTION() do { \
        JUMP_FUNCTION(V21_IRQ_HANDLER) = (uint32_t)&TIM2_IRQHandler1; \
    }while(0);
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_PendingTick;
#endif

AP_TIM_TypeDef* const TimerIndex[FREE_TIMER_NUMBER]= {AP_TIM5,AP_TIM6};
static ap_tm_hdl_t s_ap_callback = NULL;

static int hal_timer_clear_int(AP_TIM_TypeDef* TIMx)
{
    return TIMx->EOI;
}

static void hal_timer_stop_counter(AP_TIM_TypeDef* TIMx)
{
    TIMx->ControlReg = 0;
    TIMx->LoadCount = 0;   //0x0
    TIMx->CurrentCount = 0;//0x4
}

static void hal_timer_set_loadtimer(AP_TIM_TypeDef* TIMx, int time)
{
    if(time>0)
    {
        TIMx->ControlReg = 0x0;
        TIMx->ControlReg = 0x2;
        TIMx->LoadCount = 4*time;// 4MHz system timer, * 4 to convert to 1MHz timer
        TIMx->ControlReg = 0x3;
    }
    else
    {
        TIMx->ControlReg = 0x0;
    }
}
#if (TIM2_IRQHANDLER1_ENABLE==TRUE)
void TIM2_IRQHandler1(void)
{
    HAL_ENTER_CRITICAL_SECTION();

    if (AP_TIM2->status & 0x1)
    {
        clear_timer_int(AP_TIM2);
        g_TIM2_IRQ_TIM3_CurrCount = AP_TIM3->CurrentCount;
        g_TIM2_IRQ_PendingTick = AP_TIM2->CurrentCount;
        uint32_t tim2_delay_tick, tim3_DeltTick;
        static uint32_t AP_TIM3_LastCount = 0, Tim3_tick_Remainder = 0;
        tim3_DeltTick = (AP_TIM3_LastCount > g_TIM2_IRQ_TIM3_CurrCount)
                        ? (AP_TIM3_LastCount - g_TIM2_IRQ_TIM3_CurrCount + Tim3_tick_Remainder)
                        : (0xFFFFFD - g_TIM2_IRQ_TIM3_CurrCount + AP_TIM3_LastCount + Tim3_tick_Remainder);
        AP_TIM3_LastCount = g_TIM2_IRQ_TIM3_CurrCount;
        tim2_delay_tick = (tim3_DeltTick / 2500);

        if (tim2_delay_tick >= 1)
        {
            Tim3_tick_Remainder = (tim3_DeltTick - tim2_delay_tick * 2500);
            osal_sys_tick += tim2_delay_tick;
        }
        else
        {
            Tim3_tick_Remainder = (2500 - tim3_DeltTick);
            osal_sys_tick++;
        }
    }

    HAL_EXIT_CRITICAL_SECTION();
}
#endif

void __attribute__((used)) hal_TIMER5_IRQHandler(void)
{
    if(AP_TIM5->status & 0x1)
    {
        hal_timer_clear_int(AP_TIM5);

        if(s_ap_callback)
            s_ap_callback(HAL_EVT_TIMER_5);
    }
}

void __attribute__((used)) hal_TIMER6_IRQHandler(void)
{
    if(AP_TIM6->status & 0x1)
    {
        hal_timer_clear_int(AP_TIM6);

        if(s_ap_callback)
            s_ap_callback(HAL_EVT_TIMER_6);
    }
}

static void hal_timer_wakeup_handler(void)
{
    if(s_ap_callback)
        s_ap_callback(HAL_EVT_WAKEUP);
}

void hal_timer_sleep_handler(void)
{
    if(s_ap_callback)
        s_ap_callback(HAL_EVT_SLEEP);
}

int hal_timer_mask_int(User_Timer_e timeId, bool en)
{
    volatile AP_TIM_TypeDef* TIMx;
    TIMx = TimerIndex[timeId-AP_TIMER_ID_5];

    if(en)
        TIMx->ControlReg |= (1 << 2);
    else
        TIMx->ControlReg &= ~(1 << 2);

    return PPlus_SUCCESS;
}

int hal_timer_set(User_Timer_e timeId, uint32_t us)
{
    uint32_t time = us;

    switch(timeId)
    {
    case AP_TIMER_ID_5:
        JUMP_FUNCTION(TIM5_IRQ_HANDLER)                  =   (uint32_t)&hal_TIMER5_IRQHandler;
        NVIC_EnableIRQ((IRQn_Type)TIM5_IRQn);
        NVIC_SetPriority((IRQn_Type)TIM5_IRQn, IRQ_PRIO_HAL);
        hal_timer_set_loadtimer(AP_TIM5, time);
        hal_clk_gate_enable(MOD_TIMER5);
        break;

    case AP_TIMER_ID_6:
        JUMP_FUNCTION(TIM6_IRQ_HANDLER)                  =   (uint32_t)&hal_TIMER6_IRQHandler;
        NVIC_EnableIRQ((IRQn_Type)TIM6_IRQn);
        NVIC_SetPriority((IRQn_Type)TIM6_IRQn, IRQ_PRIO_HAL);
        hal_timer_set_loadtimer(AP_TIM6, time);
        hal_clk_gate_enable(MOD_TIMER6);
        break;

    default:
        return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

int hal_timer_stop(User_Timer_e timeId)
{
    switch(timeId)
    {
    case AP_TIMER_ID_5:
        JUMP_FUNCTION(TIM5_IRQ_HANDLER) = 0;
        hal_timer_stop_counter(AP_TIM5);
        NVIC_DisableIRQ((IRQn_Type)TIM5_IRQn);
        hal_clk_gate_disable(MOD_TIMER5);
        break;

    case AP_TIMER_ID_6:
        JUMP_FUNCTION(TIM6_IRQ_HANDLER) = 0;
        hal_timer_stop_counter(AP_TIM6);
        NVIC_DisableIRQ((IRQn_Type)TIM6_IRQn);
        hal_clk_gate_disable(MOD_TIMER6);
        break;

    default:
        return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

int hal_timer_init(ap_tm_hdl_t callback)
{
    s_ap_callback = callback;
    hal_timer_stop(AP_TIMER_ID_5);
    hal_timer_stop(AP_TIMER_ID_6);
    return hal_pwrmgr_register(MOD_TIMER, hal_timer_sleep_handler, hal_timer_wakeup_handler);
}

int hal_timer_deinit(void)
{
    s_ap_callback = NULL;
    return hal_pwrmgr_unregister(MOD_TIMER);
}
