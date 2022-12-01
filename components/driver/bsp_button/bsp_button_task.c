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
    Filename:       bsp_button_task.c
    Revised:        $Date $
    Revision:       $Revision $
**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include <string.h>
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "pwrmgr.h"
#include "bsp_button_task.h"

uint8 Bsp_Btn_TaskID;

bool bsp_btn_timer_flag = FALSE;
bool bsp_btn_gpio_flag = FALSE;
bool bsp_btn_kscan_flag = FALSE;
bsp_btn_callback_t bsp_btn_cb = NULL;

uint8_t Bsp_Btn_Get_Index(gpio_pin_e pin)
{
    uint8_t i;

    if (hal_gpio_btn_get_index(pin, &i) == PPlus_SUCCESS)
    {

        return (i);
    }

    return 0xFF;
}

static void Bsp_Btn_Check(uint8_t ucKeyCode)
{

    hal_gpio_btn_cb(ucKeyCode);

    if (bsp_btn_timer_flag == TRUE)
    {
        if (bsp_KeyEmpty() == TRUE)
        {
            osal_stop_timerEx(Bsp_Btn_TaskID, BSP_BTN_EVT_SYSTICK);
            bsp_btn_timer_flag = FALSE;
        }
    }
}

void gpio_btn_pin_event_handler(gpio_pin_e pin, IO_Wakeup_Pol_e type)
{

    if (((GPIO_SINGLE_BTN_IDLE_LEVEL == 0) && (POL_RISING == type)) ||
        ((GPIO_SINGLE_BTN_IDLE_LEVEL == 1) && (POL_RISING != type)))
    {
        bsp_btn_timer_flag = TRUE;
        osal_start_reload_timer(Bsp_Btn_TaskID, BSP_BTN_EVT_SYSTICK, BTN_SYS_TICK);
        bsp_set_key_value_by_index(Bsp_Btn_Get_Index(pin), 1);
    }
    else
    {
        bsp_set_key_value_by_index(Bsp_Btn_Get_Index(pin), 0);
    }
}

void Bsp_Btn_Init(uint8 task_id)
{
    Bsp_Btn_TaskID = task_id;

    if (bsp_btn_gpio_flag == TRUE)
    {
        ;
    }
    else
    {
        LOG("btn config error %d %d %d\n", __LINE__, bsp_btn_gpio_flag, bsp_btn_kscan_flag);
        return;
    }

    for (int i = 0; i < BSP_TOTAL_BTN_NUM; i++)
    {
        usr_sum_btn_array[i].KeyConfig = (BSP_BTN_PD_CFG | BSP_BTN_UP_CFG | BSP_BTN_LPS_CFG | BSP_BTN_LPK_CFG);
    }

#if (BSP_COMBINE_BTN_NUM > 0)
    if (PPlus_SUCCESS != bsp_InitBtn(usr_sum_btn_array, BSP_TOTAL_BTN_NUM, BSP_SINGLE_BTN_NUM, usr_combine_btn_array))
#else
    if (PPlus_SUCCESS != bsp_InitBtn(usr_sum_btn_array, BSP_TOTAL_BTN_NUM, 0, NULL))
#endif

    {
        LOG("bsp button init error\n");
    }

    // hal_pwrmgr_register(MOD_USR8, NULL, wake_test);
}

uint16 Bsp_Btn_ProcessEvent(uint8 task_id, uint16 events)
{
    uint8_t ucKeyCode;

    if (Bsp_Btn_TaskID != task_id)
    {
        return 0;
    }

    if (events & BSP_BTN_EVT_SYSTICK)
    {

        ucKeyCode = bsp_KeyPro();

        if (ucKeyCode != BTN_NONE)
        {
            Bsp_Btn_Check(ucKeyCode);
        }

        return (events ^ BSP_BTN_EVT_SYSTICK);
    }

    return 0;
}
