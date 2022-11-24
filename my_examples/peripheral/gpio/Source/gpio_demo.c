/*
 * @Author: xuhongv | 半颗心脏 xuhongv@yeah.net
 * @Date: 2022-11-23 17:32:11
 * @LastEditors: xuhongv | 半颗心脏 xuhongv@yeah.net
 * @LastEditTime: 2022-11-24 10:38:25
 * @FilePath: \PHY6252_6222_SDK\my_examples\peripheral\gpio\Source\gpio_demo.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "OSAL.h"
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "key.h"

static uint8 Blink_TaskID;
static uint8 Blink_Task_EVT = 0xF0;

// for  ai-thinker PB-03-Kit GREEN LED
#define LED_GPIO GPIO_P11

void LED_Toggle(GPIO_Pin_e pin)
{
    hal_gpio_pin_init(pin, OEN);
    if (hal_gpio_read(pin))
        hal_gpio_write(pin, 0);
    else
        hal_gpio_write(pin, 1);
}

void Blink_Init(uint8 task_id)
{

    LOG("%s task_id=%d \n", __FUNCTION__, task_id);

    Blink_TaskID = task_id;

    // LED 转换
    hal_gpio_pin_init(LED_GPIO, OEN);
    hal_gpio_write(LED_GPIO, 1);

    osal_start_timerEx(Blink_TaskID, Blink_Task_EVT, 1000);
}

uint16 Blink_ProcessEvent(uint8 task_id, uint16 events)
{
    if (task_id != Blink_TaskID)
    {
        return 0;
    }

    if (events & Blink_Task_EVT)
    {
        // LED 转换
        LED_Toggle(LED_GPIO);
        //延迟一秒
        WaitMs(1000);
        LOG("%s \n", __FUNCTION__);
        osal_start_timerEx(Blink_TaskID, Blink_Task_EVT, 2000);
        return (events ^ Blink_Task_EVT);
    }
    return 0;
}
