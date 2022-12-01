
#include <string.h>
#include "gpio.h"
#include "bsp_gpio.h"
#include "kscan.h"
#include "log.h"
#include "bsp_button.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "button.h"
#include "bsp_button_task.h"
#include "bsp_gpio.h"

static uint8 Demo_TaskID;

BTN_T usr_sum_btn_array[BSP_TOTAL_BTN_NUM];

#if (BSP_COMBINE_BTN_NUM > 0)
uint32_t usr_combine_btn_array[BSP_COMBINE_BTN_NUM] =
{
    (BIT(0) | BIT(1)),
    (BIT(0) | BIT(2)),
};
#endif

#define LED_H(io)                            \
    {                                        \
        hal_gpio_pull_set(io, STRONG_PULL_UP); \
        hal_gpio_write(io, 1);               \
    }
#define LED_L(io)                         \
    {                                     \
        hal_gpio_pull_set(io, PULL_DOWN); \
        hal_gpio_write(io, 0);            \
    }

// for  ai-thinker PB-03-Kit GREEN LED
#define LED_GPIO GPIO_P18
#define LED_GPIO1 GPIO_P07
#define LED_GPIO2 GPIO_P11

void LED_blink(GPIO_Pin_e pin)
{
    LED_H(pin);
    //延迟一秒
    WaitMs(1000);
    LED_L(pin);
}

void LED_toggle(GPIO_Pin_e pin)
{
    static uint8 state = 1;
    if (state)
    {
        LED_H(pin);
        state = 0;
    }
    else
    {
        LED_L(pin);
        state = 1;
    }
}

void hal_bsp_btn_callback(uint8_t evt)
{

    switch (BSP_BTN_TYPE(evt))
    {
    case BSP_BTN_PD_TYPE:
        LOG("press down ");
        break;

    case BSP_BTN_UP_TYPE:
        LOG("press up ");
        // LED 转换
        LED_toggle(LED_GPIO);

        break;

    case BSP_BTN_LPS_TYPE:
        LOG("long press start ");
        break;

    case BSP_BTN_LPK_TYPE:
        LOG("long press keep ");
        break;

    default:
        LOG("unexpected ");
        break;
    }
}

Gpio_Btn_Info gpio_btn_info =
{
    {GPIO_P15},
    hal_bsp_btn_callback,
};

void Key_Demo_Init(uint8 task_id)
{
    Demo_TaskID = task_id;

    LED_H(LED_GPIO);
    LED_L(LED_GPIO1);
    LED_L(LED_GPIO2);

    LOG("Key_Demo_Init key=%d \n", gpio_btn_info.s_key[0]);

    if (PPlus_SUCCESS == hal_gpio_btn_init(&gpio_btn_info))
    {
        bsp_btn_gpio_flag = TRUE;
    }
    else
    {
        LOG("hal_gpio_btn_init error:%d\n", __LINE__);
    }
}

uint16 Key_ProcessEvent(uint8 task_id, uint16 events)
{
    if (Demo_TaskID != task_id)
    {
        return 0;
    }

    return 0;
}
