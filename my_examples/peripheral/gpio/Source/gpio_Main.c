/*
 * @Author: xuhongv | 半颗心脏 xuhongv@yeah.net
 * @Date: 2022-11-23 17:22:43
 * @LastEditors: xuhongv | 半颗心脏 xuhongv@yeah.net
 * @LastEditTime: 2022-11-23 17:40:56
 * @FilePath: \PHY6252_6222_SDK\my_examples\peripheral\gpio\Source\gpio_Main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"

#include "ll_sleep.h"


#include "uart.h"
/**************************************************************************************************
    FUNCTIONS
 **************************************************************************************************/
#include "comdef.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_Timers.h"
#include "OSAL_PwrMgr.h"

#include "timer.h"
#include "ll_sleep.h"
#include "jump_function.h"
#include "global_config.h"
extern pwrmgr_attribute_t pwrmgr_attribute;
extern uint32 ll_remain_time;

/*********************************************************************
    EXTERNAL VARIABLES
*/


/**************************************************************************************************
    @fn          main

    @brief       Start of application.

    @param       none

    @return      none
 **************************************************************************************************
*/
int app_main(void)
{
    /* Initialize the operating system */
    osal_init_system();
    osal_pwrmgr_device( PWRMGR_BATTERY );
    /* Start OSAL */
    osal_start_system(); // No Return from here
    return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
