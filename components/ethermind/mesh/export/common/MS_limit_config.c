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
/* ------------------------------- Header File Inclusion */
#include "MS_brr_api.h"
#include "MS_prov_api.h"
#include "MS_access_api.h"
#include "blebrr.h"
#include "ll.h"
#include "MS_trn_api.h"
#include "pwrmgr.h"
#include "ll_sleep.h"
#include "led_light.h"
#include "access_internal.h"
#include "trn_extern.h"
#include "net_extern.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Global Definitions */

/** Element Count */
UINT8 ms_access_element_count;

/** Model Count */
UINT8 ms_access_model_count;

/** SubNet&LPNs Count */
UINT8 ms_max_subnets,ms_max_lpns;

/** Count of Device Key Entries */
UINT16 ms_max_dev_keys;

/** Count of AppKey Entries */
UINT8 ms_max_apps;

/** Count of Virtual&non-Virtual Address Entries */
UINT8 ms_max_virtual_addrs,ms_max_non_virtual_addrs;

/** Count of proxy filter**/
UINT8 ms_proxy_filter_list_size;






/* --------------------------------------------- Macros */

/* --------------------------------------------- Structures/Data Types */



/* --------------------------------------------- Global Variables */
/** Element List */
MS_DEFINE_GLOBAL_ARRAY(MS_ACCESS_ELEMENT_TYPE, ms_access_element_list, MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT));

/** Model List */
MS_DEFINE_GLOBAL_ARRAY(MS_ACCESS_MODEL_TYPE, ms_access_model_list, MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT));

MS_DEFINE_GLOBAL_ARRAY(MS_NETKEY_ENTRY, ms_subnet_table, (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)));

MS_DEFINE_GLOBAL_ARRAY(MS_NET_ADDR, ms_key_refresh_whitelist, (MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS)));


/**
    List of Element Address Entries.

    0-th entry is for local device.
    Rest for the friend elements.
*/
MS_DEFINE_GLOBAL_ARRAY(MS_ELEMENT_ADDR_ENTRY, ms_element_addr_table, 1 + MS_CONFIG_LIMITS(MS_MAX_LPNS));

#ifdef MS_FRIEND_SUPPORT
    /** LowPower Node Element Database */
    MS_DEFINE_GLOBAL_ARRAY(TRN_LPN_ELEMENT, lpn_element, MS_CONFIG_LIMITS(MS_MAX_LPNS));
#endif

/** List of Device Key Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_DEV_KEY_ENTRY, ms_dev_key_table, MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS));

/** List of AppKey Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_APPKEY_ENTRY, ms_appkey_table, MS_CONFIG_LIMITS(MS_MAX_APPS));

/**
    All addresses - Unicast, Group and Virtual Addresses are maintained
    in following two tables.

    Subscription list will index to both these tables.
    Index 0 to (MS_MAX_NON_VIRTUAL_ADDRS - 1) will correspond to non-virtual address.
    Index MS_MAX_NON_VIRTUAL_ADDRS to (MS_MAX_ADDRS - 1) will correspond to virtual address.

    Note: MS_MAX_ADDRS is defined as (MS_MAX_NON_VIRTUAL_ADDRS + MS_MAX_VIRTUAL_ADDRS)
*/

/** List of Virtual Address Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_VIRTUAL_ADDR_ENTRY, ms_virtual_addr_table, MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS));

/** List of non-Virtual Address Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_NON_VIRTUAL_ADDR_ENTRY, ms_non_virtual_addr_table, MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));

#ifdef MS_PROXY_SERVER
    MS_DEFINE_GLOBAL_ARRAY(PROXY_FILTER_LIST, net_proxy_list, PROXY_FILTER_LIST_COUNT);
#endif




/* ------------------------------- Functions */
void MS_limit_config(void)
{
    ms_access_element_count = MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT);
    ms_access_model_count = MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT);
    ms_max_subnets = MS_CONFIG_LIMITS(MS_MAX_SUBNETS);
    ms_max_lpns = MS_CONFIG_LIMITS(MS_MAX_LPNS);
    ms_max_dev_keys = MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS);
    ms_max_apps = MS_CONFIG_LIMITS(MS_MAX_APPS);
    ms_max_virtual_addrs = MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS);
    ms_max_non_virtual_addrs = MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS);
    ms_proxy_filter_list_size = MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE);
}

