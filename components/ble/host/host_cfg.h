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
**************************************************************************************************/

#ifndef HOST_CFG_H
#define HOST_CFG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "sm.h"
#include "linkdb.h"
#include "l2cap_internal.h"
#include "sm_internal.h"
#include "gap_internal.h"
#include "att_internal.h"
#include "gatt_internal.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
// This is the link database, 1 record for each connection
static linkDBItem_t glinkDB[MAX_NUM_LL_CONN];

// Table of callbacks to make when a connection changes state
static pfnLinkDBCB_t glinkCBs[MAX_NUM_LL_CONN+LINKDB_STACK_CALLBACK_NUM];
static smPairingParams_t* smPairingParam[MAX_NUM_LL_CONN];
static uint16 gMTU_Size[MAX_NUM_LL_CONN];
gapAuthStateParams_t* gAuthenLink[MAX_NUM_LL_CONN];
l2capReassemblePkt_t l2capReassembleBuf[MAX_NUM_LL_CONN];
l2capSegmentBuff_t   l2capSegmentBuf[MAX_NUM_LL_CONN];
gattClientInfo_t    gattClientInfo[GATT_MAX_NUM_CONN];
gattServerInfo_t    gattServerInfo[GATT_MAX_NUM_CONN];

/*********************************************************************
    FUNCTIONS
*/
extern uint8 Host_InitContext(       uint8 max_link_num,
                                     linkDBItem_t* plinkDB,pfnLinkDBCB_t* plinkCBs,
                                     smPairingParams_t** param,
                                     uint16* mtu_size_buf,
                                     gapAuthStateParams_t** Authen_link,
                                     l2capReassemblePkt_t* ressembleBuf,l2capSegmentBuff_t* segmentBuf,
                                     gattClientInfo_t* clientInfo,
                                     gattServerInfo_t* serverInfo);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HOST_CFG_H */
