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
/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "osal_cbTimer.h"
#include "osal_snv.h"
#include "hci_tl.h"
#include "l2cap.h"
#include "linkdb.h"
#include "gap.h"
#include "gapbondmgr.h"
#include "central.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

// Profile Events
#define START_ADVERTISING_EVT         0x0001
#define RSSI_READ_EVT                 0x0002
#define UPDATE_PARAMS_TIMEOUT_EVT     0x0004

// Profile OSAL Message IDs
#define GAPCENTRALROLE_RSSI_MSG_EVT   0xE0

/*********************************************************************
    TYPEDEFS
*/

// RSSI read data structure
typedef struct
{
    uint16        period;
    uint16        connHandle;
    uint8         timerId;
} gapCentralRoleRssi_t;

// OSAL event structure for RSSI timer events
typedef struct
{
    osal_event_hdr_t      hdr;
    gapCentralRoleRssi_t*  pRssi;
} gapCentralRoleRssiEvent_t;

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

// Task ID
static uint8 gapCentralRoleTaskId;

// App callbacks
static gapCentralRoleCB_t* pGapCentralRoleCB;

// Array of RSSI read structures
static gapCentralRoleRssi_t gapCentralRoleRssi[GAPCENTRALROLE_NUM_RSSI_LINKS];

#ifdef EXT_ADV_ENABLE
    //extcentralCB
    static const gapExtCentralCBs_t* pGapExtCentralCB = NULL;
    //extscan rptconfig
    gapExtScan_AdvInfo_t gapExtScan_AdvInfo[GAP_EXTSCAN_MAX_SCAN_NUM];
#endif
/*********************************************************************
    Profile Parameters - reference GAPCENTRALROLE_PROFILE_PARAMETERS for
    descriptions
*/

static uint8  gapCentralRoleIRK[KEYLEN];
static uint8  gapCentralRoleSRK[KEYLEN];
static uint32 gapCentralRoleSignCounter;
static uint8  gapCentralRoleBdAddr[B_ADDR_LEN];
static uint8  gapCentralRoleMaxScanRes = 0;

/*********************************************************************
    LOCAL FUNCTIONS
*/
static void gapCentralRole_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void gapCentralRole_ProcessGAPMsg( gapEventHdr_t* pMsg );
static gapCentralRoleRssi_t* gapCentralRole_RssiAlloc( uint16 connHandle );
static gapCentralRoleRssi_t* gapCentralRole_RssiFind( uint16 connHandle );
static void gapCentralRole_RssiFree( uint16 connHandle );
static void gapCentralRole_timerCB( uint8* pData );

#ifdef EXT_ADV_ENABLE
    static uint8 gapExtScan_legacyadv_rptfilter;
    bStatus_t GAPExtCentralRole_InitExtScanParams(void);
    bStatus_t GAPExtCentralRole_InitExtConnParams(void);
    void GAPExtCentralRole_Reassemble_Advrpt(hciEvt_BLEExtAdvPktReport_t* pMsg);
#endif
/*********************************************************************
    PUBLIC FUNCTIONS
*/

/**
    @brief   Start the device in Central role.  This function is typically
            called once during system startup.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_StartDevice( gapCentralRoleCB_t* pAppCallbacks )
{
    if ( pAppCallbacks )
    {
        pGapCentralRoleCB = pAppCallbacks;
    }

    return GAP_DeviceInit( gapCentralRoleTaskId, GAP_PROFILE_CENTRAL,
                           gapCentralRoleMaxScanRes, gapCentralRoleIRK,
                           gapCentralRoleSRK, &gapCentralRoleSignCounter );
}

/**
    @brief   Set a parameter in the Central Profile.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_SetParameter( uint16 param, uint8 len, void* pValue )
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {
    case GAPCENTRALROLE_IRK:
        if ( len == KEYLEN )
        {
            VOID osal_memcpy( gapCentralRoleIRK, pValue, KEYLEN ) ;
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPCENTRALROLE_SRK:
        if ( len == KEYLEN )
        {
            VOID osal_memcpy( gapCentralRoleSRK, pValue, KEYLEN ) ;
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPCENTRALROLE_SIGNCOUNTER:
        if ( len == sizeof ( uint32 ) )
        {
            gapCentralRoleSignCounter = *((uint32*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPCENTRALROLE_MAX_SCAN_RES:
        if ( len == sizeof ( uint8 ) )
        {
            gapCentralRoleMaxScanRes = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    default:
        ret = INVALIDPARAMETER;
        break;
    }

    return ret;
}

/**
    @brief   Get a parameter in the Central Profile.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_GetParameter( uint16 param, void* pValue )
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {
    case GAPCENTRALROLE_IRK:
        VOID osal_memcpy( pValue, gapCentralRoleIRK, KEYLEN ) ;
        break;

    case GAPCENTRALROLE_SRK:
        VOID osal_memcpy( pValue, gapCentralRoleSRK, KEYLEN ) ;
        break;

    case GAPCENTRALROLE_SIGNCOUNTER:
        *((uint32*)pValue) = gapCentralRoleSignCounter;
        break;

    case GAPCENTRALROLE_BD_ADDR:
        VOID osal_memcpy( pValue, gapCentralRoleBdAddr, B_ADDR_LEN ) ;
        break;

    case GAPCENTRALROLE_MAX_SCAN_RES:
        *((uint8*)pValue) = gapCentralRoleMaxScanRes;
        break;

    default:
        ret = INVALIDPARAMETER;
        break;
    }

    return ret;
}

/**
    @brief   Terminate a link.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_TerminateLink( uint16 connHandle )
{
    return GAP_TerminateLinkReq( gapCentralRoleTaskId, connHandle, HCI_DISCONNECT_REMOTE_USER_TERM ) ;
}

/**
    @brief   Establish a link to a peer device.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_EstablishLink( uint8 highDutyCycle, uint8 whiteList,
                                        uint8 addrTypePeer, uint8* peerAddr )
{
    gapEstLinkReq_t params;
    params.taskID = gapCentralRoleTaskId;
    params.highDutyCycle = highDutyCycle;
    params.whiteList = whiteList;
    params.addrTypePeer = addrTypePeer;
    VOID osal_memcpy( params.peerAddr, peerAddr, B_ADDR_LEN );
    return GAP_EstablishLinkReq( &params );
}

/**
    @brief   Update the link connection parameters.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_UpdateLink( uint16 connHandle, uint16 connIntervalMin,
                                     uint16 connIntervalMax, uint16 connLatency,
                                     uint16 connTimeout )
{
    return (bStatus_t) HCI_LE_ConnUpdateCmd( connHandle, connIntervalMin,
                                             connIntervalMax, connLatency,
                                             connTimeout, 0, 0 );
}

/**
    @brief   Start a device discovery scan.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_StartDiscovery( uint8 mode, uint8 activeScan, uint8 whiteList )
{
    gapDevDiscReq_t params;
    params.taskID = gapCentralRoleTaskId;
    params.mode = mode;
    params.activeScan = activeScan;
    params.whiteList = whiteList;
    return GAP_DeviceDiscoveryRequest( &params );
}

/**
    @brief   Cancel a device discovery scan.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_CancelDiscovery( void )
{
    return GAP_DeviceDiscoveryCancel( gapCentralRoleTaskId );
}

/**
    @brief   Start periodic RSSI reads on a link.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_StartRssi( uint16 connHandle, uint16 period )
{
    gapCentralRoleRssi_t*  pRssi;

    // Verify link is up
    if (!linkDB_Up(connHandle))
    {
        return bleIncorrectMode;
    }

    // If already allocated
    if ((pRssi = gapCentralRole_RssiFind( connHandle )) != NULL)
    {
        // Stop timer
        osal_CbTimerStop( pRssi->timerId );
    }
    // Allocate structure
    else if ((pRssi = gapCentralRole_RssiAlloc( connHandle )) != NULL)
    {
        pRssi->period = period;
    }
    // Allocate failed
    else
    {
        return bleNoResources;
    }

    // Start timer
    osal_CbTimerStart( gapCentralRole_timerCB, (uint8*) pRssi,
                       period, &pRssi->timerId );
    return SUCCESS;
}

/**
    @brief   Cancel periodic RSSI reads on a link.

    Public function defined in central.h.
*/
bStatus_t GAPCentralRole_CancelRssi(uint16 connHandle )
{
    gapCentralRoleRssi_t*  pRssi;

    if ((pRssi = gapCentralRole_RssiFind( connHandle )) != NULL)
    {
        // Stop timer
        osal_CbTimerStop( pRssi->timerId );
        // Free RSSI structure
        gapCentralRole_RssiFree( connHandle );
        return SUCCESS;
    }

    // Not found
    return bleIncorrectMode;
}

/**
    @brief   Central Profile Task initialization function.

    @param   taskId - Task ID.

    @return  void
*/
void GAPCentralRole_Init( uint8 taskId )
{
    uint8 i;
    gapCentralRoleTaskId = taskId;

    // Initialize internal data
    for ( i = 0; i < GAPCENTRALROLE_NUM_RSSI_LINKS; i++ )
    {
        gapCentralRoleRssi[i].connHandle = GAP_CONNHANDLE_ALL;
        gapCentralRoleRssi[i].timerId = INVALID_TIMER_ID;
    }

    // Initialize parameters
    #ifdef EXT_ADV_ENABLE
    osal_memset(gapExtScan_AdvInfo,0,sizeof(gapExtScan_AdvInfo_t)*GAP_EXTSCAN_MAX_SCAN_NUM);
    GAPExtCentralRole_InitExtScanParams();
    GAPExtCentralRole_InitExtConnParams();
    #endif
    // Retore items from NV
//  VOID osal_snv_read( BLE_NVID_IRK, KEYLEN, gapCentralRoleIRK );
//  VOID osal_snv_read( BLE_NVID_CSRK, KEYLEN, gapCentralRoleSRK );
//  VOID osal_snv_read( BLE_NVID_SIGNCOUNTER, sizeof( uint32 ), &gapCentralRoleSignCounter );
    // Register for HCI messages (for RSSI)
    GAP_RegisterForHCIMsgs( taskId );
}

/**
    @brief   Central Profile Task event processing function.

    @param   taskId - Task ID
    @param   events - Events.

    @return  events not processed
*/
uint16 GAPCentralRole_ProcessEvent( uint8 taskId, uint16 events )
{
    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( gapCentralRoleTaskId )) != NULL )
        {
            gapCentralRole_ProcessOSALMsg( (osal_event_hdr_t*) pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & GAP_EVENT_SIGN_COUNTER_CHANGED )
    {
        // Sign counter changed, save it to NV
        //  VOID osal_snv_write( BLE_NVID_SIGNCOUNTER, sizeof( uint32 ), &gapCentralRoleSignCounter );
        return ( events ^ GAP_EVENT_SIGN_COUNTER_CHANGED );
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      gapCentralRole_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void gapCentralRole_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case HCI_GAP_EVENT_EVENT:
        if ( pMsg->status == HCI_COMMAND_COMPLETE_EVENT_CODE )
        {
            hciEvt_CmdComplete_t* pPkt = (hciEvt_CmdComplete_t*) pMsg;

            if ( pPkt->cmdOpcode == HCI_READ_RSSI )
            {
                uint16 connHandle = BUILD_UINT16( pPkt->pReturnParam[1], pPkt->pReturnParam[2] );
                int8 rssi = (int8) pPkt->pReturnParam[3];

                // Report RSSI to app
                if ( pGapCentralRoleCB && pGapCentralRoleCB->rssiCB )
                {
                    pGapCentralRoleCB->rssiCB( connHandle, rssi );
                }
            }
        }
        else if(pMsg->status == HCI_LE_EVENT_CODE)
        {
            #ifdef EXT_ADV_ENABLE
            uint8 BLEEventCode = ((hciEvt_BLEExtAdvPktReport_t*)(pMsg))->BLEEventCode;

            if(BLEEventCode == HCI_BLE_EXT_ADV_REPORT_EVENT)
            {
                if(gapExtScan_legacyadv_rptfilter && (((hciEvt_BLEExtAdvPktReport_t*)(pMsg))->rptInfo->eventType & LE_ADV_PROP_LEGACY_BITMASK))
                    break;

                //save advrpt
                GAPExtCentralRole_Reassemble_Advrpt((hciEvt_BLEExtAdvPktReport_t*)(pMsg));

                if(pGapExtCentralCB && pGapExtCentralCB->pfnProcessExtScanningEvt)
                {
                    pGapExtCentralCB->pfnProcessExtScanningEvt((hciEvt_BLEExtAdvPktReport_t*)(pMsg));
                }
            }
            else if(BLEEventCode == HCI_BLE_SCAN_TIMEOUT_EVENT)
            {
                if(pGapExtCentralCB && pGapExtCentralCB->pfnProcessEvtScanTimeoutEvt)
                {
                    pGapExtCentralCB->pfnProcessEvtScanTimeoutEvt((hciEvt_ScanTimeout_t*)(pMsg));
                }

                osal_memset(gapExtScan_AdvInfo,0,sizeof(gapExtScan_AdvInfo_t)*GAP_EXTSCAN_MAX_SCAN_NUM);
            }

            #endif
        }

        break;

    case GAP_MSG_EVENT:
        gapCentralRole_ProcessGAPMsg( (gapEventHdr_t*) pMsg );
        break;

    case GAPCENTRALROLE_RSSI_MSG_EVT:
    {
        gapCentralRoleRssi_t* pRssi = ((gapCentralRoleRssiEvent_t*) pMsg)->pRssi;

        // If link is up and RSSI reads active
        if (pRssi->connHandle != GAP_CONNHANDLE_ALL &&
                linkDB_Up(pRssi->connHandle))
        {
            // Restart timer
            osal_CbTimerStart( gapCentralRole_timerCB, (uint8*) pRssi,
                               pRssi->period, &pRssi->timerId );
            // Read RSSI
            VOID HCI_ReadRssiCmd( pRssi->connHandle );
        }
    }
    break;

    default:
        break;
    }
}

/*********************************************************************
    @fn      gapCentralRole_ProcessGAPMsg

    @brief   Process an incoming task message from GAP.

    @param   pMsg - message to process

    @return  none
*/
static void gapCentralRole_ProcessGAPMsg( gapEventHdr_t* pMsg )
{
    switch ( pMsg->opcode )
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        gapDeviceInitDoneEvent_t* pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        if ( pPkt->hdr.status == SUCCESS )
        {
            // Save off the generated keys
            // VOID osal_snv_write( BLE_NVID_IRK, KEYLEN, gapCentralRoleIRK );
            // VOID osal_snv_write( BLE_NVID_CSRK, KEYLEN, gapCentralRoleSRK );
            // Save off the information
            VOID osal_memcpy( gapCentralRoleBdAddr, pPkt->devAddr, B_ADDR_LEN );
        }
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t* pPkt = (gapEstLinkReqEvent_t*) pMsg;

        if (pPkt->hdr.status == SUCCESS)
        {
            // Notify the Bond Manager of the connection
            VOID GAPBondMgr_LinkEst( pPkt->devAddrType, pPkt->devAddr,
                                     pPkt->connectionHandle, GAP_PROFILE_CENTRAL );
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        uint16 connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
        VOID GAPBondMgr_ProcessGAPMsg( (gapEventHdr_t*)pMsg );
        // Cancel RSSI reads
        GAPCentralRole_CancelRssi( connHandle );
    }
    break;

    // temporary workaround
    case GAP_SLAVE_REQUESTED_SECURITY_EVENT:
        VOID GAPBondMgr_ProcessGAPMsg( pMsg );
        break;

    default:
        break;
    }

    // Pass event to app
    if ( pGapCentralRoleCB && pGapCentralRoleCB->eventCB )
    {
        pGapCentralRoleCB->eventCB( (gapCentralRoleEvent_t*) pMsg );
    }
}

/*********************************************************************
    @fn      gapCentralRole_RssiAlloc

    @brief   Allocate an RSSI structure.

    @param   connHandle - Connection handle

    @return  pointer to structure or NULL if allocation failed.
*/
static gapCentralRoleRssi_t* gapCentralRole_RssiAlloc( uint16 connHandle )
{
    uint8 i;

    // Find free RSSI structure
    for ( i = 0; i < GAPCENTRALROLE_NUM_RSSI_LINKS; i++ )
    {
        if ( gapCentralRoleRssi[i].connHandle == GAP_CONNHANDLE_ALL )
        {
            gapCentralRoleRssi[i].connHandle = connHandle;
            return &gapCentralRoleRssi[i];
        }
    }

    // No free structure found
    return NULL;
}

/*********************************************************************
    @fn      gapCentralRole_RssiFind

    @brief   Find an RSSI structure.

    @param   connHandle - Connection handle

    @return  pointer to structure or NULL if not found.
*/
static gapCentralRoleRssi_t* gapCentralRole_RssiFind( uint16 connHandle )
{
    uint8 i;

    // Find free RSSI structure
    for ( i = 0; i < GAPCENTRALROLE_NUM_RSSI_LINKS; i++ )
    {
        if ( gapCentralRoleRssi[i].connHandle == connHandle )
        {
            return &gapCentralRoleRssi[i];
        }
    }

    // Not found
    return NULL;
}

/*********************************************************************
    @fn      gapCentralRole_RssiFree

    @brief   Free an RSSI structure.

    @param   connHandle - Connection handle

    @return  none
*/
static void gapCentralRole_RssiFree( uint16 connHandle )
{
    uint8 i;

    // Find RSSI structure
    for ( i = 0; i < GAPCENTRALROLE_NUM_RSSI_LINKS; i++ )
    {
        if ( gapCentralRoleRssi[i].connHandle == connHandle )
        {
            gapCentralRoleRssi[i].connHandle = GAP_CONNHANDLE_ALL;
            break;
        }
    }
}

/*********************************************************************
    @fn      gapCentralRole_timerCB

    @brief   OSAL timer callback function

    @param   pData - Data pointer

    @return  none
*/
static void gapCentralRole_timerCB( uint8* pData )
{
    gapCentralRoleRssiEvent_t* pMsg;
    // Timer has expired so clear timer ID
    ((gapCentralRoleRssi_t*) pData)->timerId = INVALID_TIMER_ID;
    // Send OSAL message
    pMsg = (gapCentralRoleRssiEvent_t*) osal_msg_allocate( sizeof(gapCentralRoleRssiEvent_t) );

    if ( pMsg )
    {
        pMsg->hdr.event = GAPCENTRALROLE_RSSI_MSG_EVT;
        pMsg->pRssi = (gapCentralRoleRssi_t*) pData;
        osal_msg_send ( gapCentralRoleTaskId, (uint8*) pMsg );
    }
}

#ifdef EXT_ADV_ENABLE
static gapExtScan_Parameters_t gap_ExtScanParameter;
static gapExtInit_Parameters_t gap_ExtInitParameter;
static uint8 gapExtScan_filter_duplicates;
static uint16 gapExtScan_duration;

//register extcenteral CBs
bStatus_t GAPExtCentralRole_CB( gapExtCentralCBs_t* pAppCallbacks )
{
    if ( pAppCallbacks )
    {
        pGapExtCentralCB = pAppCallbacks;
    }

    return SUCCESS;
}

/*********************************************************************
    @brief   set extscan parameters


*/
bStatus_t GAPExtCentralRole_SetParameter(uint16 param, uint16 len, void* pValue)
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {
    case GAPEXTSCAN_OWNADDRTYPE:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtScanParameter.own_address_type = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_FILTERPOLICY:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtScanParameter.scanning_filter_policy = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_SCANPHYS:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtScanParameter.scanning_PHYs = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_1MPHY_SCANTYPE:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtScanParameter.scan_type[0] = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_1MPHY_SCANINTERVAL:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtScanParameter.scan_interval[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_1MPHY_SCANWINDOW:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtScanParameter.scan_window[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_CODEDPHY_SCANTYPE:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtScanParameter.scan_type[1] = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_CODEDPHY_SCANINTERVAL:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtScanParameter.scan_interval[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_CODEDPHY_SCANWINDOW:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtScanParameter.scan_window[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_FILTERDUPLICATES:
        if ( len == sizeof ( uint8 ) )
        {
            gapExtScan_filter_duplicates = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_SCANDURATION:
        if ( len == sizeof ( uint16 ) )
        {
            gapExtScan_duration = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTSCAN_LEGACYADV_RPTFILTER:
        if ( len == sizeof ( uint8 ) )
        {
            gapExtScan_legacyadv_rptfilter = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_FILTERPOLICY:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtInitParameter.initiator_filter_policy = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_OWNADDRTYPE:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtInitParameter.own_address_type = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_INITPHYS:
        if ( len == sizeof ( uint8 ) )
        {
            gap_ExtInitParameter.initiating_PHYs = *((uint8*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_1MPHY_SCANINTERVAL:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.scan_interval[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_1MPHY_SCANWINDOW:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.scan_window[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_1MPHY_CONNINTERVAL_MIN:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_interval_min[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_1MPHY_CONNINTERVAL_MAX:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_interval_max[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_1MPHY_CONNLATENCY:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_latency[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_1MPHY_TIMEOUT:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.supervision_timeout[0] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_2MPHY_SCANINTERVAL:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.scan_interval[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_2MPHY_SCANWINDOW:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.scan_window[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_2MPHY_CONNINTERVAL_MIN:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_interval_min[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_2MPHY_CONNINTERVAL_MAX:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_interval_max[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_2MPHY_CONNLATENCY:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_latency[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_2MPHY_TIMEOUT:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.supervision_timeout[1] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_CODEDPHY_SCANINTERVAL:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.scan_interval[2] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_CODEDPHY_SCANWINDOW:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.scan_window[2] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_CODEDPHY_CONNINTERVAL_MIN:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_interval_min[2] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_CODEDPHY_CONNINTERVAL_MAX:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_interval_max[2] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_CODEDPHY_CONNLATENCY:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.conn_latency[2] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    case GAPEXTINIT_CODEDPHY_TIMEOUT:
        if ( len == sizeof ( uint16 ) )
        {
            gap_ExtInitParameter.supervision_timeout[2] = *((uint16*)pValue);
        }
        else
        {
            ret = bleInvalidRange;
        }

        break;

    default:
        ret = INVALIDPARAMETER;
        break;
    }

    return ret;
}

bStatus_t GAPExtCentralRole_GetParameter(uint16 param,  void* pValue)
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {
    case GAPEXTSCAN_OWNADDRTYPE:
        *((uint8*)pValue) = gap_ExtScanParameter.own_address_type;
        break;

    case GAPEXTSCAN_FILTERPOLICY:
        *((uint8*)pValue) = gap_ExtScanParameter.scanning_filter_policy;
        break;

    case GAPEXTSCAN_SCANPHYS:
        *((uint8*)pValue) = gap_ExtScanParameter.scanning_PHYs;
        break;

    case GAPEXTSCAN_1MPHY_SCANTYPE:
        *((uint8*)pValue) = gap_ExtScanParameter.scan_type[0];
        break;

    case GAPEXTSCAN_1MPHY_SCANINTERVAL:
        *((uint16*)pValue) = gap_ExtScanParameter.scan_interval[0];
        break;

    case GAPEXTSCAN_1MPHY_SCANWINDOW:
        *((uint16*)pValue) = gap_ExtScanParameter.scan_window[0];
        break;

    case GAPEXTSCAN_CODEDPHY_SCANTYPE:
        *((uint8*)pValue) = gap_ExtScanParameter.scan_type[1];
        break;

    case GAPEXTSCAN_CODEDPHY_SCANINTERVAL:
        *((uint16*)pValue) = gap_ExtScanParameter.scan_interval[1];
        break;

    case GAPEXTSCAN_CODEDPHY_SCANWINDOW:
        *((uint16*)pValue) = gap_ExtScanParameter.scan_window[1];
        break;

    case GAPEXTSCAN_FILTERDUPLICATES:
        *((uint8*)pValue) = gapExtScan_filter_duplicates;
        break;

    case GAPEXTSCAN_SCANDURATION:
        *((uint16*)pValue) = gapExtScan_duration;
        break;

    case GAPEXTSCAN_LEGACYADV_RPTFILTER:
        *((uint8*)pValue) = gapExtScan_legacyadv_rptfilter;
        break;

    case GAPEXTINIT_FILTERPOLICY:
        *((uint8*)pValue) = gap_ExtInitParameter.initiator_filter_policy;
        break;

    case GAPEXTINIT_OWNADDRTYPE:
        *((uint8*)pValue) = gap_ExtInitParameter.own_address_type;
        break;

    case GAPEXTINIT_INITPHYS:
        *((uint8*)pValue) = gap_ExtInitParameter.initiating_PHYs;
        break;

    case GAPEXTINIT_1MPHY_SCANINTERVAL:
        *((uint16*)pValue) = gap_ExtInitParameter.scan_interval[0];
        break;

    case GAPEXTINIT_1MPHY_SCANWINDOW:
        *((uint16*)pValue) = gap_ExtInitParameter.scan_window[0];
        break;

    case GAPEXTINIT_1MPHY_CONNINTERVAL_MIN:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_interval_min[0];
        break;

    case GAPEXTINIT_1MPHY_CONNINTERVAL_MAX:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_interval_max[0];
        break;

    case GAPEXTINIT_1MPHY_CONNLATENCY:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_latency[0];
        break;

    case GAPEXTINIT_1MPHY_TIMEOUT:
        *((uint16*)pValue) = gap_ExtInitParameter.supervision_timeout[0];
        break;

    case GAPEXTINIT_2MPHY_SCANINTERVAL:
        *((uint16*)pValue) = gap_ExtInitParameter.scan_interval[1];
        break;

    case GAPEXTINIT_2MPHY_SCANWINDOW:
        *((uint16*)pValue) = gap_ExtInitParameter.scan_window[1];
        break;

    case GAPEXTINIT_2MPHY_CONNINTERVAL_MIN:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_interval_min[1];
        break;

    case GAPEXTINIT_2MPHY_CONNINTERVAL_MAX:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_interval_max[1];
        break;

    case GAPEXTINIT_2MPHY_CONNLATENCY:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_latency[1];
        break;

    case GAPEXTINIT_2MPHY_TIMEOUT:
        *((uint16*)pValue) = gap_ExtInitParameter.supervision_timeout[1];
        break;

    case GAPEXTINIT_CODEDPHY_SCANINTERVAL:
        *((uint16*)pValue) = gap_ExtInitParameter.scan_interval[2];
        break;

    case GAPEXTINIT_CODEDPHY_SCANWINDOW:
        *((uint16*)pValue) = gap_ExtInitParameter.scan_window[2];
        break;

    case GAPEXTINIT_CODEDPHY_CONNINTERVAL_MIN:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_interval_min[2];
        break;

    case GAPEXTINIT_CODEDPHY_CONNINTERVAL_MAX:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_interval_max[2];
        break;

    case GAPEXTINIT_CODEDPHY_CONNLATENCY:
        *((uint16*)pValue) = gap_ExtInitParameter.conn_latency[2];
        break;

    case GAPEXTINIT_CODEDPHY_TIMEOUT:
        *((uint16*)pValue) = gap_ExtInitParameter.supervision_timeout[2];
        break;

    default:
        ret = INVALIDPARAMETER;
        break;
    }

    return ret;
}

bStatus_t GAPExtCentralRole_InitExtScanParams(void)
{
    gap_ExtScanParameter.own_address_type           = LL_DEV_ADDR_TYPE_PUBLIC;
    gap_ExtScanParameter.scanning_filter_policy     = LL_SCAN_WL_POLICY_ANY_ADV_PKTS;
    gap_ExtScanParameter.scanning_PHYs              = LL_SCAN_PHY_1M_BITMASK | LL_SCAN_PHY_CODED_BITMASK;
    gap_ExtScanParameter.scan_type[0]               = LL_SCAN_ACTIVE;
    gap_ExtScanParameter.scan_interval[0]           = 1600;   //N * 0.625 ms
    gap_ExtScanParameter.scan_window[0]             = 1600;   //N * 0.625 ms
    gap_ExtScanParameter.scan_type[1]               = LL_SCAN_ACTIVE;
    gap_ExtScanParameter.scan_interval[1]           = 1600;   //N * 0.625 ms
    gap_ExtScanParameter.scan_window[1]             = 1600;   //N * 0.625 ms
    gapExtScan_filter_duplicates                   = 0;
    gapExtScan_duration                            = 300;  //N * 10 ms
    return SUCCESS;
}

bStatus_t GAPExtCentralRole_EnableExtScan(void)
{
    HCI_LE_SetExtendedScanParametersCmd(gap_ExtScanParameter.own_address_type,
                                        gap_ExtScanParameter.scanning_filter_policy,
                                        gap_ExtScanParameter.scanning_PHYs,
                                        gap_ExtScanParameter.scan_type,
                                        gap_ExtScanParameter.scan_interval,
                                        gap_ExtScanParameter.scan_window);
    HCI_LE_SetExtendedScanEnableCmd(TRUE,
                                    gapExtScan_filter_duplicates,
                                    gapExtScan_duration,
                                    0);
    return SUCCESS;
}

bStatus_t GAPExtCentralRole_DisableExtScan(void)
{
    HCI_LE_SetExtendedScanEnableCmd(FALSE,0,0,0);
    osal_memset(gapExtScan_AdvInfo,0,sizeof(gapExtScan_AdvInfo_t)*GAP_EXTSCAN_MAX_SCAN_NUM);
    return SUCCESS;
}

bStatus_t GAPExtCentralRole_InitExtConnParams(void)
{
    gap_ExtInitParameter.initiator_filter_policy       = LL_INIT_WL_POLICY_USE_PEER_ADDR;
    gap_ExtInitParameter.own_address_type              = LL_DEV_ADDR_TYPE_PUBLIC;
    gap_ExtInitParameter.initiating_PHYs               = LL_SCAN_PHY_1M_BITMASK | LL_CONN_PHY_2M_BITMASK | LL_SCAN_PHY_CODED_BITMASK;
    //1M extinit
    gap_ExtInitParameter.scan_interval[0]              = 1600; //N * 0.625 ms
    gap_ExtInitParameter.scan_window[0]                = 1600; //N * 0.625 ms
    gap_ExtInitParameter.conn_interval_min[0]          = 80; //N * 1.25 ms
    gap_ExtInitParameter.conn_interval_max[0]          = 160; //N * 1.25 ms
    gap_ExtInitParameter.conn_latency[0]               = 0;
    gap_ExtInitParameter.supervision_timeout[0]        = 200; //N * 10 ms
    gap_ExtInitParameter.minimum_CE_length[0]          = 0;
    gap_ExtInitParameter.maximum_CE_length[0]          = 0;
    //2M extinit
    gap_ExtInitParameter.scan_interval[1]              = 1600; //N * 0.625 ms
    gap_ExtInitParameter.scan_window[1]                = 1600; //N * 0.625 ms
    gap_ExtInitParameter.conn_interval_min[1]          = 80; //N * 1.25 ms
    gap_ExtInitParameter.conn_interval_max[1]          = 160; //N * 1.25 ms
    gap_ExtInitParameter.conn_latency[1]               = 0;
    gap_ExtInitParameter.supervision_timeout[1]        = 200; //N * 10 ms
    gap_ExtInitParameter.minimum_CE_length[1]          = 0;
    gap_ExtInitParameter.maximum_CE_length[1]          = 0;
    //codedphy extinit
    gap_ExtInitParameter.scan_interval[2]              = 1600; //N * 0.625 ms
    gap_ExtInitParameter.scan_window[2]                = 1600; //N * 0.625 ms
    gap_ExtInitParameter.conn_interval_min[2]          = 80; //N * 1.25 ms
    gap_ExtInitParameter.conn_interval_max[2]          = 160; //N * 1.25 ms
    gap_ExtInitParameter.conn_latency[2]               = 0;
    gap_ExtInitParameter.supervision_timeout[2]        = 200; //N * 10 ms
    gap_ExtInitParameter.minimum_CE_length[2]          = 0;
    gap_ExtInitParameter.maximum_CE_length[2]          = 0;
    return SUCCESS;
}

bStatus_t GAPExtCentralRole_Createconn(uint8 peer_address_type,uint8* peer_address)
{
    HCI_LE_ExtendedCreateConnectionCmd(gap_ExtInitParameter.initiator_filter_policy,
                                       gap_ExtInitParameter.own_address_type,
                                       peer_address_type,
                                       peer_address,
                                       gap_ExtInitParameter.initiating_PHYs,
                                       gap_ExtInitParameter.scan_interval,
                                       gap_ExtInitParameter.scan_window,
                                       gap_ExtInitParameter.conn_interval_min,
                                       gap_ExtInitParameter.conn_interval_max,
                                       gap_ExtInitParameter.conn_latency,
                                       gap_ExtInitParameter.supervision_timeout,
                                       gap_ExtInitParameter.minimum_CE_length,
                                       gap_ExtInitParameter.maximum_CE_length);
    return SUCCESS;
}

void GAPExtCentralRole_Reassemble_Advrpt(hciEvt_BLEExtAdvPktReport_t* pMsg)
{
    for(uint8_t i=0; i < GAP_EXTSCAN_MAX_SCAN_NUM; i++)
    {
        if(gapExtScan_AdvInfo[i].inuse==FALSE)
        {
            osal_memcpy(&(gapExtScan_AdvInfo[i].advrpt),pMsg->rptInfo,23);
            gapExtScan_AdvInfo[i].advrpt.dataLen = pMsg->rptInfo->dataLen;

            if(pMsg->rptInfo->dataLen>GAP_EXTSCAN_DATALEN)
                gapExtScan_AdvInfo[i].advrpt.dataLen = GAP_EXTSCAN_DATALEN;

            osal_memcpy(&(gapExtScan_AdvInfo[i].advrpt.rptData[0]),&(pMsg->rptInfo->rptData[0]),gapExtScan_AdvInfo[i].advrpt.dataLen);
            gapExtScan_AdvInfo[i].inuse = TRUE;

            if(pMsg->rptInfo->eventType & 0x10)
                gapExtScan_AdvInfo[i].datacompleted = 1;

            return;
        }
        else
        {
            if(osal_memcmp(pMsg->rptInfo->addr,gapExtScan_AdvInfo[i].advrpt.advAddr,B_ADDR_LEN) &&
                    gapExtScan_AdvInfo[i].datafull==0 &&
                    gapExtScan_AdvInfo[i].datacompleted==0 &&
                    gapExtScan_AdvInfo[i].datatruncated==0)
            {
                if(pMsg->rptInfo->eventType==0x40)
                {
                    gapExtScan_AdvInfo[i].datatruncated=1;
                    break;
                }

                if(gapExtScan_AdvInfo[i].advrpt.dataLen+pMsg->rptInfo->dataLen>=GAP_EXTSCAN_DATALEN)
                {
                    osal_memcpy(&(gapExtScan_AdvInfo[i].advrpt.rptData[gapExtScan_AdvInfo[i].advrpt.dataLen]),&(pMsg->rptInfo->rptData[0]),GAP_EXTSCAN_DATALEN-gapExtScan_AdvInfo[i].advrpt.dataLen);
                    gapExtScan_AdvInfo[i].datafull=1;
                    gapExtScan_AdvInfo[i].advrpt.dataLen = GAP_EXTSCAN_DATALEN;
                    break;
                }

                osal_memcpy(&(gapExtScan_AdvInfo[i].advrpt.rptData[gapExtScan_AdvInfo[i].advrpt.dataLen]),&(pMsg->rptInfo->rptData[0]),pMsg->rptInfo->dataLen);
                gapExtScan_AdvInfo[i].advrpt.dataLen += pMsg->rptInfo->dataLen;

                if((pMsg->rptInfo->eventType & 0x60)==0)
                    gapExtScan_AdvInfo[i].datacompleted=1;

                break;
            }
            else if(osal_memcmp(pMsg->rptInfo->addr,gapExtScan_AdvInfo[i].advrpt.advAddr,B_ADDR_LEN))
                break;
        }
    }
}
#endif
/*********************************************************************
*********************************************************************/
