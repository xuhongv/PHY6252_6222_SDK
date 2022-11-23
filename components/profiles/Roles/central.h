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
#ifndef CENTRAL_H
#define CENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "gap.h"

/*********************************************************************
    CONSTANTS
*/

/** @defgroup GAPCENTRALROLE_PROFILE_PARAMETERS GAP Central Role Parameters
    @{
*/
#define GAPCENTRALROLE_IRK                 0x400  //!< Identity Resolving Key. Read/Write. Size is uint8[KEYLEN]. Default is all 0, which means that the IRK will be randomly generated.
#define GAPCENTRALROLE_SRK                 0x401  //!< Signature Resolving Key. Read/Write. Size is uint8[KEYLEN]. Default is all 0, which means that the SRK will be randomly generated.
#define GAPCENTRALROLE_SIGNCOUNTER         0x402  //!< Sign Counter. Read/Write. Size is uint32. Default is 0.
#define GAPCENTRALROLE_BD_ADDR             0x403  //!< Device's Address. Read Only. Size is uint8[B_ADDR_LEN]. This item is read from the controller.
#define GAPCENTRALROLE_MAX_SCAN_RES        0x404  //!< Maximum number of discover scan results to receive. Default is 0 = unlimited.
/** @} End GAPCENTRALROLE_PROFILE_PARAMETERS */

/**
    Number of simultaneous links with periodic RSSI reads
*/
#ifndef GAPCENTRALROLE_NUM_RSSI_LINKS
#define GAPCENTRALROLE_NUM_RSSI_LINKS     4
#endif

#define HCI_BLE_SCAN_TIMEOUT_EVENT                     0x11
#define GAP_EXTSCAN_MAX_SCAN_NUM                       3
#define GAP_EXTSCAN_DATALEN                            1300
/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/

/**
    Central Event Structure
*/
typedef union
{
    gapEventHdr_t             gap;                //!< GAP_MSG_EVENT and status.
    gapDeviceInitDoneEvent_t  initDone;           //!< GAP initialization done.
    gapDeviceInfoEvent_t      deviceInfo;         //!< Discovery device information event structure.
    gapDevDiscEvent_t         discCmpl;           //!< Discovery complete event structure.
    gapEstLinkReqEvent_t      linkCmpl;           //!< Link complete event structure.
    gapLinkUpdateEvent_t      linkUpdate;         //!< Link update event structure.
    gapTerminateLinkEvent_t   linkTerminate;      //!< Link terminated event structure.
} gapCentralRoleEvent_t;

/**
    RSSI Read Callback Function
*/
typedef void (*pfnGapCentralRoleRssiCB_t)
(
    uint16 connHandle,                    //!< Connection handle.
    int8  rssi                            //!< New RSSI value.
);

/**
    Central Event Callback Function
*/
typedef void (*pfnGapCentralRoleEventCB_t)
(
    gapCentralRoleEvent_t* pEvent         //!< Pointer to event structure.
);

/**
    Central Callback Structure
*/
typedef struct
{
    pfnGapCentralRoleRssiCB_t   rssiCB;   //!< RSSI callback.
    pfnGapCentralRoleEventCB_t  eventCB;  //!< Event callback.
} gapCentralRoleCB_t;

//for extscan report
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8  BLEEventCode;
} hciEvt_ScanTimeout_t;
typedef void (*gapProcessExtScanningEvt_t)( hciEvt_BLEExtAdvPktReport_t* pMsg );
typedef void (*gapProcessExtScanTimeoutEvt_t)(hciEvt_ScanTimeout_t* pMsg);
typedef struct
{
    //gapProcessHCICmdEvt_t   pfnProcessHCICmdEvt;   // When HCI Command Event received
    gapProcessExtScanningEvt_t pfnProcessExtScanningEvt; // When Scanning Report received
    gapProcessExtScanTimeoutEvt_t pfnProcessEvtScanTimeoutEvt;   //When scantimeout event received
} gapExtCentralCBs_t;

typedef struct
{
    uint16  advEvt;
    uint8   advAddrType;
    uint8   advAddr[B_ADDR_LEN];
    uint8   primaryPHY;
    uint8   secondaryPHY;
    uint8   advertisingSID;
    uint8   txPower;
    int8    rssi;
    uint16  periodicAdvertisingInterval;
    uint8   directAddrType;
    uint8   directAddr[B_ADDR_LEN];
    uint16   dataLen;
    uint8   rptData[GAP_EXTSCAN_DATALEN];
} gapExtScan_AdvRpt_t;

typedef struct
{
    uint8   inuse;
    uint8   datafull;
    uint8   datacompleted;
    uint8   datatruncated;
    gapExtScan_AdvRpt_t   advrpt;
} gapExtScan_AdvInfo_t;

typedef struct
{
    uint8 own_address_type;
    uint8 scanning_filter_policy;
    uint8 scanning_PHYs;
    uint8 scan_type[2];
    uint16 scan_interval[2];
    uint16 scan_window[2];
} gapExtScan_Parameters_t;

typedef struct
{
    uint8      initiator_filter_policy;
    uint8      own_address_type;
    uint8      peer_address_type;
    uint8      peer_address[B_ADDR_LEN];
    uint8      initiating_PHYs;
    uint16     scan_interval[3];
    uint16     scan_window[3];
    uint16     conn_interval_min[3];
    uint16     conn_interval_max[3];
    uint16     conn_latency[3];
    uint16     supervision_timeout[3];
    uint16     minimum_CE_length[3];
    uint16     maximum_CE_length[3];
} gapExtInit_Parameters_t;

typedef enum
{
    //extscan param
    GAPEXTSCAN_OWNADDRTYPE           =0,
    GAPEXTSCAN_FILTERPOLICY          =1,
    GAPEXTSCAN_SCANPHYS,
    GAPEXTSCAN_1MPHY_SCANTYPE,
    GAPEXTSCAN_1MPHY_SCANINTERVAL,
    GAPEXTSCAN_1MPHY_SCANWINDOW,
    GAPEXTSCAN_CODEDPHY_SCANTYPE,
    GAPEXTSCAN_CODEDPHY_SCANINTERVAL,
    GAPEXTSCAN_CODEDPHY_SCANWINDOW,
    GAPEXTSCAN_FILTERDUPLICATES,
    GAPEXTSCAN_SCANDURATION,
    //extscan rptfilter
    GAPEXTSCAN_LEGACYADV_RPTFILTER,
    //extinit param
    GAPEXTINIT_FILTERPOLICY,
    GAPEXTINIT_OWNADDRTYPE,
    GAPEXTINIT_INITPHYS,

    GAPEXTINIT_1MPHY_SCANINTERVAL,
    GAPEXTINIT_1MPHY_SCANWINDOW,
    GAPEXTINIT_1MPHY_CONNINTERVAL_MIN,
    GAPEXTINIT_1MPHY_CONNINTERVAL_MAX,
    GAPEXTINIT_1MPHY_CONNLATENCY,
    GAPEXTINIT_1MPHY_TIMEOUT,

    GAPEXTINIT_2MPHY_SCANINTERVAL,
    GAPEXTINIT_2MPHY_SCANWINDOW,
    GAPEXTINIT_2MPHY_CONNINTERVAL_MIN,
    GAPEXTINIT_2MPHY_CONNINTERVAL_MAX,
    GAPEXTINIT_2MPHY_CONNLATENCY,
    GAPEXTINIT_2MPHY_TIMEOUT,

    GAPEXTINIT_CODEDPHY_SCANINTERVAL,
    GAPEXTINIT_CODEDPHY_SCANWINDOW,
    GAPEXTINIT_CODEDPHY_CONNINTERVAL_MIN,
    GAPEXTINIT_CODEDPHY_CONNINTERVAL_MAX,
    GAPEXTINIT_CODEDPHY_CONNLATENCY,
    GAPEXTINIT_CODEDPHY_TIMEOUT

} gapExtCentralConfig_e;

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    API FUNCTIONS
*/

/*  -------------------------------------------------------------------
    Central Profile Public APIs
*/

/**
    @defgroup CENTRAL_PROFILE_API Central Profile API Functions

    @{
*/

/**
    @brief   Start the device in Central role.  This function is typically
            called once during system startup.

    @param   pAppCallbacks - pointer to application callbacks

    @return  SUCCESS: Operation successful.<BR>
            bleAlreadyInRequestedMode: Device already started.<BR>
*/
extern bStatus_t GAPCentralRole_StartDevice( gapCentralRoleCB_t* pAppCallbacks );

/**
    @brief   Set a parameter in the Central Profile.

    @param   param - profile parameter ID: @ref GAPCENTRALROLE_PROFILE_PARAMETERS
    @param   len - length of data to write
    @param   pValue - pointer to data to write.  This is dependent on
            the parameter ID and WILL be cast to the appropriate
            data type.

    @return  SUCCESS: Operation successful.<BR>
            INVALIDPARAMETER: Invalid parameter ID.<BR>
*/
extern bStatus_t GAPCentralRole_SetParameter( uint16 param, uint8 len, void* pValue );

/**
    @brief   Get a parameter in the Central Profile.

    @param   param - profile parameter ID: @ref GAPCENTRALROLE_PROFILE_PARAMETERS
    @param   pValue - pointer to buffer to contain the read data

    @return  SUCCESS: Operation successful.<BR>
            INVALIDPARAMETER: Invalid parameter ID.<BR>
*/
extern bStatus_t GAPCentralRole_GetParameter( uint16 param, void* pValue );

/**
    @brief   Terminate a link.

    @param   connHandle - connection handle of link to terminate
            or @ref GAP_CONN_HANDLE_DEFINES

    @return  SUCCESS: Terminate started.<BR>
            bleIncorrectMode: No link to terminate.<BR>
*/
extern bStatus_t GAPCentralRole_TerminateLink( uint16 connHandle );

/**
    @brief   Establish a link to a peer device.

    @param   highDutyCycle -  TRUE to high duty cycle scan, FALSE if not
    @param   whiteList - determines use of the white list: @ref GAP_WHITELIST_DEFINES
    @param   addrTypePeer - address type of the peer device: @ref GAP_ADDR_TYPE_DEFINES
    @param   peerAddr - peer device address

    @return  SUCCESS: started establish link process.<BR>
            bleIncorrectMode: invalid profile role.<BR>
            bleNotReady: a scan is in progress.<BR>
            bleAlreadyInRequestedMode: can?t process now.<BR>
            bleNoResources: too many links.<BR>
*/
extern bStatus_t GAPCentralRole_EstablishLink( uint8 highDutyCycle, uint8 whiteList,
                                               uint8 addrTypePeer, uint8* peerAddr );

/**
    @brief   Update the link connection parameters.

    @param   connHandle - connection handle
    @param   connIntervalMin - minimum connection interval in 1.25ms units
    @param   connIntervalMax - maximum connection interval in 1.25ms units
    @param   connLatency - number of LL latency connection events
    @param   connTimeout - connection timeout in 10ms units

    @return  SUCCESS: Connection update started started.<BR>
            bleIncorrectMode: No connection to update.<BR>
*/
extern bStatus_t GAPCentralRole_UpdateLink( uint16 connHandle, uint16 connIntervalMin,
                                            uint16 connIntervalMax, uint16 connLatency,
                                            uint16 connTimeout );
/**
    @brief   Start a device discovery scan.

    @param   mode - discovery mode: @ref GAP_DEVDISC_MODE_DEFINES
    @param   activeScan - TRUE to perform active scan
    @param   whiteList - TRUE to only scan for devices in the white list

    @return  SUCCESS: Discovery scan started.<BR>
            bleIncorrectMode: Invalid profile role.<BR>
            bleAlreadyInRequestedMode: Not available.<BR>
*/
extern bStatus_t GAPCentralRole_StartDiscovery( uint8 mode, uint8 activeScan, uint8 whiteList );

/**
    @brief   Cancel a device discovery scan.

    @return  SUCCESS: Cancel started.<BR>
            bleInvalidTaskID: Not the task that started discovery.<BR>
            bleIncorrectMode: Not in discovery mode.<BR>
*/
extern bStatus_t GAPCentralRole_CancelDiscovery( void );

/**
    @brief   Start periodic RSSI reads on a link.

    @param   connHandle - connection handle of link
    @param   period - RSSI read period in ms

    @return  SUCCESS: Terminate started.<BR>
            bleIncorrectMode: No link.<BR>
            bleNoResources: No resources.<BR>
*/
extern bStatus_t GAPCentralRole_StartRssi( uint16 connHandle, uint16 period );

/**
    @brief   Cancel periodic RSSI reads on a link.

    @param   connHandle - connection handle of link

    @return  SUCCESS: Operation successful.<BR>
            bleIncorrectMode: No link.<BR>
*/
extern bStatus_t GAPCentralRole_CancelRssi(uint16 connHandle );

/**
    @}
*/

/*  -------------------------------------------------------------------
    TASK API - These functions must only be called by OSAL.
*/

/**
    @internal

    @brief   Central Profile Task initialization function.

    @param   taskId - Task ID.

    @return  void
*/
extern void GAPCentralRole_Init( uint8 taskId );

/**
    @internal

    @brief   Central Profile Task event processing function.

    @param   taskId - Task ID
    @param   events - Events.

    @return  events not processed
*/
extern uint16 GAPCentralRole_ProcessEvent( uint8 taskId, uint16 events );
#ifdef EXT_ADV_ENABLE
extern bStatus_t GAPExtCentralRole_SetParameter(uint16 param, uint16 len, void* pValue);
extern bStatus_t GAPExtCentralRole_GetParameter(uint16 param,  void* pValue);
extern bStatus_t GAPExtCentralRole_EnableExtScan(void);
extern bStatus_t GAPExtCentralRole_DisableExtScan(void);
extern bStatus_t GAPExtCentralRole_Createconn(uint8 peer_address_type,uint8* peer_address);
#endif
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CENTRAL_H */
