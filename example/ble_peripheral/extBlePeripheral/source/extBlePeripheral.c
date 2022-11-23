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
    Filename:       extBlePeripheral.c
    Revised:
    Revision:

    Description:    This file contains the Simple BLE Peripheral sample application


**************************************************************************************************/
/*********************************************************************
    INCLUDES
*/
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleIBeaconProfile_ota.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "extBlePeripheral.h"
#include "ll.h"
#include "ll_def.h"
#include "hci_tl.h"
#include "hci.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     800//24//32//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800//48//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE//TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Default passcode
#define DEFAULT_PASSCODE                      0//19655

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#define RESOLVING_LIST_ENTRY_NUM              10

#define RPA_TEST                              0
/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    EXTERNAL VARIABLES
*/
//volatile uint8_t g_current_advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;

//extern wtnrTest_t wtnrTest;
//extern l2capSARDbugCnt_t g_sarDbgCnt;
extern uint32 g_osal_mem_allo_cnt;
extern uint32 g_osal_mem_free_cnt;

extern uint32         g_advSlotPeriodic;           // us
/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/
static uint8 extBlePeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

#if(RPA_TEST ==1)
    static uint8  peerIrkList[RESOLVING_LIST_ENTRY_NUM][LL_ENC_IRK_LEN];
    static uint8  localIrkList[RESOLVING_LIST_ENTRY_NUM][LL_ENC_IRK_LEN];
    static uint8  peerAddrList[RESOLVING_LIST_ENTRY_NUM][LL_DEVICE_ADDR_LEN];
    static uint8  peerAddrType[RESOLVING_LIST_ENTRY_NUM];
#endif

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x12,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x50,   // 'P'
    0x48,   // 'H'
    0x59,   // 'Y'
    0x2b,   // '+'
    0x41,   // 'A'
    0x32,   // '2'
    0x2d,   // '-'
    0x20,   // ''
    0x2d,   // '-'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'


    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0,       // 0dBm
    0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1, //300
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2 //600
};


//static uint8 otaAdvIntv         = 100;     // advert data for iBeacon
static uint8 advertData[] =
{
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x1A, // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
    0x4c, // Company ID - Fixed
    0x00, // Company ID - Fixed
    0x02, // Data Type - Fixed
    0x15, // Data Length - Fixed
    0xFD, // UUID
    0xA5, // UUID
    0x06, // UUID
    0x93, // UUID
    0xA4, // UUID
    0xE2, // UUID
    0x4F, // UUID
    0xB1, // UUID
    0xAF, // UUID
    0xCF, // UUID
    0xC6, // UUID
    0xEB, // UUID
    0x07, // UUID
    0x64, // UUID
    0x78, // UUID
    0x25, // UUID
    0x27, // Major
    0x74, // Major
    0x6b,//0x04, // Minor
    0xed,//0xb0, // Minor
    0xc5, // Power - The 2's complement of the calibrated Tx Power
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1, //300
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2, //600
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3, //900
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4 //1200
};
//static uint8 otaConnIntvMax     = DEFAULT_DESIRED_MIN_CONN_INTERVAL>>2;        //unit is 5ms
//static uint8 otaConnIntvMin     = DEFAULT_DESIRED_MAX_CONN_INTERVAL>>2;        //uiit is 5ms
//static uint8 otaConnIntvLatency = DEFAULT_DESIRED_SLAVE_LATENCY;        //
//static uint8 otaConnTimeOut     = DEFAULT_DESIRED_CONN_TIMEOUT/100;        //unit is second

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PHY+A2- -FFFFFF ";
static  uint8_t notifyBuf[256];
static uint16 notifyCnt = 0;
/*********************************************************************
    LOCAL FUNCTIONS
*/
static void extBlePeripheral_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );

static void peripheralStateReadRssiCB( int8 rssi  );

#if(RPA_TEST ==1)
    static void initResolvingList(void);
#endif


char* bdAddr2Str( uint8* pAddr );

//static uint8_t extBlePeripheral_ScanRequestFilterCBack(void);
/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t extBlePeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks, add 2017-11-15
static gapBondCBs_t extBlePeripheral_BondMgrCBs =
{
    NULL,                     // Passcode callback (not used by application)
    NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t extBlePeripheral_SimpleProfileCBs =
{
    simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/
/*********************************************************************
    @fn      extBlePeripheral_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void extBlePeripheral_Init( uint8 task_id )
{
    extBlePeripheral_TaskID = task_id;
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
    //setup GAP ext adv parameters
    {
        uint8 adv_hdl;
        uint8 adv_sid;
        uint8 adv_prop;
        uint8 priPHY = GAP_ADV_PHY_1MBPS;//GAP_ADV_PHY_1MBPS;//GAP_ADV_PHY_CODED;
        uint8 secPHY = GAP_ADV_PHY_1MBPS;//GAP_ADV_PHY_1MBPS;//GAP_ADV_PHY_2MBPS;//GAP_ADV_PHY_CODED;
        uint32 priAdvInt_Min = 320;
        uint32 priAdvInt_Max = 640;
        // 1st adv set
        adv_hdl = 0;
        adv_sid = 0;
        adv_prop =GAP_EXTADV_PROP_CONNECTABLE_NONSCAN;//GAP_EXTADV_PROP_NONCONN_NONSCAN
        // uint8 peeraddrType = 0;
        // uint8 peeraddr[6] = {0x22,0x18,0x11,0x44,0x55,0x66};
        // GAPRole_extAdv_SetParameter(adv_hdl, GAP_PEER_ADDR_TYPE, sizeof(peeraddrType), &peeraddrType);
        // GAPRole_extAdv_SetParameter(adv_hdl, GAP_PEER_ADDR, sizeof(peeraddr), peeraddr);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_PRI_CHN_ADV_INT_MIN, sizeof(priAdvInt_Min), &priAdvInt_Min);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_PRI_CHN_ADV_INT_MAX, sizeof(priAdvInt_Max), &priAdvInt_Max);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_EXT_ADV_DATA, sizeof(advertData), advertData);
        //GAPRole_extAdv_SetParameter(adv_hdl, GAP_EXT_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_PRI_ADV_PHY, 1, &priPHY);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_SEC_ADV_PHY, 1, &secPHY);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_ADV_SID, 1, &adv_sid);
        GAPRole_extAdv_SetParameter(adv_hdl, GAP_ADV_EVENT_PROP, 1, &adv_prop);
    }
    // Setup the GAP Peripheral Role Profile
    {
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Setup the GAP Bond Manager, add 2017-11-15
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8 mitm = FALSE;
        uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
        uint8 bonding = TRUE;
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
    }
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
    // Setup the SimpleProfile Characteristic Values
    {
        uint8  uuid_setting[IBEACON_UUID_LEN] =
        {
            0xFD,
            0xA5,
            0x06,
            0x93,
            0xA4,
            0xE2,
            0x4F,
            0xB1,
            0xAF,
            0xCF,
            0xC6,
            0xEB,
            0x07,
            0x64,
            0x78,
            0x25
        };
        uint16 major = 0x2774;
        uint16 minor = 0x6bed;
        uint8 power = 0x0f;
        uint8 reset[IBEACON_ATT_LONG_PKT];

        for(uint8 i=0; i<IBEACON_ATT_LONG_PKT; i++)
        {
            reset[i]=(i<6) ? 0 : i;
        }

        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, IBEACON_UUID_LEN, uuid_setting);
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint16 ), &major );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint16 ), &minor );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &power );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, IBEACON_ATT_LONG_PKT, &reset );
    }
    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &extBlePeripheral_SimpleProfileCBs );
    {
        //feature set
        uint8_t mtuSet = 247;
        llInitFeatureSet2MPHY(TRUE);
        llInitFeatureSetDLE(TRUE);
        ATT_SetMTUSizeMax(mtuSet);
        LOG("[2Mbps | DLE | MTU %d] \n",mtuSet);
    }
    // Setup a delayed profile startup
    osal_set_event( extBlePeripheral_TaskID, SBP_START_DEVICE_EVT );

    //intial notifyBuf
    for(int i =0 ; i<255; i++)
        notifyBuf[i]=i;

    // ========================= For extend advertiser test
    // for receive HCI complete message
//    GAP_RegisterForHCIMsgs(extBlePeripheral_TaskID);
    // init RPA list
//    initResolvingList();
    LOG("=============extBlePeripheral_Init Done===========\n");
}

/*********************************************************************
    @fn      extBlePeripheral_ProcessEvent

    @brief   Simple BLE Peripheral Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 extBlePeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( extBlePeripheral_TaskID )) != NULL )
        {
            extBlePeripheral_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &extBlePeripheral_PeripheralCBs );
        // ===================
        // Start Bond Manager
        VOID GAPBondMgr_Register( &extBlePeripheral_BondMgrCBs );
        return ( events ^ SBP_START_DEVICE_EVT );
    }

    // enable adv
    if ( events & SBP_RESET_ADV_EVT )
    {
        LOG("========== enable Extended Advertiser ========== \r\n");
        //enable adv set 0
        GAPRole_extAdv_EnableAdvSet(0);
        //osal_start_reload_timer(extBlePeripheral_TaskID,SBP_PEROID_EVT,3000);
        //osal_start_timerEx(extBlePeripheral_TaskID,SBP_STOP_ADV_EVT,5000);
        return ( events ^ SBP_RESET_ADV_EVT );
    }

    if ( events & SBP_STOP_ADV_EVT )
    {
        LOG("========== disable Extended Advertiser ========== \r\n");
        GAPRole_extAdv_DisableAdvSet(0);          // disable adv set 0
        return ( events ^ SBP_STOP_ADV_EVT );
    }

    if ( events & SBP_PEROID_EVT )
    {
        LOG("getscanreq:%d,sendscanrsp:%d,getconnreq:%d\n",g_pmCounters.ll_recv_scan_req_cnt,g_pmCounters.ll_send_scan_rsp_cnt,g_pmCounters.ll_recv_conn_req_cnt);
        return ( events ^ SBP_PEROID_EVT );
    }

    if ( events & SBP_NOTIFY_TEST_EVT )
    {
        notifyBuf[0]=HI_UINT16(notifyCnt);
        notifyBuf[1]=LO_UINT16(notifyCnt);
        uint8 ret = simpleProfile_Notify(SIMPLEPROFILE_CHAR6,ATT_GetCurrentMTUSize(0)-3,notifyBuf);

        if(ret==SUCCESS)
        {
            LOG("notify NO.%d success\n",notifyCnt);
            notifyCnt++;
        }
        else
        {
            LOG("notify NO.%d ret:%d\n",notifyCnt,ret);
        }

        return ( events ^ SBP_NOTIFY_TEST_EVT );
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      extBlePeripheral_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void extBlePeripheral_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    hciEvt_CmdComplete_t* pHciMsg;

    switch ( pMsg->event )
    {
    case HCI_GAP_EVENT_EVENT:
    {
        switch( pMsg->status )
        {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
            pHciMsg = (hciEvt_CmdComplete_t*)pMsg;
            LOG("==> HCI: %x\n", pHciMsg->cmdOpcode);
            //safeToDealloc = gapProcessHCICmdCompleteEvt( (hciEvt_CmdComplete_t *)pMsg );
            break;

        default:
            //safeToDealloc = FALSE;  // Send to app
            break;
        }
    }
    }
}
/*********************************************************************
    @fn      peripheralStateReadRssiCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
*/
static void peripheralStateReadRssiCB( int8  rssi )
{
//    notifyBuf[15]++;
//    notifyBuf[16]=rssi;
//    notifyBuf[17]=HI_UINT16(g_conn_param_foff);
//    notifyBuf[18]=LO_UINT16(g_conn_param_foff);;
//    notifyBuf[19]=g_conn_param_carrSens;
}

/*********************************************************************
    @fn      peripheralStateNotificationCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
*/
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
    switch ( newState )
    {
    case GAPROLE_STARTED:
    {
        //LOG("=========> GAPROLE_STARTED\r\n");
        uint8 ownAddress[B_ADDR_LEN];
        uint8 str_addr[14]= {0};
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];
        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;
        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
        osal_memcpy(&attDeviceName[9],&str_addr[6],8);
        osal_memcpy(&scanRspData[11],&str_addr[6],8);
        // Set the GAP Characteristics
        GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
        GAPRole_extAdv_SetParameter(0, GAP_EXT_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
        osal_start_timerEx(extBlePeripheral_TaskID, SBP_RESET_ADV_EVT, 500);
    }
    break;

    case GAPROLE_ADVERTISING:
        break;

    case GAPROLE_CONNECTED:
        break;

    case GAPROLE_CONNECTED_ADV:
        break;

    case GAPROLE_WAITING:
        osal_stop_timerEx(extBlePeripheral_TaskID,SBP_NOTIFY_TEST_EVT);
        osal_start_timerEx(extBlePeripheral_TaskID, SBP_RESET_ADV_EVT, 500);
        break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
        osal_stop_timerEx(extBlePeripheral_TaskID,SBP_NOTIFY_TEST_EVT);
        osal_start_timerEx(extBlePeripheral_TaskID, SBP_RESET_ADV_EVT, 500);
        break;

    case GAPROLE_ERROR:
        break;

    default:
        break;
    }

    gapProfileState = newState;
    LOG("[GAP ROLE %d]\n",newState);
    VOID gapProfileState;
}


/*********************************************************************
    @fn      simpleProfileChangeCB

    @brief   Callback from SimpleBLEProfile indicating a value change

    @param   paramID - parameter ID of the value that was changed.

    @return  none
*/
static void simpleProfileChangeCB( uint8 paramID )
{
    uint8 newValue[IBEACON_ATT_LONG_PKT];

    switch( paramID )
    {
    case SIMPLEPROFILE_CHAR5:
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR5,newValue);

        //for notify test:[00 00] stop notify;[00 1f] 1 second 1 notify
        if(newValue[0]==0)
        {
            if(newValue[1]==0)
            {
                osal_stop_timerEx(extBlePeripheral_TaskID,SBP_NOTIFY_TEST_EVT);
            }
            else
            {
                osal_start_reload_timer(extBlePeripheral_TaskID,SBP_NOTIFY_TEST_EVT,newValue[1]<<5);
            }
        }

        break;

    default:
        // not process other attribute change
        break;
    }
}

/*********************************************************************
    @fn      bdAddr2Str

    @brief   Convert Bluetooth address to string. Only needed when
           LCD display is used.

    @return  none
*/
char* bdAddr2Str( uint8* pAddr )
{
    uint8       i;
    char        hex[] = "0123456789ABCDEF";
    static char str[B_ADDR_STR_LEN];
    char*        pStr = str;
    *pStr++ = '0';
    *pStr++ = 'x';
    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for ( i = B_ADDR_LEN; i > 0; i-- )
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }

    *pStr = 0;
    return str;
}

#if(RPA_TEST ==1)
static void initResolvingList(void)
{
    int i;
    uint8 temp;
    uint8 devAddr[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

    for (i = 0; i < RESOLVING_LIST_ENTRY_NUM; i++)
    {
        memset(&peerIrkList[i], (i + 1), LL_ENC_IRK_LEN);
//        memset(&peerIrkList[i], 0, LL_ENC_IRK_LEN);
        memset(&localIrkList[i], (i + 1), LL_ENC_IRK_LEN);
//      memset(&localIrkList[i], 0, LL_ENC_IRK_LEN);
        temp = ((i + 1) << 4) | (i + 1);
        memset(&peerAddrList[i], temp, LL_DEVICE_ADDR_LEN);
        peerAddrType[i] = LL_DEV_ADDR_TYPE_PUBLIC;      // LL_DEV_ADDR_TYPE_RANDOM
    }

    LL_SetRandomAddress(devAddr);
}
#endif
/*********************************************************************
*********************************************************************/
