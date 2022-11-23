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
#include "OSAL_PwrMgr.h"
#include "OSAL_bufmgr.h"
#include "gatt.h"
#include "ll.h"
#include "ll_common.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile_ota.h"
#include "extBLECentral.h"
#include "timer.h"
#include "log.h"
#include "ll_def.h"
#include "global_config.h"
#include "flash.h"
#include "rflib.h"
#include "clock.h"
/*********************************************************************
    MACROS
*/

// Length of bd addr as a string
//#define B_ADDR_STR_LEN                        15

/*********************************************************************
    CONSTANTS
*/
// Ext Scan duration in 10ms
#define EXT_SCAN_DURATION                     500       // * 10ms

// TRUE to use active scan
#define EXT_SCAN_ACTIVE_SCAN                  TRUE

// TRUE to use white list during scan
#define EXT_SCAN_WHITELIST                    FALSE

// TRUE to use white list when extinit creating link
#define EXT_INIT_WHITELIST                    FALSE

// EXTscan report only extadv
#define DEFAULT_ONLY_REPORT_EXTADV            TRUE

// Default passcode
#define DEFAULT_PASSCODE                      123456//19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE//GAPBOND_PAIRING_MODE_INITIATE//GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           3000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Application states
enum
{
    BLE_STATE_IDLE,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
    BLE_DISC_STATE_IDLE,                // Idle
    BLE_DISC_STATE_SVC,                 // Service discovery
    BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// add by zhufei.zhang 2018.10.25
#define PRIMARY_SERVICE_RES         10//0xFFFF
#define Characteristic_LEN                  20
#define Characteristic_ValueIndex       1
#define Characteristic_NotifyIndex  2
#define Central_Test_ToDoList               6
//#define Build_TESTVECTOR 0
/*********************************************************************
    TYPEDEFS
*/
// Service and Characteristic
typedef struct
{
    uint16_t  charStartHandle;
    uint16_t  charUuid;
    uint8_t     UUID_Len;
    uint8_t     UUID[ATT_UUID_SIZE];
    uint8_t   Properties;
} SimpleCharacteristic;
typedef struct
{
    // Service Info , User Don't Care
    uint8_t     Attribute_Data_Len;
    uint16_t    Attribute_StartHandle;
    uint16_t    End_Group_Handle;
    // User Care
    // Caculate UUID Len
    uint8_t     UUID_Len;
    uint8_t     UUID[ATT_UUID_SIZE];
    // Characteristic
    uint8_t     CharacNum;
    SimpleCharacteristic Characteristic[Characteristic_LEN];
} SimpleGATTReadGroupRsp;
typedef struct
{
    // Primary Service Num
    uint8_t                                 PrimaryServiceCnt;
    // Primary Service Characteristic Index
    uint8_t                                 PrimaryCharacIndex;
    // Characteristic Find Index
    uint8_t                                 CharacFindIndex;
    SimpleGATTReadGroupRsp  ServerGroupService[PRIMARY_SERVICE_RES];
} SimpleGattScanServer;

/*********************************************************************
    GLOBAL VARIABLES
*/
perStatsByChan_t g_perStatsByChanTest;
/*********************************************************************
    EXTERNAL VARIABLES
*/
extern uint32 g_osal_mem_allo_cnt;
extern uint32 g_osal_mem_free_cnt;
extern l2capSARDbugCnt_t g_sarDbgCnt;
extern llGlobalStatistics_t g_pmCounters;
extern gapExtScan_AdvInfo_t gapExtScan_AdvInfo[GAP_EXTSCAN_MAX_SCAN_NUM];
/*********************************************************************
    EXTERNAL FUNCTIONS
*/
extern uint8 llPermit_ctrl_procedure(uint16 connId);
/*********************************************************************
    LOCAL VARIABLES
*/

// Task ID for internal task/event processing
static uint8 extBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";
static const uint8 peerDeviceName[GAP_DEVICE_NAME_LEN]      = "Device Magic";

// Connection handle of current connection
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Value to write
volatile static uint8 simpleBLECharVal = 0;

static uint16 dleTxOctets=251;
static uint16 dleTxTime=2120;

static uint8 phyModeCtrl=0x01;

//static uint16 dleTxOctetsSlave=251;
//volatile static uint16 dleTxTimeSlave=2120;

//static uint8 phyModeCtrlSlave=0x01;

// add by zhufei.zhang 2018.10.25
static SimpleGattScanServer             SimpleClientInfo;
//static SimpleClientADV_ScanData     simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

static uint8 mtu = 247;

// #define EXTENDED_SCAN_DATA_BUFFER_LEN          1000
// static uint8 scanDataBuffer[EXTENDED_SCAN_DATA_BUFFER_LEN];

uint8 peer_address_type=0;
uint8 peer_address[B_ADDR_LEN]= {0,0,0,0,0,0};
static bool findDevByName=FALSE;
/*********************************************************************
    LOCAL FUNCTIONS
*/
static void extBLECentralProcessGATTMsg( gattMsgEvent_t* pMsg );
static void extBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void extBLECentralEventCB( gapCentralRoleEvent_t* pEvent );
static void extBLECentralPasscodeCB( uint8* deviceAddr, uint16 connectionHandle,
                                     uint8 uiInputs, uint8 uiOutputs );
static void extBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//static void extBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void extBLECentral_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t* pMsg );
static void extBLECentralStartDiscoveryService( void );

// add by zhufei.zhang
extern uint32 osal_memory_statics(void);

//static void extBLECentralScanEvtCB( hciEvt_BLEExtAdvPktReport_t* pMsg );
static void extBLECentralScantimeoutEvtCB(hciEvt_ScanTimeout_t* pMsg);
bStatus_t GAPExtCentralRole_CB( gapExtCentralCBs_t* pAppCallbacks );
void ebc_char_data_test(void);
/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
    extBLECentralRssiCB,       // RSSI callback
    extBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
    extBLECentralPasscodeCB,
    extBLECentralPairStateCB
};

static const gapExtCentralCBs_t extBLECentralCB =
{
    NULL,//extBLECentralScanEvtCB,
    extBLECentralScantimeoutEvtCB
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/
void check_PerStatsProcess(void);

/*********************************************************************
    @fn      extBLECentral_Init

    @brief   Initialization function for the Simple BLE Central App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notification).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void extBLECentral_Init( uint8 task_id )
{
    extBLETaskId = task_id;
    //config extscan
    {
        // uint8 extscan_phys = LL_SCAN_PHY_1M_BITMASK | LL_SCAN_PHY_CODED_BITMASK;
        // GAPExtCentralRole_SetParameter(GAPEXTSCAN_SCANPHYS,sizeof(uint8),&extscan_phys);
        uint16 scan_duration = EXT_SCAN_DURATION;
        GAPExtCentralRole_SetParameter(GAPEXTSCAN_SCANDURATION,sizeof ( uint16 ),&scan_duration);
        uint8 scantype = EXT_SCAN_ACTIVE_SCAN;
        GAPExtCentralRole_SetParameter(GAPEXTSCAN_1MPHY_SCANTYPE,sizeof ( uint8 ),&scantype);
        GAPExtCentralRole_SetParameter(GAPEXTSCAN_CODEDPHY_SCANTYPE,sizeof ( uint8 ),&scantype);
        uint8 extscan_whitelist = EXT_SCAN_WHITELIST;
        GAPExtCentralRole_SetParameter(GAPEXTSCAN_FILTERPOLICY,sizeof ( uint8 ),&extscan_whitelist);
        uint8 extscan_legacyadv_rptfilter = DEFAULT_ONLY_REPORT_EXTADV;
        GAPExtCentralRole_SetParameter(GAPEXTSCAN_LEGACYADV_RPTFILTER,sizeof ( uint8 ),&extscan_legacyadv_rptfilter);
    }
    //config extinit
    {
        // uint8 extinit_phys = LL_SCAN_PHY_1M_BITMASK | LL_CONN_PHY_2M_BITMASK | LL_SCAN_PHY_CODED_BITMASK;//LL_SCAN_PHY_1M_BITMASK | LL_CONN_PHY_2M_BITMASK;//LL_SCAN_PHY_CODED_BITMASK
        // GAPExtCentralRole_SetParameter(GAPEXTINIT_INITPHYS,sizeof(uint8),&extinit_phys);
        uint8 extinit_whitelist = EXT_INIT_WHITELIST;
        GAPExtCentralRole_SetParameter(GAPEXTINIT_FILTERPOLICY,sizeof ( uint8 ),&extinit_whitelist);
        uint16 conn_interval_min = 80; //N * 1.25 ms
        uint16 conn_interval_max = 160; //N * 1.25 ms
        uint16 conn_latency = 0;
        uint16 conn_timeout = 200; //N * 10 ms
        //1M extinit config
        GAPExtCentralRole_SetParameter(GAPEXTINIT_1MPHY_CONNINTERVAL_MIN,sizeof ( uint16 ),&conn_interval_min);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_1MPHY_CONNINTERVAL_MAX,sizeof ( uint16 ),&conn_interval_max);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_1MPHY_CONNLATENCY,sizeof ( uint16 ),&conn_latency);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_1MPHY_TIMEOUT,sizeof ( uint16 ),&conn_timeout);
        //2M extinit config
        GAPExtCentralRole_SetParameter(GAPEXTINIT_2MPHY_CONNINTERVAL_MIN,sizeof ( uint16 ),&conn_interval_min);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_2MPHY_CONNINTERVAL_MAX,sizeof ( uint16 ),&conn_interval_max);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_2MPHY_CONNLATENCY,sizeof ( uint16 ),&conn_latency);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_2MPHY_TIMEOUT,sizeof ( uint16 ),&conn_timeout);
        //codedphy extinit config
        GAPExtCentralRole_SetParameter(GAPEXTINIT_CODEDPHY_CONNINTERVAL_MIN,sizeof ( uint16 ),&conn_interval_min);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_CODEDPHY_CONNINTERVAL_MAX,sizeof ( uint16 ),&conn_interval_max);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_CODEDPHY_CONNLATENCY,sizeof ( uint16 ),&conn_latency);
        GAPExtCentralRole_SetParameter(GAPEXTINIT_CODEDPHY_TIMEOUT,sizeof ( uint16 ),&conn_timeout);
    }
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8*)simpleBLEDeviceName);
    // Setup the GAP Bond Manager
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = DEFAULT_PAIRING_MODE;
        uint8 mitm = DEFAULT_MITM_MODE;
        uint8 ioCap = DEFAULT_IO_CAPABILITIES;
        uint8 bonding = DEFAULT_BONDING_MODE;
        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8), &bonding);
    }
    // Initialize GATT Client
    VOID GATT_InitClient();
    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(extBLETaskId);
    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);         // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
    LL_PLUS_PerStats_Init(&g_perStatsByChanTest);
    // Setup a delayed profile startup
    osal_set_event(extBLETaskId, START_DEVICE_EVT);
    //LL_InitExtendedScan(scanDataBuffer, EXTENDED_SCAN_DATA_BUFFER_LEN);
    GAPExtCentralRole_CB((gapExtCentralCBs_t*)&extBLECentralCB);
    LOG("[PEER ADDR]");

    for (int i = 0; i < 6; i++)
    {
        peer_address[i] = *(volatile uint8_t*)(0x11004008 + i);
        LOG("%02x", peer_address[i]);
    }

    LOG("\n");

    if(peer_address[0]==0xff)
    {
        LOG("[PEER NAME] L=%d ",osal_strlen((char*)peerDeviceName));
        LOG("%s",peerDeviceName);
        LOG("\n");
        findDevByName=TRUE;
    }
    else
        HCI_LE_AddWhiteListCmd(peer_address_type, peer_address);

    LOG("extBLECentral_Init\n");
}

/*********************************************************************
    @fn      extBLECentral_ProcessEvent

    @brief   Simple BLE Central Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 extBLECentral_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if (events & SYS_EVENT_MSG)
    {
        uint8* pMsg;

        if ((pMsg = osal_msg_receive(extBLETaskId)) != NULL)
        {
            extBLECentral_ProcessOSALMsg((osal_event_hdr_t*)pMsg);
            // Release the OSAL message
            VOID osal_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & START_DEVICE_EVT)
    {
        // Start the Device
        VOID GAPCentralRole_StartDevice((gapCentralRoleCB_t*)&simpleBLERoleCB);
        // Register with bond manager after starting device
        GAPBondMgr_Register((gapBondCBs_t*)&simpleBLEBondCB);
        return (events ^ START_DEVICE_EVT);
    }

    if (events & CENTRAL_INIT_DONE_EVT)
    {
        //LOG("BLE as central Init Done \r\n");
        GAPExtCentralRole_EnableExtScan();
        return (events ^ CENTRAL_INIT_DONE_EVT);
    }

    if (events & CENTRAL_DISCOVER_DEVDONE_EVT)
    {
        //LOG("CENTRAL_DISCOVER_DEVDONE_EVT \n");
        GAPExtCentralRole_Createconn(peer_address_type,peer_address);
        return (events ^ CENTRAL_DISCOVER_DEVDONE_EVT);
    }

    if (events & START_DISCOVERY_SERVICE_EVT)
    {
        extBLECentralStartDiscoveryService();
        return (events ^ START_DISCOVERY_SERVICE_EVT);
    }

    if (events & EBC_CANCEL_CONN)
    {
        if (simpleBLEState != BLE_STATE_CONNECTED) // not connected
        {
            GAPCentralRole_TerminateLink(simpleBLEConnHandle);
            LOG("Establish Link Time Out.\r\n");
        }

        return (events ^ EBC_CANCEL_CONN);
    }

    if (events & EBC_TERMINATED_CONN)
    {
        if (simpleBLEState == BLE_STATE_CONNECTED) // not connected
        {
            GAPCentralRole_TerminateLink(simpleBLEConnHandle);
        }

        LOG("Terminated Link .s%d d%d\r\n", simpleBLEState, simpleBLEConnHandle);
        return (events ^ EBC_TERMINATED_CONN);
    }

    if(events & START_CHAR_DATA_TEST)
    {
        ebc_char_data_test();
        return (events ^ START_CHAR_DATA_TEST);
    }

    if (events & UPD_CHAN_MAP_EVENT)
    {
        uint8 chanMap[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};

        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT )
        {
            if(llPermit_ctrl_procedure(simpleBLEConnHandle))
            {
                HCI_LE_SetHostChanClassificationCmd(chanMap);
                AT_LOG("CHAN MAP UPDATE\r\n");
            }
            else
                osal_start_timerEx(extBLETaskId,UPD_CHAN_MAP_EVENT,500);
        }

        return (events ^ UPD_CHAN_MAP_EVENT);
    }

    if (events & UPD_CONN_PARAM)
    {
        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)
        {
            if(llPermit_ctrl_procedure(simpleBLEConnHandle))
            {
                uint16 connIntv = 200;//*1.25ms
                uint16 connLatency = 0;
                uint16 connTimeOut = 200;//*10ms
                AT_LOG("UPD CONNPARAM[ %2d %2d %2d %2d]\r\n", connIntv, connIntv, connLatency, connTimeOut);
                GAPCentralRole_UpdateLink(simpleBLEConnHandle, connIntv, connIntv, connLatency, connTimeOut);
            }
            else
                osal_start_timerEx(extBLETaskId,UPD_CONN_PARAM,500);
        }

        return (events ^ UPD_CONN_PARAM);
    }

    if (events & UPD_DATA_LENGTH_EVT)
    {
        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)
        {
            if(llPermit_ctrl_procedure(simpleBLEConnHandle))
            {
                HCI_LE_SetDataLengthCmd(simpleBLEConnHandle, dleTxOctets, dleTxTime);
                AT_LOG("DLE[ %2d %2d ]\r\n", dleTxOctets, dleTxTime);
            }
            else
                osal_start_timerEx(extBLETaskId,UPD_DATA_LENGTH_EVT,500);
        }

        return (events ^ UPD_DATA_LENGTH_EVT);
    }

    if (events & UPD_PHY_MODE_EVT)
    {
        if (simpleBLEConnHandle != GAP_CONNHANDLE_INIT)
        {
            if(llPermit_ctrl_procedure(simpleBLEConnHandle))
            {
                uint8 allPhy = 0x00;
                uint8 txPhy = phyModeCtrl;
                uint8 rxPhy = phyModeCtrl;
                uint16 phyOption = 0x00;
                HCI_LE_SetPhyMode(simpleBLEConnHandle, allPhy, txPhy, rxPhy, phyOption);
                AT_LOG("PHY[ %2d %2d %2d]\r\n", allPhy, txPhy, rxPhy);
            }
            else
                osal_start_timerEx(extBLETaskId,UPD_PHY_MODE_EVT,500);
        }

        return (events ^ UPD_PHY_MODE_EVT);
    }

    if (events & EBC_STOP_EXTSCAN)
    {
        LOG("EBC_STOP_EXTSCAN\r\n");
        GAPExtCentralRole_DisableExtScan();
        return (events ^ EBC_STOP_EXTSCAN);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      extBLECentral_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void extBLECentral_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case GATT_MSG_EVENT:
        extBLECentralProcessGATTMsg( (gattMsgEvent_t*) pMsg );
        break;
    }
}

/*********************************************************************
    @fn      extBLECentralProcessGATTMsg

    @brief   Process GATT messages

    @return  none
*/

static void extBLECentralProcessGATTMsg( gattMsgEvent_t* pMsg )
{
    if ( simpleBLEState != BLE_STATE_CONNECTED )
    {
        // In case a GATT message came after a connection has dropped,
        // ignore the message
        return;
    }

    if(pMsg->hdr.status==bleTimeout)
    {
        AT_LOG("[GATT TO] %x\n",pMsg->method);
        return;
    }

//  LOG("extBLECentralProcessGATTMsg pMsg->method 0x%02X,pMsg->msg.errorRsp.reqOpcode 0x%02X\r\n",pMsg->method,pMsg->msg.errorRsp.reqOpcode);
    if ( ( pMsg->method == ATT_READ_RSP ) ||
            ( ( pMsg->method == ATT_ERROR_RSP ) &&
              ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
    {
        if ( pMsg->method == ATT_ERROR_RSP )
        {
            //uint8 status = pMsg->msg.errorRsp.errCode;
            LOG( "Read Error %d\r\n", pMsg->msg.errorRsp.errCode );
        }
        else
        {
            // After a successful read, display the read value
            //uint8 valueRead = pMsg->msg.readRsp.value[0];
            LOG( "Read rsp Len : %d\r\n", pMsg->msg.readRsp.len);

            if(pMsg->msg.readRsp.len>20)
            {
                for(unsigned char i = 0; i < 10; i++)
                {
                    LOG( "0x%02X,", pMsg->msg.readRsp.value[i]);
                }

                LOG("<--->");

                for(unsigned char i = pMsg->msg.readRsp.len-10; i < pMsg->msg.readRsp.len; i++)
                {
                    LOG( "0x%02X,", pMsg->msg.readRsp.value[i]);
                }

                LOG("\r\n");
            }
            else
            {
                for(unsigned char i = 0; i < pMsg->msg.readRsp.len; i++)
                {
                    LOG( "0x%02X,", pMsg->msg.readRsp.value[i]);
                }

                LOG("\r\n");
            }
        }

        // osal_start_timerEx( extBLETaskId, START_CHAR_DATA_TEST,100 );
        //    simpleBLEProcedureInProgress = FALSE;
    }
    else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
              ( ( pMsg->method == ATT_ERROR_RSP ) &&
                ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
    {
        if ( pMsg->method == ATT_ERROR_RSP )
        {
            //uint8 status = pMsg->msg.errorRsp.errCode;
            LOG( "Write Error: %d\r\n", pMsg->msg.errorRsp.errCode );
        }
        else
        {
            // After a succesful write, display the value that was written and increment value
            LOG( "Write sent: %d\r\n", simpleBLECharVal++);
        }

        //osal_start_timerEx( extBLETaskId, START_CHAR_DATA_TEST,100 );
        //    simpleBLEProcedureInProgress = FALSE;
    }
    else if( pMsg->method == ATT_HANDLE_VALUE_NOTI  ||
             ( ( pMsg->method == ATT_ERROR_RSP ) &&
               ( pMsg->msg.errorRsp.reqOpcode == ATT_HANDLE_VALUE_NOTI ) ) )
    {
        if ( pMsg->method == ATT_ERROR_RSP ||
                pMsg->msg.handleValueNoti.len > ATT_GetCurrentMTUSize(0)-3 )
        {
            //uint8 status = pMsg->msg.errorRsp.errCode;
            LOG( "Ntf Error: %d\r\n", pMsg->msg.errorRsp.errCode );
        }
        else
        {
            LOG( "Read Ntf Len : %d\r\n", pMsg->msg.handleValueNoti.len);

            if(pMsg->msg.handleValueNoti.len>20)
            {
                for(unsigned char i = 0; i < 10; i++)
                {
                    LOG( "0x%02X,", pMsg->msg.handleValueNoti.value[i]);
                }

                LOG("<--->");

                for(unsigned char i =  pMsg->msg.handleValueNoti.len-10; i < pMsg->msg.handleValueNoti.len; i++)
                {
                    LOG( "0x%02X,", pMsg->msg.handleValueNoti.value[i]);
                }
            }
            else
            {
                for(unsigned char i = 0; i < pMsg->msg.handleValueNoti.len; i++)
                {
                    LOG( "0x%02X,", pMsg->msg.handleValueNoti.value[i]);
                }
            }

            LOG("\r\n");
        }
    }
    else //if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
    {
        simpleBLEGATTDiscoveryEvent( pMsg );
    }
}

/*********************************************************************
    @fn      extBLECentralRssiCB

    @brief   RSSI callback.

    @param   connHandle - connection handle
    @param   rssi - RSSI

    @return  none
*/
static void extBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LOG( "RSSI -dB: %d\r\n", (uint8) (-rssi) );
}

/*********************************************************************
    @fn      extBLECentralEventCB

    @brief   Central event callback function.

    @param   pEvent - pointer to event structure

    @return  none
*/
static void extBLECentralEventCB( gapCentralRoleEvent_t* pEvent )
{
//    static int try_num = 0;
    switch ( pEvent->gap.opcode )
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        osal_set_event(extBLETaskId,CENTRAL_INIT_DONE_EVT);
    }
    break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        AT_LOG("\n== GAP_LINK_ESTABLISHED_EVENT ==\r\n");

        if ( pEvent->gap.hdr.status == SUCCESS )
        {
            simpleBLEState = BLE_STATE_CONNECTED;
            simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
            HCI_PPLUS_ConnEventDoneNoticeCmd(extBLETaskId, NULL);

            // //--------------------------------------------------------------------------------------
            // //MTU Size Exchange
            if(mtu>23)
            {
                ATT_SetMTUSizeMax(mtu);
                attExchangeMTUReq_t pReq;
                pReq.clientRxMTU = mtu;
                uint8 status =GATT_ExchangeMTU(simpleBLEConnHandle,&pReq, extBLETaskId);
                LOG( "[MTU Req]%d %d\n",status,pReq.clientRxMTU);
            }
            else
            {
                ATT_SetMTUSizeMax(23);
            }

            //-------------------------------------------------------------------------------------
            // DLE
            llInitFeatureSetDLE(FALSE);

            if(dleTxOctets>27)
            {
                llInitFeatureSetDLE(TRUE);
                osal_start_timerEx( extBLETaskId, UPD_DATA_LENGTH_EVT, 100 );
            }

            //-------------------------------------------------------------------------------------
            //phy update
            HCI_LE_SetDefaultPhyMode(0,0x03,0x01,0x01);
            llInitFeatureSet2MPHY(FALSE);

            if(phyModeCtrl !=0x01)
            {
                llInitFeatureSet2MPHY(TRUE);
                HCI_LE_SetDefaultPhyMode(0,0x00,0x03,0x03);
                osal_start_timerEx( extBLETaskId, UPD_PHY_MODE_EVT, 300 );
            }

            HCI_LE_ReadRemoteUsedFeaturesCmd(simpleBLEConnHandle);
            HCI_ReadRemoteVersionInfoCmd(simpleBLEConnHandle);
            // osal_start_timerEx( extBLETaskId, UPD_CHAN_MAP_EVENT, 20000 );
            // osal_start_timerEx( extBLETaskId, UPD_CONN_PARAM, 30000 );
            osal_start_timerEx( extBLETaskId, START_DISCOVERY_SERVICE_EVT, \
                                DEFAULT_SVC_DISCOVERY_DELAY );
        }
        else if(pEvent->gap.hdr.status==LL_STATUS_WARNING_WAITING_LLIRQ)
        {
            AT_LOG( "[WAITING LL_IRQ]. " );
            AT_LOG( "Reason: 0x%02x\r\n", pEvent->gap.hdr.status);
            osal_set_event(extBLETaskId,CENTRAL_DISCOVER_DEVDONE_EVT);
        }
        else
        {
            simpleBLEState = BLE_STATE_IDLE;
            simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
            simpleBLEDiscState = BLE_DISC_STATE_IDLE;
            osal_start_timerEx( extBLETaskId, CENTRAL_INIT_DONE_EVT, \
                                1000 );
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        AT_LOG("[TVEC] %08x DISC.R[0x%2x]\n",getMcuPrecisionCount(),pEvent->linkTerminate.reason);
        check_PerStatsProcess();
        AT_LOG("[PMCNT] rdErr %d rstErr %d trgErr %d\n",g_pmCounters.ll_rfifo_read_err,g_pmCounters.ll_rfifo_rst_err,g_pmCounters.ll_trigger_err);
        // Terminate Link , memset SimpleGattScanServer structure
        osal_memset(&SimpleClientInfo,0,sizeof(SimpleGattScanServer));
        osal_start_timerEx( extBLETaskId, CENTRAL_INIT_DONE_EVT, \
                            10*1000 );
    }
    break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        LOG( "Server Request for Param Update\r\n");
    }
    break;

    default:
        LOG(" extBLECentralEventCB --> pEvent->gap.opcode: 0x%02X\r\n", pEvent->gap.opcode);
        break;
    }
}

/*********************************************************************
    @fn      pairStateCB

    @brief   Pairing state callback.

    @return  none
*/
static void extBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
    LOG("extBLECentralPairStateCB in param state 0x%02X,status 0x%02X\r\n",state,status);

    if ( state == GAPBOND_PAIRING_STATE_STARTED )
    {
        LOG( "Pairing started\n" );
    }
    else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
    {
        if ( status == SUCCESS )
        {
            LOG( "Pairing success\n" );
        }
        else
        {
            LOG( "Pairing fail\n" );
        }
    }
    else if ( state == GAPBOND_PAIRING_STATE_BONDED )
    {
        if ( status == SUCCESS )
        {
            LOG( "Bonding success\n" );
        }
    }
}

/*********************************************************************
    @fn      extBLECentralPasscodeCB

    @brief   Passcode callback.

    @return  none
*/
static void extBLECentralPasscodeCB( uint8* deviceAddr, uint16 connectionHandle,
                                     uint8 uiInputs, uint8 uiOutputs )
{
    LOG("extBLECentralPasscodeCB\r\n");
    #if (HAL_LCD == TRUE)
    uint32  passcode;
    uint8   str[7];
    // Create random passcode
    LL_Rand( ((uint8*) &passcode), sizeof( uint32 ));
    passcode %= 1000000;

    // Display passcode to user
    if ( uiOutputs != 0 )
    {
        LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( (char*) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
    }

    #endif
    uint32 passcode = 0;
    GAPBondMgr_GetParameter( GAPBOND_DEFAULT_PASSCODE,&passcode);
    // Send passcode response
    GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
}

/*********************************************************************
    @fn      extBLECentralStartDiscoveryService

    @brief   Start service discovery.

    @return  none
*/
static void extBLECentralStartDiscoveryService( void )
{
    LOG("==>extBLECentralStartDiscoveryService\r\n");
    // add by zhufei.zhang 2018.10.25
    // before start discovery , memset the variable
    osal_memset(&SimpleClientInfo,0,sizeof(SimpleGattScanServer));
    volatile uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                              HI_UINT16(SIMPLEPROFILE_SERV_UUID)
                                            };
    simpleBLEDiscState = BLE_DISC_STATE_SVC;
    // Discovery simple BLE service
    GATT_DiscAllPrimaryServices(simpleBLEConnHandle,extBLETaskId);
}

/*********************************************************************
    @fn      simpleBLEGATTDiscoveryEvent

    @brief   Process GATT discovery event

    @return  none
*/
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t* pMsg )
{
    volatile attReadByTypeReq_t req;

    if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
    {
        // add by zhufei.zhang 2018/10/24
        if( ( pMsg->method ==  ATT_READ_BY_GRP_TYPE_RSP) &&
                ( pMsg->msg.readByGrpTypeRsp.numGrps > 0 ) )
        {
            for(unsigned char i = 0 ; i < pMsg->msg.readByGrpTypeRsp.numGrps; i++)
            {
                // Current Attribute LEN
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].Attribute_Data_Len = \
                        pMsg->msg.readByGrpTypeRsp.len;
                // Current Attribute Handle
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].Attribute_StartHandle = \
                        BUILD_UINT16(   pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i], \
                                        pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 1]);
                // Current Attribute End Group Handle
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].End_Group_Handle = \
                        BUILD_UINT16(   pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i+2], \
                                        pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 3]);
                // Caculate Primary Service UUID Length
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].UUID_Len = \
                        pMsg->msg.readByGrpTypeRsp.len - 4;
                // Copy UUID
                osal_memcpy(SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].UUID,\
                            &(pMsg->msg.readByGrpTypeRsp.dataList[pMsg->msg.readByGrpTypeRsp.len * i + 4]),\
                            SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryServiceCnt].UUID_Len);
                // Primary Service Count Index ++
                SimpleClientInfo.PrimaryServiceCnt++;
            }
        }
        else if ( ( pMsg->method == ATT_READ_BY_GRP_TYPE_RSP  &&
                    pMsg->hdr.status == bleProcedureComplete ) ||
                  ( pMsg->method == ATT_ERROR_RSP ) )
        {
            // Primary Service Discover OK , Prepare Discover Characteristic
            simpleBLEDiscState = BLE_DISC_STATE_CHAR;

            if( SimpleClientInfo.PrimaryServiceCnt > 0 )
            {
                // Characteristic Find Index Init
                SimpleClientInfo.PrimaryCharacIndex = 0;
                SimpleClientInfo.CharacFindIndex = 0;
                GATT_DiscAllChars(  simpleBLEConnHandle,
                                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Attribute_StartHandle,\
                                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].End_Group_Handle,
                                    extBLETaskId );
            }
        }
    }
    else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
    {
        // Characteristic found, store handle
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
                pMsg->msg.readByTypeRsp.numPairs > 0 )
        {
            // Iterate through all three pairs found.
            for(unsigned char i = 0; i < pMsg->msg.readByTypeRsp.numPairs ; i++)
            {
                // Extract the starting handle, ending handle, and UUID of the current characteristic.
                // characteristic Handle
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[SimpleClientInfo.CharacFindIndex].charStartHandle = \
                        BUILD_UINT16(   pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i], \
                                        pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 1]);
                // Characteristic Properties
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[SimpleClientInfo.CharacFindIndex].Properties = \
                        pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 2];
                // Characteristic UUID
                SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[SimpleClientInfo.CharacFindIndex].charUuid = \
                        BUILD_UINT16(   pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 5], \
                                        pMsg->msg.readByTypeRsp.dataList[pMsg->msg.readByTypeRsp.len * i + 6]);
                SimpleClientInfo.CharacFindIndex++;
            }
        }
        else if(( pMsg->method == ATT_READ_BY_TYPE_RSP  &&
                  pMsg->hdr.status == bleProcedureComplete ) ||
                ( pMsg->method == ATT_ERROR_RSP ))
        {
            SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].CharacNum = \
                    SimpleClientInfo.CharacFindIndex;
            SimpleClientInfo.CharacFindIndex = 0;
            AT_LOG(" Service 0x%02X%02X Characteristic Find Success \r\n",\
                   SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].UUID[1],\
                   SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].UUID[0]);

            for(unsigned char i = 0; i< SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].CharacNum; i++)
            {
                AT_LOG("Chars found handle is :0x%04X,Properties is: 0x%02X,uuid is:0x%04X\r\n", \
                       SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[i].charStartHandle,\
                       SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[i].Properties,\
                       SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Characteristic[i].charUuid);
            }

            if(++SimpleClientInfo.PrimaryCharacIndex < SimpleClientInfo.PrimaryServiceCnt)
            {
                GATT_DiscAllChars(  simpleBLEConnHandle,
                                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].Attribute_StartHandle,\
                                    SimpleClientInfo.ServerGroupService[SimpleClientInfo.PrimaryCharacIndex].End_Group_Handle,
                                    extBLETaskId );
            }
            else
            {
                AT_LOG("All Characteristic Discover Success \r\n");
                simpleBLEDiscState = BLE_DISC_STATE_IDLE;
                osal_start_timerEx( extBLETaskId, START_CHAR_DATA_TEST,100 );
            }
        }
    }
    else
    {
        LOG("simpleBLEDiscState = BLE_DISC_STATE_IDLE \r\n");
    }
}

/*********************************************************************
    @fn      bdAddr2Str

    @brief   Convert Bluetooth address to string

    @return  none
*/
// char* bdAddr2Str( uint8* pAddr )
// {
//     uint8       i;
//     char        hex[] = "0123456789ABCDEF";
//     static char str[B_ADDR_STR_LEN];
//     char*        pStr = str;
// //  *pStr++ = '0';
// //  *pStr++ = 'x';
//     // Start from end of addr
//     pAddr += B_ADDR_LEN;

//     for ( i = B_ADDR_LEN; i > 0; i-- )
//     {
//         *pStr++ = hex[*--pAddr >> 4];
//         *pStr++ = hex[*pAddr & 0x0F];
//     }

//     *pStr = 0;
//     return str;
// }

void check_PerStatsProcess(void)
{
    perStats_t perStats;
    uint16 perRxNumTotal=0;
    uint16 perRxCrcErrTotal=0;
    uint16 perTxNumTotal=0;
    uint16 perTxAckTotal=0;
    uint16 perRxToCntTotal=0;
    uint16 perConnEvtTotal=0;
    LOG("[PER STATS Notify]\r");
    LOG("----- ch connN rxNum rxCrc rxToN txAck txRty \r");

    for(uint8 i=0; i<37; i++)
    {
        LL_PLUS_PerStasReadByChn(i,&perStats);
        LOG("[PER] %02d %05d %05d %05d %05d %05d %05d\n",i,perStats.connEvtCnt,
            perStats.rxNumPkts,
            perStats.rxNumCrcErr,
            perStats.rxToCnt,
            perStats.TxNumAck,
            perStats.txNumRetry);
        perConnEvtTotal+= perStats.connEvtCnt;
        perRxNumTotal+= perStats.rxNumPkts;
        perRxCrcErrTotal+= perStats.rxNumCrcErr;
        perRxToCntTotal+= perStats.rxToCnt;
        perTxAckTotal+= perStats.TxNumAck;
        perTxNumTotal+= perStats.txNumRetry;
    }

    LOG("TOTAL ch connN rxNum rxCrc rxToN txAck txRty \r");
    LOG("\n[PER] -- %05d %05d %05d %05d %05d %05d\n",perConnEvtTotal,
        perRxNumTotal,
        perRxCrcErrTotal,
        perRxToCntTotal,
        perTxAckTotal,
        perTxNumTotal);
    LL_PLUS_PerStatsReset();
}

// static void extBLECentralScanEvtCB( hciEvt_BLEExtAdvPktReport_t* pMsg )
// {}

static void extBLECentralScantimeoutEvtCB(hciEvt_ScanTimeout_t* pMsg)
{
    //LOG("extBLECentralScantimeoutEvtCB\n");
    int Index = -1;

    for(int i=0; i<GAP_EXTSCAN_MAX_SCAN_NUM; i++)
    {
        if(gapExtScan_AdvInfo[i].inuse)
        {
            //analyse advinfo get localname
            uint16 datalen=0;
            uint8 localname_len=0;
            uint8 localname[255];

            // DATA FORMAT : Length + AD Type + AD Data
            while(datalen<gapExtScan_AdvInfo[i].advrpt.dataLen)
            {
                uint8 ADLen = gapExtScan_AdvInfo[i].advrpt.rptData[datalen];
                uint8 ADType = gapExtScan_AdvInfo[i].advrpt.rptData[datalen+1];

                if(ADLen<2)//ADType error
                    break;

                if(ADType == GAP_ADTYPE_LOCAL_NAME_SHORT || ADType == GAP_ADTYPE_LOCAL_NAME_COMPLETE)
                {
                    localname_len= ADLen-1;
                    osal_memcpy(localname, &(gapExtScan_AdvInfo[i].advrpt.rptData[datalen+2]),localname_len);
                    localname[localname_len] = '\0';
                    //LOG("localname:%s\n",localname);
                    break;
                }

                datalen+= ADLen+1;
            }

            //show advinfo
            LOG("\nNO.%d,advEvtType:0x%02x,advAddrType:%d,advaddr:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x ",i,\
                gapExtScan_AdvInfo[i].advrpt.advEvt,gapExtScan_AdvInfo[i].advrpt.advAddrType,gapExtScan_AdvInfo[i].advrpt.advAddr[0],gapExtScan_AdvInfo[i].advrpt.advAddr[1],\
                gapExtScan_AdvInfo[i].advrpt.advAddr[2],gapExtScan_AdvInfo[i].advrpt.advAddr[3],gapExtScan_AdvInfo[i].advrpt.advAddr[4],gapExtScan_AdvInfo[i].advrpt.advAddr[5]);

            if(localname_len)
                LOG("localname:%s",localname);

            LOG("\nprimPHY:%d,secPHY:%d,advSID:%d,Txpower:%d,Rssi:%d\n",gapExtScan_AdvInfo[i].advrpt.primaryPHY,gapExtScan_AdvInfo[i].advrpt.secondaryPHY,\
                gapExtScan_AdvInfo[i].advrpt.advertisingSID,gapExtScan_AdvInfo[i].advrpt.txPower,gapExtScan_AdvInfo[i].advrpt.rssi);
            LOG("datafull:%d,datacompleted:%d,datatruncated:%d\n",gapExtScan_AdvInfo[i].datafull,gapExtScan_AdvInfo[i].datacompleted,gapExtScan_AdvInfo[i].datatruncated);
            LOG("datalen:%d",gapExtScan_AdvInfo[i].advrpt.dataLen);

            for(int j=0; j< gapExtScan_AdvInfo[i].advrpt.dataLen; j++)
            {
                if((j%20)==0)
                    LOG("\n");

                LOG("0x%02x ",gapExtScan_AdvInfo[i].advrpt.rptData[j]);
            }

            LOG("\n\n");

            //find target device
            if(findDevByName && (localname_len==osal_strlen((char*)peerDeviceName)))
            {
                if(osal_memcmp(localname,peerDeviceName, localname_len))
                {
                    Index = i;
                    peer_address_type = gapExtScan_AdvInfo[i].advrpt.advAddrType;
                    osal_memcpy(peer_address,gapExtScan_AdvInfo[i].advrpt.advAddr,B_ADDR_LEN);
                }
            }

            if(!findDevByName)
            {
                if(osal_memcmp(gapExtScan_AdvInfo[i].advrpt.advAddr,peer_address, B_ADDR_LEN))
                {
                    Index = i;
                    peer_address_type = gapExtScan_AdvInfo[i].advrpt.advAddrType;
                }
            }
        }
    }

    if(Index == -1)
    {
        LOG("\npeerdevice not found\n");
        osal_set_event(extBLETaskId,CENTRAL_INIT_DONE_EVT);
    }
    else
    {
        LOG("\nfind target,Index:%d start establish\n",Index);
        osal_set_event(extBLETaskId,CENTRAL_DISCOVER_DEVDONE_EVT);
    }
}

void ebc_char_data_test(void)
{
    for(uint8 service_index=0; service_index<SimpleClientInfo.PrimaryServiceCnt; service_index++)
    {
        //LOG("CHARnum%d\n",SimpleClientInfo.ServerGroupService[service_index].CharacNum);
        for(uint8 char_index=0; char_index<SimpleClientInfo.ServerGroupService[service_index].CharacNum; char_index++)
        {
            //LOG("%x\n",SimpleClientInfo.ServerGroupService[service_index].Characteristic[char_index].Properties);
            if( SimpleClientInfo.ServerGroupService[service_index].Characteristic[char_index].Properties & GATT_PROP_NOTIFY )
            {
                attWriteReq_t* pReq;
                pReq = osal_mem_alloc(sizeof(attWriteReq_t));
                pReq->sig = 0;
                pReq->cmd = 0;
                pReq->handle = SimpleClientInfo.ServerGroupService[service_index].\
                               Characteristic[char_index].charStartHandle + Characteristic_NotifyIndex;
                pReq->len = 2;
                pReq->value[0] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY);
                pReq->value[1] = (unsigned char)(GATT_CLIENT_CFG_NOTIFY >> 8);
                bStatus_t status = GATT_WriteCharValue(simpleBLEConnHandle, pReq, extBLETaskId);

                if(status == SUCCESS)
                {
                    LOG("GATT_WriteCharValue EnableNotify Handle:0x%2X success \r\n",pReq->handle);
                }
                else
                {
                    LOG("GATT_WriteCharValue EnableNotify Handle:0x%2X ERROR %d\r\n",pReq->handle,status);
                }

                osal_mem_free(pReq);
                return;
            }
        }
    }
}
/*********************************************************************
*********************************************************************/

