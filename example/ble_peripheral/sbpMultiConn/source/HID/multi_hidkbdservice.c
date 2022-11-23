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

/******************************************************************************

 *****************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "gattservapp.h"
#include "multi_hidkbdservice.h"
#include "peripheralMultiConn.h"
#include "battservice.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/
uint8 multi_hid_task_id=0XFF;
/*********************************************************************
    GLOBAL VARIABLES
*/
// HID service
CONST uint8 hidServUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_SERV_UUID), HI_UINT16(HID_SERV_UUID)
};

// HID Boot Keyboard Input Report characteristic
CONST uint8 hidBootKeyInputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_KEY_INPUT_UUID), HI_UINT16(BOOT_KEY_INPUT_UUID)
};

// HID Boot Mouse Input Report characteristic
CONST uint8 hidBootMouseInputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_MOUSE_INPUT_UUID), HI_UINT16(BOOT_MOUSE_INPUT_UUID)
};

// HID Boot Keyboard Output Report characteristic
CONST uint8 hidBootKeyOutputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_KEY_OUTPUT_UUID), HI_UINT16(BOOT_KEY_OUTPUT_UUID)
};

// HID Information characteristic
CONST uint8 hidInfoUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_INFORMATION_UUID), HI_UINT16(HID_INFORMATION_UUID)
};

// HID Report Map characteristic
CONST uint8 hidReportMapUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(REPORT_MAP_UUID), HI_UINT16(REPORT_MAP_UUID)
};

// HID Control Point characteristic
CONST uint8 hidControlPointUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_CTRL_PT_UUID), HI_UINT16(HID_CTRL_PT_UUID)
};

// HID Report characteristic
CONST uint8 hidReportUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(REPORT_UUID), HI_UINT16(REPORT_UUID)
};

// HID Protocol Mode characteristic
CONST uint8 hidProtocolModeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(PROTOCOL_MODE_UUID), HI_UINT16(PROTOCOL_MODE_UUID)
};

/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

// HID Information characteristic value
static CONST uint8 hidInfo[HID_INFORMATION_LEN] =
{
    LO_UINT16(0x0111), HI_UINT16(0x0111),             // bcdHID (USB HID version)
    0x00,                                             // bCountryCode
    HID_KBD_FLAGS                                     // Flags
};


static CONST uint8 hidReportMap[] =
{

    #if EN_CONSUMER_MODE
    0x05, 0x0C,                     // Usage Page (Consumer)  ---- 0x0c
    0x09, 0x01,                     // Usage (Consumer Control)
    0xA1, 0x01,                     // Collection (Application)
    0x85, HID_RPT_ID_CC_IN,                     //     Report Id (2)

    //0x09, 0xCD,                     //     Usage (Play/Pause)
    //0x09, 0xE9,                     //
    //0x09, 0xB5,                     //     Usage (Scan Next Track)
    //0x09, 0xB6,                     //     Usage (Scan Previous Track)
    //0x09, 0xEA,                     //     Usage (Volume Down)
    0x09, 0xE9,                     //     Usage (Volume Up)
    //0x09, 0xE9,                     //     Power
    //0x09, 0x41,                     //     Usage (Menu Pick)

    //0x15, 0x00,                     //     Logical minimum (0)
    0x25, 0x01,                     //     Logical maximum (1)
    0x75, 0x08,                     //     Report Size (1)
    0x95, 0x01,                     //     Report Count (8)
    0x81, 0x02,                     //
    0xC0,
    #endif

};

#define DEFAULT_HID_IDLE_TIMEOUT              0
// HID Dev configuration
static hidDevCfg_t hidKbdCfg =
{
    DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
    HID_KBD_FLAGS               // HID feature flags
};


// HID report map length
uint16 hidReportMapLen = sizeof(hidReportMap);

// HID report mapping table
hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

/*********************************************************************
    Profile Attributes - variables
*/

// HID Service attribute
static CONST gattAttrType_t hidService = { ATT_BT_UUID_SIZE, hidServUUID };

// Include attribute (Battery service)
//static uint16 include = GATT_INVALID_HANDLE;

// HID Information characteristic
static uint8 hidInfoProps = GATT_PROP_READ;

//HID Report Map characteristic
static uint8 hidReportMapProps = GATT_PROP_READ;

// HID External Report Reference Descriptor
//static uint8 hidExtReportRefDesc[ATT_BT_UUID_SIZE] =
//{ LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID) };

// HID Control Point characteristic
static uint8 hidControlPointProps = GATT_PROP_WRITE_NO_RSP;
static uint8 hidControlPoint;

// HID Protocol Mode characteristic
static uint8 hidProtocolModeProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
uint8 hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID Report characteristic, key input
static uint8 hidReportKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportKeyIn;
static gattCharCfg_t hidReportKeyInClientCharCfg[GATT_MAX_NUM_CONN];

// HID Report Reference characteristic descriptor, key input
static uint8 hidReportRefKeyIn[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, LED output
//static uint8 hidReportLedOutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportLedOut;

// HID Report Reference characteristic descriptor, LED output
//static uint8 hidReportRefLedOut[HID_REPORT_REF_LEN] =
//{ HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT };

// HID Boot Keyboard Input Report
//static uint8 hidReportBootKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
//static uint8 hidReportBootKeyIn;
//static gattCharCfg_t hidReportBootKeyInClientCharCfg[GATT_MAX_NUM_CONN];

//// HID Boot Keyboard Output Report
//static uint8 hidReportBootKeyOutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportBootKeyOut;

// HID Boot Mouse Input Report
//static uint8 hidReportBootMouseInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
//static uint8 hidReportBootMouseIn;
//static gattCharCfg_t hidReportBootMouseInClientCharCfg[GATT_MAX_NUM_CONN];

// Feature Report
//static uint8 hidReportFeatureProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 hidReportFeature;

// HID Report Reference characteristic descriptor, Feature
//static uint8 hidReportRefFeature[HID_REPORT_REF_LEN] =
//{ HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE };

#if EN_MOUSE_REPORT

// HID Report Reference characteristic descriptor, mouse input
static uint8 hidReportRefMouseIn[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT };

static uint8 hidReportMouseInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportMouseIn;
static gattCharCfg_t hidReportMouseInClientCharCfg[GATT_MAX_NUM_CONN];

#endif
#if EN_CONSUMER_MODE
// HID Report Reference characteristic descriptor, consumer control input
static uint8 hidReportRefCCIn[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, consumer control input
static uint8 hidReportCCInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportCCIn;
static gattCharCfg_t hidReportCCInClientCharCfg[GATT_MAX_NUM_CONN];

#endif

static uint8 HidDev_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                uint8* pValue, uint16* pLen, uint16 offset, uint8 maxLen );
static bStatus_t HidDev_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                     uint8* pValue, uint16 len, uint16 offset );
static hidRptMap_t* hidDevRptByHandle( uint16 handle );
static hidRptMap_t* hidDevRptByCccdHandle( uint16 handle );

static uint8 hidKbdRcvReport( uint8 len, uint8* pData );
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint16* pLen, uint8* pData );
static void hidKbdEvtCB( uint8 evt );


static hidRptMap_t* pHidDevRptTbl;

static uint8 hidDevRptTblLen;

static hidDevCB_t* pHidDevCB;

static hidDevCfg_t* pHidDevCfg;


// TRUE if boot mouse enabled
static uint8 hidBootMouseEnabled = FALSE;

/*********************************************************************
    Profile Attributes - Table
*/

static gattAttribute_t hidAttrTbl[] =
{
    // HID Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)& hidService                      /* pValue */
    },

    #if 1

    // HID Information characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidInfoProps
    },

    // HID Information characteristic
    {
        { ATT_BT_UUID_SIZE, hidInfoUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8*) hidInfo
    },
    #endif

    // HID Control Point characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidControlPointProps
    },

    // HID Control Point characteristic
    {
        { ATT_BT_UUID_SIZE, hidControlPointUUID },
        GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidControlPoint
    },

    // HID Protocol Mode characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidProtocolModeProps
    },

    // HID Protocol Mode characteristic
    {
        { ATT_BT_UUID_SIZE, hidProtocolModeUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidProtocolMode
    },


    // HID Report Map characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportMapProps
    },

    // HID Report Map characteristic
    {
        { ATT_BT_UUID_SIZE, hidReportMapUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8*) hidReportMap
    },




    // HID Report characteristic, key input declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportKeyInProps
    },

    // HID Report characteristic, key input
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportKeyIn
    },

    // HID Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8*)& hidReportKeyInClientCharCfg
    },

    // HID Report Reference characteristic descriptor, key input
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefKeyIn
    },


    #if EN_CONSUMER_MODE
    // HID Report characteristic declaration, consumer control
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportCCInProps
    },

    // HID Report characteristic, consumer control
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportCCIn
    },

    // HID Report characteristic client characteristic configuration, consumer control
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8*)& hidReportCCInClientCharCfg
    },

    // HID Report Reference characteristic descriptor, consumer control
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefCCIn
    },

    #endif

};

/*********************************************************************
    LOCAL FUNCTIONS
*/

/*********************************************************************
    PROFILE CALLBACKS
*/

// Service Callbacks
CONST gattServiceCBs_t hidKbdCBs =
{
    HidDev_ReadAttrCB,  // Read callback function pointer
    HidDev_WriteAttrCB, // Write callback function pointer
    NULL                // Authorization callback function pointer
};

/*********************************************************************
    PROFILE CALLBACKS
*/

static hidDevCB_t hidKbdHidCBs =
{
    hidKbdRptCB,
    hidKbdEvtCB,
    NULL
};


void HidKbd_Serive_reset_ccd(void)
{
    for ( uint8 i = 0; i < GATT_MAX_NUM_CONN; i++ )
    {
        hidReportCCInClientCharCfg[i].connHandle = INVALID_CONNHANDLE;
        hidReportCCInClientCharCfg[i].value = GATT_CFG_NO_OPERATION;
        hidReportKeyInClientCharCfg[i].connHandle = INVALID_CONNHANDLE;
        hidReportKeyInClientCharCfg[i].value = GATT_CFG_NO_OPERATION;
    }
}


/*********************************************************************
    @fn      hidDevRptByHandle

    @brief   Find the HID report structure for the given handle.

    @param   handle - ATT handle

    @return  Pointer to HID report structure
*/
static hidRptMap_t* hidDevRptByHandle( uint16 handle )
{
    uint8       i;
    hidRptMap_t* p = pHidDevRptTbl;

    for ( i = hidDevRptTblLen; i > 0; i--, p++ )
    {
        if ( p->handle == handle && p->mode == hidProtocolMode)
        {
            return p;
        }
    }

    return NULL;
}


static hidRptMap_t* hidDevRptByCccdHandle( uint16 handle )
{
    uint8       i;
    hidRptMap_t* p = pHidDevRptTbl;

    for ( i = hidDevRptTblLen; i > 0; i--, p++ )
    {
        if ( p->cccdHandle == handle)
        {
            return p;
        }
    }

    return NULL;
}


/*********************************************************************
    @fn      hidKbdRptCB

    @brief   HID Dev report callback.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   uuid - attribute uuid.
    @param   oper - operation:  read, write, etc.
    @param   len - Length of report.
    @param   pData - Report data.

    @return  GATT status code.
*/
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint16* pLen, uint8* pData )
{
    uint8 status = SUCCESS;

    // write
    if ( oper == HID_DEV_OPER_WRITE )
    {
        if ( uuid == REPORT_UUID )
        {
            // process write to LED output report; ignore others
            if ( type == HID_REPORT_TYPE_OUTPUT )
            {
                status = hidKbdRcvReport( *pLen, pData );
            }
        }

        if ( status == SUCCESS )
        {
            status = HidKbd_SetParameter( id, type, uuid, *pLen, pData );
        }
    }
    // read
    else if ( oper == HID_DEV_OPER_READ )
    {
        status = HidKbd_GetParameter( id, type, uuid, pLen, pData );
    }
    // notifications enabled
    else if ( oper == HID_DEV_OPER_ENABLE )
    {
        if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
        {
            hidBootMouseEnabled = TRUE;
            hidBootMouseEnabled=hidBootMouseEnabled;//clear warning
        }
    }
    // notifications disabled
    else if ( oper == HID_DEV_OPER_DISABLE )
    {
        if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
        {
            hidBootMouseEnabled = FALSE;
        }
    }

    return status;
}

/*********************************************************************
    @fn      hidKbdEvtCB

    @brief   HID Dev event callback.

    @param   evt - event ID.

    @return  HID response code.
*/
static void hidKbdEvtCB( uint8 evt )
{
    // process enter/exit suspend or enter/exit boot mode
    return;
}

/*********************************************************************
    @fn      hidKbdRcvReport

    @brief   Process an incoming HID keyboard report.

    @param   len - Length of report.
    @param   pData - Report data.

    @return  status
*/
static uint8 hidKbdRcvReport( uint8 len, uint8* pData )
{
    return 0;
}

/*********************************************************************
    @fn          HidDev_ReadAttrCB

    @brief       HID Dev attribute read callback.

    @param       connHandle - connection message was received on
    @param       pAttr - pointer to attribute
    @param       pValue - pointer to data to be read
    @param       pLen - length of data to be read
    @param       offset - offset of the first octet to be read
    @param       maxLen - maximum length of data to be read
    @param       method - type of read message

    @return      SUCCESS, blePending or Failure
*/
uint8 HidDev_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                         uint8* pValue, uint16* pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t   status = SUCCESS;
    hidRptMap_t* pRpt;
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

    // Only report map is long
    if ( offset > 0 && uuid != REPORT_MAP_UUID )
    {
        return ( ATT_ERR_ATTR_NOT_LONG );
    }

    if ( uuid == REPORT_UUID ||
            uuid == BOOT_KEY_INPUT_UUID ||
            uuid == BOOT_KEY_OUTPUT_UUID ||
            uuid == BOOT_MOUSE_INPUT_UUID )
    {
        // find report ID in table
        if ( (pRpt = hidDevRptByHandle(pAttr->handle)) != NULL )
        {
            // execute report callback
            status  = (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
                                              HID_DEV_OPER_READ, pLen, pValue );
        }
        else
        {
            *pLen = 0;
        }
    }
    else if ( uuid == REPORT_MAP_UUID )
    {
        // verify offset
        if ( offset >= hidReportMapLen )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // determine read length
            *pLen = MIN( maxLen, (hidReportMapLen - offset) );
            // copy data
            osal_memcpy( pValue, pAttr->pValue + offset, *pLen );
            LOG("map\n");
        }
    }
    else if ( uuid == HID_INFORMATION_UUID )
    {
        *pLen = HID_INFORMATION_LEN;
        osal_memcpy( pValue, pAttr->pValue, HID_INFORMATION_LEN );
    }
    else if ( uuid == GATT_REPORT_REF_UUID )
    {
        *pLen = HID_REPORT_REF_LEN;
        osal_memcpy( pValue, pAttr->pValue, HID_REPORT_REF_LEN );
    }
    else if ( uuid == PROTOCOL_MODE_UUID )
    {
        *pLen = HID_PROTOCOL_MODE_LEN;
        pValue[0] = pAttr->pValue[0];
    }
    else if ( uuid == GATT_EXT_REPORT_REF_UUID )
    {
        *pLen = HID_EXT_REPORT_REF_LEN;
        osal_memcpy( pValue, pAttr->pValue, HID_EXT_REPORT_REF_LEN );
    }

    return ( status );
}

/*********************************************************************
    @fn      HidDev_WriteAttrCB

    @brief   HID Dev attribute read callback.

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/
bStatus_t HidDev_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                              uint8* pValue, uint16 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
    hidRptMap_t* pRpt;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if ( offset > 0 )
    {
        return ( ATT_ERR_ATTR_NOT_LONG );
    }

    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

    if ( uuid == REPORT_UUID ||
            uuid == BOOT_KEY_OUTPUT_UUID )
    {
        // find report ID in table
        if ((pRpt = hidDevRptByHandle(pAttr->handle)) != NULL)
        {
            // execute report callback
            status  = (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
                                              HID_DEV_OPER_WRITE, &len, pValue );
        }
    }
    else if ( uuid == HID_CTRL_PT_UUID )
    {
        // Validate length and value range
        if ( len == 1 )
        {
            if ( pValue[0] == HID_CMD_SUSPEND ||  pValue[0] == HID_CMD_EXIT_SUSPEND )
            {
                // execute HID app event callback
                (*pHidDevCB->evtCB)( (pValue[0] == HID_CMD_SUSPEND) ?
                                     HID_DEV_SUSPEND_EVT : HID_DEV_EXIT_SUSPEND_EVT );
            }
            else
            {
                status = ATT_ERR_INVALID_VALUE;
            }
        }
        else
        {
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
    }
    else if ( uuid == GATT_CLIENT_CHAR_CFG_UUID )
    {
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );

        if ( status == SUCCESS )
        {
            uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );

            // find report ID in table
            if ( (pRpt = hidDevRptByCccdHandle(pAttr->handle)) != NULL )
            {
                // execute report callback
                (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
                                        (charCfg == GATT_CLIENT_CFG_NOTIFY) ?
                                        HID_DEV_OPER_ENABLE : HID_DEV_OPER_DISABLE,
                                        &len, pValue );
            }
        }
    }
    else if ( uuid == PROTOCOL_MODE_UUID )
    {
        if ( len == HID_PROTOCOL_MODE_LEN )
        {
            if ( pValue[0] == HID_PROTOCOL_MODE_BOOT ||
                    pValue[0] == HID_PROTOCOL_MODE_REPORT )
            {
                pAttr->pValue[0] = pValue[0];
                // execute HID app event callback
                (*pHidDevCB->evtCB)( (pValue[0] == HID_PROTOCOL_MODE_BOOT) ?
                                     HID_DEV_SET_BOOT_EVT : HID_DEV_SET_REPORT_EVT );
            }
            else
            {
                status = ATT_ERR_INVALID_VALUE;
            }
        }
        else
        {
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
    }

    return ( status );
}

/*********************************************************************
    @fn      HidDev_RegisterReports

    @brief   Register the report table with HID Dev.

    @param   numReports - Length of report table.
    @param   pRpt - Report table.

    @return  None.
*/
void HidDev_RegisterReports( uint8 numReports, hidRptMap_t* pRpt )
{
    pHidDevRptTbl = pRpt;
    hidDevRptTblLen = numReports;
}

/*********************************************************************
    @fn      HidDev_Register

    @brief   Register a callback function with HID Dev.

    @param   pCfg - Parameter configuration.
    @param   pfnServiceCB - Callback function.

    @return  None.
*/
void HidDev_Register( hidDevCfg_t* pCfg, hidDevCB_t* pCBs )
{
    pHidDevCB = pCBs;
    pHidDevCfg = pCfg;
    pHidDevCfg=pHidDevCfg;//clear warning
}



static void HidService_Profile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
                ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
                  ( !linkDB_Up( connHandle ) ) ) )
        {
            GATTServApp_InitCharCfg( connHandle, hidReportKeyInClientCharCfg );
            GATTServApp_InitCharCfg( connHandle, hidReportCCInClientCharCfg );
            HidKbd_Serive_reset_ccd();
        }
    }
}



/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      HidKbd_AddService

    @brief   Initializes the HID Service by registering
            GATT attributes with the GATT server.

    @return  Success or Failure
*/
bStatus_t HidKbd_AddService( void )
{
    uint8 status = SUCCESS;
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportKeyInClientCharCfg );
    // GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportBootKeyInClientCharCfg );
    //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportBootMouseInClientCharCfg );
    // GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportMouseInClientCharCfg );
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportCCInClientCharCfg );
    VOID linkDB_Register( HidService_Profile_HandleConnStatusCB );
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( hidAttrTbl, GATT_NUM_ATTRS( hidAttrTbl ), &hidKbdCBs );
    // Construct map of reports to characteristic handles
    // Each report is uniquely identified via its ID and type
    // Key input report
    hidRptMap[0].id = hidReportRefKeyIn[0];
    hidRptMap[0].type = hidReportRefKeyIn[1];
    hidRptMap[0].handle = hidAttrTbl[HID_REPORT_KEY_IN_IDX].handle;
    hidRptMap[0].cccdHandle = hidAttrTbl[HID_REPORT_KEY_IN_CCCD_IDX].handle;
    hidRptMap[0].mode = HID_PROTOCOL_MODE_REPORT;
    #if EN_CONSUMER_MODE
    // Consumer Control input report
    hidRptMap[8].id = hidReportRefCCIn[0];
    hidRptMap[8].type = hidReportRefCCIn[1];
    hidRptMap[8].handle = hidAttrTbl[HID_REPORT_CC_IN_IDX].handle;
    hidRptMap[8].cccdHandle = hidAttrTbl[HID_REPORT_CC_IN_CCCD_IDX].handle;
    hidRptMap[8].mode = HID_PROTOCOL_MODE_REPORT;
    #endif
    // Setup report ID map
    HidDev_RegisterReports( HID_NUM_REPORTS, hidRptMap );
    return ( status );
}

/*********************************************************************
    @fn      HidKbd_SetParameter

    @brief   Set a HID Kbd parameter.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   uuid - attribute uuid.
    @param   len - length of data to right.
    @param   pValue - pointer to data to write.  This is dependent on
            the input parameters and WILL be cast to the appropriate
            data type (example: data type of uint16 will be cast to
            uint16 pointer).

    @return  GATT status code.
*/
uint8 HidKbd_SetParameter( uint8 id, uint8 type, uint16 uuid, uint16 len, void* pValue )
{
    bStatus_t ret = SUCCESS;

    switch ( uuid )
    {
    case REPORT_UUID:
        if ( type ==  HID_REPORT_TYPE_OUTPUT )
        {
            if ( len == 1 )
            {
                hidReportLedOut = *((uint8*)pValue);
                hidReportLedOut=hidReportLedOut;
            }
            else
            {
                ret = ATT_ERR_INVALID_VALUE_SIZE;
            }
        }
        else if ( type == HID_REPORT_TYPE_FEATURE )
        {
            if ( len == 1 )
            {
                hidReportFeature = *((uint8*)pValue);
                hidReportFeature=hidReportFeature;
            }
            else
            {
                ret = ATT_ERR_INVALID_VALUE_SIZE;
            }
        }
        else
        {
            ret = ATT_ERR_ATTR_NOT_FOUND;
        }

        break;

    case BOOT_KEY_OUTPUT_UUID:
        if ( len == 1 )
        {
            hidReportBootKeyOut = *((uint8*)pValue);
        }
        else
        {
            ret = ATT_ERR_INVALID_VALUE_SIZE;
        }

        break;

    default:
        // ignore the request
        break;
    }

    return ( ret );
}

/*********************************************************************
    @fn      HidKbd_GetParameter

    @brief   Get a HID Kbd parameter.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   uuid - attribute uuid.
    @param   pLen - length of data to be read
    @param   pValue - pointer to data to get.  This is dependent on
            the input parameters and WILL be cast to the appropriate
            data type (example: data type of uint16 will be cast to
            uint16 pointer).

    @return  GATT status code.
*/
uint8 HidKbd_GetParameter( uint8 id, uint8 type, uint16 uuid, uint16* pLen, void* pValue )
{
    switch ( uuid )
    {
    case REPORT_UUID:
        if ( type ==  HID_REPORT_TYPE_OUTPUT )
        {
            *((uint8*)pValue) = hidReportLedOut;
            *pLen = 1;
        }
        else if ( type == HID_REPORT_TYPE_FEATURE )
        {
            *((uint8*)pValue) = hidReportFeature;
            *pLen = 1;
        }
        else
        {
            *pLen = 0;
        }

        break;

    case BOOT_KEY_OUTPUT_UUID:
        *((uint8*)pValue) = hidReportBootKeyOut;
        hidReportBootKeyOut=hidReportBootKeyOut;
        *pLen = 1;
        break;

    default:
        *pLen = 0;
        break;
    }

    return ( SUCCESS );
}

static void multi_hidService_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case HCI_GAP_EVENT_EVENT:
    {
        default:
            //safeToDealloc = FALSE;  // Send to app
            break;
        }
    }
}


void multi_hidService_init( uint8 task_id )
{
    multi_hid_task_id=task_id;
    // Set up HID keyboard service
    HidKbd_AddService();
    // Register for HID Dev callback
    HidDev_Register(&hidKbdCfg, &hidKbdHidCBs);
}

/*********************************************************************
    @fn      multi_hidService_ProcessEvent

    @brief   Application task entry point for the ANCS App.

    @param   a0, a1 - not used.

    @return  None.
*/
uint16  multi_hidService_ProcessEvent( uint8 task_id, uint16 events )
{
    // Fetch any available messages that might have been sent from the stack
    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( multi_hid_task_id )) != NULL )
        {
            multi_hidService_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            //AncsApp_processStackMsg(pMsg);
            osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

//      if ( events & START_DEVICE_EVT )
//      {
//
//
//        return ( events ^ START_DEVICE_EVT );
//      }
    // Service discovery event.
    return 0;
}


/*********************************************************************
*********************************************************************/
