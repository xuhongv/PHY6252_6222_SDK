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
    Filename:       simpleGATTprofile_ota.c
    Revised:
    Revision:

    Description:    This file contains the Simple GATT profile sample GATT service
                  profile for use with the BLE sample application.


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "simpleProfile.h"

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

#define SERVAPP_NUM_ATTR_SUPPORTED        21

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

// Simple GATT Profile Service UUID: 0xFD50
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(SIMPLEPROFILE_SERVICE_UUID), HI_UINT16(SIMPLEPROFILE_SERVICE_UUID)
};

// Characteristic 1 UUID: 0xFD52
CONST uint8 simpleProfile_data_in_UUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(SIMPLEPROFILE_DATA_IN_UUID), HI_UINT16(SIMPLEPROFILE_DATA_IN_UUID)
};

// Characteristic 2 UUID: 0xFD54
CONST uint8 simpleProfile_data_out_UUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(SIMPLEPROFILE_DATA_OUT_UUID), HI_UINT16(SIMPLEPROFILE_DATA_OUT_UUID)
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

static simpleProfile_CB* simpleProfile_cb = NULL;


/*********************************************************************
    Profile Attributes - variables
*/

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE, simpleProfileServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 simpleProfile_data_in_Props = GATT_PROP_WRITE_NO_RSP;

// Characteristic 1 Value
static uint8 simpleProfile_data_in_val[20];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};

// Simple Profile Characteristic 2 Properties
static uint8 simpleProfile_data_out_Props = GATT_PROP_NOTIFY;

// Characteristic 2 Value
static uint8 simpleProfile_data_out_val[20];

static gattCharCfg_t simpleProfile_data_out_cccd[GATT_MAX_NUM_CONN];


/*********************************************************************
    Profile Attributes - Table
*/
static gattAttribute_t simpleProfileAttrTbl[7] =
{
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)& simpleProfileService               /* pValue */
    },
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &simpleProfile_data_in_Props
    },
    {
        { ATT_BT_UUID_SIZE, simpleProfile_data_in_UUID },
        GATT_PERMIT_WRITE,
        0,
        &simpleProfile_data_in_val[0]
    },
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &simpleProfile_data_out_Props
    },
    {
        { ATT_BT_UUID_SIZE, simpleProfile_data_out_UUID },
        0,
        0,
        &simpleProfile_data_out_val[0]
    },
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8*)& simpleProfile_data_out_cccd
    },
};


/*********************************************************************
    LOCAL FUNCTIONS
*/
static uint8 simpleProfile_ReadCB( uint16 connHandle, gattAttribute_t* pAttr,
                                   uint8* pValue, uint16* pLen, uint16 offset, uint8 maxLen );
static bStatus_t simpleProfile_WriteCB( uint16 connHandle, gattAttribute_t* pAttr,
                                        uint8* pValue, uint16 len, uint16 offset );

static void simpleProfile_HandleConnCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
    PROFILE CALLBACKS
*/
// Simple Profile Service Callbacks
CONST gattServiceCBs_t simpleProfileCBs =
{
    simpleProfile_ReadCB,  // Read callback function pointer
    simpleProfile_WriteCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      SimpleProfile_AddService

    @brief   Initializes the Simple Profile service by registering
            GATT attributes with the GATT server.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.

    @return  Success or Failure
*/
bStatus_t SimpleProfile_InitService( simpleProfile_CB* appCallbacks )
{
    uint8 status = SUCCESS;
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfile_data_out_cccd );
    // Register with Link DB to receive link status change callback
    VOID linkDB_Register( simpleProfile_HandleConnCB );
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simpleProfileAttrTbl,
                                          GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                          &simpleProfileCBs );

    if (status != SUCCESS)
        LOG("Add Simple Profile service failed!\n");

    simpleProfile_cb = appCallbacks;
    return ( status );
}

/*********************************************************************
    @fn          simpleProfile_ReadAttrCB

    @brief       Read an attribute.

    @param       connHandle - connection message was received on
    @param       pAttr - pointer to attribute
    @param       pValue - pointer to data to be read
    @param       pLen - length of data to be read
    @param       offset - offset of the first octet to be read
    @param       maxLen - maximum length of data to be read

    @return      Success or Failure
*/
static uint8 simpleProfile_ReadCB( uint16 connHandle, gattAttribute_t* pAttr,
                                   uint8* pValue, uint16* pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t status = SUCCESS;

    // If attribute permissions require authorization to read, return error
    if ( gattPermitAuthorRead( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if ( offset > 0 )
    {
        return ( ATT_ERR_ATTR_NOT_LONG );
    }

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

        switch ( uuid )
        {
        // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
        // gattserverapp handles those reads
        case SIMPLEPROFILE_DATA_IN_UUID:
            *pLen = 20;
            VOID osal_memcpy( pValue, pAttr->pValue, 20 );
            break;

        case SIMPLEPROFILE_DATA_OUT_UUID:
            //case SIMPLEPROFILE_CHAR4_UUID:
            *pLen = 20;
            VOID osal_memcpy( pValue, pAttr->pValue, *pLen );
            break;

        default:
            // Should never get here! (characteristics 3 and 4 do not have read permissions)
            *pLen = 0;
            status = ATT_ERR_ATTR_NOT_FOUND;
            break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    return ( status );
}

/*********************************************************************
    @fn      simpleProfile_WriteAttrCB

    @brief   Validate attribute data prior to a write operation

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/
// TODO: test this function

static bStatus_t simpleProfile_WriteCB( uint16 connHandle, gattAttribute_t* pAttr,
                                        uint8* pValue, uint16 len, uint16 offset )
{
    bStatus_t status = SUCCESS;

    // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

        switch ( uuid )
        {
        case SIMPLEPROFILE_DATA_IN_UUID:

            //Validate the value
            // Make sure it's not a blob oper
            if ( offset != 0 )
            {
                status = ATT_ERR_ATTR_NOT_LONG;
            }

            if (SUCCESS == status)
            {
                LOG("private data in\n");

                if (NULL != simpleProfile_cb)
                {
                    simpleProfile_cb->simpleProfile_data_in_cb
                    (
                        connHandle,
                        offset,
                        len,
                        pValue
                    );
                }
            }

            break;

        case GATT_CLIENT_CHAR_CFG_UUID:
            status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                     offset, GATT_CLIENT_CFG_NOTIFY );

            if ( status == SUCCESS )
            {
                uint16 t_cccd_val = BUILD_UINT16( pValue[0], pValue[1] );
                t_cccd_val = (GATT_CLIENT_CFG_NOTIFY == t_cccd_val) ?\
                             TRUE : FALSE;

                if (NULL != simpleProfile_cb)
                {
                    simpleProfile_cb->simpleProfile_data_out_ccd_cb
                    (
                        connHandle,
                        t_cccd_val
                    );
                }
            }

            break;

        default:
            // Should never get here! (characteristics 2 and 4 do not have write permissions)
            status = ATT_ERR_ATTR_NOT_FOUND;
            break;
        }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    return ( status );
}

/*********************************************************************
    @fn          simpleProfile_HandleConnStatusCB

    @brief       Simple Profile link status change handler function.

    @param       connHandle - connection handle
    @param       changeType - type of change

    @return      none
*/
static void simpleProfile_HandleConnCB( uint16 connHandle, uint8 changeType )
{
    // Make sure this is not loopback connection
    if ( connHandle != LOOPBACK_CONNHANDLE )
    {
        // Reset Client Char Config if connection has dropped
        if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
                ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) &&
                  ( !linkDB_Up( connHandle ) ) ) )
        {
            GATTServApp_InitCharCfg( connHandle, simpleProfile_data_out_cccd );
        }
    }
}


/*********************************************************************
*********************************************************************/
