
/**
    \file blebrr_pl.c


*/

/*
    Copyright (C) 2018. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
/* Platform Stack Headers */
#include <hci.h>
#include <gap.h>
#include <gatt.h>
#undef BLE_CLIENT_ROLE

/* Povisioning API headers */
#include "MS_prov_api.h"

/* BLE Bearer related Headers */
#include "blebrr.h"

#include "simpleProfile.h"

#include "appl_main.h"
#include "model_state_handler_pl.h"

#include "MS_net_api.h"
#include "cliface.h"
#include "access_extern.h"


extern uint8             llState, llSecondaryState;
extern MS_ACCESS_MODEL_HANDLE   UI_generic_onoff_server_model_handle;
extern /*static*/ MS_STATE_GENERIC_ONOFF_STRUCT UI_generic_onoff;

/* Platform log to be mapped */
#define BLEBRRPL_LOG            printf
#define BLEBRRPL_dump_bytes     appl_dump_bytes

/* Active connection handle used to send measurements */
uint16_t private_conn_hndl = 0xFFFF;

static uint16_t appl_simpleProfile_data_out_ccd_cb(uint16_t conn_hndl, uint8_t enabled);
static uint16_t appl_simpleProfile_data_in_wt_cb
(
    uint16_t conn_hndl,
    uint16_t offset,
    uint16_t length,
    uint8_t*   value
);
static simpleProfile_CB simpleProfile_cb =
{
    .simpleProfile_data_in_cb      = appl_simpleProfile_data_in_wt_cb,
    .simpleProfile_data_out_ccd_cb = appl_simpleProfile_data_out_ccd_cb,
};

void appl_simpleProfile_add_service(void)
{
    SimpleProfile_InitService(&simpleProfile_cb);
}

static uint16_t appl_simpleProfile_data_out_ccd_cb(uint16_t conn_hndl, uint8_t enabled)
{
    BLEBRRPL_LOG("appl_simpleProfile_data_out handle %4x\n", conn_hndl);

    if (TRUE == enabled)
    {
        private_conn_hndl = conn_hndl;
        BLEBRRPL_LOG("Simple Profile Out CCD Enabled");
        MS_private_server_adv_stop();
        blebrr_scan_enable();
    }
    else
    {
        BLEBRRPL_LOG("Simple Profile Out CCD Disabled");
    }

//    blebrr_gatt_com_channel_setup_pl
//    (
//        BLEBRR_SERVER_ROLE,
//        BLEBRR_GATT_PROXY_MODE,
//        (enabled) ? BLEBRR_COM_CHANNEL_CONNECT : BLEBRR_COM_CHANNEL_DISCONNECT
//    );
    return 0x0000;
}
static uint16_t appl_simpleProfile_data_in_wt_cb
(
    uint16_t conn_hndl,
    uint16_t offset,
    uint16_t length,
    uint8_t*   value
)
{
    uint8_t cmd[256];
    API_RESULT retval;
    osal_memcpy(&cmd, value, length);

    if (NULL != value)
    {
        BLEBRRPL_LOG("Simple Profile Data IN received");

        if(length > 1)
        {
            UINT16  dst_addr;
            UINT16 data_len;
            //      UINT32 opcode;
            UINT16 marker = 0;
            UINT8 data[64];
            appl_dump_bytes(value, length);
            MS_PACK_BE_2_BYTE(&dst_addr, &cmd[marker]);
            marker += 2;
            MS_PACK_BE_2_BYTE(&data_len, &cmd[marker]);
            BLEBRRPL_LOG("dst_addr %04x,data_len %04x\n",dst_addr,data_len);
            marker += 2;
            MS_PACK_BE_N_BYTE(data,&cmd[marker],data_len);
            //cmd parse param (node to node)
            retval = MS_access_raw_data(&UI_generic_onoff_server_model_handle,0x8202,dst_addr,0x00,data,data_len,0);

            if(retval != API_SUCCESS)
                BLEBRRPL_LOG("retval %04x\n",retval);
        }
        else
        {
            //cmd on/off
            generic_onoff_set_pl(cmd[0]);
            UI_generic_onoff.onoff = cmd[0];
        }

//        blebrr_recv_mesh_packet_pl
//        (
//            &conn_hndl,
//            offset,
//            value,
//            length
//        );
    }

    return 0x0000;
}

