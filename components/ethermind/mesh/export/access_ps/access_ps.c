
/**
    \file access_ps.c

    Persistent Storage related implementation for Access Layer.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "access_internal.h"
#include "access_extern.h"
#include "net_extern.h"
#include "MS_net_api.h"
#include "flash.h"


#ifdef MS_STORAGE
#include "nvsto.h"

extern uint16_t crc16(uint16_t seed, const volatile void* p_data, uint32_t size);


/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/** Data structure related to persistent storage (PS) load/store */
typedef struct _MS_PS_RECORDS
{
    /** Information to load/store */
    void* info;

    /** Start Offset in PS */
    UINT32 start_offset;

    /** Size of Information (in bytes) */
    UINT32 size;

    /** Function Pointer to Store */
    void (* store) (void);

    /** Function Pointer to Load */
    void (* load) (void);

} MS_PS_RECORDS;

/* Store Elements */
static void ms_store_elements(void);
/* Store Models */
static void ms_store_models(void);
/* Store Subnets */
static void ms_store_subnets(void);
/* Store Device Key */
static void ms_store_dev_key(void);
/* Store AppKeys */
static void ms_store_appkeys(void);
/* Store Element Addresses */
static void ms_store_element_addresses(void);
/* Store Virtual Addresses */
static void ms_store_virtual_addresses(void);
/* Store Non-virtual Addresses */
static void ms_store_non_virtual_addresses(void);
/* Store Network Sequence Number */
static void ms_store_seq_number(void);
/* Store Tx States */
static void ms_store_tx_states_and_features(void);
/* Store Provisioner Address */
static void ms_store_provisioner_address(void);

static void ms_store_proxy_filter_address(void);


/* Load Elements */
static void ms_load_elements(void);
/* Load Models */
static void ms_load_models(void);
/* Load Subnets */
static void ms_load_subnets(void);
/* Load Device Key */
static void ms_load_dev_key(void);
/* Load AppKeys */
static void ms_load_appkeys(void);
/* Load Element Addresses */
static void ms_load_element_addresses(void);
/* Load Virtual Addresses */
static void ms_load_virtual_addresses(void);
/* Load Non-virtual Addresses */
static void ms_load_non_virtual_addresses(void);
/* Load Network Sequence Number */
static void ms_load_seq_number(void);
/* Load Tx States */
static void ms_load_tx_states_and_features(void);
/* Load Provisioner Address */
static void ms_load_provisioner_address(void);

static void ms_load_proxy_filter_address(void);



/**
    List of persistent store records.
*/
static MS_PS_RECORDS ms_ps_records[] =
{
    /** Information to load/store, Start Offset, Size, Store, Load */
    {NULL, 0 /* MS_PS_RECORD_ELEMENTS_OFFSET */,          0 /* MS_PS_RECORD_ELEMENTS_SIZE */,          ms_store_elements,               ms_load_elements},
    {NULL, 0 /* MS_PS_RECORD_MODELS_OFFSET */,            0 /* MS_PS_RECORD_MODELS_SIZE */,            ms_store_models,                 ms_load_models},
    {NULL, 0 /* MS_PS_RECORD_SUBNETS_OFFSET */,           0 /* MS_PS_RECORD_SUBNETS_SIZE */,           ms_store_subnets,                ms_load_subnets},
    {NULL, 0 /* MS_PS_RECORD_DEV_KEYS_OFFSET */,          0 /* MS_PS_RECORD_DEV_KEYS_SIZE */,          ms_store_dev_key,                ms_load_dev_key},
    {NULL, 0 /* MS_PS_RECORD_APP_KEYS_OFFSET */,          0 /* MS_PS_RECORD_APP_KEYS_SIZE */,          ms_store_appkeys,                ms_load_appkeys},
    {NULL, 0 /* MS_PS_RECORD_ELEMENT_ADDRS_OFFSET */,     0 /* MS_PS_RECORD_ELEMENT_ADDRS_SIZE */,     ms_store_element_addresses,      ms_load_element_addresses},
    {NULL, 0 /* MS_PS_RECORD_VIRTUAL_ADDRS_OFFSET */,     0 /* MS_PS_RECORD_VIRTUAL_ADDRS_SIZE */,     ms_store_virtual_addresses,      ms_load_virtual_addresses},
    {NULL, 0 /* MS_PS_RECORD_NON_VIRTUAL_ADDRS_OFFSET */, 0 /* MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE */, ms_store_non_virtual_addresses,  ms_load_non_virtual_addresses},
    {NULL, 0 /* MS_PS_RECORD_SEQ_NUMBER_OFFSET */,        0 /* MS_PS_RECORD_SEQ_NUMBER_SIZE */,        ms_store_seq_number,             ms_load_seq_number},
    {NULL, 0 /* MS_PS_RECORD_TX_STATES_OFFSET */,         0 /* MS_PS_RECORD_TX_STATES_SIZE */,         ms_store_tx_states_and_features, ms_load_tx_states_and_features},
    {NULL, 0 /* MS_PS_RECORD_TX_STATES_OFFSET */,         0 /* MS_PS_RECORD_TX_STATES_SIZE */,         ms_store_provisioner_address,    ms_load_provisioner_address},
    {NULL, 0 /* MS_PS_RECORD_PROXY_FILTER_OFFSET */,      0 /* MS_PS_RECORD_PROXY_FILTER_SIZE */,      ms_store_proxy_filter_address,   ms_load_proxy_filter_address}
};

/* Nonvolatile storage handle */
static NVSTO_HANDLE access_nvsto_handle;
static UINT8   ms_ps_store_disable_flag = 0;


UINT8   ms_ps_count;
UINT16  ms_crc16_code;




/* --------------------------------------------- Functions */
/* Initialize the access ps store */
void access_ps_init (void)
{
    /** Information to load/store, Start Offset, Size, Store, Load */
    ms_ps_records[0].start_offset = MS_PS_RECORD_ELEMENTS_OFFSET;
    ms_ps_records[0].size = MS_PS_RECORD_ELEMENTS_SIZE;
    ms_ps_records[1].start_offset = MS_PS_RECORD_MODELS_OFFSET;
    ms_ps_records[1].size = MS_PS_RECORD_MODELS_SIZE;
    ms_ps_records[2].start_offset = MS_PS_RECORD_SUBNETS_OFFSET;
    ms_ps_records[2].size = MS_PS_RECORD_SUBNETS_SIZE;
    ms_ps_records[3].start_offset = MS_PS_RECORD_DEV_KEYS_OFFSET;
    ms_ps_records[3].size = MS_PS_RECORD_DEV_KEYS_SIZE;
    ms_ps_records[4].start_offset = MS_PS_RECORD_APP_KEYS_OFFSET;
    ms_ps_records[4].size = MS_PS_RECORD_APP_KEYS_SIZE;
    ms_ps_records[5].start_offset = MS_PS_RECORD_ELEMENT_ADDRS_OFFSET;
    ms_ps_records[5].size = MS_PS_RECORD_ELEMENT_ADDRS_SIZE;
    ms_ps_records[6].start_offset = MS_PS_RECORD_VIRTUAL_ADDRS_OFFSET;
    ms_ps_records[6].size = MS_PS_RECORD_VIRTUAL_ADDRS_SIZE;
    ms_ps_records[7].start_offset = MS_PS_RECORD_NON_VIRTUAL_ADDRS_OFFSET;
    ms_ps_records[7].size = MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE;
    ms_ps_records[8].start_offset = MS_PS_RECORD_SEQ_NUMBER_OFFSET;
    ms_ps_records[8].size = MS_PS_RECORD_SEQ_NUMBER_SIZE;
    ms_ps_records[9].start_offset = MS_PS_RECORD_TX_STATES_FEATURES_OFFSET;
    ms_ps_records[9].size = MS_PS_RECORD_TX_STATES_FEATURES_SIZE;
    ms_ps_records[10].start_offset = MS_PS_RECORD_PROVISIONER_ADDR_OFFSET;
    ms_ps_records[10].size = MS_PS_RECORD_PROVISIONER_ADDR_SIZE;
    ms_ps_records[11].start_offset = MS_PS_RECORD_PROXY_FILTER_ADDR_OFFSET;
    ms_ps_records[11].size = MS_PS_RECORD_PROXY_FILTER_ADDR_SIZE;
    nvsto_register_ps
    (
        (MS_PS_RECORD_PROXY_FILTER_ADDR_OFFSET + MS_PS_RECORD_PROXY_FILTER_ADDR_SIZE),
        &access_nvsto_handle
    );
//    printf("[nvsto_register_ps]MS_PS_RECORD_CORE_MODULES_OFFSET = 0x%08X\n",MS_PS_RECORD_CORE_MODULES_OFFSET);
}

/* Store Elements */
static void ms_store_elements(void)
{
    UINT16 element_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_ELEMENTS_SIZE];
    /* Init Element list */
    memset(element_buf,0,MS_PS_RECORD_ELEMENTS_SIZE);
    offset = 0;
    /* Copy Element Count */
    element_count = MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT);
    size = sizeof(element_count);
    memcpy(&element_buf[offset], (void*)&element_count, size);
    offset += size;
    /* Copy Element List */
    size = sizeof(MS_ACCESS_ELEMENT_TYPE) * MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT);
    memcpy(&element_buf[offset], (void*)&ms_access_element_list[0], size);
    offset += size;
    /* Copy Composition Header */
    size = sizeof(composition_data_hdr);
    memcpy(&element_buf[offset], (void*)&composition_data_hdr, size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_ELEMENTS_SIZE);
    /* Write Element to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_ELEMENTS_SIZE
    );
}

/* Store Models */
static void ms_store_models(void)
{
    UINT16 model_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_MODELS_SIZE];
    /* Init Model list */
    memset(element_buf,0,MS_PS_RECORD_MODELS_SIZE);
    offset = 0;
    /* Copy Model Count */
    model_count = MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT);
    size = sizeof(model_count);
    memcpy(&element_buf[offset], (void*)&model_count, size);
    offset += size;
    /* Copy Model List */
    size = (MS_ACCESS_MODEL_TYPE_SIZE) * MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT);
    memcpy(&element_buf[offset], (void*)&ms_access_model_list[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_MODELS_SIZE);
    /* Write Models to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_MODELS_SIZE
    );
}

/* Store Subnets */
static void ms_store_subnets(void)
{
    UINT16 subnet_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_SUBNETS_SIZE];
    /* Init Subnets list */
    memset(element_buf,0,MS_PS_RECORD_ELEMENTS_SIZE);
    offset = 0;
    /* Copy Subnet Count */
    subnet_count = (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS));
    size = sizeof(subnet_count);
    memcpy(&element_buf[offset], (void*)&subnet_count, size);
    offset += size;
    /* Copy IV Index and Update Flag */
    size = sizeof(ms_iv_index);
    memcpy(&element_buf[offset], (void*)&ms_iv_index, size);
    offset += size;
    /* Copy NetKey Count */
    size = sizeof(ms_netkey_count);
    memcpy(&element_buf[offset], (void*)&ms_netkey_count, size);
    offset += size;
    /* Copy Subnet Table */
    size = MS_NETKEY_ENTRY_SIZE * (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS));
    memcpy(&element_buf[offset], (void*)&ms_subnet_table[0], size);
    offset += size;
    /* Copy Key refresh Table */
    size = sizeof(MS_NET_ADDR) * (MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS));
    memcpy(&element_buf[offset], (void*)&ms_key_refresh_whitelist[0], size);
    offset += size;
    /* Copy Key refresh Count */
    size = sizeof(ms_key_refresh_count);
    memcpy(&element_buf[offset], (void*)&ms_key_refresh_count, size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_SUBNETS_SIZE);
    /* Write Subnets to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_SUBNETS_SIZE
    );
}

/* Store Device Key */
static void ms_store_dev_key(void)
{
    UINT16 dev_key_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_DEV_KEYS_SIZE];
    /* Init Dev Key list */
    memset(element_buf,0,MS_PS_RECORD_DEV_KEYS_SIZE);
    offset = 0;
    /* Copy Devkey Count */
    dev_key_count = MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS);
    size = sizeof(dev_key_count);
    memcpy(&element_buf[offset], (void*)&dev_key_count, size);
    offset += size;
    /* Copy Devkey Entry Count */
    size = sizeof(ms_dev_key_table_entries);
    memcpy(&element_buf[offset], (void*)&ms_dev_key_table_entries, size);
    offset += size;
    /* Copy Devkey Pointer */
    size = sizeof(ms_dev_key_table_pointer);
    memcpy(&element_buf[offset], (void*)&ms_dev_key_table_pointer, size);
    offset += size;
    /* Copy Devkey Table */
    size = (sizeof(MS_DEV_KEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS));
    memcpy(&element_buf[offset], (void*)&ms_dev_key_table[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_DEV_KEYS_SIZE);
    /* Write Dev Key to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_DEV_KEYS_SIZE
    );
}

/* Store AppKeys */
static void ms_store_appkeys(void)
{
    UINT16 appkey_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_APP_KEYS_SIZE];
    /* Init AppKeys list */
    memset(element_buf,0,MS_PS_RECORD_APP_KEYS_SIZE);
    offset = 0;
    /* Copy Appkey Count */
    appkey_count = MS_CONFIG_LIMITS(MS_MAX_APPS);
    size = sizeof(appkey_count);
    memcpy(&element_buf[offset], (void*)&appkey_count, size);
    offset += size;
    /* Copy Appkey Table */
    size = (sizeof(MS_APPKEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_APPS));
    memcpy(&element_buf[offset], (void*)&ms_appkey_table[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_APP_KEYS_SIZE);
    /* Write AppKeys to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_APP_KEYS_SIZE
    );
}

/* Store Element Addresses */
static void ms_store_element_addresses(void)
{
    UINT16 element_addr_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_ELEMENT_ADDRS_SIZE];
    /* Init Element Addresses list */
    memset(element_buf,0,MS_PS_RECORD_ELEMENT_ADDRS_SIZE);
    offset = 0;
    /* Copy Element Address Count */
    element_addr_count = (1 + MS_CONFIG_LIMITS(MS_MAX_LPNS));
    size = sizeof(element_addr_count);
    memcpy(&element_buf[offset], (void*)&element_addr_count, size);
    offset += size;
    /* Copy Element Address Table */
    size = (sizeof(MS_ELEMENT_ADDR_ENTRY) * (1 + MS_CONFIG_LIMITS(MS_MAX_LPNS)));
    memcpy(&element_buf[offset], (void*)&ms_element_addr_table[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_ELEMENT_ADDRS_SIZE);
    /* Write Element Addresses to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_ELEMENT_ADDRS_SIZE
    );
}

/* Store Virtual Addresses */
static void ms_store_virtual_addresses(void)
{
    UINT16 virtual_addr_count;
    /* Init Virtual Addresses list */
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_VIRTUAL_ADDRS_SIZE];
    memset(element_buf,0,MS_PS_RECORD_VIRTUAL_ADDRS_SIZE);
    offset = 0;
    /* Copy Virtual Address Count */
    virtual_addr_count = MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS);
    size = sizeof(virtual_addr_count);
    memcpy(&element_buf[offset], (void*)&virtual_addr_count, size);
    offset += size;
    /* Copy Virtual Address Table */
    size = (sizeof(MS_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS));
    memcpy(&element_buf[offset], (void*)&ms_virtual_addr_table[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_VIRTUAL_ADDRS_SIZE);
    /* Write Virtual Addresses to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_VIRTUAL_ADDRS_SIZE
    );
}

/* Store Non-virtual Addresses */
static void ms_store_non_virtual_addresses(void)
{
    UINT16 non_virtual_addr_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE];
    /* Init Non-Virtual Addresses list */
    memset(element_buf,0,MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE);
    offset = 0;
    /* Copy Non-Virtual Address Count */
    non_virtual_addr_count = MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS);
    size = sizeof(non_virtual_addr_count);
    memcpy(&element_buf[offset], (void*)&non_virtual_addr_count, size);
    offset += size;
    /* Copy Non-Virtual Address Table */
    size = (sizeof(MS_NON_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));
    memcpy(&element_buf[offset], (void*)&ms_non_virtual_addr_table[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE);
    /* Write Non-Virtual Addresses to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_NON_VIRTUAL_ADDRS_SIZE
    );
}

/* Store Network Sequence Number */
static void ms_store_seq_number(void)
{
    NET_SEQ_NUMBER_STATE seq_num_state;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_SEQ_NUMBER_SIZE];
    MS_net_get_seq_num_state(&seq_num_state);
    /* Init Sequence Number list */
    memset(element_buf,0,MS_PS_RECORD_SEQ_NUMBER_SIZE);
    offset = 0;
    /* Copy block end of Network Sequence Number */
    size = sizeof(seq_num_state.block_seq_num_max);
    memcpy(&element_buf[offset], (void*)&seq_num_state.block_seq_num_max, size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_SEQ_NUMBER_SIZE);
    /* Write Sequence Number to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_SEQ_NUMBER_SIZE
    );
}

/* Store Tx States */
static void ms_store_tx_states_and_features(void)
{
    UINT16 tx_state_count;
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_TX_STATES_FEATURES_SIZE];
    /* Init Tx States&features list */
    memset(element_buf,0,MS_PS_RECORD_TX_STATES_FEATURES_SIZE);
    offset = 0;
    /* Copy Tx State Count */
    tx_state_count = 2;
    size = sizeof(tx_state_count);
    memcpy(&element_buf[offset], (void*)&tx_state_count, size);
    offset += size;
    /* Copy Tx State */
    size = sizeof(ms_tx_state);
    memcpy(&element_buf[offset], (void*)&ms_tx_state, size);
    offset += size;
    /* Copy MS Features */
    size = sizeof(ms_features);
    memcpy(&element_buf[offset], (void*)&ms_features, size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_TX_STATES_FEATURES_SIZE);
    /* Write Tx States&features to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_TX_STATES_FEATURES_SIZE
    );
}


/* Store Provisioner Address */
static void ms_store_provisioner_address(void)
{
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_PROVISIONER_ADDR_SIZE];
    /* Init Tx States&features list */
    memset(element_buf,0,MS_PS_RECORD_PROVISIONER_ADDR_SIZE);
    offset = 0;
    /* Copy Tx State */
    size = sizeof(ms_provisioner_addr);
    memcpy(&element_buf[offset], (void*)&ms_provisioner_addr, size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_PROVISIONER_ADDR_SIZE);
    /* Write Tx States&features to flash */
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_PROVISIONER_ADDR_SIZE
    );
}

static void ms_store_proxy_filter_address(void)
{
//  printf
//    ("[ACCESS]: ms_store_proxy_filter_address \n");
//  for (INT16 i = 0; i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i++)
//  {
//      printf (
//               "[PROXY]: Proxy Filter addr list 0x%04x\n", net_proxy_list[0].p_addr[i]);
//
//  }
    UINT16   offset,size;
    UCHAR   element_buf[MS_PS_RECORD_PROXY_FILTER_ADDR_SIZE];
    /* Init Tx States&features list */
    memset(element_buf,0,MS_PS_RECORD_PROXY_FILTER_ADDR_SIZE);
    offset = 0;
    /* Copy Tx State */
    size = sizeof(MS_NET_ADDR)*MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE);
    memcpy(&element_buf[offset], (void*)net_proxy_list[0].p_addr, size);
//    memcpy(&element_buf[offset], (void*)&net_proxy_list[0].p_addr[0], size);
    offset += size;
    //Calc crc code
    ms_crc16_code = crc16(ms_crc16_code, &element_buf[0], MS_PS_RECORD_PROXY_FILTER_ADDR_SIZE);
    /* Write Tx States&features to flash */
//  for (UINT16 i = 0; i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i++){
//      printf (
//       "[PROXY]:Proxy Filter addr list 0x%04x,0x%04x\n", ms_proxy_filter_addr[i],element_buf[0]);
//
//  }
    nvsto_write_ps
    (
        access_nvsto_handle,
        &element_buf[0],
        MS_PS_RECORD_PROXY_FILTER_ADDR_SIZE
    );
}

/**
    \brief Store Access Layer information in persistent storage.

    \par Description
    This function saves in RAM data structures/configurations in persistent storage.
    This function will be called whenever there is change in any of the data
    structures/configurations which need to be stored.

    \param [in] records    This is a bitmask identifying the records to be stored.
*/
void ms_access_ps_store (/* IN */ UINT32 records)
{
    UINT32 index;
    INT16 ret,count;
    ACCESS_TRC
    ("[ACCESS]: PS Store. Records 0x%08X\n", records);

    if(ms_ps_store_disable_flag == MS_FALSE)
    {
        /* Open the storage */
        ret = nvsto_open_pswrite (access_nvsto_handle);

        if (0 > ret)
        {
            ACCESS_ERR
            ("[ACCESS]: PS Open Failed\n");
            return;
        }

        /*Erase flash*/
        nvsto_erase_ps(access_nvsto_handle);
        nvsto_write_header_ps(access_nvsto_handle,0xffffff03); //writing
        ms_crc16_code = 0;

        /** Store information in listed order */
        for (index = 0; index < MS_PS_ACCESS_MAX_RECORDS; index++)
        {
            /* Seek to the corresponding location in PS */
            nvsto_seek_ps(access_nvsto_handle, ms_ps_records[index].start_offset);
            /* Only using the load/store function pointers */
            ms_ps_records[index].store();

            if(index == (MS_PS_ACCESS_MAX_RECORDS-1))
            {
                blebrr_scan_pl(FALSE);
            }
        }

        count = ++ms_ps_count;
        nvsto_write_header_ps(access_nvsto_handle, (((UINT32)ms_crc16_code)<<16)|((MS_PS_ACCESS_MAX_RECORDS-12)<<12)|((count&0xf)<<8)|0x1); //writing ok
        /* Close the storage */
        nvsto_close_ps (access_nvsto_handle);
    }

    return;
}

/* Load Elements */
static void ms_load_elements(void)
{
    UINT16 element_count;
    UINT32 index;
    /* Read Element Count */
    element_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &element_count,
        sizeof(element_count)
    );

    /* Check Element Count */
    if (MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT) != element_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Element Count 0x%04X. Returning\n", element_count);
        return;
    }

    /* Read Element List */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_access_element_list[0],
        sizeof(MS_ACCESS_ELEMENT_TYPE) * MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT)
    );
    /* Read Composition Header */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &composition_data_hdr,
        sizeof(composition_data_hdr)
    );

    /* TODO: Find a better solution to register elements/modules */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT); index++)
    {
        /* Mark Elements Invalid, so that these can reused */
        ms_access_element_list[index].valid = 0x00;
        ms_access_element_list[index].num_s = 0x00;
        ms_access_element_list[index].num_v = 0x00;
    }
}

/* Load Models */
static void ms_load_models(void)
{
    UINT16 model_count;
    UINT32 index;
    /* Read Model Count */
    model_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &model_count,
        sizeof(model_count)
    );

    /* Check Model Count */
    if (MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT) != model_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Model Count 0x%04X. Returning\n", model_count);
        return;
    }

    /* Read Model List */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_access_model_list[0],
        (MS_ACCESS_MODEL_TYPE_SIZE) * MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT)
    );

    /* TODO: Find a better solution to register elements/modules */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); index++)
    {
        /* Mark Models Invalid, so that these can reused */
        ms_access_model_list[index].fixed.valid = 0x00;
    }
}

/* Load Subnets */
static void ms_load_subnets(void)
{
    UINT16 subnet_count;
    /* Read Subnet Count */
    subnet_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &subnet_count,
        sizeof(subnet_count)
    );

    /* Check Subnet Count */
    if ((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) != subnet_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Subnet Count 0x%04X. Returning\n", subnet_count);
        return;
    }

    /* Read IV Index and Update Flag */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_iv_index,
        sizeof(ms_iv_index)
    );
    /* Read NetKey Count */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_netkey_count,
        sizeof(ms_netkey_count)
    );
    /* Read Subnet Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_subnet_table[0],
        (MS_NETKEY_ENTRY_SIZE * (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)))
    );
    /* Read Key refresh Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_key_refresh_whitelist[0],
        (sizeof(MS_NET_ADDR) * (MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS)))
    );
    /* Read Key refresh Count */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_key_refresh_count,
        sizeof(ms_key_refresh_count)
    );
}

/* Load Device Key */
static void ms_load_dev_key(void)
{
    UINT16 dev_key_count;
    /* Read Devkey Count */
    dev_key_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &dev_key_count,
        sizeof(dev_key_count)
    );

    /* Check Devkey Count */
    if (MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS) != dev_key_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Devkey Count 0x%04X. Returning\n", dev_key_count);
        return;
    }

    /* Read Devkey Entry Count */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_dev_key_table_entries,
        sizeof(ms_dev_key_table_entries)
    );
    /* Read Devkey Pointer */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_dev_key_table_pointer,
        sizeof(ms_dev_key_table_pointer)
    );
    /* Read Devkey Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_dev_key_table[0],
        (sizeof(MS_DEV_KEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS))
    );
}

/* Load AppKeys */
static void ms_load_appkeys(void)
{
    UINT16 appkey_count;
    /* Read Appkey Count */
    appkey_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &appkey_count,
        sizeof(appkey_count)
    );

    /* Check Appkey Count */
    if (MS_CONFIG_LIMITS(MS_MAX_APPS) != appkey_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Appkey Count 0x%04X. Returning\n", appkey_count);
        return;
    }

    /* Read Appkey Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_appkey_table[0],
        (sizeof(MS_APPKEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_APPS))
    );
}

/* Load Element Addresses */
static void ms_load_element_addresses(void)
{
    UINT16 element_addr_count;
    /* Read Element Address Count */
    element_addr_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &element_addr_count,
        sizeof(element_addr_count)
    );

    if ((1 + MS_CONFIG_LIMITS(MS_MAX_LPNS)) != element_addr_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Element Address Count 0x%04X. Returning\n", element_addr_count);
        return;
    }

    /* Read Element Address Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_element_addr_table[0],
        (sizeof(MS_ELEMENT_ADDR_ENTRY) * (1 + MS_CONFIG_LIMITS(MS_MAX_LPNS)))
    );
    /* TODO: Remove when PS implementation is complete */
    ms_element_addr_table[0].element_count = 0;
}

/* Load Virtual Addresses */
static void ms_load_virtual_addresses(void)
{
    UINT16 virtual_addr_count;
    /* Read Virtual Address Count */
    virtual_addr_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &virtual_addr_count,
        sizeof(virtual_addr_count)
    );

    /* Check Virtual Address Count */
    if (MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS) != virtual_addr_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Virtual Element Address Count 0x%04X. Returning\n", virtual_addr_count);
        return;
    }

    /* Read Virtual Address Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_virtual_addr_table[0],
        (sizeof(MS_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS))
    );
}

/* Load Non-virtual Addresses */
static void ms_load_non_virtual_addresses(void)
{
    UINT16 non_virtual_addr_count;
    /* Read Non-Virtual Address Count */
    non_virtual_addr_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &non_virtual_addr_count,
        sizeof(non_virtual_addr_count)
    );

    /* Check Non-Virtual Address Count */
    if (MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS) != non_virtual_addr_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Non-Virtual Element Address Count 0x%04X. Returning\n", non_virtual_addr_count);
        return;
    }

    /* Read Virtual Address Table */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_non_virtual_addr_table[0],
        (sizeof(MS_NON_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS))
    );
}

/* Load Network Sequence Number */
static void ms_load_seq_number(void)
{
    NET_SEQ_NUMBER_STATE seq_number_state;
    INT16 ret;
    /* Read block end of Network Sequence Number */
    ret=nvsto_read_ps
        (
            access_nvsto_handle,
            &seq_number_state.block_seq_num_max,
            sizeof(seq_number_state.block_seq_num_max)
        );

    if(ret!=-1)
    {
        /* Set current Network Sequence Number as the saved block end */
        if(seq_number_state.block_seq_num_max == 0xffffffff) //0xffffffff should set to 0 by hq
            seq_number_state.block_seq_num_max = 0;

        seq_number_state.seq_num = seq_number_state.block_seq_num_max;
        MS_net_set_seq_num_state(&seq_number_state);
    }
}

/* Load Tx States */
static void ms_load_tx_states_and_features(void)
{
    UINT16 tx_state_count;
    /* Read Tx State Count */
    tx_state_count = 0;
    nvsto_read_ps
    (
        access_nvsto_handle,
        &tx_state_count,
        sizeof(tx_state_count)
    );

    /* Check Tx State Count */
    if (2 != tx_state_count)
    {
        ACCESS_ERR(
            "[PS] Invalid Tx State Count 0x%04X. Returning\n", tx_state_count);
        return;
    }

    /* Read Tx State */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_tx_state,
        sizeof(ms_tx_state)
    );
    /* Read MS Features */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_features,
        sizeof(ms_features)
    );
}

/* Load Provisioner Address */
static void ms_load_provisioner_address(void)
{
    /* Read Provisioner Address */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &ms_provisioner_addr,
        sizeof(ms_provisioner_addr)
    );
}

static void ms_load_proxy_filter_address(void)
{
    /* Read Provisioner Address */
    nvsto_read_ps
    (
        access_nvsto_handle,
        &net_proxy_list[0].p_addr[0],
        (sizeof(MS_NET_ADDR)*MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE))
    );
}

/**
    \brief Load Access Layer information from persistent storage.

    \par Description
    This function loads data structures/configurations which are already
    saved in persistent storage to in RAM data structures.
    This function will generally be called only once during the power of
    initialization.

    \param [in] records    This is a bitmask identifying the records to be loaded.
*/
void ms_access_ps_load (/* IN */ UINT32 records)
{
    UINT32 index;
    INT16 ret;
    ACCESS_TRC
    ("[ACCESS]: PS Load. Records 0x%08X\n", records);
//  printf
//    ("[ACCESS]: PS Load. Records 0x%08X\n", records);
    /* Open the storage */
    ret = nvsto_open_psread (access_nvsto_handle);

    if (0 > ret)
    {
        ACCESS_ERR
        ("[ACCESS]: PS Open Failed\n");
//      printf
//        ("[ACCESS]: PS Open Failed\n");
        /* Close the storage */
        nvsto_close_ps (access_nvsto_handle);
        return;
    }

    /** Load information in listed order */
    for (index = 0; index < MS_PS_ACCESS_MAX_RECORDS; index++)
    {
        if (0 != (records & (1 << index)))
        {
            /* Seek to the corresponding location in PS */
            nvsto_seek_ps(access_nvsto_handle, ms_ps_records[index].start_offset);
            /* Only using the load/store function pointers */
            ms_ps_records[index].load();
        }
    }

    /* Close the storage */
    nvsto_close_ps (access_nvsto_handle);
    return;
}

/**
    \brief Get Core Modules storage handle and offset from persistent storage.

    \par Description
    This function returns the storage handle and offset for Core Modules.

    \param [out] ps_handle  Persistent Storage Handle.
    \param [out] offset     The memory to be filled with the storage offset information.
*/
API_RESULT MS_access_ps_get_handle_and_offset
(
    /* OUT */ NVSTO_HANDLE* ps_handle,
    /* OUT */ UINT32*        offset
)
{
    API_RESULT retval;

    if ((NULL == ps_handle) || (NULL == offset))
    {
        retval = API_FAILURE;
        /* TODO: Return invalid parameter */
    }
    else
    {
        *ps_handle = access_nvsto_handle;
        *offset = MS_PS_RECORD_CORE_MODULES_OFFSET;
        retval = API_SUCCESS;
    }

    return retval;
}

void MS_access_ps_store_all_record(void)
{
    ms_access_ps_store(MS_PS_ACCESS_ALL_RECORDS);
}

API_RESULT MS_access_ps_store_disable(UINT8 enable)
{
    ms_ps_store_disable_flag = enable;
    return API_SUCCESS;
}


API_RESULT MS_access_ps_crc_check(INT16 ret)
{
    UINT32 index;
    UINT16  crc16_code,crc16_code_local;
    UINT32  start_store_addr;
    UINT8 id;

    if (0 > ret)
    {
        ACCESS_ERR
        ("[ACCESS]: PS Open Failed\n");
        return API_FAILURE;
    }

    crc16_code = 0;
    start_store_addr = (ret == NVS_FLASH_FIRST_OK) ? nvs_flash_base1 : nvs_flash_base2;
    start_store_addr &= 0xffffff;
    start_store_addr += 0x11000000;
    hal_flash_read((unsigned int)start_store_addr+2 + nvs_crc_pos,(UINT8*)&crc16_code_local,2);
    hal_flash_read((unsigned int)start_store_addr+1 + nvs_crc_pos,(UINT8*)&id,1);

    if((crc16_code_local == 0xffff)&&(id == 0xff))
    {
        return API_FAILURE;
    }

    for (index = 0; index < (12 + (id>>4)&0xf); index++)
    {
        /* Seek to the corresponding location in PS */
        nvsto_seek_ps(access_nvsto_handle, ms_ps_records[index].start_offset);
        crc16_code = crc16(crc16_code, (UINT8*)((unsigned int)start_store_addr +
                                                ms_ps_records[index].start_offset), ms_ps_records[index].size);
    }

    if(crc16_code_local != crc16_code)
        return API_FAILURE;

    return API_SUCCESS;
}


#endif /* MS_STORAGE */
