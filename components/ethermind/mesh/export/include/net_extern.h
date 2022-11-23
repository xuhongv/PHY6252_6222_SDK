
/**
    \file net_extern.h

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_NET_EXTERN_
#define _H_NET_EXTERN_

#include "MS_net_api.h"

/* --------------------------------------------- Global Definitions */
/**
    Network Proxy Filter List Count is ONE less that configured network interface.
    First (0-th) network interface is used for Advertising Channel.
*/
#define PROXY_FILTER_LIST_COUNT (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) - 1)



/* --------------------------------------------- Data Types/ Structures */
/** Proxy Filter List */
typedef struct _PROXY_FILTER_LIST
{
    /* Proxy Address List */
    MS_DEFINE_GLOBAL_ARRAY(PROXY_ADDR, p_addr, MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE));
    MS_DEFINE_GLOBAL_ARRAY(PROXY_ADDR, v_addr, MS_CONFIG_LIMITS(MS_PROXY_FILTER_DYNAMIC_LIST_SIZE));

    /* Proxy Address List Active Count */
    UINT16             p_count;

    /* Proxy Address List Active Count */
    UINT16             v_count;

    /* Proxy List Filter Type */
    PROXY_FILTER_TYPE  type;

    /* Proxy Server/Client Role */
    UCHAR              role;

} PROXY_FILTER_LIST;


/* --------------------------------------------- External Global Definitions */
/* Module Mutex */
MS_DEFINE_MUTEX_TYPE(extern, net_mutex)

/* Module callback pointer */
extern NET_NTF_CB net_callback;

/* Network Sequence Number */
extern NET_SEQ_NUMBER_STATE net_seq_number_state;

extern UINT8 seq_num_init_flag;
extern UINT32 g_iv_update_index;
extern UINT8 g_iv_update_state;
extern UINT8 g_iv_update_start_timer;
extern UINT8   MS_key_refresh_active;

extern UINT16 net_cache_start;
extern UINT16 net_cache_size;

extern PROXY_FILTER_LIST net_proxy_list[PROXY_FILTER_LIST_COUNT];

extern UINT8 ms_proxy_filter_list_size;

/* Network Cache */
//MS_DECLARE_GLOBAL_ARRAY(NET_CACHE_ELEMENT, net_cache, MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE));

//extern UINT16 net_cache_start;
//extern UINT16 net_cache_size;

API_RESULT net_delete_from_cache
(
    /* IN */ MS_NET_ADDR  addr
);

API_RESULT net_proxy_server_filter_op
(
    /* IN */ NETIF_HANDLE*   handle,
    /* IN */ UCHAR           opcode,
    /* IN */ UCHAR*          pdu,
    /* IN */ UINT16          pdu_len,
    /* IN */ UCHAR           proxy_fitlter_flg
);


#endif /* _H_NET_EXTERN_ */

