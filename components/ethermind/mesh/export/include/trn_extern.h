
/**
    \file trn_extern.h

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_TRN_EXTERN_
#define _H_TRN_EXTERN_

/* Individual Queue Elements */
typedef struct _TRN_FRN_Q_ELEMENT
{
    /* Associated Network Header */
    MS_NET_HEADER net_hdr;

    /* Packet Length */
    UINT8         ltrn_pkt_length;

    /* Is Ack? */
    UINT8         is_ack;

    /* Lower Transport Packet */
    UINT8         pdu[16 /* NET_MAX_PAYLOAD_SIZE */];

} TRN_FRN_Q_ELEMENT;

/**
      Friend Queue

      0-th Index: Contains completely received segmented or unsegmented
                  Transport Messages
      1-st Index: Contains segmented Transport Messages currently being
                  received. After all the segments are received, segments
                  from [#1] will be copied to [#0]
*/
typedef struct _TRN_FRIEND_QUEUE
{
    MS_DEFINE_GLOBAL_ARRAY(TRN_FRN_Q_ELEMENT, queue, MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE));
    UINT16 queue_start;
    UINT16 queue_size;
} TRN_FRIEND_QUEUE;


/** LowPower Node Element Structure */
typedef struct _TRN_LPN_ELEMENT
{
    MS_TRN_FRNDSHIP_INFO node;

    /* Friend Clear Context */
    struct _clrctx
    {
        /* Previous friend address */
        MS_NET_ADDR frnd_addr;

        /* Clear Retry Timeout */
        UINT32 retry_tmo;

        /* Timer handle */
        EM_timer_handle thandle;
    } clrctx;

    /**
        Friend Queue

        0-th Index: Contains completely received segmented or unsegmented
                   Transport Messages
        1-st Index: Contains segmented Transport Messages currently being
                   received. After all the segments are received, segments
                   from [#1] will be copied to [#0]
    */
    TRN_FRIEND_QUEUE friend_queue[2];

    /* Subscription List */
    MS_DEFINE_GLOBAL_ARRAY(MS_NET_ADDR, subscription_list, MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE));

    /* LPN Poll Timeout (in ms) */
    UINT32 poll_timeout;

    /*
        Validity index. Supports following Values:
        - 0x00: Entity Invalid
        - 0x01: Entity Valid Permanent
        - 0xFF: Entity Valid Temporary
    */
    UCHAR valid;

    /* Message Queue Start Index */
    UINT16 mqstart;

    /* Message Queue End Index */
    UINT16 mqend;

    /* LPN Receive Delay (in Ms) */
    UCHAR rx_delay;

    /* Number of elements in the LPN */
    UCHAR num_elements;

    /* RSSI of the friend request packet */
    UCHAR rssi;

    /* Friend Sequence Number in FriendPoll */
    UCHAR fsn;

    /* Timer for the element - TODO: Single global timer? */
    EM_timer_handle thandle;

} TRN_LPN_ELEMENT;

/** Friend Element Structure */ /* TODO: See if the structures can be clubbed */
typedef struct _TRN_FRND_ELEMENT
{
    /* Friendship information */
    MS_TRN_FRNDSHIP_INFO node;

    /* Adjusted Poll timeout */
    UINT32 poll_to;

    /* The main subnet handle */
    MS_SUBNET_HANDLE subnet_handle;

    /* Friend Address */
    MS_NET_ADDR addr;

    /* Friend Receive Windows */
    UCHAR rx_window;

    /* Transaction Number for Management PDUs */
    UCHAR txn_no;

    /* Current Friend sequence number */
    UCHAR fsn;

    /* Friend sequence number last sent */
    UCHAR sfsn;

    /* Timer for the element */
    EM_timer_handle thandle;

    /* Poll retry tracker */
    UCHAR poll_retry_count;

} TRN_FRND_ELEMENT;

/** Hearbeat Publication state */
typedef struct _TRN_HEARTBEAT_PUBLICATION_STATE
{
    /**
        Destination address for Heartbeat messages
    */
    MS_NET_ADDR daddr;

    /**
        Count to control the number of periodic heartbeat
        transport messages to be sent
    */
    UINT8 count_log;

    /**
        Period to control the cadence of periodic heartbeat
        transport messages
    */
    UINT8 period_log;

    /**
        TTL value to be used when sending Heartbeat messages
    */
    UINT8 ttl;

    /**
        Features that trigger sending Heartbeat messages when changed
    */
    UINT16 features;

    /**
        Global NetKey index of the NetKey to be used to send Heartbeat messges
    */
    UINT16 netkey_index;

    /** Associated Subnet Handle */
    MS_SUBNET_HANDLE subnet_handle;

    /* Period Timer */
    EM_timer_handle timer_handle;

    /* Tx Count */
    UINT32           tx_count;

} TRN_HEARTBEAT_PUBLICATION_STATE;

/** Hearbeat Subscription state */
typedef struct _TRN_HEARTBEAT_SUBSCRIPTION_STATE
{
    /**
        Source address for Heartbeat messages that a node shall process
    */
    MS_NET_ADDR saddr;

    /**
        Destination address for Heartbeat messages
    */
    MS_NET_ADDR daddr;

    /**
        Counter that tracks the number of periodic heartbeat transport message
        received since receiving the most recent Config Heartbeat Subscription
        Set message
    */
    UINT16 count;

    /**
        Logarithmic value of the above count
    */
    UINT8 count_log;

    /**
        Period that controls the period for processing periodical Heartbeat
        transport control messages
    */
    UINT8 period_log;

    /**
        Minimum hops value registered when receiving heartbeat messages since
        receiving the most recent Config Heartbeat Subscription Set message
    */
    UINT16 min_hops;

    /**
        Maximum hops value registered when receiving heartbeat messages since
        receiving the most recent Config Heartbeat Subscription Set message
    */
    UINT16 max_hops;

    /* Period Timer */
    EM_timer_handle timer_handle;

} TRN_HEARTBEAT_SUBSCRIPTION_STATE;


/* --------------------------------------------- External Global Definitions */
/* Module Mutex */
MS_DEFINE_MUTEX_TYPE(extern, trn_mutex)

/* Module callbacks */
extern TRN_NTF_CB trn_ctrl_callback;
extern TRN_NTF_CB trn_access_callback;
extern TRN_HEARTBEAT_RCV_CB    trn_heartbeat_rcv_callback;
extern TRN_HEARTBEAT_RCV_TIMEOUT_CB    trn_heartbeat_rcv_timeout_callback;



/** Friend Role */
extern UCHAR trn_frnd_role;

/** LowPower Node Element Database */
MS_DECLARE_GLOBAL_ARRAY(TRN_LPN_ELEMENT, lpn_element, MS_CONFIG_LIMITS(MS_MAX_LPNS));

/** Friend Element Database */
extern TRN_FRND_ELEMENT frnd_element;

/** Global Friend counter */
extern UINT16 trn_frnd_counter;

/* Global LPN counter */
extern UINT16 trn_lpn_counter;

/* Friendship Setup timeout */
extern UINT32 frnd_setup_timeout;

/* Friendship setup time lapsed count */
extern UINT32 frnd_setup_time_lapsed;

/* Heartbeat Publication Information */
extern TRN_HEARTBEAT_PUBLICATION_STATE heartbeat_pub;

/* Heartbeat Subscription Information */
extern TRN_HEARTBEAT_SUBSCRIPTION_STATE heartbeat_sub;

/* Friendship callback */
extern TRN_FRND_CB frnd_cb;

extern UINT16 ms_max_dev_keys;


#endif /* _H_TRN_EXTERN_ */

