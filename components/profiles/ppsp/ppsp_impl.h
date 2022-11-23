#ifndef PPSP_IMPL_H
#define PPSP_IMPL_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
    INCLUDES
*/
#include "comdef.h"

/*********************************************************************
    CONSTANTS
*/
#define PPSP_IMPL_CFGS_TIMR_TOUT                              (100)


typedef struct
{
    volatile uint32 ppsp_impl_appl_bins_vers;    // firmware/resource vers @ local flsh
    volatile uint32 ppsp_impl_appl_bins_addr;    // firmware/resource addr @ local flsh
    volatile uint32 ppsp_impl_appl_bins_size;    // firmware/resource size @ local flsh
    volatile uint32 ppsp_impl_appl_mtus;        // valid xfer buff size (-3)

    int32  (*ppsp_impl_prog_eras_hdlr)( uint32 addr);
    int32  (*ppsp_impl_prog_read_hdlr)( uint32 addr, void* valu, uint16 size );
    int32  (*ppsp_impl_prog_writ_hdlr)( uint32 addr, void* valu, uint16 size );

    void   (*ppsp_impl_appl_writ_hdlr)( uint8 para, void* valu, uint16 coun );
    void   (*ppsp_impl_appl_noti_hdlr)( uint8 para, void* valu, uint16 coun );

    void   (*ppsp_impl_appl_timr_hdlr)( void );
    void   (*ppsp_impl_appl_tout_hdlr)( void );

    void   (*ppsp_impl_appl_rset_hdlr)( void );
} ppsp_impl_clit_hdlr_t;


int32
ppsp_impl_reg_clit_appl( ppsp_impl_clit_hdlr_t* clit_hdlr );

int32
ppsp_impl_reg_serv_appl( ppsp_impl_clit_hdlr_t* clit_hdlr );

/*
    callback of tick, on which used as watch guard and ticker for xfer
*/
// void
// ppsp_impl_appl_timr_hdlr(void);

// uint8
// ppsp_impl_ini(void);

// uint32
// ppsp_impl_get_stat(void);

// uint8
// ppsp_impl_get_pids(uint8* pids);

// uint8
// ppsp_impl_get_macs(uint8* macs);

// uint8
// ppsp_impl_get_scrt(uint8* scrt);

// uint8
// ppsp_impl_cal_keys(const uint8* rand, uint8 rsiz, const uint8* pids, uint8 psiz, const uint8* macs, uint8 msiz, const uint8* scrt, uint8 ssiz);

// uint8
// ppsp_impl_enc_text(uint8* text, uint8* cipr);

// uint8
// ppsp_impl_dec_cipr(uint8* text, uint8* cipr);

/*
    callback of connection stat changes
*/
// void
// ppsp_impl_ack_conn(uint8 flag);

// bStatus_t
// ppsp_serv_ntf_para( uint8 para, void *valu, uint16 lnth );

// uint8
// ppsp_impl_set_msgs();

#ifdef __cplusplus
}
#endif

#endif /*  */
