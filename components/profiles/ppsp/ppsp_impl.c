
/*********************************************************************
    INCLUDES
*/
#include "ppsp_impl.h"
#include "ppsp_serv.h"
#include "error.h"
#include "flash.h"
#include "OSAL.h"
#include "log.h"


/*
 ******************************************************************************
    Defines
 ******************************************************************************
*/
#define PPSP_IMPL_CFGS_LOGS_TAGS        "PPSP_IMPL"
#ifdef  PPSP_IMPL_CFGS_LOGS_TAGS

#define logs_logs_fmt(str, scp, fmt, ...)                                 \
    LOG("[%s][%s] "fmt"\n\r", str, scp, ##__VA_ARGS__);

/* ERROR */
#define logs_err(fmt, ...)                                                \
    logs_logs_fmt("ERR", PPSP_IMPL_CFGS_LOGS_TAGS, fmt, ##__VA_ARGS__)

/* WARNING */
#define logs_war(fmt, ...)                                                \
    logs_logs_fmt("WAR", PPSP_IMPL_CFGS_LOGS_TAGS, fmt, ##__VA_ARGS__)

/* INFORMATION */
#define logs_inf(fmt, ...)                                                \
    logs_logs_fmt("INF", PPSP_IMPL_CFGS_LOGS_TAGS, fmt, ##__VA_ARGS__)

/* VERB */
#define logs_ver(fmt, ...)                                                \
    // logs_logs_fmt("VER", PPSP_IMPL_CFGS_LOGS_TAGS, fmt, ##__VA_ARGS__)

/* Function entry */
#define logs_ent(fmt, ...)                                                \
    // LOG("[%s][%s] %s("fmt")\n\r", "ENT", PPSP_IMPL_CFGS_LOGS_TAGS, __func__, ##__VA_ARGS__)

/* function exit */
#define logs_exi(fmt, ...)                                                \
    // LOG("[%s][%s] %s("fmt")\n\r", "EXI", PPSP_IMPL_CFGS_LOGS_TAGS, __func__, ##__VA_ARGS__)

/* DUMP */
static void
ppsp_impl_dbg_dump_byte(uint8* data, uint32 size)
{
    for ( uint32 itr0 = 0; itr0 < size; itr0 += 1 )
    {
        LOG("%02x ",data[itr0]);
    }

    LOG("\r\n");
}
#define logs_dmp(data, size)                                              \
    ppsp_impl_dbg_dump_byte(data, size)

#else
/* ERROR */
#define logs_err(fmt, ...)

/* WARNING */
#define logs_war(fmt, ...)

/* INFORMATION */
#define logs_inf(fmt, ...)

/* VERB */
#define logs_ver(fmt, ...)

/* Function entry */
#define logs_ent(fmt, ...)

/* function exit */
#define logs_exi(fmt, ...)

#define logs_dmp(data, size)

#endif /* CORE_CFG_BUILD_DEBUG */

/* ASSERT */
#define ppsp_impl_ast_expr_halt(expr)                                       \
    if ( !(expr) )                                                              \
    {                                                                           \
        logs_err("!! ASSERT FAILURE !! @ %s @ %d", __FILE__, __LINE__);         \
        while ( 1 ) ;                                                           \
    }

#define ppsp_impl_ast_expr_abrt(expr, rslt)                                 \
    if ( !(expr) )                                                              \
    {                                                                           \
        logs_err("!! ASSERT FAILURE !! @ %s @ %d", __FILE__, __LINE__);         \
        return ( rslt ) ;                                                       \
    }

#define PPSP_IMPL_CFGS_BUSY_STAT_BITS   0x00000001
#define PPSP_IMPL_CFGS_AUTH_STAT_BITS   0x00000002
#define PPSP_IMPL_CFGS_OTAS_STAT_BITS   0x00000004  // ota bgns
#define PPSP_IMPL_CFGS_OTAE_STAT_BITS   0x00000008  // ota ends

#define PPSP_IMPL_CFGS_ALIS_PIDS_COUN   4
#define PPSP_IMPL_CFGS_ALIS_MACS_COUN   6
#define PPSP_IMPL_CFGS_ALIS_SCRT_COUN   16

#define PPSP_IMPL_CFGS_MSGS_CRYP_ENAB   0//1
#if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    // #include "sha256.h"
    #include "tinycrypt/sha256.h"
    #include "tinycrypt/aes.h"
#endif

#define PPSP_IMPL_CFGS_MSGS_HDER_SIZE   4

#define PPSP_IMPL_CFGS_PACK_FRAM_NUMB   (0x0f)  // maxi frame numb per package (0x00~0x0f)

#define PPSP_IMPL_CFGS_OPCO_MISN_ISSU   0x02    // issu by master, resp by slave
#define PPSP_IMPL_CFGS_OPCO_MISR_RESP   0x03    // resp by slave, issu by master
#define PPSP_IMPL_CFGS_OPCO_MNSI_ISSU   0x04    // issu by slave, resp by master
#define PPSP_IMPL_CFGS_OPCO_MRSI_RESP   0x05    // resp by master, issu by slave
#define PPSP_IMPL_CFGS_OPCO_NRSP_ISSU   0x06    // issu W/O resp

#define PPSP_IMPL_CFGS_OPCO_RAND_ISSU   0x10    // issu RAND by master, resp by slave
#define PPSP_IMPL_CFGS_OPCO_CIPR_RESP   0x11    // resp RAND by slave, issu by master. CIPR = ENCR(RAND, BKEY(RAND,MACS,SCRT))
#define PPSP_IMPL_CFGS_OPCO_VERF_ISSU   0x12    // issu VERF by master, resp by slave. verify result of encryption
#define PPSP_IMPL_CFGS_OPCO_VERF_RESP   0x13    // resp VERF by slave, issu by master
#define PPSP_IMPL_CFGS_OPCO_NWRK_ISSU   0x14    // issu NWRK by master, resp by slave. networking rsult, unprov/proved
#define PPSP_IMPL_CFGS_OPCO_NWRK_RESP   0x15    // resp NWRK by slave, issu by master. confirm of networking rsult

#define PPSP_IMPL_CFGS_OPCO_VERS_ISSU   0x20    // issu VERS by master, resp by slave
#define PPSP_IMPL_CFGS_OPCO_VERS_RESP   0x21    // resp VERS by slave, issu by master
#define PPSP_IMPL_CFGS_OPCO_UPDA_ISSU   0x22    // issu UPDA by master, resp by slave
#define PPSP_IMPL_CFGS_OPCO_UPDA_RESP   0x23    // resp UPDA by slave, issu by master. confirm, rcvd size, fast mode
#define PPSP_IMPL_CFGS_OPCO_PACK_ISSU   0x2F    // issu PACK by master, issu W/O resp.
#define PPSP_IMPL_CFGS_OPCO_PACK_RESP   0x24    // issu PACK by slave, issu W/O resp.  confirm of total frame
#define PPSP_IMPL_CFGS_OPCO_COMP_ISSU   0x25    // issu COMP by master, resp by slave. complete of transfer
#define PPSP_IMPL_CFGS_OPCO_COMP_RESP   0x26    // resp COMP by slave, issu by master. reply with crc

#define PPSP_IMPL_CFGS_OPCO_USER_ISSU   0xFE    // issu USER, resp by slave
#define PPSP_IMPL_CFGS_OPCO_USER_RESP   0xFF    // issu USER, resp by slave

#define PPSP_IMPL_CFGS_PROG_ADDR_BASE   (0x11000000)
#define PPSP_IMPL_CFGS_PROG_SCTR_SIZE   (0x1000)    // program sector size in byte
#define PPSP_IMPL_CFGS_PROG_ADDR_BGNS   (0x55000)   // program data bgn address of flash
#define PPSP_IMPL_CFGS_PROG_ADDR_ENDS   (0x80000)   // program data end address of flash
#define PPSP_IMPL_CFGS_PROG_FLSH_SIZE   (PPSP_IMPL_CFGS_PROG_ADDR_ENDS-PPSP_IMPL_CFGS_PROG_ADDR_BGNS)   // program total size in byte

#define PPSP_IMPL_CFGS_PROG_VERS_MAJR   (1)
#define PPSP_IMPL_CFGS_PROG_VERS_MINR   (0)
#define PPSP_IMPL_CFGS_PROG_VERS_REVI   (0)
#define PPSP_IMPL_CFGS_PROG_VERS_RSRV   (0)

#define PPSP_IMPL_CFGS_PROG_TYPE_SIZE   (1)
#define PPSP_IMPL_CFGS_PROG_VERS_SIZE   (4)
#define PPSP_IMPL_CFGS_PROG_SIZE_SIZE   (4)
#define PPSP_IMPL_CFGS_PROG_CRCS_SIZE   (2)
#define PPSP_IMPL_CFGS_PROG_FLAG_SIZE   (1)

#define PPSP_IMPL_CFGS_PROG_BUFF_SIZE   (256)   // 4BYTE ALLIGNED


/* para, callback hdlr exchanged between appl & ppsp */
static ppsp_impl_clit_hdlr_t* __ppsp_impl_clit_hdlr = 0;

static ppsp_serv_appl_hdlr_t  __ppsp_serv_appl_hdlr =
{
    .ppsp_serv_char_upda_hdlr = 0,
};


static const uint8
__ppsp_impl_opco_prim_list[] =
{
    PPSP_IMPL_CFGS_OPCO_MISN_ISSU,
    PPSP_IMPL_CFGS_OPCO_MNSI_ISSU,
    PPSP_IMPL_CFGS_OPCO_MISR_RESP,
    PPSP_IMPL_CFGS_OPCO_MRSI_RESP,
    PPSP_IMPL_CFGS_OPCO_NRSP_ISSU,

    PPSP_IMPL_CFGS_OPCO_RAND_ISSU,
    PPSP_IMPL_CFGS_OPCO_CIPR_RESP,
    PPSP_IMPL_CFGS_OPCO_VERF_ISSU,
    PPSP_IMPL_CFGS_OPCO_VERF_RESP,
    PPSP_IMPL_CFGS_OPCO_NWRK_ISSU,
    PPSP_IMPL_CFGS_OPCO_NWRK_RESP,

    PPSP_IMPL_CFGS_OPCO_VERS_ISSU,
    PPSP_IMPL_CFGS_OPCO_VERS_RESP,
    PPSP_IMPL_CFGS_OPCO_UPDA_ISSU,
    PPSP_IMPL_CFGS_OPCO_UPDA_RESP,
    PPSP_IMPL_CFGS_OPCO_PACK_ISSU,
    PPSP_IMPL_CFGS_OPCO_PACK_RESP,
    PPSP_IMPL_CFGS_OPCO_COMP_ISSU,
    PPSP_IMPL_CFGS_OPCO_COMP_RESP,

    PPSP_IMPL_CFGS_OPCO_USER_ISSU,
    PPSP_IMPL_CFGS_OPCO_USER_RESP,
};
static uint8
__ppsp_impl_opco_prim_coun = sizeof(__ppsp_impl_opco_prim_list);

static uint8*
__ppsp_impl_opco_user_list = 0;
static uint8
__ppsp_impl_opco_user_coun = 0;

#if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    static uint8
    __ppsp_impl_auth_keys_data[16];
    static uint8
    __ppsp_impl_auth_verf_data[16];
#endif


static uint32
__ppsp_impl_stat_bits_flag = 0x00;
static uint32
__ppsp_impl_msgs_numb = 0;  // expect sequ numb of next

uint8   __ppsp_impl_upda_buff[PPSP_IMPL_CFGS_PROG_BUFF_SIZE];
uint32  __ppsp_impl_upda_frsz;


/*
    private function prototype
*/


/*
    private function implimentation
*/
#define ppsp_impl_get_auth_rslt(rslt)                       \
    {                                                           \
        (rslt)  = ((__ppsp_impl_stat_bits_flag & PPSP_IMPL_CFGS_AUTH_STAT_BITS)?1:0);   \
    }

#define ppsp_impl_set_auth_rslt(flag)                       \
    {                                                           \
        if ( 0 == (flag) )                                      \
            __ppsp_impl_stat_bits_flag &= ~PPSP_IMPL_CFGS_AUTH_STAT_BITS;   \
        else                                                    \
            __ppsp_impl_stat_bits_flag |= PPSP_IMPL_CFGS_AUTH_STAT_BITS;    \
    }

#define ppsp_impl_get_msgs_numb(msgs, numb)                     \
    {                                                           \
        if ( 0 != (msgs) )                                      \
            (numb) = ((uint8*)(msgs))[0]&0x0F;                  \
    }

#define ppsp_impl_set_msgs_numb(msgs, numb)                     \
    {                                                           \
        if ( 0 != (msgs) )                                      \
            ((uint8*)(msgs))[0] = (((uint8*)(msgs))[0]&~0x0F) | (numb)&0x0F;\
    }

#define ppsp_impl_get_msgs_encr(msgs, flag)                     \
    {                                                           \
        if ( 0 != (msgs) )                                      \
            (flag) = ((((uint8*)(msgs))[0]&0x10)>>4);           \
    }

#define ppsp_impl_set_msgs_encr(msgs, flag)                     \
    {                                                           \
        if ( 0 != (msgs) )                                      \
            ((uint8*)(msgs))[0] = (((uint8*)(msgs))[0]&~(0x01<<4)) | (((flag)&0x01)<<4);\
    }

#define ppsp_impl_get_msgs_opco(msgs, opco)                     \
    {                                                           \
        if ( 0 != (msgs) )                                      \
            (opco) = ((uint8*)(msgs))[1]&0xFF;                  \
    }

#define ppsp_impl_set_msgs_opco(msgs, opco)                 \
    {                                                           \
        if ( 0 != (msgs) )                                        \
            ((uint8*)(msgs))[1] = (opco)&0xFF;                              \
    }

#define ppsp_impl_get_msgs_seqn(msgs, alln, seqn)           \
    {                                                           \
        if ( 0 != (msgs) ) {                                      \
            alln = (((uint8*)(msgs))[2]&0xF0)>>4;                           \
            seqn = (((uint8*)(msgs))[2]&0x0F)>>0;                           \
        }                                                       \
    }

#define ppsp_impl_set_msgs_seqn(msgs, alln, seqn)           \
    {                                                           \
        if ( 0 != (msgs) )                                        \
            ((uint8*)(msgs))[2] = ((alln&0x0F))<<4 | ((seqn&0x0F)<<0);      \
    }

/*
    desc: get msg payload length, byte 3 of header
*/
#define ppsp_impl_get_msgs_frsz(msgs, frsz)                 \
    {                                                           \
        if ( 0 != (msgs) )                                        \
            (frsz) = ((uint8*)(msgs))[3]&0xFF;                              \
    }

#define ppsp_impl_set_msgs_frsz(msgs, frsz)                 \
    {                                                           \
        if ( 0 != (msgs) )                                        \
            ((uint8*)(msgs))[3] = (frsz)&0xFF;                              \
    }

/*
    desc: get msg payload
*/
#define ppsp_impl_get_msgs_plds(msgs, data)                 \
    {                                                           \
        if ( 0 != msgs )                                        \
            (/* (uint8*) */(data)) = (((uint8*)(msgs))+PPSP_IMPL_CFGS_MSGS_HDER_SIZE);    \
    }

/*
    desc: set msg payload
*/
#define ppsp_impl_set_msgs_plds(msgs, data, coun)           \
    {                                                           \
        if ( 0 != (uint8*)(msgs) )                              \
            osal_memcpy(((uint8*)(msgs))+PPSP_IMPL_CFGS_MSGS_HDER_SIZE, (uint8*)(data), coun);  \
    }

/*
    desc: PKCS#7 padding
*/
#define ppsp_impl_get_pkcs_7pad(bksz, dasz, pval)           \
    {                                                           \
        (pval) = (bksz) - ((dasz) % (bksz));                            \
    }

/*
    desc: chk mesg package size
*/
static int32
ppsp_impl_chk_msgs_lnth(const void* mesg, uint16 coun)
{
    // logs_ent("mesg:#X%08x, coun:#d%d", mesg, coun);
    return ( (NULL != mesg && PPSP_IMPL_CFGS_MSGS_HDER_SIZE <= coun) ? SUCCESS : FAILURE );
}

/*
    desc: chk mesg id
*/
static int32
ppsp_impl_chk_msgs_numb(const void* mesg, uint16 coun)
{
    // logs_ent("mesg:#X%08x, coun:#d%d", mesg, coun);
    int32 rslt = FAILURE;

    if ( NULL != mesg )
    {
        uint8 numb;
        ppsp_impl_get_msgs_numb(mesg, numb);
        // rslt = (/* (0 <= numb) &&  */(15 >= numb)); // comment for compuler warning

        if ( 0xff == __ppsp_impl_msgs_numb )
        {
            // serv only,
            // first mesg received, be the one
            __ppsp_impl_msgs_numb = numb;
        }

        if ( (__ppsp_impl_msgs_numb == numb) && (15 >= numb) )
        {
            rslt = SUCCESS;
        }

        logs_err("mesg numb:rcvd:#X%d, xpct:#X%d", numb, __ppsp_impl_msgs_numb);
    }

    return ( rslt );
}

/*
    desc: chk mesg opcode
*/
static int32
ppsp_impl_chk_msgs_opco(const void* mesg, uint16 coun)
{
    // logs_ent("mesg:#X%08x, coun:#d%d", mesg, coun);
    int32 rslt = FAILURE;

    if ( NULL != mesg )
    {
        uint8 opco;
        ppsp_impl_get_msgs_opco(mesg, opco);

        for ( int itr0 = 0; itr0 <  __ppsp_impl_opco_prim_coun; itr0 += 1 )
        {
            if ( opco == __ppsp_impl_opco_prim_list[itr0] )
            {
                rslt = SUCCESS;
                break;
            }
        }

        if ( SUCCESS != rslt )
            for ( int itr0 = 0; itr0 <  __ppsp_impl_opco_user_coun; itr0 += 1 )
            {
                if ( opco == __ppsp_impl_opco_user_list[itr0] )
                {
                    rslt = SUCCESS;
                    break;
                }
            }
    }

    return ( rslt );
}

/*
    desc: chk mesg segment number + total segment number
*/
static int32
ppsp_impl_chk_msgs_seqn(const void* mesg, uint16 coun)
{
    // logs_ent("mesg:#X%08x, coun:#d%d", mesg, coun);
    int32 rslt = FAILURE;

    if ( NULL != mesg )
    {
        uint8 alln, seqn;
        ppsp_impl_get_msgs_seqn(mesg, alln, seqn);

        if ( (15 >= alln) && (15 >= seqn) && (alln >= seqn) )
        {
            rslt = SUCCESS;
        }
    }

    return ( rslt );
}

/*
    desc: chk mesg validation
*/
static int32
ppsp_impl_chk_msgs_vali(const uint8* mesg, uint16 coun)
{
    int32 rslt = SUCCESS;

    /* */
    if ( SUCCESS == rslt )
    {
        rslt = ppsp_impl_chk_msgs_lnth(mesg, coun);
    }

    /* */
    // if ( SUCCESS == rslt )
    // {
    //     rslt = ppsp_impl_chk_msgs_numb(mesg, coun);
    // }

    /* */
    if ( SUCCESS == rslt )
    {
        rslt = ppsp_impl_chk_msgs_opco(mesg, coun);
    }

    /* */
    if ( SUCCESS == rslt )
    {
        rslt = ppsp_impl_chk_msgs_seqn(mesg, coun);
    }

    return ( rslt );
}

/*
    desc: make message with header & payload
*/
static void*
ppsp_impl_new_msgs_raws(uint8 numb, uint8 encr, uint8 opco, uint8 alln, uint8 seqn, uint8* data, uint16 leng)
{
    logs_ent("numb:#X%02x,encr:#X%02x,opco:#X%02x,alln:#X%02x,seqn:#X%02x,data:#X%08x,leng:#X%08x,",
             numb,       encr,       opco,       alln,       seqn,       data,       leng);
    uint8* msgs = 0;
    /* only consider unsegmented case */
    // LOG("osal_mem_use: %d \r\n", osal_memory_statics());
    msgs = osal_mem_alloc(PPSP_IMPL_CFGS_MSGS_HDER_SIZE+leng);

    if ( 0 != msgs )
    {
        osal_memset(msgs, 0, PPSP_IMPL_CFGS_MSGS_HDER_SIZE+leng);
        ppsp_impl_set_msgs_numb(msgs, numb);
        ppsp_impl_set_msgs_encr(msgs, encr);
        // ppsp_impl_set_msgs_vers(msgs, numb);
        ppsp_impl_set_msgs_opco(msgs, opco);
        ppsp_impl_set_msgs_seqn(msgs, alln, seqn);    // frame numb, frame sequ
        ppsp_impl_set_msgs_frsz(msgs, leng);    // frame size
        ppsp_impl_set_msgs_plds(msgs, data, leng); // payload
    }

    return ( msgs );
}

static int32
ppsp_impl_cvt_hexs_char(uint8* dest, uint32 dsiz, const uint8* srcs, uint32 ssiz, int revs)
{
    if ( (0 == dest) || (0 == srcs) )
    {
        logs_err("!! (0==dest) || (0==srcs) !! @ %s @ %d", __FILE__, __LINE__);
        return ( FAILURE );
    }

    if ( (ssiz * 2) >  dsiz )
    {
        logs_err("!! (ssiz * 2) >  dsiz !! @ %s @ %d", __FILE__, __LINE__);
        return ( FAILURE );
    }

    const uint8 hmap[] = "0123456789abcdef";    // hex map

    if ( revs )
    {
        srcs += ssiz;

        for ( uint32 itr0 = ssiz; itr0 >  0; itr0 -= 1 )
        {
            *dest++ = hmap[*--srcs >> 4];
            *dest++ = hmap[*srcs & 0x0F];
        }
    }
    else
    {
        for ( uint32 itr0 = 0; itr0 <  ssiz; itr0 += 1 )
        {
            *dest++ = hmap[*srcs     >> 4];
            *dest++ = hmap[*srcs++ & 0x0F];
        }
    }

    return ( SUCCESS );
}

static int32
ppsp_impl_era_prog_data(uint32 addr)
{
    int32 rslt = FAILURE;

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return ( FAILURE );
    }

    if ( 0 == __ppsp_impl_clit_hdlr->ppsp_impl_prog_eras_hdlr )
    {
        logs_ver("!! NULL PROG ERASE HDLR, USES DEFAULT !!");

        // flash address range check
        if ( PPSP_IMPL_CFGS_PROG_ADDR_BGNS >  addr || PPSP_IMPL_CFGS_PROG_ADDR_ENDS <= addr )
        {
            return ( FAILURE );
        }

        #if 0   // PRIME: 6202/6212
        flash_sector_erase(PPSP_IMPL_CFGS_PROG_ADDR_BASE + (addr&0xFFF000));
        #else   // BBB: 6222/6252
        hal_flash_erase_sector(PPSP_IMPL_CFGS_PROG_ADDR_BASE + (addr&0xFFF000));
        #endif
        return ( SUCCESS );
    }

    rslt = __ppsp_impl_clit_hdlr->ppsp_impl_prog_eras_hdlr(addr);
    return ( rslt );
}

static int32
ppsp_impl_pul_prog_data(uint32 addr, void* valu, uint16 size)
{
    int32 rslt = FAILURE;

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return ( FAILURE );
    }

    if ( 0 == __ppsp_impl_clit_hdlr->ppsp_impl_prog_read_hdlr )
    {
        logs_ver("!! NULL PROG READ HDLR, USES DEFAULT !!");

        // flash address range check
        if ( PPSP_IMPL_CFGS_PROG_ADDR_BGNS >  addr || PPSP_IMPL_CFGS_PROG_ADDR_ENDS <= addr )
        {
            return ( FAILURE );
        }

        if ( NULL == valu )
        {
            return ( FAILURE );
        }

        osal_memcpy(valu, (const void*)(PPSP_IMPL_CFGS_PROG_ADDR_BASE+addr), size);
        return ( SUCCESS );
    }

    rslt = __ppsp_impl_clit_hdlr->ppsp_impl_prog_read_hdlr(addr, valu, size);
    return ( rslt );
}

static int32
ppsp_impl_psh_prog_data(uint32 addr, void* valu, uint32 size)
{
    int32 rslt = FAILURE;

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return ( FAILURE );
    }

    if ( 0 == __ppsp_impl_clit_hdlr->ppsp_impl_prog_writ_hdlr )
    {
        logs_ver("!! NULL APPL WRIT HDLR, USES DEFAULT !!");

        // flash address range check
        if ( PPSP_IMPL_CFGS_PROG_ADDR_BGNS >  addr || PPSP_IMPL_CFGS_PROG_ADDR_ENDS <= addr )
        {
            return ( FAILURE );
        }

        // 4 bytes aligned
        if ( addr & 0x000003 )
        {
            return ( FAILURE );
        }

        #if 0   // PRIME: 6202/6212
        uint32  coun = (size+sizeof(uint32)-1)/sizeof(uint32);
        uint32  dwrd;   // data in word

        for ( int itr0 = 0; itr0 < coun; itr0 += 1 )
        {
            /* !! need swap variable, WTF !! */
            dwrd  = 0xffffffff;
            osal_memcpy(&dwrd, (uint32*)valu+itr0, sizeof(dwrd));

            if ( 0 == WriteFlash(PPSP_IMPL_CFGS_PROG_ADDR_BASE+addr+itr0*sizeof(uint32), dwrd) )
            {
                logs_war("!! WRIT FAIL, ABRT ALLS !!");
                return ( FAILURE );
            }
        }

        #else   // BBB: 6222/6252

        // if ( 0 != hal_flash_write(PPSP_IMPL_CFGS_PROG_ADDR_BASE+addr, valu, size) ) {
        if ( 0 != hal_flash_write_by_dma(PPSP_IMPL_CFGS_PROG_ADDR_BASE+addr, valu, size) )
        {
            logs_war("!! WRIT FAIL, ABRT ALLS !!");
            return ( FAILURE );
        }

        #endif
        return ( SUCCESS );
    }

    rslt = __ppsp_impl_clit_hdlr->ppsp_impl_prog_writ_hdlr(addr, valu, size);
    return ( rslt );
}

/*****************************************************************************/
// Authorization relative
static int32
ppsp_impl_get_auth_rand(uint8* rand)
{
    // int32   rslt = FAILURE;
    if ( 0 == rand )
    {
        return ( FAILURE );
    }

    // gens rand numb of utf8
    // for ( uint8 itr0 = 0; itr0 < PPSP_IMPL_CFGS_ALIS_PIDS_COUN; itr0 += 1 )
    // {
    //     //hal_flash_read(0x4030+itr0,&pids[PPSP_IMPL_CFGS_ALIS_PIDS_COUN-itr0-1],1);
    //     rslt = ppsp_impl_pul_prog_data(0x4030+itr0,&pids[PPSP_IMPL_CFGS_ALIS_PIDS_COUN-itr0-1],1);
    //     if ( SUCCESS != rslt ) return ( FAILURE );
    // }
    // pre-cfg value for debug only
    const uint8* zstr = "drfiHgbsvomOieog";
    osal_memcpy(rand, zstr, osal_strlen((char*)zstr));
    return ( SUCCESS );
}

static int32
ppsp_impl_get_auth_pids(uint8* pids)
{
    // int32   rslt = FAILURE;
    if ( 0 == pids )
    {
        return ( FAILURE );
    }

    #if 1
    // pre-cfg value for debug only
    const uint8 valu[PPSP_IMPL_CFGS_ALIS_PIDS_COUN] = { 0xe2,0x93,0x02,0x00, };
    osal_memcpy(pids, valu, PPSP_IMPL_CFGS_ALIS_PIDS_COUN);
    #else

    // load PID
    for ( uint8 itr0 = 0; itr0 < PPSP_IMPL_CFGS_ALIS_PIDS_COUN; itr0 += 1 )
    {
        //hal_flash_read(0x4030+itr0,&pids[PPSP_IMPL_CFGS_ALIS_PIDS_COUN-itr0-1],1);
        rslt = ppsp_impl_pul_prog_data(0x4030+itr0,&pids[PPSP_IMPL_CFGS_ALIS_PIDS_COUN-itr0-1],1);

        if ( SUCCESS != rslt ) return ( FAILURE );
    }

    #endif
    return ( SUCCESS );
}

static int32
ppsp_impl_get_auth_macs(uint8* macs)
{
    // int32  rslt = FAILURE;
    if ( 0 == macs )
    {
        return ( FAILURE );
    }

    #if 1
    // pre-cfg value for debug only
    const uint8 valu[PPSP_IMPL_CFGS_ALIS_MACS_COUN] = { 0xf3,0xf2,0xf1,0xf0,0xcd,0xab, };
    osal_memcpy(macs, valu, PPSP_IMPL_CFGS_ALIS_MACS_COUN);
    #else
    uint32 addr = 0x4000;

    if ( NULL != macs )
    {
        // load MAC
        //hal_flash_read(addr ++,&macs [3],1);
        //hal_flash_read(addr ++,&macs [2],1);
        //hal_flash_read(addr ++,&macs [1],1);
        //hal_flash_read(addr ++,&macs [0],1);
        //hal_flash_read(addr ++,&macs [5],1);
        //hal_flash_read(addr ++,&macs [4],1);
//        macs [3] = (uint8)ReadFlash(addr ++);
//        macs [2] = (uint8)ReadFlash(addr ++);
//        macs [1] = (uint8)ReadFlash(addr ++);
//        macs [0] = (uint8)ReadFlash(addr ++);
//        macs [5] = (uint8)ReadFlash(addr ++);
//        macs [4] = (uint8)ReadFlash(addr);
    }

    #endif
    return ( SUCCESS );
}

static int32
ppsp_impl_get_auth_scrt(uint8* scrt)
{
    uint8 rslt = 1;

    if ( 0 == scrt )
    {
        return ( FAILURE );
    }

    #if 1
    // pre-cfg value for debug only
    const uint8 valu[PPSP_IMPL_CFGS_ALIS_SCRT_COUN] = { 0xd1,0xc6,0x53,0xb2,0x8e,0x22,0x90,0xa5,0x43,0x04,0x12,0x5f,0x07,0xed,0x73,0x59, };
    osal_memcpy(scrt, valu, PPSP_IMPL_CFGS_ALIS_SCRT_COUN);
    #else

    for ( uint8 itr0 = 0; itr0 < PPSP_IMPL_CFGS_ALIS_SCRT_COUN; itr0 ++ )
    {
        //hal_flash_read(0x4010+itr0,&scrt[itr0],1);
        //scrt[itr0] = (uint8_t)ReadFlash(0x4010+itr0);
    }

    #endif
    return ( rslt );
}

/* calc auth keys */
#if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

static int32
ppsp_impl_cal_auth_keys(const uint8* rand, uint8 rsiz, const uint8* pids, uint8 psiz, const uint8* macs, uint8 msiz, const uint8* scrt, uint8 ssiz)
{
    // LOG("[ENT]: %s(rand:#X08%x,rsiz:#D%d,pids#X08%x,psiz:#D%d,macs#X08%x,msiz:#D%d,scrt#X08%x,ssiz:#D%d) \r\n",
    //     __func__,  rand,       rsiz,     pids,      psiz,     macs,      msiz,     scrt,      ssiz);
    int32 rslt = SUCCESS;
    uint8 temp[128];
    uint8 posi = 0;
    /* rand numb + ',' */
    osal_memcpy(temp+posi, rand, rsiz);
    posi += rsiz;
    temp[posi] = ',';
    posi += 1;
    /* pids in hex str + ',' */
    // hex2Str(pids, temp+posi, psiz, 1); posi += psiz * 2;
    ppsp_impl_cvt_hexs_char(temp+posi, sizeof(temp)-posi, pids, psiz, 1);
    posi += psiz * 2;
    temp[posi] = ',';
    posi += 1;
    /* mac in hex str + ',' */
    // hex2Str(macs, temp+posi, msiz, 1); posi += msiz * 2;
    ppsp_impl_cvt_hexs_char(temp+posi, sizeof(temp)-posi, macs, msiz, 1);
    posi += msiz * 2;
    temp[posi] = ',';
    posi += 1;
    /* secret */
    // hex2Str(scrt, temp+posi, ssiz, 0); posi += ssiz * 2;
    ppsp_impl_cvt_hexs_char(temp+posi, sizeof(temp)-posi, scrt, ssiz, 0);
    posi += ssiz * 2;
    // mbedtls_sha256_context ctxt;
    struct tc_sha256_state_struct hash;
    uint8 sha256sum[32];
    // mbedtls_sha256_init(&ctxt);
    tc_sha256_init(&hash);
    // rslt = mbedtls_sha256_starts_ret(&ctxt, 0);
    // if ( rslt == 0 ) {
    //     rslt = mbedtls_sha256_update_ret(&ctxt, temp, posi);
    // }
    // if ( rslt == 0 ) {
    //     rslt = mbedtls_sha256_finish_ret(&ctxt, sha256sum);
    // }
    tc_sha256_update(&hash, (const uint8_t*)temp, posi);
    tc_sha256_final(sha256sum, &hash);
    osal_memcpy(__ppsp_impl_auth_keys_data, sha256sum, sizeof(__ppsp_impl_auth_keys_data));
    logs_ver("== rand text givn: ==");
    // my_dump_byte((uint8_t*)rand, rsiz);
    logs_dmp((uint8*)rand, rsiz);
    logs_ver("== pids valu givn: ==");
    // my_dump_byte((uint8_t*)pids, rsiz);
    logs_dmp((uint8*)pids, psiz);
    logs_ver("== macs addr givn: ==");
    // my_dump_byte((uint8_t*)macs, msiz);
    logs_dmp((uint8*)macs, msiz);
    logs_ver("== scrt valu givn: ==");
    // my_dump_byte((uint8_t*)scrt, ssiz);
    logs_dmp((uint8*)scrt, ssiz);
    logs_ver("== cryp keys gens: ==");
    // my_dump_byte((uint8_t*)__ppsp_impl_auth_keys_data, sizeof(__ppsp_impl_auth_keys_data));
    logs_dmp(__ppsp_impl_auth_keys_data, sizeof(__ppsp_impl_auth_keys_data));
    #if 0
    gen_aligenie_auth_key((uint8*)rand, rsiz, (uint8*)pids, psiz, (uint8*)macs, msiz, (uint8*)scrt, ssiz);
    cpy_aligenie_auth_key(__ppsp_impl_auth_keys_data);
    #endif
    return ( rslt );
}

static int32
ppsp_impl_enc_text(uint8* text, uint8* cipr)
{
    logs_ent("");
    // uint8 rslt   = 1;
    uint8 iv[16] = { 0x31, 0x32, 0x33, 0x61, 0x71, 0x77, 0x65, 0x64,
                     0x23, 0x2a, 0x24, 0x21, 0x28, 0x34, 0x6a, 0x75,
                   };
    uint8 data[16];
    uint8 itr0 = 0;
    osal_memcpy(data, text, 16);

    while ( itr0 <  16 )
    {
        data[itr0] ^= iv[itr0];
        itr0       += 1;
    }

    struct tc_aes_key_sched_struct s;

    if ( 0 == tc_aes128_set_encrypt_key(&s, __ppsp_impl_auth_keys_data) )
    {
        LOG("AES128 %s (NIST encr test) failed.\n", __func__);
        // rslt = 0;
        // goto RSLT_FAIL_ENCR;
        return ( FAILURE );
    }

    if (tc_aes_encrypt(cipr, data, &s) == 0)
    {
        LOG("AES128 %s (NIST encr test) failed.\n", __func__);
        // rslt = 0;
        // goto RSLT_FAIL_ENCR;
        return ( FAILURE );
    }

    // rslt = 1;
// RSLT_FAIL_ENCR:
    // return ( rslt );
    return ( SUCCESS );
}

static int32
ppsp_impl_dec_cipr(uint8* text, uint8* cipr)
{
    logs_ent("");
    // uint8 rslt = 1;
    struct tc_aes_key_sched_struct s;

    if ( 0 == tc_aes128_set_decrypt_key(&s, __ppsp_impl_auth_keys_data) )
    {
        LOG("AES128 %s (NIST decr test) failed.\n", __func__);
        // rslt = 0;
        // goto RSLT_FAIL_ENCR;
        return ( FAILURE );
    }

    if ( 0 == tc_aes_decrypt(text, cipr, &s) )
    {
        LOG("AES128 %s (NIST decr test) failed.\n", __func__);
        // rslt = 0;
        // goto RSLT_FAIL_ENCR;
        return ( FAILURE );
    }

    uint8 aesiv[16] = { 0x31, 0x32, 0x33, 0x61, 0x71, 0x77, 0x65, 0x64,
                        0x23, 0x2a, 0x24, 0x21, 0x28, 0x34, 0x6a, 0x75,
                      };
    uint8 itr0 = 0;

    while ( itr0 <  16 )
    {
        text[itr0] ^= aesiv[itr0];
        itr0       += 1;
    }

//     rslt = 1;
// RSLT_FAIL_ENCR:
//     return ( rslt );
    return ( SUCCESS );
}
#endif

static void
ppsp_impl_ack_serv_msgs_cipr(void* cipr, uint32 lnth)
{
    logs_ent("");
    logs_ver("acks trgt rand cipr ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    uint8* msgs_xfer = 0;
    uint8* msgs_data = cipr;
    uint16 msgs_size = lnth;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    0,                              // flag of encryption
                    PPSP_IMPL_CFGS_OPCO_CIPR_RESP,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD8_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

static void
ppsp_impl_ack_serv_msgs_verf(uint8 rslt)
{
    logs_ent("");
    logs_ver("acks trgt auth verify rslt ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    /* 0:succ, 1:fail */
    ppsp_impl_set_auth_rslt(!rslt);
    /* */
    // shared btle xfer buff & otas buff
    // rply text
    __ppsp_impl_upda_buff[0] = rslt;
    __ppsp_impl_upda_frsz    = 1;
    uint8* msgs_xfer = 0;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    0,                              // flag of encryption
                    PPSP_IMPL_CFGS_OPCO_VERF_RESP,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD8_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

static void
ppsp_impl_ack_serv_msgs_nets(uint8 rslt)
{
}

static void
ppsp_impl_prc_serv_msgs_rand(uint8* msgs)
{
    logs_ent("");
    logs_inf("proc reqs rand keys gens ...");
    // uint8   msgn;
    uint8   rsiz;
    uint8*  rand;
    // ppsp_impl_get_msgs_numb(msgs, msgn);   // mesg id
    ppsp_impl_get_msgs_frsz(msgs, rsiz);   // rand size
    ppsp_impl_get_msgs_plds(msgs, rand);   // rand pointer
    uint8   pids[PPSP_IMPL_CFGS_ALIS_PIDS_COUN];
    uint8   macs[PPSP_IMPL_CFGS_ALIS_MACS_COUN];
    uint8   scrt[PPSP_IMPL_CFGS_ALIS_SCRT_COUN];
    (void)(rand);
    (void)(rsiz);
    ppsp_impl_get_auth_pids(pids);   // load PIDs
    ppsp_impl_get_auth_macs(macs);   // load MACs
    ppsp_impl_get_auth_scrt(scrt);   // load SCRT
    uint8*  cipr = __ppsp_impl_upda_buff;  // ciper
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    osal_memset(__ppsp_impl_auth_keys_data, 0xFF, sizeof(__ppsp_impl_auth_keys_data));
    ppsp_impl_cal_auth_keys(rand, rsiz, pids, sizeof(pids), macs, sizeof(macs), scrt, sizeof(scrt));
    ppsp_impl_enc_text(rand, cipr);
    logs_ver("== rand text givn: ==");
    ppsp_impl_dbg_dump_byte(rand, 16);
    logs_ver("=====================");
    logs_ver("== cryp keys gens: ==");
    ppsp_impl_dbg_dump_byte(__ppsp_impl_auth_keys_data, sizeof(__ppsp_impl_auth_keys_data));
    logs_ver("=====================");
    logs_ver("== encr cipr gens: ==");
    ppsp_impl_dbg_dump_byte(cipr, 16);
    logs_ver("=====================");
    #endif
    /* xfer cipr back to clit, skip vali encr chck, clit will chck */
    ppsp_impl_ack_serv_msgs_cipr(cipr, 16);
}

static void
ppsp_impl_prc_serv_msgs_verf(uint8* msgs)
{
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x01 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8* text_data = __ppsp_impl_upda_buff;
    uint16 text_size = frsz;

    if ( 0x01 == encr )
    {
        ppsp_impl_dec_cipr(text_data, msgs_data);
        msgs_data = text_data;
        msgs_size = text_size;
    }

    #endif
    ppsp_impl_ack_serv_msgs_verf(msgs_data[0]);
}

static void
ppsp_impl_prc_serv_msgs_nets(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc info network status ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x01 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8* text_data = __ppsp_impl_upda_buff;
    uint16 text_size = frsz;

    if ( 0x01 == encr )
    {
        ppsp_impl_dec_cipr(text_data, msgs_data);
        msgs_data = text_data;
        msgs_size = text_size;
    }

    #endif
    logs_inf("info network status: ");
    logs_inf("    network is %s ", (0x00==msgs_data[0])?"FIRM":"EXIT");
    // shall reply a acks for confirmation
    ppsp_impl_ack_serv_msgs_nets(msgs_data[0]);
}

/*****************************************************************************/
// ota relative
// uint8   __ppsp_impl_upda_buff[PPSP_IMPL_CFGS_PROG_BUFF_SIZE];
// uint32  __ppsp_impl_upda_frsz;
uint16  __ppsp_impl_upda_crcs;  // crc from upstream
uint32  __ppsp_impl_upda_offs;  // offset of filled posi

uint32  __ppsp_impl_upda_alln;  // expect sequ numb of next
uint32  __ppsp_impl_upda_seqn;  // expect sequ numb of next


static void
ppsp_impl_rst_appl_devi(void)
{
    __ppsp_impl_msgs_numb = 0xff;

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    if ( 0 == __ppsp_impl_clit_hdlr->ppsp_impl_appl_rset_hdlr )
    {
        logs_war("!! NULL APPL RSET HDLR, USES DEFAULT !!");
        hal_system_soft_reset();

        while ( 1 );
    }

    __ppsp_impl_clit_hdlr->ppsp_impl_appl_rset_hdlr();
    // while ( 1 );
}

static uint16
ppsp_impl_cal_crc16_CCITT_FALSE(uint16 crci, uint8* data, uint32 coun)
{
    uint16 wCRCin = crci;
    uint16 wCPoly = 0x1021;

    while (coun--)
    {
        wCRCin ^= (*(data++) << 8);

        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }

    return (wCRCin);
}

static void
ppsp_impl_ack_serv_msgs_vers(/* uint8 encr,  */uint8 type)
{
    logs_ent("");
    logs_ver("acks trgt firmware version ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    // rply text
    __ppsp_impl_upda_buff[0] = type; // type
    __ppsp_impl_upda_buff[1] = PPSP_IMPL_CFGS_PROG_VERS_REVI; // revision
    __ppsp_impl_upda_buff[2] = PPSP_IMPL_CFGS_PROG_VERS_MINR; // minor
    __ppsp_impl_upda_buff[3] = PPSP_IMPL_CFGS_PROG_VERS_MAJR; // major
    __ppsp_impl_upda_buff[4] = 0x00; // reserved
    __ppsp_impl_upda_frsz    = 5;
    logs_inf("trgt bins type:%x,vers:%d.%d.%d,rsrv:%x",
             // type       // major      // minor      // revision   // reserved
             __ppsp_impl_upda_buff[0],
             __ppsp_impl_upda_buff[3], __ppsp_impl_upda_buff[2], __ppsp_impl_upda_buff[1],
             __ppsp_impl_upda_buff[4]);
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8 encr;
    ppsp_impl_get_auth_rslt(encr);

    if ( 1 == encr )
    {
        uint8   padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_VERS_RESP,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD8_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: acks message of issue requesting target binary update
    para: prmt: permition of update, 1: upda allow, 0: upda deny
          offs: last update position used for resume
          frsz: frame size of a pack
*/
static void
ppsp_impl_ack_serv_msgs_upda(/* uint8 encr,  */uint8 prmt, uint32 offs, uint8 alln)
{
    logs_ent("");
    logs_ver("acks reqs binary update ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    /* resume of last trans point is not supported */
    __ppsp_impl_upda_offs = 0x00000000;
    __ppsp_impl_upda_alln = 0x0f<alln?0x0f:alln;
    __ppsp_impl_upda_seqn = 0;
    logs_inf("trgt asks upda:%02x,offs:%08x,alln:%02x",
             prmt,
             __ppsp_impl_upda_offs,
             __ppsp_impl_upda_alln);
    /* */
    // shared btle xfer buff & otas buff
    // rply text
    __ppsp_impl_upda_buff[0] = prmt; // permition
    __ppsp_impl_upda_buff[1] = (__ppsp_impl_upda_offs>> 0)&0xff; // last
    __ppsp_impl_upda_buff[2] = (__ppsp_impl_upda_offs>> 8)&0xff; // last
    __ppsp_impl_upda_buff[3] = (__ppsp_impl_upda_offs>>16)&0xff; // last
    __ppsp_impl_upda_buff[4] = (__ppsp_impl_upda_offs>>24)&0xff; // last
    __ppsp_impl_upda_buff[5] = __ppsp_impl_upda_alln; // maxi frame in pack(0x00~0x0f:1~16)
    __ppsp_impl_upda_frsz    = 6;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8 encr;
    ppsp_impl_get_auth_rslt(encr);

    if ( 1 == encr )
    {
        uint8  padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_UPDA_RESP,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD8_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: acks message of issue pushing target binary pack
    para: alln: all numb
          seqn: seq numb
          offs: binary size accepted
*/
static void
ppsp_impl_ack_serv_msgs_pack(/* uint8 encr,  */uint8 alln, uint8 seqn, uint32 offs)
{
    logs_ent("");
    logs_ver("acks reqs binary pack ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    logs_inf("trgt acks alln:%02x,seqn:%02x,offs:%08x",
             alln,
             seqn,
             offs);
    /* resume of last trans point is not supported */
    __ppsp_impl_upda_buff[0] = ((alln&0x0F)<<4)|((seqn&0x0F)<<0); //
    __ppsp_impl_upda_buff[1] = (offs>> 0)&0xff; // last
    __ppsp_impl_upda_buff[2] = (offs>> 8)&0xff; // last
    __ppsp_impl_upda_buff[3] = (offs>>16)&0xff; // last
    __ppsp_impl_upda_buff[4] = (offs>>24)&0xff; // last
    __ppsp_impl_upda_frsz    = 5;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8 encr;
    ppsp_impl_get_auth_rslt(encr);

    if ( 1 == encr )
    {
        uint8  padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    0,                              // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_PACK_RESP,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD8_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: acks message of issue pushing target binary pack
    para: rslt: result
*/
static void
ppsp_impl_ack_serv_msgs_comp(/* uint8 encr,  */uint8 rslt)
{
    logs_ent("");
    logs_ver("acks rslt binary comp ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    logs_inf("trgt acks rslt:%02x", rslt);
    /* complete of bins, should resp crcs chck */
    __ppsp_impl_upda_buff[0] = rslt; // 1:succ, 0:fail
    __ppsp_impl_upda_frsz    = 1;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8 encr;
    ppsp_impl_get_auth_rslt(encr);

    if ( 1 == encr )
    {
        uint8  padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    0,                              // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_COMP_RESP,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD8_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

static void
ppsp_impl_prc_serv_msgs_vers(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc reqs firmware version ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x01 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8* text_data = __ppsp_impl_upda_buff;

    if ( 0x01 == encr )
    {
        ppsp_impl_dec_cipr(text_data, msgs_data);
        msgs_data = text_data;
        msgs_size = frsz - text_data[frsz-1];
        // if ( 0x01 != msgs_size )
        // {
        //     logs_err("!! INVALID MESG CONTENT !!");
        //     return;
        // }
    }

    #endif
    logs_inf("reqs vers type:%x",
             // type
             msgs_data[0]);
    ppsp_impl_ack_serv_msgs_vers(/* encr,  */((0x00==msgs_data[0])?0x00:0xff));
}

static void
ppsp_impl_prc_serv_msgs_upda(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc reqs binary update ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    // LOG("msgn = %x\r\n",msgn);
    // LOG("encr = %x\r\n",encr);
    // LOG("frsz = %x\r\n",frsz);
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x0C != frsz) )
    {
        LOG("!! INVALID MESG CONTENT !! \r\n");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8* text_data = __ppsp_impl_upda_buff;

    if ( 0x01 == encr )
    {
        ppsp_impl_dec_cipr(text_data, msgs_data);
        msgs_data = text_data;
        msgs_size = frsz - text_data[frsz-1];
    }

    #endif
    uint32 logv;
    logs_inf("new binary info: ");
    msgs_size  = 0;
    /* copy update type */
    osal_memcpy(&logv, msgs_data+msgs_size, PPSP_IMPL_CFGS_PROG_TYPE_SIZE);
    msgs_size += PPSP_IMPL_CFGS_PROG_TYPE_SIZE;
    logs_inf("    type:%d ",       (uint8)logv);
    /* copy prog vers */
    // __ppsp_impl_clit_hdlr->ppsp_impl_clit_bins_vers = 0xffffffff;
    osal_memcpy((void*)&__ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_vers, msgs_data+msgs_size, PPSP_IMPL_CFGS_PROG_VERS_SIZE);
    msgs_size += PPSP_IMPL_CFGS_PROG_VERS_SIZE;
    osal_memcpy(&logv, msgs_data+msgs_size-PPSP_IMPL_CFGS_PROG_VERS_SIZE, PPSP_IMPL_CFGS_PROG_VERS_SIZE);
    logs_inf("    vers:%08x ",    (uint32)logv);
    /* copy prog size */
    osal_memcpy((void*)&__ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size, msgs_data+msgs_size, PPSP_IMPL_CFGS_PROG_SIZE_SIZE);
    msgs_size += PPSP_IMPL_CFGS_PROG_SIZE_SIZE;
    osal_memcpy(&logv, msgs_data+msgs_size-PPSP_IMPL_CFGS_PROG_SIZE_SIZE, PPSP_IMPL_CFGS_PROG_SIZE_SIZE);
    logs_inf("    size:%d, %08x", (uint32)logv,(uint32)logv);
    /* copy prog crcs */
    osal_memcpy((void*)&__ppsp_impl_upda_crcs, msgs_data+msgs_size, PPSP_IMPL_CFGS_PROG_CRCS_SIZE);
    msgs_size += PPSP_IMPL_CFGS_PROG_CRCS_SIZE;
    osal_memcpy(&logv, msgs_data+msgs_size-PPSP_IMPL_CFGS_PROG_CRCS_SIZE, PPSP_IMPL_CFGS_PROG_CRCS_SIZE);
    logs_inf("    crcs:%08x ",    (uint16)logv);
    /* copy prog flag */
    osal_memcpy(&logv, msgs_data+msgs_size, PPSP_IMPL_CFGS_PROG_FLAG_SIZE);
    msgs_size += PPSP_IMPL_CFGS_PROG_FLAG_SIZE;
    logs_inf("    flag:%d ",       (uint8)logv);

    if ( PPSP_IMPL_CFGS_PROG_FLSH_SIZE < __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size )
    {
        logs_err("!! FIRMWARE SIZE EXCEED LIMIT OF %d BYTES !! \r\n", PPSP_IMPL_CFGS_PROG_FLSH_SIZE);
        return;
    }

    uint32 sctr_numb = (__ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size+PPSP_IMPL_CFGS_PROG_SCTR_SIZE-1)/PPSP_IMPL_CFGS_PROG_SCTR_SIZE;

    for ( uint8 itr0 = 0; itr0 < sctr_numb; itr0 += 1 )
    {
        ppsp_impl_era_prog_data(PPSP_IMPL_CFGS_PROG_ADDR_BGNS + (itr0<<12));
    }

    logs_ver("Eras Flsh @ addr:%08x @ numb:%d", PPSP_IMPL_CFGS_PROG_ADDR_BGNS, sctr_numb);
    /* vers chck shall be make to ensure a upgrade */
    ppsp_impl_ack_serv_msgs_upda(/* encr,  */1, 0, PPSP_IMPL_CFGS_PACK_FRAM_NUMB);
}

static void
ppsp_impl_prc_serv_msgs_pack(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc push of binary pack ...");
    uint8   msgn;
    // uint8   encr;
    uint8   alln, seqn;
    uint8   frsz;
    uint8*  plds;
    ppsp_impl_get_msgs_numb(msgs, msgn);
    // ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_seqn(msgs, alln, seqn);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    if ( seqn != __ppsp_impl_upda_seqn )
    {
        logs_war("mesg numb:rcvd:#X%d, xpct:#X%d", msgn, __ppsp_impl_msgs_numb);
        logs_war("mesg numb:alln:#X%d, seqn:#X%d, xpct:#X%d", alln, seqn, __ppsp_impl_upda_seqn);
        logs_war("!! MSGS LOSS, ALLS DROP !! \r\n",);
        // THIS FRAME AND ALL FOLLOWS WILL BE DROP
        // AND RSPN LAST SUCC UPDA SIZE
        ppsp_impl_ack_serv_msgs_pack(
            /*  #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                encr,
                #else
                0,
                #endif */
            alln,
            __ppsp_impl_upda_seqn,
            __ppsp_impl_upda_offs);
    }
    else
    {
        __ppsp_impl_upda_seqn += 1;
        /* */
        // shared btle xfer buff & otas buff
        uint8* msgs_data = plds;
        uint16 msgs_size = frsz;
        ppsp_impl_psh_prog_data(
            PPSP_IMPL_CFGS_PROG_ADDR_BGNS + __ppsp_impl_upda_offs,
            msgs_data,
            msgs_size);
        __ppsp_impl_upda_offs += msgs_size;

        if ( alln != seqn )
        {
            // SHALL RSPN NO MSG
            logs_ver("wait next binary pack ...");
        }
        else
        {
            __ppsp_impl_upda_seqn =  0;
            ppsp_impl_ack_serv_msgs_pack(
                /*  #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,
                    #else
                    0,
                    #endif */
                alln,
                seqn,
                __ppsp_impl_upda_offs);
        }
    }
}

static void
ppsp_impl_prc_serv_msgs_comp(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc issu of binary comp ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x01 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !! \r\n");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8* text_data = __ppsp_impl_upda_buff;
    uint16 text_size = (uint16)-1;

    if ( 0x01 == encr )
    {
        ppsp_impl_dec_cipr(text_data, msgs_data);
        msgs_data = text_data;
        msgs_size = text_size;
        // if ( 0x01 != msgs_size )
        // {
        //     logs_err("!! INVALID MESG CONTENT !! \r\n");
        //     return;
        // }
    }

    #endif
    logs_inf("issu comp flag:%x",
             // flag
             msgs_data[0]);
    /* calc prog crcs */
    #if 0
    uint32 itr0 = __ppsp_impl_clit_hdlr->ppsp_impl_clit_bins_size;
    uint32 valu;
    uint32 size;
    uint32 addr = __ppsp_impl_clit_hdlr->ppsp_impl_clit_bins_addr;
    uint32 offs = 0;
    uint32 crcs = 0xffff;

    while ( itr0 )
    {
        size = ((itr0 >= sizeof(valu)) ? sizeof(valu) : itr0);
        __ppsp_impl_clit_hdlr->ppsp_impl_prog_read_hdlr(addr + offs, &valu, size);
        crcs = ppsp_impl_cal_crc16_CCITT_FALSE(crcs, (uint8*)&valu, size);
        itr0 -= size;
        offs += size;
    }

    #else
    uint32 size = __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size;
    uint32 addr = PPSP_IMPL_CFGS_PROG_ADDR_BASE+PPSP_IMPL_CFGS_PROG_ADDR_BGNS;//__ppsp_impl_clit_hdlr->ppsp_impl_clit_bins_addr;
    uint32 crcs = 0xffff;
    crcs = ppsp_impl_cal_crc16_CCITT_FALSE(crcs, (uint8*)addr, size);
    #endif
    logs_ver("Calc crcs:%04x, crcd:%04x, fwsz:%08x, dnsz:%08x \r\n",
             __ppsp_impl_upda_crcs,
             crcs,
             __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size,
             __ppsp_impl_upda_offs);
    /* complete of bins, should write tags for bldr */
    logs_ver("complete of bins, will write tags for bldr");
    logs_inf("COMP CRCS: %s", (__ppsp_impl_upda_crcs==crcs)?"SUCC":"FAIL");
    ppsp_impl_ack_serv_msgs_comp(/* encr,  */__ppsp_impl_upda_crcs==crcs);
    logs_inf("!! ============================ !!");
    logs_inf("   PROG OTAs DONE, RESETING ...");
    LOG("COMP CRCS: %s \r\n", (__ppsp_impl_upda_crcs==crcs)?"SUCC":"FAIL");
    LOG("!! ============================ !!");
    LOG("   PROG OTAs DONE, RESETING ...");
    LOG("\r\n\r\n");

    if ( __ppsp_impl_upda_crcs==crcs )
    {
        ppsp_impl_psh_prog_data(PPSP_IMPL_CFGS_PROG_ADDR_BGNS, "OTAF", 4); // tag:"OTAF"
        ppsp_impl_rst_appl_devi();
    }
}



static void
ppsp_impl_mak_clit_msgs_rand(uint8* msgs)
{
    logs_ent("");
    logs_ver("push trgt rand numb ...");
    uint8   rand[16];
    uint8   pids[PPSP_IMPL_CFGS_ALIS_PIDS_COUN];
    uint8   macs[PPSP_IMPL_CFGS_ALIS_MACS_COUN];
    uint8   scrt[PPSP_IMPL_CFGS_ALIS_SCRT_COUN];
    ppsp_impl_get_auth_rand(rand);
    ppsp_impl_get_auth_pids(pids);   // load PIDs
    ppsp_impl_get_auth_macs(macs);   // load MACs
    ppsp_impl_get_auth_scrt(scrt);   // load SCRT
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    osal_memset(__ppsp_impl_auth_keys_data, 0xFF, sizeof(__ppsp_impl_auth_keys_data));
    ppsp_impl_cal_auth_keys(rand, sizeof(rand), pids, sizeof(pids), macs, sizeof(macs), scrt, sizeof(scrt));
    ppsp_impl_enc_text(rand, __ppsp_impl_auth_verf_data);
    logs_ver("== rand text givn: ==");
    ppsp_impl_dbg_dump_byte(rand, 16);
    logs_ver("=====================");
    logs_ver("== cryp keys gens: ==");
    ppsp_impl_dbg_dump_byte(__ppsp_impl_auth_keys_data, sizeof(__ppsp_impl_auth_keys_data));
    logs_ver("=====================");
    logs_ver("== encr cipr gens: ==");
    ppsp_impl_dbg_dump_byte(__ppsp_impl_auth_verf_data, 16);
    logs_ver("=====================");
    #endif
    uint8* msgs_xfer = 0;
    uint8* msgs_data = rand;
    uint16 msgs_size = 16;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    0,                              // flag of encryption
                    PPSP_IMPL_CFGS_OPCO_RAND_ISSU,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD5_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

static void
ppsp_impl_mak_clit_msgs_verf(uint8 rslt)
{
    logs_ent("");
    logs_ver("push trgt auth rslt ...");
    uint8* msgs_xfer = 0;
    uint8* msgs_data = &rslt;
    uint16 msgs_size = 1;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    0,                              // flag of encryption
                    PPSP_IMPL_CFGS_OPCO_VERF_ISSU,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD5_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: make message of issue requesting target curr version
*/
static void
ppsp_impl_mak_clit_msgs_vers(uint8 encr, uint8 type)
{
    logs_ent("");
    logs_ver("reqs trgt firmware version ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    /* */
    // shared btle xfer buff & otas buff
    __ppsp_impl_upda_buff[0] = 0x00;
    __ppsp_impl_upda_frsz    = 1;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        uint8   padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_VERS_ISSU,  // op-code
                    0,                              // segment numb
                    0,                              // seg seq numb
                    msgs_data,                      // payload
                    msgs_size);                     // size

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD5_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: make message of issue requesting target new otas
*/
static void
ppsp_impl_mak_clit_msgs_upda(uint8 encr, uint8 type)
{
    logs_ent("");
    logs_ver("reqs trgt binary update ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    uint32 logv;
    logs_ver("new binary info: ");
    /* */
    __ppsp_impl_upda_buff[0] = 0x00;
    __ppsp_impl_upda_frsz    = 0;
    // issu text
    /* read fw type, fw vers, fw size, fw crcs, fw flag */
    __ppsp_impl_upda_buff[__ppsp_impl_upda_frsz] = type;    // type: 0 full; 1 part
    __ppsp_impl_upda_frsz   += PPSP_IMPL_CFGS_PROG_TYPE_SIZE;
    osal_memcpy(&logv, __ppsp_impl_upda_buff+__ppsp_impl_upda_frsz-PPSP_IMPL_CFGS_PROG_TYPE_SIZE, PPSP_IMPL_CFGS_PROG_TYPE_SIZE);
    logs_ver("    type:%d ", (uint8)logv);
    /* copy prog vers */
    // bin version has not yet implement, make them all 0xff
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_vers = 0xffffffff;
    osal_memcpy(&__ppsp_impl_upda_buff[__ppsp_impl_upda_frsz],
                (const void*)&__ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_vers,
                PPSP_IMPL_CFGS_PROG_VERS_SIZE);
    __ppsp_impl_upda_frsz   += PPSP_IMPL_CFGS_PROG_VERS_SIZE;
    osal_memcpy(&logv, __ppsp_impl_upda_buff+__ppsp_impl_upda_frsz-PPSP_IMPL_CFGS_PROG_VERS_SIZE, PPSP_IMPL_CFGS_PROG_VERS_SIZE);
    logs_ver("    vers:%08x ", logv);
    /* copy prog size */
    osal_memcpy(&__ppsp_impl_upda_buff[__ppsp_impl_upda_frsz],
                (const void*)&__ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size,
                PPSP_IMPL_CFGS_PROG_SIZE_SIZE);
    __ppsp_impl_upda_frsz   += PPSP_IMPL_CFGS_PROG_SIZE_SIZE;
    osal_memcpy(&logv, __ppsp_impl_upda_buff+__ppsp_impl_upda_frsz-PPSP_IMPL_CFGS_PROG_SIZE_SIZE, PPSP_IMPL_CFGS_PROG_SIZE_SIZE);
    logs_ver("    size:%d, %x08x", logv, logv);
    /* calc prog crcs */
    #if 0
    uint32 itr0 = __ppsp_impl_clit_hdlr->ppsp_impl_clit_bins_size;
    uint32 valu;
    uint32 size;
    uint32 addr = __ppsp_impl_clit_hdlr->ppsp_impl_clit_bins_addr;
    uint32 offs = 0;
    uint32 crcs = 0xffff;

    while ( itr0 )
    {
        size = ((itr0 >= sizeof(valu)) ? sizeof(valu) : itr0);
        __ppsp_impl_clit_hdlr->ppsp_impl_prog_read_hdlr(addr + offs, &valu, size);
        crcs = ppsp_impl_cal_crc16_CCITT_FALSE(crcs, (uint8*)&valu, size);
        itr0 -= size;
        offs += size;
    }

    #else
    uint32 size = __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size;
    uint32 addr = __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_addr;
    uint32 crcs = 0xffff;
    crcs = ppsp_impl_cal_crc16_CCITT_FALSE(crcs, (uint8*)addr, size);
    #endif
    osal_memcpy(&__ppsp_impl_upda_buff[__ppsp_impl_upda_frsz], &crcs, PPSP_IMPL_CFGS_PROG_CRCS_SIZE);
    __ppsp_impl_upda_frsz   += PPSP_IMPL_CFGS_PROG_CRCS_SIZE;
    osal_memcpy(&logv, __ppsp_impl_upda_buff+__ppsp_impl_upda_frsz-PPSP_IMPL_CFGS_PROG_CRCS_SIZE, PPSP_IMPL_CFGS_PROG_CRCS_SIZE);
    logs_ver("    crcs:%08x ", logv);
    /* calc prog flag */
    __ppsp_impl_upda_buff[__ppsp_impl_upda_frsz] = 0x00;    // flag
    __ppsp_impl_upda_frsz   += PPSP_IMPL_CFGS_PROG_FLAG_SIZE;
    osal_memcpy(&logv, __ppsp_impl_upda_buff+__ppsp_impl_upda_frsz-PPSP_IMPL_CFGS_PROG_FLAG_SIZE, PPSP_IMPL_CFGS_PROG_FLAG_SIZE);
    logs_ver("    flag:%d ", (uint8)logv);
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        uint8  padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_UPDA_ISSU,
                    0,
                    0,
                    msgs_data,
                    msgs_size);

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    // call under line xfer
    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD5_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: make message of issue pushing target a segment of binary data
*/
static void
ppsp_impl_mak_clit_msgs_pack(uint8 encr)
{
    logs_ent("");
    logs_ver("push trgt binary pack ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    /* copy prog data */
    /* 4byte alligned */
    __ppsp_impl_upda_frsz = ((__ppsp_impl_clit_hdlr->ppsp_impl_appl_mtus - PPSP_IMPL_CFGS_MSGS_HDER_SIZE) & ~0x00000003UL);
    // calc pack size, frsz should less than remaining size of firmware not yet being ota-ed.
    uint32 size = __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size - __ppsp_impl_upda_offs;

    if ( __ppsp_impl_upda_frsz > size )
    {
        __ppsp_impl_upda_frsz = size;
    }

    ppsp_impl_pul_prog_data(__ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_addr + __ppsp_impl_upda_offs,
                            __ppsp_impl_upda_buff,
                            __ppsp_impl_upda_frsz);
    logs_inf("make pack @ addr:0x%08x of byts:%d,0x%08x",
             __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_addr + __ppsp_impl_upda_offs,
             __ppsp_impl_upda_frsz, __ppsp_impl_upda_frsz);
    uint8* msgs_xfer = 0;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    0,
                    PPSP_IMPL_CFGS_OPCO_PACK_ISSU,
                    __ppsp_impl_upda_alln,
                    __ppsp_impl_upda_seqn,
                    msgs_data,
                    msgs_size);

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD5_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
    /* prep for next pack */
    __ppsp_impl_upda_offs += __ppsp_impl_upda_frsz;

    if ( __ppsp_impl_upda_seqn >= __ppsp_impl_upda_alln )
        __ppsp_impl_upda_seqn  = 0;
    else
        __ppsp_impl_upda_seqn += 1;
}

/*
    desc: make message of issue acking target complete of pack xfer
*/
static void
ppsp_impl_mak_clit_msgs_comp(uint8 encr)
{
    logs_ent("");
    logs_ver("acks trgt binary comp ...");

    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    /* */
    __ppsp_impl_upda_buff[0] = 0x01;
    __ppsp_impl_upda_frsz    = 1;
    uint8* msgs_data = __ppsp_impl_upda_buff;
    uint16 msgs_size = __ppsp_impl_upda_frsz;
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        uint8  padd_valu;
        ppsp_impl_get_pkcs_7pad(16, msgs_size, padd_valu);
        osal_memset(msgs_data+msgs_size, padd_valu, padd_valu);
        ppsp_impl_enc_text(msgs_data, msgs_data);
        // msgs_data =     // msgs_data now been encrypted
        msgs_size = 16; // msgs_size cipr size
    }

    #endif
    uint8* msgs_xfer = 0;
    msgs_xfer = ppsp_impl_new_msgs_raws(
                    __ppsp_impl_msgs_numb,          // msg numb, auto incr
                    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
                    encr,                              // flag of encryption
                    #else
                    0,                              // flag of encryption
                    #endif
                    PPSP_IMPL_CFGS_OPCO_COMP_ISSU,
                    0,
                    0,
                    msgs_data,
                    msgs_size);

    if ( 0 == msgs_xfer )
    {
        logs_err("!! NEW XFER MSGS FAIL, SKIP !!");
        return;
    }

    __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr(
        PPSP_SERV_CFGS_CHAR_FFD5_INDX,
        msgs_xfer,
        PPSP_IMPL_CFGS_MSGS_HDER_SIZE+msgs_size);
    osal_mem_free(msgs_xfer);
}

/*
    desc: proc message of response acking cipr of rand
*/
static void
ppsp_impl_prc_clit_msgs_cipr(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc trgt cipr of rand ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    /* msgs header validation check */
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x10 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_data);
    (void)(msgs_size);
    uint8 rslt = 1; // default failure
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
    uint8* verf_data = __ppsp_impl_auth_verf_data;
    uint16 verf_size = sizeof(__ppsp_impl_auth_verf_data);

    if ( msgs_size == verf_size )
    {
        rslt = !osal_memcmp(verf_data, msgs_data, verf_size);
    }

    #endif
    /* issue */
    ppsp_impl_mak_clit_msgs_verf(rslt);
}

/*
    desc: proc message of response acking verf rslt
*/
static void
ppsp_impl_prc_clit_msgs_verf(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc trgt acks of verf rslt ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    /* msgs header validation check */
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x01 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    logs_inf("trgt verf rslt:%d, %s", msgs_data[0], (0==msgs_data[0])?"SUCC":"FAIL");
    logs_inf("!! AUTHORIZATION COMPLETE !!");

    /*  */
    if ( 0 != msgs_data[0] )
    {
        // ends link
        ppsp_impl_set_auth_rslt(0);
    }
    else
    {
        ppsp_impl_set_auth_rslt(1);
        /* issue */
        // ppsp_impl_mak_clit_msgs_upda(0);
        ppsp_impl_mak_clit_msgs_vers(1, 0);
    }
}

/*
    desc: proc message of response acking curr version of target
*/
static void
ppsp_impl_prc_clit_msgs_vers(uint8* msgs)
{
    logs_ent("");
    logs_ver("proc trgt firmware version ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    /* msgs header validation check */
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x05 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        ppsp_impl_dec_cipr(__ppsp_impl_upda_buff, msgs_data);
        msgs_data = __ppsp_impl_upda_buff;
        msgs_size = frsz - msgs_data[frsz-1];
    }

    #endif
    logs_inf("trgt bins type:%x,vers:%d.%d.%d,rsrv:%x",
             // type       // major      // minor      // revision   // reserved
             msgs_data[0], msgs_data[3], msgs_data[2], msgs_data[1], msgs_data[4]);
    /* issue */
    // type: 0 full; 1 part
    ppsp_impl_mak_clit_msgs_upda(encr, 0);
}

/*
    desc: proc message of response replying binary otas request
*/
static void
ppsp_impl_prc_clit_msgs_upda(uint8* msgs)
{
    logs_ent("");
    logs_ver("trgt rspn firmware upda ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    /* msgs header validation check */
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x06 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        ppsp_impl_dec_cipr(__ppsp_impl_upda_buff, msgs_data);
        msgs_data = __ppsp_impl_upda_buff;
        msgs_size = frsz - msgs_data[frsz-1];
    }

    #endif
    /* copy last ota-ed posi */
    // last upda posi
    osal_memcpy((void*)&__ppsp_impl_upda_offs, (void*)&msgs_data[1], sizeof(__ppsp_impl_upda_offs));
    /* copy maxi frame numb of packing */
    __ppsp_impl_upda_alln = msgs_data[5]; // maxi frame in pack(0x00~0x0f:1~16)
    __ppsp_impl_upda_seqn = 0;
    logs_inf("trgt rspn upda:%02x,offs:%08x,alln:%02x",
             msgs_data[0], __ppsp_impl_upda_offs, __ppsp_impl_upda_alln);

    /* otas reqs deny by trgt, skip */
    // 1: upda allow, 0: upda deny
    if ( 0 == msgs_data[0] )
    {
        logs_war("trget DO NOT want firmware update, SKIP !!");
        return;
    }

    /* trgt want to upda, & timr available */
    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    if ( 0 == __ppsp_impl_clit_hdlr->ppsp_impl_appl_timr_hdlr )
    {
        logs_err("!! NULL TIMR HDLR, SKIP !!");
        return;
    }

    __ppsp_impl_clit_hdlr->ppsp_impl_appl_timr_hdlr();
}

/*
    desc: proc message of response acking packing position
*/
static void
ppsp_impl_prc_clit_msgs_pack(uint8* msgs)
{
    logs_ent("");
    logs_ver("trgt rspn firmware pack ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    /* msgs header validation check */
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x05 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        ppsp_impl_dec_cipr(__ppsp_impl_upda_buff, msgs_data);
        msgs_data = __ppsp_impl_upda_buff;
        msgs_size = frsz - msgs_data[frsz-1];
    }

    #endif
    /* complete of a package, should resp rcvd seqn, rcvd size */
    uint8   alln, seqn;
    alln = ((msgs_data[0] >> 4) & 0x0F);
    seqn = ((msgs_data[0] >> 0) & 0x0F); //
    /* we will use last posi rspn to re-posi streaming in case of FRAME-LOSS or SUSPEND-RESUME */
    // last upda posi
    osal_memcpy(&__ppsp_impl_upda_offs, &msgs_data[1], sizeof(__ppsp_impl_upda_offs));
    LOG("[INF] trgt acks pack: alln:%02x,seqn:%02x,dnsz:%08x \r\n",
        alln,     seqn,     __ppsp_impl_upda_offs);

    /* trgt want to upda, & timr available */
    if ( 0 == __ppsp_impl_clit_hdlr )
    {
        logs_err("!! NULL APPL HDLR, SKIP !!");
        return;
    }

    if ( 0 == __ppsp_impl_clit_hdlr->ppsp_impl_appl_timr_hdlr )
    {
        logs_err("!! NULL TIMR HDLR, SKIP !!");
        return;
    }

    if ( __ppsp_impl_upda_offs < __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size )
    {
        __ppsp_impl_clit_hdlr->ppsp_impl_appl_timr_hdlr();
    }
    else
    {
        ppsp_impl_mak_clit_msgs_comp(
            #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
            1
            #else
            0
            #endif
        );
    }
}

/*
    desc: proc message of response acking comp with crcs rslt
*/
static void
ppsp_impl_prc_clit_msgs_comp(void* msgs)
{
    logs_ent("");
    logs_ver("trgt rspn comp with crcs rslt ...");
    // uint8   msgn;
    uint8   encr;
    uint8   frsz;
    uint8*  plds;
    // ppsp_impl_get_msgs_numb(msgs, msgn);
    ppsp_impl_get_msgs_encr(msgs, encr);
    ppsp_impl_get_msgs_frsz(msgs, frsz);
    ppsp_impl_get_msgs_plds(msgs, plds);

    /* msgs header validation check */
    if ( (0x01 == encr && 0x10 != frsz) || (0x00 == encr && 0x01 != frsz) )
    {
        logs_err("!! INVALID MESG CONTENT !!");
        return;
    }

    uint8* msgs_data = plds;
    uint16 msgs_size = frsz;
    (void)(msgs_size);
    #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)

    if ( 1 == encr )
    {
        ppsp_impl_dec_cipr(__ppsp_impl_upda_buff, msgs_data);
        msgs_data = __ppsp_impl_upda_buff;
        msgs_size = frsz - msgs_data[frsz-1];
    }

    #endif

    /* msgs payload validation check */

    if ( 0x01 == msgs_data[0] )
    {
    }
    else
    {
    }

    logs_inf("trgt rspn comp with %s", 0x01 == msgs_data[0] ? "SUCC" : "FAIL");
}

static void
ppsp_impl_serv_rmsg_hdlr(uint8 para, const void* valu, uint16 coun)
{
    logs_ent("para:%d, valu:%08x, coun:%d", para, valu, coun);
    // ppsp_impl_dbg_dump_byte(valu, coun);

    if ( SUCCESS != ppsp_impl_chk_msgs_vali(valu, coun) )
    {
        logs_err("!! RCVD INVL MSGS, SKIP !!");
        return;
    }

    uint8 msgn;
    uint8 opco;
    ppsp_impl_get_msgs_numb(valu, msgn);
    ppsp_impl_get_msgs_opco(valu, opco);
    __ppsp_impl_msgs_numb = msgn;

    if ( PPSP_IMPL_CFGS_OPCO_RAND_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_rand((uint8*)valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_VERF_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_verf((uint8*)valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_NWRK_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_nets((uint8*)valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_VERS_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_vers((uint8*)valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_UPDA_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_upda((uint8*)valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_PACK_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_pack((uint8*)valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_COMP_ISSU == opco )
    {
        ppsp_impl_prc_serv_msgs_comp((uint8*)valu);
    }
    else
    {
        logs_war("!! SOON WILL IMPL, SKIP !!");
    }

    if ( 15 <= __ppsp_impl_msgs_numb )
        __ppsp_impl_msgs_numb  = 0;
    else
        __ppsp_impl_msgs_numb += 1;
}

/*
    desc: ppsp received msgs handler on client end
*/
static void
ppsp_impl_clit_rmsg_hdlr(uint8 para, void* valu, uint16 coun)
{
    logs_ent("para:%d, valu:%08x, coun:%d \r\n", para, valu, coun);
    logs_dmp(valu, coun);

    if ( SUCCESS != ppsp_impl_chk_msgs_vali(valu, coun) )
    {
        logs_err("!! RCVD INVL MSGS, SKIP !!");
        return;
    }

    // if ( 15 <= __ppsp_impl_msgs_numb )
    //     __ppsp_impl_msgs_numb  = 0;
    // else
    //     __ppsp_impl_msgs_numb += 1;
    uint8 opco;
    ppsp_impl_get_msgs_opco(valu, opco);

    if ( PPSP_IMPL_CFGS_OPCO_CIPR_RESP == opco )
    {
        ppsp_impl_prc_clit_msgs_cipr(valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_VERF_RESP == opco )
    {
        ppsp_impl_prc_clit_msgs_verf(valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_VERS_RESP == opco )
    {
        ppsp_impl_prc_clit_msgs_vers(valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_UPDA_RESP == opco )
    {
        ppsp_impl_prc_clit_msgs_upda(valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_PACK_RESP == opco )
    {
        ppsp_impl_prc_clit_msgs_pack(valu);
    }
    else if ( PPSP_IMPL_CFGS_OPCO_COMP_RESP == opco )
    {
        ppsp_impl_prc_clit_msgs_comp(valu);
    }
    else
    {
        logs_war("!! SOON WILL IMPL, SKIP !!");
    }
}

static void
ppsp_impl_clit_tout_hdlr(void)
{
    logs_ent("");

    if ( __ppsp_impl_upda_offs < __ppsp_impl_clit_hdlr->ppsp_impl_appl_bins_size )
    {
        ppsp_impl_mak_clit_msgs_pack(0);

        if ( 0 <  __ppsp_impl_upda_seqn )
        {
            if ( 0 != __ppsp_impl_clit_hdlr )
                if ( 0 != __ppsp_impl_clit_hdlr->ppsp_impl_appl_timr_hdlr )
                {
                    __ppsp_impl_clit_hdlr->ppsp_impl_appl_timr_hdlr();
                }
        }
    }
    else                // firmwawre download complete
    {
        ppsp_impl_mak_clit_msgs_comp(
            #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
            1
            #else
            0
            #endif
        );
    }
}

int32
ppsp_impl_reg_serv_appl( ppsp_impl_clit_hdlr_t* clit_hdlr )
{
    ppsp_impl_set_auth_rslt(0);
    __ppsp_impl_msgs_numb = 0xff;

    if ( clit_hdlr )
    {
        __ppsp_impl_clit_hdlr = clit_hdlr;
        __ppsp_impl_clit_hdlr->ppsp_impl_appl_writ_hdlr = (void(*)())ppsp_serv_set_para;    // func type casting
        // __ppsp_impl_clit_hdlr->ppsp_impl_appl_noti_hdlr = ppsp_impl_serv_rmsg_hdlr;
        __ppsp_serv_appl_hdlr.ppsp_serv_char_upda_hdlr = ppsp_impl_serv_rmsg_hdlr;
        ppsp_serv_reg_appl(&__ppsp_serv_appl_hdlr);
        return ( SUCCESS );
    }
    else
    {
        //
        return ( FAILURE );
    }
}

int32
ppsp_impl_reg_clit_appl( ppsp_impl_clit_hdlr_t* clit_hdlr )
{
    ppsp_impl_set_auth_rslt(0);
    __ppsp_impl_msgs_numb = 0x00;

    if ( clit_hdlr )
    {
        __ppsp_impl_clit_hdlr = clit_hdlr;
        __ppsp_impl_clit_hdlr->ppsp_impl_appl_noti_hdlr = ppsp_impl_clit_rmsg_hdlr;
        __ppsp_impl_clit_hdlr->ppsp_impl_appl_tout_hdlr = ppsp_impl_clit_tout_hdlr;
        #if (1 == PPSP_IMPL_CFGS_MSGS_CRYP_ENAB)
        ppsp_impl_mak_clit_msgs_rand(0);
        #else
        ppsp_impl_mak_clit_msgs_vers(0, 0);
        #endif
        return ( SUCCESS );
    }
    else
    {
        //
        return ( FAILURE );
    }
}
