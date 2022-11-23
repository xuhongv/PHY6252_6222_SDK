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

/*******************************************************************************
    @file     fs.c
    @brief    Contains all functions support for spi driver
    @version  0.0
    @date     18. Oct. 2017
    @author



*******************************************************************************/
#include "rom_sym_def.h"
#include "OSAL.h"
#include "fs.h"
#include "flash.h"
#include "error.h"
#include "log.h"

//#define FS_DBBUG
#ifdef FS_DBBUG
    #define FS_LOG(...)  {LOG(__VA_ARGS__);LOG("\r\n");}
#else
    #define FS_LOG(...)
#endif

#define FS_RET_FS_REPARED                     ((int)0x7fff0001)

/* FS_ITEM_LEN should be 16, 32, or 64 */

#ifndef FS_ITEM_LEN
    #define FS_ITEM_LEN                                                             16
#endif

/*********************************

    fs struct:
    for data sector:
    sector_head(16)
    file_head(4byte)+file_data
    ...
    file_head(4byte)+file_data

    for exchange sector:
    sector_head_exch
    fffffffffffffffffffffffffff
    ...
    fffffffffffffffffffffffffff

**********************************/


/*>>>>>>>>>>>>>>>> please DO NOT modify the following parameters >>>>>>>>>>>>>>>*/
#define FS_FSIZE_MAX                        4095
#define FS_SECTOR_SIZE                      4096
#define FS_ITEM_HEAD_LEN                    4
#define FS_ITEM_DATA_LEN                    (FS_ITEM_LEN - FS_ITEM_HEAD_LEN)
#define FS_ITEMS_PER_SECT                   (FS_SECTOR_SIZE/FS_ITEM_LEN - 1)
#define FS_ABSADDR(offset)                  (s_fs_ctx.cfg.fs_addr + offset)
#define FS_ITEMADDR(sectaddr, item_idx)     (sectaddr + (item_idx+1)*FS_ITEM_LEN)
/*<<<<<<<<<<<<<<<<    please DO NOT modify the up parameters     <<<<<<<<<<<<<<<*/



typedef enum     /*state machine for fs analyze*/
{
    FS_ANALYZE_UNCHECK       = 0,//before analysis fs
    FS_ANALYZE_NEW           = 1,//new fs,all data should be 0xFF, need do create
    FS_ANALYZE_NORMAL        = 2,//fs initialized, can access normally
    FS_ANALYZE_ERROR         = 4,//fs initialized, but data is broken, need do rebuild
} fs_analyze_st;

typedef enum
{
    ITEM_DEL      = 0x00,//zone is deleted
    ITEM_RESERVED = 0x01,//to be extend
    ITEM_USED     = 0x02,//zone is used
    ITEM_UNUSED   = 0x03,//zone is free
} item_pro;

typedef enum
{
    ITEM_MF_E   = 0x00,//multiple frame file,end frame
    ITEM_MF_F   = 0x01,//multiple frame file,first frame
    ITEM_MF_C   = 0x02,//multiple frame file,continue frame
    ITEM_SF     = 0x03,//single frame file
} item_frame;

/*
    file head struct:
    len(12bit)+frame(2bit)+pro(2bit)+id(16bit)
*/
typedef union
{
    struct
    {
        uint32_t id:16;//file id
        uint32_t pro:2;//file property
        uint32_t frame:2;//file frame
        uint32_t len:12;//file length
    } b;
    uint32_t reg;
} fs_item_t;

/*sector head struct: */
/*fs_addr(one word)+(ff+index+item_len+fs_snum)(one word)+(0xffffffff)(one word)~(0xffffffff)(one word)*/
/*when do exchanging, EXCH_TAG>>DATA_TAG, DATA_TAG>>EXCHING_TAG*/
#define FS_SECT_NEW_TAG       0xffffffff   /*uninited sector*/
#define FS_SECT_DATA_TAG      0x48485346   /*"FSHH"*/
#define FS_SECT_EXCH_TAG      0x58485346   /*"FSHX"*/
#define FS_SECT_EXCHING_TAG   0x40485346   /*"FSH@", for doing exchange*/
typedef struct
{
    uint32_t tag;       //data sector: "FSH@"; exch sector "FSHX"
    uint32_t fs_addr;   //fs start address
    uint8_t  fs_snum;   //fs sector number
    uint8_t  item_len;  //item length
    uint8_t  index;     //sector index
} fs_cfg_t;

typedef struct
{
    fs_cfg_t    cfg;
    bool        init_flg;   //flag for fs is initialized
    uint8_t     cur_sect;   //free sector index
    uint8_t     exch_sect;  //exchange sector,only use it when garbage collect
    uint16_t    cur_item;   //free position in free sector index
} fs_ctx_t;

fs_ctx_t s_fs_ctx =
{
    .init_flg = false,
    .cur_sect = 0,
    .exch_sect = 0,
    .cur_item = 0,
};

#define FS_DBG_TRIGGER
#ifdef FS_DBG_TRIGGER

    uint32_t fs_break_pos  =  0;
    uint8_t fs_break_cnt_trigger  = 0;
    uint8_t fs_break_cnt = 0;

    #define __fs_break(pos) {if(fs_break_pos == pos){if(fs_break_cnt==fs_break_cnt_trigger) NVIC_SystemReset(); fs_break_cnt ++;}}
    //#define __fs_break(pos, cnt) {if(fs_break_pos == pos){if(fs_break_cnt==fs_break_cnt_trigger && cnt >0) while(1){;} fs_break_cnt ++;}}

#else
    #define __fs_break(pos)
#endif

extern uint32_t __psr(void);//check if in int process

static void fs_erase_all_sector(void)
{
    int i;
    fs_cfg_t* pcfg = &(s_fs_ctx.cfg);

    for(i=0; i<pcfg->fs_snum; i++)
        hal_flash_erase_sector(pcfg->fs_addr+ (i*FS_SECTOR_SIZE));
}

static void fs_erase_sector(uint32_t addr_erase)
{
    hal_flash_erase_sector(addr_erase);
}

static int fs_spif_write(uint32_t addr,uint8_t* value,uint16_t len)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);

    if(__psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if((addr < pcfg->fs_addr) || (addr >= (pcfg->fs_addr+pcfg->fs_snum*FS_SECTOR_SIZE)) || ((addr&0x03)>0))
    {
        return PPlus_ERR_FS_PARAMETER;
    }

    return(hal_flash_write(addr,value,(uint32_t)len));
}

static uint32_t fs_spif_read(uint32_t addr,uint8_t* buf, uint16_t len)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);

    if(__psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    if ((addr < pcfg->fs_addr) || (addr >= (pcfg->fs_addr + pcfg->fs_snum*FS_SECTOR_SIZE)) || (buf == NULL) || (len == 0))
    {
        return PPlus_ERR_FS_PARAMETER;
    }

    return( hal_flash_read(addr,buf,len) );
}

static void check_sector_order(void)
{
    #ifdef FS_CHECK_SECT_ORDER
    int i, j;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    fs_cfg_t sect_cfg;
    uint8_t nxt_idx = 0xff;

    for(i = 0; i < pcfg->fs_snum; i++)
    {
        fs_spif_read(FS_ABSADDR(FS_SECTOR_SIZE*i),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));

        if(sect_cfg.tag == FS_SECT_DATA_TAG)
        {
            nxt_idx = (sect_cfg.index + 1);

            if(nxt_idx == pcfg->fs_snum -1)
                nxt_idx = 0xff;
        }
        else if(sect_cfg.tag == FS_SECT_EXCH_TAG)
        {
            nxt_idx = 0;
        }
        else if(sect_cfg.tag == FS_SECT_NEW_TAG)
        {
            break;
        }
        else
        {
            LOG("[Error] check_sector_order, bad head\r\n");

            while(1) {;}
        }

        j = (i + 1) % pcfg->fs_snum;
        fs_spif_read(FS_ABSADDR(FS_SECTOR_SIZE*j),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));

        if(nxt_idx != sect_cfg.index)
        {
            LOG("[Error] check_sector_order\r\n");

            while(1) {;}
        }
    }

    #endif
}

static void check_addr(uint32_t* addr)
{
    if((*addr % FS_SECTOR_SIZE) == 0)
    {
        *addr += FS_ITEM_LEN;

        if(*addr >= FS_SECTOR_SIZE *s_fs_ctx.cfg.fs_snum)
            *addr -= FS_SECTOR_SIZE *s_fs_ctx.cfg.fs_snum;
    }
}


#define  FS_GET_SECT_ID(index) ((index + 1 + s_fs_ctx.exch_sect) % s_fs_ctx.cfg.fs_snum)
#define FS_SECT_BOUNDARY_CALC(j, file_items) \
    if(j + file_items>= FS_ITEMS_PER_SECT){file_items -= FS_ITEMS_PER_SECT-(j+1); j = FS_ITEMS_PER_SECT;}\
    else{j += file_items-1;}




static int fs_search_items_using(uint16_t fid, uint32_t* faddr)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t i, cur_sect;
    uint16_t j;
    uint32_t file_items = 1;  //a file at lease have one item
    uint32_t sector_addr,item_addr;
    fs_item_t item;

    for(i = 0; i < pcfg->fs_snum-1; i++)
    {
        cur_sect = FS_GET_SECT_ID(i);

        if(file_items >= FS_ITEMS_PER_SECT)
        {
            file_items -= FS_ITEMS_PER_SECT;
            continue;
        }

        sector_addr = cur_sect * FS_SECTOR_SIZE;

        for(j = file_items-1; j < FS_ITEMS_PER_SECT; j++)
        {
            item_addr = FS_ITEMADDR(sector_addr, j);
            fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)&item,FS_ITEM_HEAD_LEN);

            switch(item.b.pro)
            {
            case ITEM_DEL:
            case ITEM_USED:
                if((ITEM_USED == item.b.pro) && (item.b.id == fid))
                {
                    *faddr = item_addr;
                    return PPlus_SUCCESS;
                }
                else
                {
                    if(item.b.frame == ITEM_MF_F)
                    {
                        file_items = (item.b.len + (FS_ITEM_DATA_LEN-1)) / FS_ITEM_DATA_LEN;
                        FS_SECT_BOUNDARY_CALC(j, file_items);
                    }
                    else
                    {
                        file_items =1;
                    }
                }

                break;

            case ITEM_UNUSED:
                return PPlus_ERR_FS_NOT_FIND_ID;

            default:
                break;
            }
        }
    }

    return PPlus_ERR_FS_NOT_FIND_ID;
}

static int fs_search_items_deleted(uint32_t* del_size,uint32_t* del_num)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t i, cur_sect;
    uint16_t j;
    uint32_t file_items = 1;  //a file at lease have one item
    uint32_t sector_addr,item_addr;
    fs_item_t item;

    for(i = 0; i < pcfg->fs_snum-1; i++)
    {
        cur_sect = FS_GET_SECT_ID(i);

        if(file_items >= FS_ITEMS_PER_SECT)
        {
            file_items -= FS_ITEMS_PER_SECT;
            continue;
        }

        sector_addr = cur_sect * FS_SECTOR_SIZE;

        for(j = file_items - 1 ; j < FS_ITEMS_PER_SECT; j++)
        {
            item_addr = FS_ITEMADDR(sector_addr, j);
            fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)&item,FS_ITEM_HEAD_LEN);

            switch(item.b.pro)
            {
            case ITEM_DEL:
            {
                if(item.b.frame == ITEM_MF_F)
                {
                    file_items = (item.b.len + (FS_ITEM_DATA_LEN-1)) / FS_ITEM_DATA_LEN;
                    *del_size += file_items*FS_ITEM_DATA_LEN;
                    *del_num += 1;
                    FS_SECT_BOUNDARY_CALC(j, file_items);
                }
                else
                {
                    file_items = 1;
                    *del_size += FS_ITEM_DATA_LEN;
                    *del_num += 1;
                }
            }
            break;

            case ITEM_USED:
            {
                if(item.b.frame == ITEM_MF_F)
                {
                    file_items = (item.b.len + (FS_ITEM_DATA_LEN-1)) / FS_ITEM_DATA_LEN;
                    FS_SECT_BOUNDARY_CALC(j, file_items);
                }
                else
                {
                    file_items = 1;
                }
            }
            break;

            case ITEM_UNUSED:
                return PPlus_SUCCESS;

            default:
                file_items = 1;
                break;
            }
        }
    }

    return PPlus_SUCCESS;
}


static int fs_del_bad_file_chain(uint8_t* sect_map, uint32_t item_addr, uint8_t item_num)
{
    int i,ret;
    fs_item_t item_hd =
    {
        .b.id = 0,
        .b.pro = ITEM_DEL,
        .b.frame = ITEM_MF_F,
        .b.len = item_num* FS_ITEM_DATA_LEN,
    };

    for (i = 0; i< item_num; i++)
    {
        ret = fs_spif_write(FS_ABSADDR(item_addr),(uint8_t*)(&item_hd),sizeof(item_hd));

        if(ret)
            return ret;

        item_hd.b.frame = ITEM_MF_C,
        check_addr(&item_addr);
    }

    return PPlus_SUCCESS;
}

/*repare function, just when file write not completed*/
static int fs_repare_normal(uint8_t* sect_map)
{
    uint8_t i = 0, j;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    bool item_st = true; //true: used or deleted items; false: should be free items
    uint32_t sect_addr,item_addr;
    uint32_t item_addr_bad;
    uint8_t num_bad;
    uint16_t tail = 0;
    fs_item_t item_hd;
    pctx->cur_sect = sect_map[0];

    for(i = 0; i< pcfg->fs_snum-1; i++)
    {
        sect_addr = FS_SECTOR_SIZE*sect_map[i];

        for(j = 0; j < FS_ITEMS_PER_SECT; j++)
        {
            item_addr = FS_ITEMADDR(sect_addr, j);
            fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)(&item_hd),sizeof(item_hd));

            if(item_st)
            {
                if(item_hd.b.pro == ITEM_DEL || item_hd.b.pro == ITEM_USED)
                {
                    pctx->cur_sect = sect_map[0];

                    if(item_hd.b.frame == ITEM_SF && tail == 0)
                    {
                        continue;
                    }
                    else if(item_hd.b.frame == ITEM_MF_F && tail == 0)
                    {
                        item_addr_bad = item_addr;
                        tail = (item_hd.b.len + FS_ITEM_DATA_LEN -1)/ FS_ITEM_DATA_LEN;
                        num_bad = tail;
                        tail --;
                    }
                    else if((item_hd.b.frame == ITEM_MF_C || item_hd.b.frame == ITEM_MF_E) && tail)
                    {
                        tail --;
                    }
                    else
                    {
                        //repare;
                        fs_del_bad_file_chain(sect_map,item_addr_bad, num_bad);
                        tail --;
                    }
                }
                else if(item_hd.b.pro == ITEM_RESERVED)
                {
                    pctx->cur_sect = sect_map[0];
                }
                else if(item_hd.reg == 0xffffffff)
                {
                    if(tail)
                    {
                        fs_del_bad_file_chain(sect_map,item_addr_bad, num_bad);
                        tail --;
                    }

                    item_st = false;
                    pctx->cur_item = j;
                }
            }
            else /*item_st == false, empty area, all FF*/
            {
                return PPlus_ERR_FATAL;
            }
        }
    }

    return FS_RET_FS_REPARED;
}

/*repare function, rebuild when do garbage not completed*/
/*case 1: old data sector >> exchanging sector, exch sector >> data sector,*/
/*case 2: old data sector erased, but did not set to exchange sector*/
/*case 3: old data sector erased and set to exchange sector, no exchanging sector*/
static int fs_repare_gcfail(uint8_t* sect_map)
{
    int ret;
    int i, j;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t   new_1st_sect = 0xff;
    fs_cfg_t  sect_cfg;
    uint8_t from_sect_id = 0xff, to_sect_id = 0xff, to_item_id = 0xff;
    uint32_t from_addr=0,to_addr=0;
    uint16_t last_to_fid = 0xffff;
    uint16_t last_to_item_off = 0;
    uint32_t item_addr;
    uint32_t addr_wr;
    fs_item_t item_hd;
    fs_cfg_t from_sect_cfg;
    fs_cfg_t to_sect_cfg;

    //step 0:
    //find id_0 data sector, if none,
    //means just set FS_SECT_EXCHING_TAG,
    //old exchange sect have not set
    for(i = 0; i < pcfg->fs_snum; i++)
    {
        fs_spif_read(FS_ABSADDR(FS_SECTOR_SIZE*i),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));

        if(sect_cfg.tag == FS_SECT_DATA_TAG && sect_cfg.index == 0)
        {
            new_1st_sect = i;
            break;
        }

        if(sect_cfg.tag == FS_SECT_EXCHING_TAG && sect_cfg.index == 0)
        {
            new_1st_sect = (i + pcfg->fs_snum -1) % pcfg->fs_snum;
            break;
        }
    }

    if(new_1st_sect ==  0xff) //need figure out from FS_SECT_EXCHING_TAG
    {
        return PPlus_ERR_FATAL; //unrecoverable problem, need format
    }

    //step1:
    //find to_addr, search from new 0 sect
    //get last file id
    to_sect_id = new_1st_sect;

    for(i = 0; i < pcfg->fs_snum -1; i++)
    {
        to_addr = FS_SECTOR_SIZE * to_sect_id;
        fs_spif_read(FS_ABSADDR(to_addr),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));

        if(sect_cfg.tag != FS_SECT_DATA_TAG)
            break;

        for(j = 0; j < FS_ITEMS_PER_SECT; j++)
        {
            item_addr = FS_ITEMADDR(to_addr, j);
            fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)(&item_hd),sizeof(item_hd));

            if(item_hd.reg == FS_SECT_NEW_TAG)
            {
                to_item_id = j;
                break;
            }

            if(item_hd.b.pro != ITEM_USED)
            {
                return PPlus_ERR_FATAL;
            }

            if(item_hd.b.frame == ITEM_SF)
            {
                last_to_fid = item_hd.b.id;
                last_to_item_off = 0;
            }
            else
            {
                if(last_to_fid != item_hd.b.id)
                {
                    last_to_fid = item_hd.b.id;
                    last_to_item_off = 0;
                }
                else
                {
                    last_to_item_off ++;
                }
            }
        }

        if(item_hd.reg == FS_SECT_NEW_TAG)
            break;

        to_sect_id = (to_sect_id+1) % pcfg->fs_snum;
    }

    //step2:
    //figure out from sect and to sect
    //find witch is FS_SECT_EXCHING_TAG or FS_SECT_NEW_TAG or FS_SECT_EXCH_TAG
    //exchanging is working sect, if new, should to next sect
    //if exch, need set to data
    from_sect_id = (new_1st_sect + pcfg->fs_snum +1) % pcfg->fs_snum;

    for(i = 0; i < pcfg->fs_snum - 1; i++)
    {
        fs_spif_read(FS_ABSADDR(FS_SECTOR_SIZE*from_sect_id),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));

        if(sect_cfg.tag == FS_SECT_EXCHING_TAG)
        {
            break;
        }
        else if(sect_cfg.tag == FS_SECT_NEW_TAG)
        {
            from_sect_cfg.tag = FS_SECT_EXCH_TAG;
            from_sect_cfg.fs_addr = pcfg->fs_addr;
            from_sect_cfg.fs_snum = pcfg->fs_snum;
            from_sect_cfg.item_len = pcfg->item_len;
            from_sect_cfg.index = 0xff;
            ret = fs_spif_write(FS_ABSADDR(FS_SECTOR_SIZE*from_sect_id),(uint8_t*)(&from_sect_cfg),sizeof(fs_cfg_t));

            if(ret) return PPlus_ERR_FS_WRITE_FAILED;

            //from_sect_id = (from_sect_id + 1) % pcfg->fs_snum;
            break;
        }
        else if(sect_cfg.tag == FS_SECT_EXCH_TAG)
        {
            from_sect_id = (from_sect_id + 1) % pcfg->fs_snum;
            break;
        }

        from_sect_id = (from_sect_id + 1) % pcfg->fs_snum;
    }

    //step 3:
    //loop count continue from step 2
    //get crush scene and repare
    //start to garbage move
    to_addr = FS_SECTOR_SIZE * to_sect_id;
    addr_wr = FS_ITEMADDR(to_addr, to_item_id);
    //rebase to_sect_id to from_sect_id
    to_sect_id = (from_sect_id + pcfg->fs_snum -1)%pcfg->fs_snum;

    for( ; i < pcfg->fs_snum - 1; i++)
    {
        uint32_t buf[FS_ITEM_LEN/4];
        from_addr = FS_SECTOR_SIZE * from_sect_id;
        fs_spif_read(FS_ABSADDR(from_addr), (uint8_t*)(&from_sect_cfg), sizeof(fs_cfg_t));
        //mark as exchanging, erase and re-config after data removed
        from_sect_cfg.tag = FS_SECT_EXCHING_TAG;
        ret = fs_spif_write(FS_ABSADDR(from_addr),(uint8_t*)(&from_sect_cfg),sizeof(fs_cfg_t));

        if(ret) return PPlus_ERR_FS_WRITE_FAILED;

        to_sect_cfg.tag = FS_SECT_DATA_TAG;
        to_sect_cfg.fs_addr = pcfg->fs_addr;
        to_sect_cfg.fs_snum = pcfg->fs_snum;
        to_sect_cfg.item_len = pcfg->item_len;
        to_sect_cfg.index = (to_sect_id + pcfg->fs_snum - new_1st_sect) % pcfg->fs_snum ;
        ret = fs_spif_write(FS_ABSADDR(FS_SECTOR_SIZE*to_sect_id),(uint8_t*)(&to_sect_cfg),sizeof(fs_cfg_t));

        if(ret) return PPlus_ERR_FS_WRITE_FAILED;

        for(j = 0; j < FS_ITEMS_PER_SECT; j ++)
        {
            item_addr = FS_ITEMADDR(from_addr, j);
            fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)(&item_hd),sizeof(item_hd));

            if(item_hd.b.pro == ITEM_USED)
            {
                fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)buf,FS_ITEM_LEN);

                if(last_to_fid == 0xffff)
                {
                    if(PPlus_SUCCESS != fs_spif_write(FS_ABSADDR(addr_wr),(uint8_t*)buf,FS_ITEM_LEN))
                        return PPlus_ERR_FS_WRITE_FAILED;

                    addr_wr += FS_ITEM_LEN;
                    check_addr(&addr_wr);
                }
                else if(last_to_fid == item_hd.b.id)
                {
                    if(last_to_item_off)
                        last_to_item_off --;
                    else
                        last_to_fid = 0xffff;
                }
            }
            else if(item_hd.b.pro == ITEM_UNUSED)
            {
                break;
            }
        }

        fs_erase_sector(FS_ABSADDR(FS_SECTOR_SIZE*from_sect_id));
        from_sect_cfg.tag = FS_SECT_EXCH_TAG;
        from_sect_cfg.fs_addr = pcfg->fs_addr;
        from_sect_cfg.fs_snum = pcfg->fs_snum;
        from_sect_cfg.item_len = pcfg->item_len;
        from_sect_cfg.index = 0xff;
        ret = fs_spif_write(FS_ABSADDR(FS_SECTOR_SIZE*from_sect_id),(uint8_t*)(&from_sect_cfg),sizeof(fs_cfg_t));

        if(ret) return PPlus_ERR_FS_WRITE_FAILED;

        from_sect_id = (from_sect_id + 1) % pcfg->fs_snum;

        if(from_sect_id == new_1st_sect)
            break;//completed

        to_sect_id  = (to_sect_id + 1) % pcfg->fs_snum;
    }

    return FS_RET_FS_REPARED;
}

static int fs_repare(void)
{
    int i;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    fs_cfg_t  sect_cfg;
    uint8_t sect_map[FS_SECTOR_NUM_MAX];
    fs_analyze_st analyze_st = FS_ANALYZE_UNCHECK;

    for(i = 0; i < pcfg->fs_snum; i++)
    {
        fs_spif_read(FS_ABSADDR(FS_SECTOR_SIZE*i),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));
        FS_LOG("sect_cfg.tag:%x",sect_cfg.tag);
        FS_LOG("sect_cfg.fs_addr:%x",sect_cfg.fs_addr);
        FS_LOG("sect_cfg.fs_snum:%x",sect_cfg.fs_snum);
        FS_LOG("sect_cfg.item_len:%x",sect_cfg.item_len);
        FS_LOG("sect_cfg.index:%x",sect_cfg.index);

        //analyze_st = fs_check_sector(&sect_cfg);
        if(sect_cfg.tag == FS_SECT_DATA_TAG || sect_cfg.tag == FS_SECT_EXCH_TAG)
        {
            if(analyze_st == FS_ANALYZE_UNCHECK && i == 0)
                analyze_st = FS_ANALYZE_NORMAL;

            if(analyze_st != FS_ANALYZE_NORMAL)
            {
                analyze_st = FS_ANALYZE_ERROR;
                break;
            }

            if(sect_cfg.tag == FS_SECT_DATA_TAG)
            {
                if((sect_cfg.fs_addr == pcfg->fs_addr) &&
                        (sect_cfg.fs_snum == pcfg->fs_snum) &&
                        (sect_cfg.item_len == pcfg->item_len))
                {
                    FS_LOG("FS_ANALYZE_NORMAL");

                    if(sect_cfg.index >= pcfg->fs_snum-1)
                    {
                        analyze_st = FS_ANALYZE_ERROR;
                        break;
                    }

                    if(sect_map[sect_cfg.index] != 0xff)
                    {
                        analyze_st = FS_ANALYZE_ERROR;
                        break;
                    }

                    sect_map[sect_cfg.index] = i;
                }
                else
                {
                    analyze_st = FS_ANALYZE_ERROR;
                    FS_LOG("FS_ANALYZE_CONTEXT_ERROR1");
                    break;
                }
            }
            else //must be FS_SECT_EXCH_TAG
            {
                if((sect_cfg.fs_addr == pcfg->fs_addr) &&
                        (sect_cfg.fs_snum == pcfg->fs_snum) &&
                        (sect_cfg.item_len == pcfg->item_len))
                {
                    pctx->exch_sect = i;
                }
                else
                {
                    analyze_st = FS_ANALYZE_ERROR;
                    FS_LOG("FS_ANALYZE_CONTEXT_ERROR1");
                    break;
                }
            }
        }
        else if(sect_cfg.tag == FS_SECT_NEW_TAG)
        {
            if(analyze_st == FS_ANALYZE_NEW || analyze_st == FS_ANALYZE_UNCHECK)
            {
                analyze_st = FS_ANALYZE_NEW;
            }
            else
            {
                analyze_st = FS_ANALYZE_ERROR;
                FS_LOG("FS_ANALYZE_ERROR2");
                break;
            }
        }
    }

    if(analyze_st == FS_ANALYZE_NORMAL)
        return fs_repare_normal(sect_map);

    return fs_repare_gcfail(sect_map);
}

static int fs_create(void)
{
    int i;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    fs_cfg_t  sect_cfg =
    {
        .tag = FS_SECT_DATA_TAG,
        .fs_addr = pcfg->fs_addr,
        .fs_snum = pcfg->fs_snum,
        .item_len = pcfg->item_len,
        .index = 0xff,
    };
    fs_erase_all_sector();

    //make data sector
    for(i = 0; i < (pcfg->fs_snum - 1); i++)
    {
        sect_cfg.index = i;

        if(PPlus_SUCCESS != fs_spif_write((FS_ABSADDR(FS_SECTOR_SIZE*i)),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t)))
        {
            FS_LOG("PPlus_ERR_FS_WRITE_FAILED");
            return PPlus_ERR_FS_WRITE_FAILED;
        }
    }

    //make exchange sector, the last sector is exchange when formated
    sect_cfg.tag = FS_SECT_EXCH_TAG;
    sect_cfg.index = 0xff;

    if(PPlus_SUCCESS != fs_spif_write((FS_ABSADDR(FS_SECTOR_SIZE*i)),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t)))
    {
        FS_LOG("PPlus_ERR_FS_WRITE_FAILED");
        return PPlus_ERR_FS_WRITE_FAILED;
    }

    pctx->cur_sect = 0;
    pctx->exch_sect = pcfg->fs_snum - 1;
    pctx->cur_item = 0;
    return PPlus_SUCCESS;
}

static int fs_loadfs(void)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t i = 0;
    uint8_t sect_map[FS_SECTOR_NUM_MAX];
    fs_analyze_st analyze_st = FS_ANALYZE_UNCHECK;
    fs_cfg_t sect_cfg;
    //osal_memset((pcfg->reserved),0xff,(FS_ITEM_LEN-7)*sizeof(uint8_t));
    osal_memset((sect_map),0xff,FS_SECTOR_NUM_MAX);
    FS_LOG("fs_loadfs:");
    check_sector_order();
    pctx->init_flg = true;

    for(i = 0; i < pcfg->fs_snum; i++)
    {
        fs_spif_read(FS_ABSADDR(FS_SECTOR_SIZE*i),(uint8_t*)(&sect_cfg),sizeof(fs_cfg_t));
        FS_LOG("sect_cfg.tag:%x",sect_cfg.tag);
        FS_LOG("sect_cfg.fs_addr:%x",sect_cfg.fs_addr);
        FS_LOG("sect_cfg.fs_snum:%x",sect_cfg.fs_snum);
        FS_LOG("sect_cfg.item_len:%x",sect_cfg.item_len);
        FS_LOG("sect_cfg.index:%x",sect_cfg.index);

        //analyze_st = fs_check_sector(&sect_cfg);
        if(sect_cfg.tag == FS_SECT_DATA_TAG || sect_cfg.tag == FS_SECT_EXCH_TAG)
        {
            if(analyze_st == FS_ANALYZE_UNCHECK && i == 0)
                analyze_st = FS_ANALYZE_NORMAL;

            if(analyze_st != FS_ANALYZE_NORMAL)
            {
                analyze_st = FS_ANALYZE_ERROR;
                break;
            }

            if(sect_cfg.tag == FS_SECT_DATA_TAG)
            {
                if((sect_cfg.fs_addr == pcfg->fs_addr) &&
                        (sect_cfg.fs_snum == pcfg->fs_snum) &&
                        (sect_cfg.item_len == pcfg->item_len))
                {
                    FS_LOG("FS_ANALYZE_NORMAL");

                    if(sect_cfg.index >= pcfg->fs_snum-1)
                    {
                        analyze_st = FS_ANALYZE_ERROR;
                        break;
                    }

                    if(sect_map[sect_cfg.index] != 0xff)
                    {
                        analyze_st = FS_ANALYZE_ERROR;
                        break;
                    }

                    sect_map[sect_cfg.index] = i;
                }
                else
                {
                    analyze_st = FS_ANALYZE_ERROR;
                    FS_LOG("FS_ANALYZE_CONTEXT_ERROR1");
                    break;
                }
            }
            else //must be FS_SECT_EXCH_TAG
            {
                if((sect_cfg.fs_addr == pcfg->fs_addr) &&
                        (sect_cfg.fs_snum == pcfg->fs_snum) &&
                        (sect_cfg.item_len == pcfg->item_len))
                {
                    pctx->exch_sect = i;
                }
                else
                {
                    analyze_st = FS_ANALYZE_ERROR;
                    FS_LOG("FS_ANALYZE_CONTEXT_ERROR1");
                    break;
                }
            }
        }
        else if(sect_cfg.tag == FS_SECT_NEW_TAG)
        {
            if(analyze_st == FS_ANALYZE_NEW || analyze_st == FS_ANALYZE_UNCHECK)
            {
                analyze_st = FS_ANALYZE_NEW;
            }
            else
            {
                analyze_st = FS_ANALYZE_ERROR;
                FS_LOG("FS_ANALYZE_ERROR2");
                break;
            }
        }
        else
        {
            analyze_st = FS_ANALYZE_ERROR;
            FS_LOG("FS_ANALYZE_ERROR2");
            break;
        }
    }

    if(analyze_st == FS_ANALYZE_NORMAL)
    {
        if((pctx->exch_sect + 1) % sect_cfg.fs_snum != sect_map[0])
            analyze_st = FS_ANALYZE_ERROR;
    }

    if(analyze_st == FS_ANALYZE_ERROR)
    {
        return fs_repare();
    }
    else if(analyze_st == FS_ANALYZE_NEW)
    {
        return fs_create();
    }
    else if(analyze_st == FS_ANALYZE_NORMAL)
    {
        //for normal fs, need parse each item when loading
        uint16_t tail = 0;
        uint16_t j = 0;
        uint32_t sect_addr,item_addr;
        fs_item_t item_hd;
        //item_st: true for data or deleted sector(used), false for empty sector
        //front area is used sector, rear area is empty sector
        bool item_st = true;
        pctx->cur_sect = sect_map[0];
        pctx->cur_item = 0xff;

        for(i = 0; i< pcfg->fs_snum-1; i++)
        {
            sect_addr = FS_SECTOR_SIZE*sect_map[i];

            for(j = 0; j < FS_ITEMS_PER_SECT; j++)
            {
                item_addr = FS_ITEMADDR(sect_addr, j);
                fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)(&item_hd),sizeof(item_hd));

                if(item_st)
                {
                    if(item_hd.b.pro == ITEM_DEL || item_hd.b.pro == ITEM_USED)
                    {
                        pctx->cur_sect = sect_map[i];

                        if(item_hd.b.frame == ITEM_SF && tail == 0)
                        {
                            continue;
                        }
                        else if(item_hd.b.frame == ITEM_MF_F && tail == 0)
                        {
                            tail = (item_hd.b.len + FS_ITEM_DATA_LEN -1)/ FS_ITEM_DATA_LEN;
                            tail --;
                        }
                        else if((item_hd.b.frame == ITEM_MF_C || item_hd.b.frame == ITEM_MF_E) && tail)
                        {
                            tail --;
                        }
                        else
                        {
                            return fs_repare();
                        }
                    }
                    else if(item_hd.b.pro == ITEM_RESERVED)
                    {
                        pctx->cur_sect = sect_map[0];

                        if(tail)
                            return fs_repare();
                    }
                    else if(item_hd.reg == 0xffffffff)
                    {
                        if(tail)
                            return fs_repare();

                        item_st = false;
                        pctx->cur_item = j;
                    }
                    else
                    {
                        return fs_repare();
                    }
                }
                else /*item_st == false, empty area, all FF*/
                {
                    if(item_hd.reg != 0xffffffff)
                    {
                        return fs_repare();
                    }
                }
            }
        }
    }

    //unrecoverable issue
    FS_LOG("FS init OK");
    return PPlus_SUCCESS;
}
int hal_fs_list(uint32_t* fnum, uint16_t* pfid, uint16_t* pfsize)
{
    int ret;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t i, cur_sect;
    uint16_t j;
    uint32_t file_items = 1;  //a file at lease have one item
    uint32_t sector_addr,item_addr;
    fs_item_t item;
    *fnum = 0;

    for(i = 0; i < pcfg->fs_snum-1; i++)
    {
        cur_sect = FS_GET_SECT_ID(i);

        if(file_items >= FS_ITEMS_PER_SECT)
        {
            file_items -= FS_ITEMS_PER_SECT;
            continue;
        }

        sector_addr = cur_sect * FS_SECTOR_SIZE;

        for(j = file_items-1; j < FS_ITEMS_PER_SECT; j++)
        {
            item_addr = FS_ITEMADDR(sector_addr, j);
            ret = fs_spif_read(FS_ABSADDR(item_addr),(uint8_t*)&item,FS_ITEM_HEAD_LEN);

            if(ret) return ret;

            switch(item.b.pro)
            {
            case ITEM_DEL:
            case ITEM_USED:
                if(item.b.frame == ITEM_MF_F)
                {
                    if(item.b.pro == ITEM_USED)
                    {
                        *fnum = *fnum + 1;

                        if(pfid)
                        {
                            *pfid = item.b.id;
                            pfid ++;
                        }

                        if(pfsize)
                        {
                            *pfsize = item.b.len;
                            pfsize ++;
                        }
                    }

                    file_items = (item.b.len + (FS_ITEM_DATA_LEN-1)) / FS_ITEM_DATA_LEN;
                    FS_SECT_BOUNDARY_CALC(j, file_items);
                }
                else if(item.b.frame == ITEM_SF)
                {
                    if(item.b.pro == ITEM_USED)
                    {
                        *fnum = *fnum + 1;

                        if(pfid)
                        {
                            *pfid = item.b.id;
                            pfid ++;
                        }

                        if(pfsize)
                        {
                            *pfsize = item.b.len;
                            pfsize ++;
                        }
                    }

                    file_items =1;
                }

                break;

            case ITEM_UNUSED:
                return PPlus_SUCCESS;

            default:
                break;
            }
        }
    }

    return PPlus_SUCCESS;
}

int hal_fs_get_size(uint32_t* fs_size, uint32_t* free_size,
                    uint32_t* fsize_max, uint8_t* item_size, uint8_t* fs_snum)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint32_t size = 0;

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if(pctx->cur_item < FS_SECTOR_SIZE)
    {
        size = ((pctx->exch_sect + pcfg->fs_snum - pctx->cur_sect - 1)%pcfg->fs_snum) * FS_ITEMS_PER_SECT;
        size += (FS_ITEMS_PER_SECT - pctx->cur_item);
        size = size * FS_ITEM_DATA_LEN;
    }

    *free_size = size;
    size = (pcfg->fs_snum-1)*FS_ITEMS_PER_SECT;
    size = size*FS_ITEM_DATA_LEN;
    *fs_size = size;
    *item_size = FS_ITEM_DATA_LEN;
    *fs_snum = pcfg->fs_snum;
    *fsize_max = FS_FSIZE_MAX;
    return PPlus_SUCCESS;
}

int hal_fs_get_free_size(uint32_t* free_size)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint32_t size = 0;
    *free_size = 0;

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if(pctx->cur_item < FS_SECTOR_SIZE)
    {
        size = ((pctx->exch_sect + pcfg->fs_snum - pctx->cur_sect - 1)% pcfg->fs_snum)* FS_ITEMS_PER_SECT;
        size += (FS_ITEMS_PER_SECT - pctx->cur_item);
        size = size * FS_ITEM_DATA_LEN;
    }

    *free_size = size;
    return PPlus_SUCCESS;
}

int hal_fs_get_garbage_size(uint32_t* garbage_fnum)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    //fs_cfg_t* pcfg = &(pctx->cfg);
    uint32_t garbage_size = 0,garbage_count = 0;
    int ret;

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    ret = fs_search_items_deleted(&garbage_size,&garbage_count);

    if(PPlus_SUCCESS == ret)
    {
        if(garbage_fnum)
            *garbage_fnum = garbage_count;

        return garbage_size;
    }

    return PPlus_ERR_FATAL;
}

int hal_fs_item_find_id(uint16_t id,uint32_t* id_addr)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    //fs_cfg_t* pcfg = &(pctx->cfg);
    int ret;

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    ret = fs_search_items_using(id,id_addr);
    return ret;
}

int hal_fs_item_write(uint16_t id,uint8_t* buf,uint16_t len)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t frame_len,wr_buf[FS_ITEM_LEN];
    uint16_t i,item_len;
    uint32_t sect_addr, item_addr;
    fs_item_t item;
    uint32_t free_size;

    if(__psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    check_sector_order();

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if((buf == NULL) || (len == 0)||(len > FS_FSIZE_MAX))
        return PPlus_ERR_FS_PARAMETER;

    hal_fs_get_free_size(&free_size);

    if(len > free_size)
        return PPlus_ERR_FS_NOT_ENOUGH_SIZE;

    //if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    //  return PPlus_ERR_FS_EXIST_SAME_ID;
    if(hal_fs_item_find_id(id,&item_addr) == PPlus_SUCCESS)
    {
        if(PPlus_SUCCESS != hal_fs_item_del(id))
            return PPlus_ERR_FATAL;
    }

    item_len = len;
    item.b.len = len;
    item.b.id = id;
    item.b.pro = ITEM_USED;

    if(len <= FS_ITEM_DATA_LEN)
        item.b.frame = ITEM_SF;

    i = 0;

    while(len > 0)
    {
        if(len > FS_ITEM_DATA_LEN)
        {
            if(item_len == len)
                item.b.frame = ITEM_MF_F;
            else
                item.b.frame = ITEM_MF_C;

            frame_len = FS_ITEM_DATA_LEN;
            len -= FS_ITEM_DATA_LEN;
        }
        else
        {
            if((item.b.frame == ITEM_MF_C) || (item.b.frame == ITEM_MF_F))
                item.b.frame = ITEM_MF_E;

            frame_len = len;
            len = 0;
        }

        sect_addr = pctx->cur_sect * FS_SECTOR_SIZE;
        item_addr = FS_ITEMADDR(sect_addr, pctx->cur_item);
        osal_memcpy(wr_buf,(uint8_t*)(&item.reg),FS_ITEM_HEAD_LEN);
        osal_memcpy((wr_buf + FS_ITEM_HEAD_LEN),(buf + i),frame_len);
        __fs_break(0x2000);

        if(PPlus_SUCCESS != fs_spif_write(FS_ABSADDR(item_addr),wr_buf,(frame_len+FS_ITEM_HEAD_LEN)))
            return PPlus_ERR_FS_WRITE_FAILED;

        i += frame_len;
        pctx->cur_item ++;

        if(pctx->cur_item == FS_ITEMS_PER_SECT)
        {
            if(((pctx->cur_sect + 1) % pcfg->fs_snum) != pctx->exch_sect)
            {
                pctx->cur_item = 0;
                pctx->cur_sect = (pctx->cur_sect + 1) % pcfg->fs_snum;
            }
        }
    }

    return PPlus_SUCCESS;
}

int hal_fs_item_read(uint16_t id,uint8_t* buf,uint16_t buf_len,uint16_t* len)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    //fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t rd_len;
    uint16_t i = 0,temp_len;
    uint32_t addr;
    fs_item_t item;

    if(__psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    check_sector_order();

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if((buf == NULL) || (buf_len == 0))
        return PPlus_ERR_FS_PARAMETER;

    if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    {
        fs_spif_read(FS_ABSADDR(addr),(uint8_t*)&item,FS_ITEM_HEAD_LEN);

        if(len != NULL)
        {
            *len = item.b.len;
        }

        temp_len = item.b.len;

        if(buf_len >= item.b.len)
        {
            while(temp_len > 0)
            {
                rd_len = (temp_len >= FS_ITEM_DATA_LEN) ? FS_ITEM_DATA_LEN : temp_len;
                fs_spif_read(FS_ABSADDR(addr + FS_ITEM_HEAD_LEN),(buf + i),rd_len);
                temp_len -= rd_len;
                addr += FS_ITEM_LEN;
                i += rd_len;
                check_addr(&addr);
            }

            return PPlus_SUCCESS;
        }

        return PPlus_ERR_FS_BUFFER_TOO_SMALL;
    }

    return PPlus_ERR_FS_NOT_FIND_ID;
}

int hal_fs_item_del(uint16_t id)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    //fs_cfg_t* pcfg = &(pctx->cfg);
    uint16_t i = 0,count = 1;
    uint32_t addr = 0;
    fs_item_t item;

    if(__psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    check_sector_order();

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if(hal_fs_item_find_id(id,&addr) == PPlus_SUCCESS)
    {
        fs_spif_read(FS_ABSADDR(addr),(uint8_t*)&item,FS_ITEM_HEAD_LEN);
        count = item.b.len/FS_ITEM_DATA_LEN + ((item.b.len % FS_ITEM_DATA_LEN)?1:0);

        for(i = 0; i < count; i++)
        {
            fs_spif_read(FS_ABSADDR(addr),(uint8_t*)&item,FS_ITEM_HEAD_LEN);
            item.b.pro = ITEM_DEL;

            if(PPlus_SUCCESS != fs_spif_write(FS_ABSADDR(addr),(uint8_t*)(&item.reg),FS_ITEM_HEAD_LEN))
                return PPlus_ERR_FS_WRITE_FAILED;

            __fs_break(0x3000);
            addr += FS_ITEM_LEN;
            check_addr(&addr);
        }

        return PPlus_SUCCESS;
    }
    else
    {
        return PPlus_ERR_FS_NOT_FIND_ID;
    }
}


int hal_fs_garbage_collect(void)
{
    int ret;
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    uint8_t  i,j,buf[FS_ITEM_LEN];
    uint8_t from_sect_id = 0,to_sect_id = 0;
    uint32_t addr_rd=0,addr_wr=0;
    fs_item_t item;
    fs_cfg_t from_sect_cfg, to_sect_cfg;

    if(__psr()&0x3f)
    {
        return PPlus_ERR_FS_IN_INT;
    }

    check_sector_order();

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    to_sect_id = pctx->exch_sect;
    from_sect_id = (pctx->exch_sect + 1) % pcfg->fs_snum;
    addr_wr = FS_SECTOR_SIZE*to_sect_id;

    for(i = 0; i < (pcfg->fs_snum - 1); i++)
    {
        addr_rd = FS_SECTOR_SIZE * from_sect_id;
        fs_spif_read(FS_ABSADDR(addr_rd), (uint8_t*)(&from_sect_cfg), sizeof(fs_cfg_t));
        __fs_break(0x1000);
        //mark as exchanging, erase and re-config after data removed
        from_sect_cfg.tag = FS_SECT_EXCHING_TAG;
        ret = fs_spif_write(FS_ABSADDR(addr_rd),(uint8_t*)(&from_sect_cfg),sizeof(fs_cfg_t));

        if(ret) return PPlus_ERR_FS_WRITE_FAILED;

        __fs_break(0x1001);
        to_sect_cfg.tag = FS_SECT_DATA_TAG;
        to_sect_cfg.fs_addr = pcfg->fs_addr;
        to_sect_cfg.fs_snum = pcfg->fs_snum;
        to_sect_cfg.item_len = pcfg->item_len;
        to_sect_cfg.index = i;
        ret = fs_spif_write(FS_ABSADDR(FS_SECTOR_SIZE*to_sect_id),(uint8_t*)(&to_sect_cfg),sizeof(fs_cfg_t));

        if(ret) return PPlus_ERR_FS_WRITE_FAILED;

        __fs_break(0x1002);

        if(i == 0)
            addr_wr += FS_ITEM_LEN;

        addr_rd += FS_ITEM_LEN;

        for(j = 0; j < FS_ITEMS_PER_SECT; j++)
        {
            fs_spif_read(FS_ABSADDR(addr_rd),(uint8_t*)&item,FS_ITEM_HEAD_LEN);
            __fs_break(0x1003);

            if(item.b.pro == ITEM_USED)
            {
                fs_spif_read(FS_ABSADDR(addr_rd),buf,FS_ITEM_LEN);
                __fs_break(0x1004);

                if(PPlus_SUCCESS != fs_spif_write(FS_ABSADDR(addr_wr),buf,FS_ITEM_LEN))
                    return PPlus_ERR_FS_WRITE_FAILED;

                __fs_break(0x1005);
                addr_wr += FS_ITEM_LEN;
                check_addr(&addr_wr);
            }
            else if(item.b.pro == ITEM_UNUSED)
            {
                break;
            }

            __fs_break(0x1006);
            addr_rd += FS_ITEM_LEN;
        }

        __fs_break(0x1007);
        fs_erase_sector(FS_ABSADDR(FS_SECTOR_SIZE*from_sect_id));
        __fs_break(0x1008);
        from_sect_cfg.tag = FS_SECT_EXCH_TAG;
        from_sect_cfg.fs_addr = pcfg->fs_addr;
        from_sect_cfg.fs_snum = pcfg->fs_snum;
        from_sect_cfg.item_len = pcfg->item_len;
        from_sect_cfg.index = 0xff;
        ret = fs_spif_write(FS_ABSADDR(FS_SECTOR_SIZE*from_sect_id),(uint8_t*)(&from_sect_cfg),sizeof(fs_cfg_t));

        if(ret) return PPlus_ERR_FS_WRITE_FAILED;

        from_sect_id = (from_sect_id + 1) % pcfg->fs_snum;
        to_sect_id   = (to_sect_id   + 1) % pcfg->fs_snum;
    }

    __fs_break(0x1009);
    //reset config parameter, reload fs
    pcfg->index     = 0xff;
    pcfg->item_len  = FS_ITEM_LEN;
    return fs_loadfs();
}

int hal_fs_format(uint32_t fs_addr, uint8_t sect_num)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);

    if(__psr()&0x3f)
        return PPlus_ERR_FS_IN_INT;

    if(pctx->init_flg == false)
        return PPlus_ERR_FS_UNINITIALIZED;

    if(fs_addr % 0x1000)
        return PPlus_ERR_INVALID_PARAM;

    pcfg->fs_snum = sect_num;
    pcfg->fs_addr = fs_addr;
    pcfg->index     = 0xff;
    pcfg->item_len  = FS_ITEM_LEN;
    return fs_create();
}

int hal_fs_init(uint32_t fs_addr, uint8_t sect_num)
{
    fs_ctx_t* pctx = &s_fs_ctx;
    fs_cfg_t* pcfg = &(pctx->cfg);
    int ret;

    if(pctx->init_flg == true)
        return PPlus_ERR_FS_HAVE_INITED;

    if(fs_addr % 0x1000)
        return PPlus_ERR_INVALID_PARAM;

    pcfg->fs_addr   = fs_addr;
    pcfg->fs_snum   = sect_num;
    pcfg->index     = 0xff;
    pcfg->item_len  = FS_ITEM_LEN;
    ret = fs_loadfs();

    if(ret == FS_RET_FS_REPARED)
        return fs_loadfs();

    return ret;
}


bool hal_fs_initialized(void)
{
    return s_fs_ctx.init_flg;
}
