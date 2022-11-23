#include "osal.h"
#include "crc16.h"
#include "fs.h"
#include "error.h"
#include "fs_autotest.h"
#include "cliface.h"
#include "clock.h"


//#define FS_OFFSET_ADDRESS         0x1103c000 //for 256KB Flash
#define FS_OFFSET_ADDRESS         0x11034000 //for 512KB Flash
#define FS_SECTOR_NUM             3
#define FS_SECTOR_NUM_BUFFER_SIZE  312/4

#define FS_TEST_BUF_SIZE 16*1024
uint8_t fs_test_buf[FS_TEST_BUF_SIZE] __attribute__((aligned(4))) ; //assume that max file size is 16K

//extern uint32_t fs_break_pos;
//extern uint8_t fs_break_cnt_trigger;
//extern uint8_t fs_break_cnt;


//preamble string add
void fst_preamble(uint8_t* params)
{
    //CLIT_preamble(params);
    CLIT_output("[%s, 0]",CLIT_preamble(params) )
}
//reset command for test
uint16_t fst_rst(uint32_t argc, uint8_t* argv[])
{
    CLIT_output("[%d, 0]",PPlus_SUCCESS);
    NVIC_SystemReset();
}
/*
    uint16_t fst_flist(uint32_t argc, uint8_t* argv[])
    {
    uint32_t fnum, i;
    uint16_t* fid = (uint16_t*)fs_test_buf;
    uint16_t* fsize = (uint16_t*)(fs_test_buf+ FS_TEST_BUF_SIZE/2);
    int ret = hal_fs_list(&fnum, fid, fsize);

    CLIT_outputs("[%d, %d , [",ret, fnum);

    for(i = 0; i< fnum; i++){
    if(i)CLIT_outputc(",");
    CLIT_outputc("[%d, %d]", fid[i], fsize[i]);
    }
    CLIT_outpute("]]");
    return 0;
    }
*/
/*
    //get total size, free size, max filesize, unit size, sector number
    uint16_t fst_info(uint32_t argc, uint8_t* argv[])
    {
    uint32_t fs_size,free_size, fsize_max;
    uint8_t item_size, sector_num;
    int ret = hal_fs_get_size(&fs_size, &free_size, &fsize_max, &item_size, &sector_num);
    CLIT_output("[%d, %d,%d,%d,%d,%d]",ret, fs_size, free_size, fsize_max, item_size, sector_num);

    return 0;
    }
*/

//return read status and cksum of file
uint16_t fst_read( uint32_t argc, uint8_t* argv[])
{
    uint16_t fid =(uint16_t)CLI_strtoi(argv[0], strlen((const char*)argv[0]), 10);
    uint16_t size =(uint16_t)CLI_strtoi(argv[1], strlen((const char*)argv[1]), 10);
    uint16_t rd_size = size;
    int iret = hal_fs_item_read(fid, fs_test_buf, 16*1024, &rd_size);

    if(iret)
    {
        CLIT_output("[%d, 0]",iret);
    }
    else
    {
        if(rd_size != size)
        {
            CLIT_output("[%d, 0]",PPlus_ERR_DATA_SIZE);
        }
        else
        {
            uint16_t crc=0;
            crc = crc16(0, fs_test_buf, size);
            CLIT_output("[%d,%d]",PPlus_SUCCESS,crc);
        }
    }

    return 0;
}

//return write res_status and cksum of file
uint16_t fst_write(uint32_t argc, uint8_t* argv[])
{
    uint32_t free_size;
    hal_fs_get_free_size(&free_size);
    int iret;
    uint16_t fid =(uint16_t)CLI_strtoi(argv[0], strlen((const char*)argv[0]), 10);
    uint16_t size =(uint16_t)CLI_strtoi(argv[1], strlen((const char*)argv[1]), 10);

    if(size <= free_size)
    {
        for(int i = 0; i<16*1024; i++)
        {
            fs_test_buf[i] = (uint8_t)(osal_rand()&0xff);
        }

        iret = hal_fs_item_write(fid, fs_test_buf, size);

        if(iret)
        {
            CLIT_output("[%d, 0]",iret);
        }
        else
        {
            uint16_t crc=0;
            crc = crc16(0, fs_test_buf, size);
            CLIT_output("[%d,%d]",PPlus_SUCCESS,crc);
        }
    }
    else
    {
        CLIT_output("[%d, 0]",PPlus_ERR_FS_NOT_ENOUGH_SIZE);
    }

    return 0;
}
//return read time of file
uint16_t fst_read_time(uint32_t argc, uint8_t* argv[])
{
    uint16_t fid =(uint16_t)CLI_strtoi(argv[0], strlen((const char*)argv[0]), 10);
    uint16_t size =(uint16_t)CLI_strtoi(argv[1], strlen((const char*)argv[1]), 10);
    uint16_t rd_size = size;
    int ret;
    uint32 t0 = hal_systick();
    ret = hal_fs_item_read(fid, fs_test_buf, 16*1024, &rd_size);
    uint32 t1= hal_systick()-t0;
    CLIT_output("[%d, %d]",t1, ret);
    return 0;
}

//return write res_status and cksum of file
uint16_t fst_write_time(uint32_t argc, uint8_t* argv[])
{
    uint32_t free_size;
    hal_fs_get_free_size(&free_size);
    int iret;
    uint16_t fid =(uint16_t)CLI_strtoi(argv[0], strlen((const char*)argv[0]), 10);
    uint16_t size =(uint16_t)CLI_strtoi(argv[1], strlen((const char*)argv[1]), 10);

    if(size <= free_size)
    {
        for(int i = 0; i<16*1024; i++)
        {
            fs_test_buf[i] = (uint8_t)(osal_rand()&0xff);
        }

        uint32 t0 = hal_systick();
        iret = hal_fs_item_write(fid, fs_test_buf, size);
        uint32 t1= hal_systick()-t0;
        CLIT_output("[%d, %d]",t1,iret);
    }

    return 0;
}

//return del file status
uint16_t fst_del(uint32_t argc, uint8_t* argv[])
{
    uint16_t fid =(uint16_t)CLI_strtoi(argv[0], strlen((const char*)argv[0]), 10);
    uint16_t iret;
    iret = hal_fs_item_del(fid);
    CLIT_output("[%d, 0]",iret);
    return 0;
}


//return available free size
uint16_t fst_free(uint32_t argc, uint8_t* argv[])
{
    uint32_t free_size;
    hal_fs_get_free_size(&free_size);
    CLIT_output("[%d, 0]",free_size);
    /*
        uint32_t total_size = FS_SECTOR_NUM * FS_SECTOR_NUM_BUFFER_SIZE;


        if( free_size> total_size)
        {

            CLIT_output("[%d, 0]",PPlus_ERR_FATAL);
        }
        else if(free_size ==0)

        {

            CLIT_output("[%d, 0]",PPlus_ERR_FS_FULL);
        }
        else
        {

            CLIT_output("[%d, 0]",PPlus_SUCCESS);

            CLIT_output("[%d, %d]",free_size, total_size);
        }
    */
    return 0;
}

//return garbage_size and garbage_count
uint16_t fst_garbage(uint32_t argc, uint8_t* argv[])
{
    uint32_t  garbage_size, garbage_count;
    garbage_size = hal_fs_get_garbage_size(&garbage_count);
    CLIT_output("[%d, %d]",garbage_size, garbage_count);
    return 0;
}


//garbage_collect and return available free size
uint16_t fst_clean(uint32_t argc, uint8_t* argv[])
{
    int iret = hal_fs_garbage_collect();
    uint32_t size = 0;
    hal_fs_get_free_size(&size);
    CLIT_output("[%d, %d]", iret, size);
    return 0;
}
/*
    uint16_t fst_trigger(uint32_t argc, uint8_t* argv[])
    {
    fs_break_pos =(uint32_t)CLI_strtoi(argv[0], strlen((const char*)argv[0]), 16);
    fs_break_cnt_trigger =(uint8_t)CLI_strtoi(argv[1], strlen((const char*)argv[1]), 10);

    fs_break_cnt = 0;

    CLIT_output("[%d]", 0);

    return 0;
    }
*/
//format and eraser all files
uint16_t fst_format(uint32_t argc, uint8_t* argv[])
{
    int result = hal_fs_format(FS_OFFSET_ADDRESS, FS_SECTOR_NUM);
    CLIT_output("[%d, 0]",result);
    return 0;
}


CLI_COMMAND fst_cmd_list[] =
{
    {"?",           "Help",         fst_help    },
    {"help",        "Help",         fst_help    },

    {"fst_rst",     "fst_reset",    fst_rst     },//reset board
    //{"fst_l",       "fst_flist",    fst_flist   },//file list
    {"fst_r",       "fst_read",     fst_read    },//read file
    {"fst_w",       "fst_write",    fst_write   },//write file
    {"fst_d",       "fst_del",      fst_del     },//delete file
    {"fst_free",    "fst_free",     fst_free    },//get free size
    {"fst_g",       "fst_garbage",  fst_garbage },//get garbage information
    {"fst_gc",      "fst_clean",    fst_clean   },//garbage collection
    {"fst_fmt",     "fst_format",   fst_format  },//fs format
    //{"fst_info",    "fst_info",     fst_info    },//fs info, get fs size, free size, item size and sector number
// {"fst_trg",     "fst_trigger",  fst_trigger },//trigger a break when do write or do garbage collect
    {"fst_rt",      "fst_read_time",     fst_read_time    },//read file time
    {"fst_wt",      "fst_write_time",    fst_write_time   },//write file time

};

static uint16_t fst_help(uint32_t argc, uint8_t* argv[])
{
    CLI_help(fst_cmd_list, sizeof(fst_cmd_list)/sizeof(CLI_COMMAND));
    return 0;
}
void fst_init(void)
{
    if(hal_fs_initialized() == FALSE)
    {
        hal_fs_init(FS_OFFSET_ADDRESS,FS_SECTOR_NUM);
    }
}

void fst_cmd_register(void)
{
    CLI_init(fst_cmd_list, sizeof(fst_cmd_list)/sizeof(CLI_COMMAND));
    fst_init();
}


