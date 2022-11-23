#ifndef _FS_AUTOTEST_H_
#define _FS_AUTOTEST_H_

#include "types.h"

//preamble string add

void fst_preamble(uint8_t* params);



//size: get bytes of section number
uint16_t fst_info(uint32_t argc, uint8_t* argv[]);

//return read status and cksum of file
uint16_t fst_read(uint32_t argc, uint8_t* argv[]);

//return write res_status and cksum of file
uint16_t fst_write(uint32_t argc, uint8_t* argv[]);

//return del file status
uint16_t fst_del(uint32_t argc, uint8_t* argv[]);

//return available free size
uint16_t fst_free(uint32_t argc, uint8_t* argv[]);

//return garbage_size and garbage_count
uint16_t fst_garbage(uint32_t argc, uint8_t* argv[]);

//garbage_collect and return available free size
uint16_t fst_clean(uint32_t argc, uint8_t* argv[]);

//format and eraser all files
uint16_t fst_format(uint32_t argc, uint8_t* argv[]);

static uint16_t fst_help(uint32_t argc, uint8_t* argv[]);
void fst_init(void);
void fst_cmd_register(void);
#endif
