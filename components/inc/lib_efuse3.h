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
    # File: lib_efuse.h
    # Hist:
     2022.7.7  YU File Creation
     2022.8.11 YU Reduce memory usage
 *******************************************************************************/
#ifndef LIB_EFUSE3_H
#define LIB_EFUSE3_H

#include <stdbool.h>
#include <stdint.h>

struct Efuse_inf
{
    unsigned int mft:2; //厂家
    unsigned int zigbee_enab:1;
    unsigned int prog_ver:4; //测试程序版本编号
    unsigned int chip_ver:4; //芯片型号
    unsigned int ble_enab:1;
    unsigned int lotnum:10;
    unsigned int site:4;
    unsigned int mesh_enab:1;
    unsigned int multirole_enab:1;
    unsigned int pass_flg:2;
    unsigned int time_stamp:31;//yyyymmddhhmmss
};

unsigned int lib_efuse_mft(void);
unsigned int lib_efuse_zigbee(void);
unsigned int lib_efuse_prog_ver(void);
unsigned int lib_efuse_chip_ver(void);
unsigned int lib_efuse_ble(void);
unsigned int lib_efuse_lotnum(void);
unsigned int lib_efuse_site(void);
unsigned int lib_efuse_time_stamp(void);
unsigned int lib_efuse_mesh(void);
unsigned int lib_efuse_multirole(void);
unsigned int lib_efuse_pass_flg(void);
void lib_efuse_load(uint32_t* efuse_data);

#endif // LIB_EFUSE3_H
