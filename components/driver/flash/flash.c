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
    @file     flash.c
    @brief    Contains all functions support for flash driver
    @version  0.0
    @date     27. Nov. 2017
    @author   qing.han



*******************************************************************************/
#include "rom_sym_def.h"
#include <string.h>
#include "types.h"
#include "flash.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"

#define SPIF_WAIT_IDLE_CYC                          (32)

#define SPIF_STATUS_WAIT_IDLE(n)                    \
    do                                              \
    {                                               \
        while((AP_SPIF->fcmd &0x02)==0x02);         \
        {                                           \
            volatile int delay_cycle = n;           \
            while (delay_cycle--){;}                \
        }                                           \
        while ((AP_SPIF->config & 0x80000000) == 0);\
    } while (0);


#define HAL_CACHE_ENTER_BYPASS_SECTION()  do{ \
        HAL_ENTER_CRITICAL_SECTION();\
        AP_CACHE->CTRL0 = 0x02; \
        AP_PCR->CACHE_RST = 0x02;\
        AP_PCR->CACHE_BYPASS = 1;    \
        HAL_EXIT_CRITICAL_SECTION();\
    }while(0);


#define HAL_CACHE_EXIT_BYPASS_SECTION()  do{ \
        HAL_ENTER_CRITICAL_SECTION();\
        AP_CACHE->CTRL0 = 0x00;\
        AP_PCR->CACHE_RST = 0x03;\
        AP_PCR->CACHE_BYPASS = 0;\
        HAL_EXIT_CRITICAL_SECTION();\
    }while(0);

#define spif_wait_nobusy(flg, tout_ns, return_val)   {if(_spif_wait_nobusy_x(flg, tout_ns)){if(return_val){ return return_val;}}}

static xflash_Ctx_t s_xflashCtx = {.rd_instr=XFRD_FCMD_READ_DUAL};

bool spif_dma_use = false;

chipMAddr_t  g_chipMAddr;
extern void ll_patch_restore(uint8_t flg);
__ATTR_SECTION_SRAM__  static inline uint32_t spif_lock()
{
    HAL_ENTER_CRITICAL_SECTION();
    uint32_t vic_iser = NVIC->ISER[0];
    //mask all irq
    NVIC->ICER[0] = 0xFFFFFFFF;
    //enable ll irq and tim1 irq
    NVIC->ISER[0] = 0x100010;
    ll_patch_restore(0);//clean ll patch
    HAL_EXIT_CRITICAL_SECTION();
    return vic_iser;
}

__ATTR_SECTION_SRAM__  static inline void spif_unlock(uint32_t vic_iser)
{
    HAL_ENTER_CRITICAL_SECTION();
    NVIC->ISER[0] = vic_iser;
    ll_patch_restore(1);//restore ll patch
    HAL_EXIT_CRITICAL_SECTION();
}

void hal_cache_tag_flush(void)
{
    HAL_ENTER_CRITICAL_SECTION();
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    volatile int dly = 8;

    if(cb==0)
    {
        AP_PCR->CACHE_BYPASS = 1;
    }

    AP_CACHE->CTRL0 = 0x02;

    while (dly--) {;};

    AP_CACHE->CTRL0 = 0x03;

    dly = 8;

    while (dly--) {;};

    AP_CACHE->CTRL0 = 0x00;

    if(cb==0)
    {
        AP_PCR->CACHE_BYPASS = 0;
    }

    HAL_EXIT_CRITICAL_SECTION();
}


static uint8_t _spif_read_status_reg_x(bool rdst_h)
{
    uint8_t status;

    if (rdst_h)
    {
        spif_cmd(FCMD_RDST_H, 0, 2, 0, 0, 0);
        SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
        spif_rddata(&status, 1);
    }
    else
    {
        spif_cmd(FCMD_RDST, 0, 2, 0, 0, 0);
        SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
        spif_rddata(&status, 1);
    }

    return status;
}

static int _spif_wait_nobusy_x(uint8_t flg, uint32_t tout_ns)
{
    uint8_t status;
    volatile int tout = (int )(tout_ns);

    for(; tout ; tout --)
    {
        status = _spif_read_status_reg_x(FALSE);

        if((status & flg) == 0)
            return PPlus_SUCCESS;

        //insert polling interval
        //5*32us
        WaitRTCCount(5);
    }

    return PPlus_ERR_BUSY;
}

static void hal_cache_init(void)
{
    volatile int dly=100;
    //clock gate
    hal_clk_gate_enable(MOD_HCLK_CACHE);
    hal_clk_gate_enable(MOD_PCLK_CACHE);
    //cache rst ahp
    AP_PCR->CACHE_RST=0x02;

    while(dly--) {};

    AP_PCR->CACHE_RST=0x03;

    hal_cache_tag_flush();

    //cache enable
    AP_PCR->CACHE_BYPASS = 0;
}


int hal_get_flash_info(void)
{
    spif_read_id(NULL);
    return PPlus_SUCCESS;
}

#if(FLASH_PROTECT_FEATURE == 1)
int hal_flash_write_status_register(uint16_t reg_data)
{
    uint8_t data[2];
    spif_cmd(FCMD_WREN, 0, 0, 0, 0, 0);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    data[0] = reg_data & 0xff;
    data[1] = reg_data>>8 & 0xff;
    spif_wrdata(data, 2);
    spif_cmd(FCMD_WRST, 0, 0, 2, 0, 0);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    return PPlus_SUCCESS;
}
int hal_flash_lock(void)
{
    if (hal_flash_get_lock_state() != 0)
    {
        return PPlus_SUCCESS;
    }

    uint32_t flash_id;
    spif_read_id(&flash_id);

    switch (flash_id)
    {
    case XT25W02D_ID:
    case TH25D20UA_ID:
    case P25D22U_ID:
    case P25D21U_ID:
        hal_flash_write_status_register(0x28);
        break;

    case UC25HQ40_ID:
    case P25D40U_ID:
    case P25Q16_ID:
    case BY25Q40_ID:
    case GT25Q40_ID:
    case TH25D40HB_ID:
    case TH25Q40UA_ID:
        hal_flash_write_status_register(0x2C);
        break;

    case GD25WD40_ID:
    case XT25W04D_ID:
    case MD25D40_ID:
    case ZB25WD40_ID:
        hal_flash_write_status_register(0x18);
        break;

    default:
        return PPlus_ERR_NOT_SUPPORTED;
    }

    return PPlus_SUCCESS;
}

int hal_flash_unlock(void)
{
    uint32_t cs = spif_lock();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    AP_SPIF->fcmd = 0x6000001;
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    AP_SPIF->fcmd_wrdata[0] = 0x00;
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    AP_SPIF->fcmd = 0x1009001;
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    spif_unlock(cs);
    return PPlus_SUCCESS;
}

uint8_t hal_flash_get_lock_state(void)
{
    uint32_t cs = spif_lock();
    uint8_t status;
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    status = _spif_read_status_reg_x(FALSE);
    uint32_t flash_id;
    spif_read_id(&flash_id);

    switch (flash_id)
    {
    case XT25W02D_ID:
    case TH25D20UA_ID:
    case P25D22U_ID:
    case P25D21U_ID:
        status = (status & 0x28)>>2;
        break;

    case UC25HQ40_ID:
    case P25D40U_ID:
    case P25Q16_ID:
    case BY25Q40_ID:
    case GT25Q40_ID:
    case TH25D40HB_ID:
    case TH25Q40UA_ID:
        status = (status & 0x2C)>>2;
        break;

    case GD25WD40_ID:
    case XT25W04D_ID:
    case MD25D40_ID:
    case ZB25WD40_ID:
        status = (status & 0x18)>>2;
        break;

    default:
        break;
    }

    if (_spif_read_status_reg_x(TRUE) & 0x40)
        status |= 0x40;

    spif_unlock(cs);
    return status;
}
#endif

#ifdef XFLASH_HIGH_SPEED
static void hw_spif_config_high_speed(sysclk_t ref_clk)
{
    volatile uint32_t tmp = AP_SPIF->config;
    tmp =  (tmp & (~ (0xf << 19))) | (0 << 19);
    AP_SPIF->config = tmp;
    subWriteReg(&AP_SPIF->rddata_capture, 4, 1, 2);
}
#endif

static void hw_spif_cache_config(void)
{
    extern volatile sysclk_t g_system_clk;
    sysclk_t spif_ref_clk = (g_system_clk > SYS_CLK_XTAL_16M) ? SYS_CLK_DLL_64M : g_system_clk;

    if(s_xflashCtx.rd_instr == XFRD_FCMD_READ_QUAD)
        spif_config(spif_ref_clk,/*div*/1,s_xflashCtx.rd_instr,0,1);
    else
        spif_config(spif_ref_clk,/*div*/1,s_xflashCtx.rd_instr,0,0);

    #ifdef XFLASH_HIGH_SPEED
    hw_spif_config_high_speed(s_xflashCtx.spif_ref_clk);
    #endif
    AP_SPIF->wr_completion_ctrl=0xff010005;//set longest polling interval
    AP_SPIF->low_wr_protection = 0;
    AP_SPIF->up_wr_protection = 0x10;
    AP_SPIF->wr_protection = 0x2;
    NVIC_DisableIRQ(SPIF_IRQn);
    NVIC_SetPriority((IRQn_Type)SPIF_IRQn, IRQ_PRIO_HAL);
    hal_cache_init();
    hal_get_flash_info();
}

int hal_spif_cache_init(xflash_Ctx_t cfg)
{
    memset(&(s_xflashCtx), 0, sizeof(s_xflashCtx));
    memcpy(&(s_xflashCtx), &cfg, sizeof(s_xflashCtx));
    hw_spif_cache_config();
    hal_pwrmgr_register(MOD_SPIF, NULL,  hw_spif_cache_config);
    return PPlus_SUCCESS;
}


int hal_flash_read(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint32_t cs = spif_lock();
    volatile uint8_t* u8_spif_addr = (volatile uint8_t*)((addr & 0x7ffff) | FLASH_BASE_ADDR);
    uint32_t cb = AP_PCR->CACHE_BYPASS;

    //read flash addr direct access
    //bypass cache
    if(cb == 0)
    {
        HAL_CACHE_ENTER_BYPASS_SECTION();
    }

    for(int i=0; i<size; i++)
        data[i]=u8_spif_addr[i];

    //bypass cache
    if(cb == 0)
    {
        HAL_CACHE_EXIT_BYPASS_SECTION();
    }

    spif_unlock(cs);
    return PPlus_SUCCESS;
}

int hal_flash_write(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t retval;
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_unlock();
    // #endif
    uint32_t cs = spif_lock();
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_write(addr,data,size);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();
    spif_unlock(cs);
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_lock();
    // #endif
    return retval;
}

int hal_flash_write_by_dma(uint32_t addr, uint8_t* data, uint32_t size)
{
    uint8_t retval;

    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_unlock();
    // #endif
    if(spif_dma_use == true)
        return PPlus_ERR_FORBIDDEN;

    spif_dma_use = true;
    uint32_t cs = spif_lock();
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_write_dma(addr,data,size);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();
    spif_unlock(cs);
    spif_dma_use = false;
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_lock();
    // #endif
    return retval;
}

int hal_flash_erase_sector(unsigned int addr)
{
    uint8_t retval;
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_unlock();
    // #endif
    uint32_t cs = spif_lock();
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_sector(addr);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    spif_unlock(cs);
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_lock();
    // #endif
    return retval;
}

int hal_flash_erase_block64(unsigned int addr)
{
    uint8_t retval;
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_unlock();
    // #endif
    uint32_t cs = spif_lock();
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_block64(addr);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    spif_unlock(cs);
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_lock();
    // #endif
    return retval;
}

int hal_flash_erase_all(void)
{
    uint8_t retval;
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_unlock();
    // #endif
    uint32_t cs = spif_lock();
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    retval = spif_erase_all();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_wait_nobusy(SFLG_WELWIP, SPIF_TIMEOUT, PPlus_ERR_BUSY);
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if(cb == 0)
    {
        hal_cache_tag_flush();
    }

    spif_unlock(cs);
    // #if(FLASH_PROTECT_FEATURE == 1)
    // hal_flash_lock();
    // #endif
    return retval;
}

int flash_write_word(unsigned int offset, uint32_t  value)
{
    uint32_t temp = value;
    offset &= 0x00ffffff;
    return (hal_flash_write_by_dma (offset, (uint8_t*) &temp, 4));
}

CHIP_ID_STATUS_e read_chip_mAddr(void)
{
    CHIP_ID_STATUS_e ret = CHIP_ID_UNCHECK;
    uint8_t b;

    for(int i=0; i<CHIP_MADDR_LEN; i++)
    {
        ret = chip_id_one_bit_hot_convter(&b, read_reg(CHIP_MADDR_FLASH_ADDRESS+(i<<2)));

        if(ret==CHIP_ID_VALID)
        {
            g_chipMAddr.mAddr[CHIP_MADDR_LEN-1-i]=b;
        }
        else
        {
            if(i>0 && ret==CHIP_ID_EMPTY)
            {
                ret =CHIP_ID_INVALID;
            }

            return ret;
        }
    }

    return ret;
}

void check_chip_mAddr(void)
{
    //chip id check
    for(int i=0; i<CHIP_MADDR_LEN; i++)
    {
        g_chipMAddr.mAddr[i]=0xff;
    }

    g_chipMAddr.chipMAddrStatus=read_chip_mAddr();
}

void LOG_CHIP_MADDR(void)
{
    LOG("\n");

    if(g_chipMAddr.chipMAddrStatus==CHIP_ID_EMPTY)
    {
        LOG("[CHIP_MADDR EMPTY]\n");
    }
    else if(g_chipMAddr.chipMAddrStatus==CHIP_ID_INVALID)
    {
        LOG("[CHIP_MADDR INVALID]\n");
    }
    else if(g_chipMAddr.chipMAddrStatus==CHIP_ID_VALID)
    {
        LOG("[CHIP_MADDR VALID]\n");

        for(int i=0; i<CHIP_MADDR_LEN; i++)
        {
            LOG("%02x",g_chipMAddr.mAddr[i]);
        }

        LOG("\n");
    }
    else
    {
        LOG("[CHIP_MADDR UNCHECKED]\n");
    }
}

