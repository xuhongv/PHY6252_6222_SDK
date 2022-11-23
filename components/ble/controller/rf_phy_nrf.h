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

/**************************************************************************************************
    Filename:       phy_plus_nrf.h
    Revised:
    Revision:

    Description:    This file contains the phyplus nrf sample application
                  definitions and prototypes.


**************************************************************************************************/

#ifndef PHYPLUSNRF_H
#define PHYPLUSNRF_H


#include "types.h"
#include "error.h"


/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/

#define NRF_CRC_SEED                                 (0xFFFF)

#define NRF_MODE_ENHANCE_SHOCKBURST                  (1)
#define NRF_MODE_SHOCKBURST                          (0)

#define NRF_ADDR_LEN_3BYTE                           (3)
#define NRF_ADDR_LEN_4BYTE                           (4)
#define NRF_ADDR_LEN_5BYTE                           (5)

#define CRC_ITU_8_LEN_1BYTE                          (1)
#define CRC_ITU_16_LEN_2BYTE                         (2)


/*********************************************************************
    MACROS
*/
#define NRF_PKT_PCF_OFFSET          ((nrfConfig.addrLen>3) ?nrfConfig.addrLen:4)
#define GET_NRF_PKT_PID(x)          (bit_rev_8(x[NRF_PKT_PCF_OFFSET])&0x03)
#define GET_NRF_PKT_NOACK(x)        (x[NRF_PKT_PCF_OFFSET+1]&0x01)

#define NRF_PHY_BUF_LEN(x)          ((nrfConfig.addrLen+nrfConfig.mode+(x)+nrfConfig.crcByte)*8 \
                                     +nrfConfig.mode)           //
#define NRF_TX_ACK_BIT_LEN          (NRF_PHY_BUF_LEN(0))        //(5+1+0+2)*8+1

/*********************************************************************
    STRUCT
*/

typedef struct nrfPkt_s
{
    uint8_t     addr[5];
    uint8_t     pldLen;
    uint8_t     pid;
    uint8_t     noAckBit;
    uint8_t     pdu[32];
    uint8_t     crc[2];
} nrfPkt_t;

typedef struct nrfPhy_s
{
    uint8_t     addrLen;
    uint8_t     mode;
    uint8_t     crcByte;
    uint32_t    version;
} nrfPhy_t;


extern nrfPkt_t nrfTxBuf;
extern nrfPkt_t nrfRxBuf;
extern nrfPkt_t nrfAckBuf;
extern nrfPhy_t nrfConfig;
/*********************************************************************
    FUNCTIONS
*/
uint8_t bit_rev_8(uint8_t b);
uint32_t nrf_pkt_init(uint8_t* addr,uint8_t addrLen,uint8_t pduLen,uint8_t crcByte,uint8_t nrfMode);
uint8_t nrf_pkt_gen(uint8_t* din, uint8_t dLen,uint8_t* dout);
uint8_t nrf_pkt_enc(nrfPkt_t* p_pkt,uint8_t* dOut, uint8_t* doLen);
uint8_t nrf_pkt_dec(uint8_t* din, uint8_t diLen, uint8_t pduLen,nrfPkt_t* p_pkt);
uint8_t nrf_txack_check(uint8_t* din);
uint8_t nrf_rxdata_check(uint8_t* din);
uint8_t nrf_pkt_crc_check(uint8_t* din);
/*********************************************************************
*********************************************************************/



#endif /* PHYPLUSPHY_H */
