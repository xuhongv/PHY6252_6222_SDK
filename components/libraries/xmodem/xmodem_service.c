

#include "types.h"
#include "OSAL.h"
#include "uart.h"
#include "flash.h"
#include "log.h"
#include "xmodem_service.h"

#define XMDM_LOG(...)   LOG(__VA_ARGS__)

#ifndef XMDM_OTA_NVM_BASE
    #include "slb.h"
    #define XMDM_OTA_NVM_BASE     SLB_EXCH_AREA_BASE
    #define XMDM_OTA_NVM_LEN      SLB_EXCH_AREA_SIZE
#endif
#define XMDM_OTA_NVM_SECT_LEN  (4*1024)

uint8_t xmodem_service_taskid = 0;


#define XMDM_START_RECV_EVT   0x0001
#define XMDM_RX_DATA_EVT      0x0002
#define XMDM_FINISH_RECV_EVT  0x0004
#define XMDM_CANCEL_RECV_EVT  0x0008
#define XMDM_TIMEOUT_EVT      0x0010

#define XMDM_TRYCH_NUMBER 16
#define XMDM_MAXRETRANS 25

enum
{
    SOH = 0x01, //128
    STX = 0x02, //1024
    EOT = 0x04, //end
    ACK = 0x06,
    NAK = 0x15,
    CAN = 0x18  //cacel
};


uint16_t calcrc(char* ptr, int count)
{
    int crc = 0;
    char i;

    while (--count >= 0)
    {
        crc = crc ^ (int) *ptr++ << 8;
        i = 8;

        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        }
        while(--i);
    }

    return (uint16_t)crc;
}

static int check(int crc, const uint8_t* buf, int sz)
{
    if (crc)
    {
        uint16_t crc2 = calcrc((char*)buf, sz);
        uint16_t tcrc = (buf[sz]<<8)+buf[sz+1];

        if (crc2 == tcrc)
            return 1;
    }
    else
    {
        int i;
        uint8_t cks = 0;

        for (i = 0; i < sz; ++i)
        {
            cks += buf[i];
        }

        if (cks == buf[sz])
            return 1;
    }

    return 0;
}


enum
{
    xmdm_cb_write = 0,
    xmdm_cb_rsp_finished = 1,
    xmdm_cb_rsp_cancel = 2,

    xmdm_cb_err_retry = -1,
    xmdm_cb_err_io = -2,
    xmdm_cb_err_busy = -3,
    xmdm_cb_err_bus = -4
};



typedef enum
{
    XMDM_ST_IDLE = 0,
    XMDM_ST_START,
    XMDM_ST_RX_HEAD,
    XMDM_ST_RX_DATA,
    XMDM_ST_RX_CHECK,
    XMDM_ST_END

} xmdm_st_t;

typedef struct
{
    xmdm_st_t st;
    char trychar;
    char trycnt;
    uint16_t block_size;

    int retrans;

    uint32_t xmit_len;
    uint32_t xmit_off;

    uint8_t block_id;
    uint16_t block_off;
    uint8_t* block_buf;

    xmdm_cb_t xmdm_cb;

    UART_INDEX_e port;
    int boardrate;
    gpio_pin_e tx;
    gpio_pin_e rx;

} xmdm_ctx_t;

xmdm_ctx_t g_xmdm_ctx;

void port_outbyte(uint8_t trychar)
{
    hal_uart_send_byte(g_xmdm_ctx.port, trychar);
}

bool xmdm_data_callback_default(int rsp_val, xmdm_cb_param_t* param)
{
    int ret = 0;
    uint8_t* p_data;
    uint32_t address, length;
    uint32_t flash_address;

    //MI_LOG_DEBUG("");
    if(rsp_val < 0 )
        return false;

    if(rsp_val == xmdm_cb_rsp_finished)
    {
        const char* ota_tag = "OTAF";
        ret = hal_flash_write_by_dma(XMDM_OTA_NVM_BASE, (uint8_t*)ota_tag, 4);
        hal_system_soft_reset();
        return true;
    }

    if(rsp_val > 0)
    {
        return true;
    }

    address = param->addr;
    length = param->len;
    p_data = param->buf;
    flash_address = XMDM_OTA_NVM_BASE + address;

    if (address < XMDM_OTA_NVM_LEN)
    {
        if (0 == (flash_address % XMDM_OTA_NVM_SECT_LEN))
        {
            ret = hal_flash_erase_sector(flash_address);

            if(ret)
                return false;
        }

        ret = hal_flash_write_by_dma(flash_address, (uint8_t*)p_data, length);

        if(ret)
            return false;
    }

    return true;
}



void xmdm_uart_hdl(uart_Evt_t* pev)
{
    char ch;

    for(int i = 0; i< pev->len; i++)
    {
        ch = pev->data[i];

        switch(g_xmdm_ctx.st)
        {
        case XMDM_ST_START:
            osal_stop_timerEx(xmodem_service_taskid, XMDM_START_RECV_EVT);
            osal_clear_event(xmodem_service_taskid, XMDM_START_RECV_EVT);

        case XMDM_ST_RX_HEAD:
        {
            g_xmdm_ctx.block_off = 0;
            XMDM_LOG(".");

            if(ch == SOH)
            {
                g_xmdm_ctx.block_size = 128;
                g_xmdm_ctx.st = XMDM_ST_RX_DATA;
                g_xmdm_ctx.block_buf[g_xmdm_ctx.block_off++] = ch;
                osal_start_timerEx(xmodem_service_taskid,XMDM_TIMEOUT_EVT, 500);
            }
            else if(ch == STX)
            {
                g_xmdm_ctx.block_size = 1024;
                g_xmdm_ctx.st = XMDM_ST_RX_DATA;
                g_xmdm_ctx.block_buf[g_xmdm_ctx.block_off++] = ch;
                osal_start_timerEx(xmodem_service_taskid,XMDM_TIMEOUT_EVT, 500);
            }
            else if(ch == EOT)
            {
                osal_stop_timerEx(xmodem_service_taskid,XMDM_TIMEOUT_EVT);
                osal_set_event(xmodem_service_taskid, XMDM_FINISH_RECV_EVT);
                g_xmdm_ctx.st = XMDM_ST_END;
                return;
            }
            else if(ch == CAN)
            {
                osal_stop_timerEx(xmodem_service_taskid,XMDM_TIMEOUT_EVT);
                osal_set_event(xmodem_service_taskid, XMDM_CANCEL_RECV_EVT);
                g_xmdm_ctx.st = XMDM_ST_END;
                return;
            }

            break;
        }

        case XMDM_ST_RX_DATA:
            g_xmdm_ctx.block_buf[g_xmdm_ctx.block_off++] = ch;

            if(g_xmdm_ctx.block_off == g_xmdm_ctx.block_size + 1+ 2 + 2)
            {
                g_xmdm_ctx.st = XMDM_ST_RX_CHECK;
                XMDM_LOG("XMDM_ST_RX_DATA\n");
                osal_clear_event(xmodem_service_taskid, XMDM_START_RECV_EVT);
                osal_set_event(xmodem_service_taskid, XMDM_RX_DATA_EVT);
            }

            break;

        default:
            break;
        }
    }
}


int xmodem_service_xmit(UART_INDEX_e port, int boardrate,
                        gpio_pin_e tx, gpio_pin_e rx,
                        uint32_t xmit_size, xmdm_cb_t cb)
{
    if(g_xmdm_ctx.st != XMDM_ST_IDLE)
        return PPlus_ERR_BUSY;

    osal_memset(&g_xmdm_ctx, 0, sizeof(g_xmdm_ctx));
    g_xmdm_ctx.st = XMDM_ST_IDLE;
    g_xmdm_ctx.trychar = 'C';
    g_xmdm_ctx.trycnt = 0;
    g_xmdm_ctx.block_size = 0;
    g_xmdm_ctx.retrans = 0;
    g_xmdm_ctx.block_id = 0;
    g_xmdm_ctx.xmit_len = xmit_size;
    g_xmdm_ctx.xmit_off = 0;
    g_xmdm_ctx.block_off = 0;
    g_xmdm_ctx.block_buf = osal_mem_alloc(1030);
    g_xmdm_ctx.xmdm_cb = cb;
    g_xmdm_ctx.port = port;
    g_xmdm_ctx.tx = tx;
    g_xmdm_ctx.rx = rx;
    g_xmdm_ctx.boardrate = boardrate;
    osal_set_event(xmodem_service_taskid, XMDM_START_RECV_EVT);
    return PPlus_SUCCESS;
}


void xmodem_service_init( uint8 task_id )
{
    xmodem_service_taskid = task_id;
    osal_memset(&g_xmdm_ctx, 0, sizeof(g_xmdm_ctx));
}


uint16_t xmodem_service_processevent( uint8 task_id, uint16 events )
{
    bool ret = false;
    XMDM_LOG("\nevt %x\n", events);

    if ( events & XMDM_START_RECV_EVT )
    {
        if(g_xmdm_ctx.st == XMDM_ST_IDLE)
        {
            uart_Cfg_t cfg =
            {
                .tx_pin = g_xmdm_ctx.tx,
                .rx_pin = g_xmdm_ctx.rx,
                .rts_pin = GPIO_DUMMY,
                .cts_pin = GPIO_DUMMY,
                .baudrate = g_xmdm_ctx.boardrate,
                .use_fifo = TRUE,
                .hw_fwctrl = FALSE,
                .use_tx_buf = FALSE,
                .parity     = FALSE,
                .evt_handler = xmdm_uart_hdl,
            };
            hal_uart_deinit(g_xmdm_ctx.port);
            ret = hal_uart_init(cfg, g_xmdm_ctx.port);

            if(ret)
            {
                g_xmdm_ctx.xmdm_cb(xmdm_cb_err_bus, NULL);
                osal_memset(&g_xmdm_ctx, 0, sizeof(g_xmdm_ctx));
                return ( events ^ XMDM_START_RECV_EVT );
            }

            g_xmdm_ctx.st = XMDM_ST_START;
        }

        if(g_xmdm_ctx.st == XMDM_ST_START)
            osal_start_timerEx(task_id,XMDM_START_RECV_EVT, 1000);

        if(g_xmdm_ctx.trychar)
        {
            port_outbyte(g_xmdm_ctx.trychar);
            g_xmdm_ctx.trycnt ++;
        }

        if(g_xmdm_ctx.trycnt >= XMDM_TRYCH_NUMBER)
        {
            if(g_xmdm_ctx.xmdm_cb)
                g_xmdm_ctx.xmdm_cb(xmdm_cb_err_retry, NULL);
        }

        g_xmdm_ctx.block_id = 1;
        return ( events ^ XMDM_START_RECV_EVT );
    }

    if ( events & XMDM_RX_DATA_EVT)
    {
        uint8_t* s_xbuff = g_xmdm_ctx.block_buf;
        uint32_t size = g_xmdm_ctx.xmit_len;
        uint16_t block_size = g_xmdm_ctx.block_size;

        if (s_xbuff[1] == (uint8_t)(~s_xbuff[2]) &&
                (s_xbuff[1] == g_xmdm_ctx.block_id || s_xbuff[1] == (uint8_t)g_xmdm_ctx.block_id-1) &&
                check(1, &s_xbuff[3], block_size))
        {
            if (s_xbuff[1] == g_xmdm_ctx.block_id)
            {
                uint32_t count = size - g_xmdm_ctx.xmit_off;

                if (count > block_size)
                    count = block_size;

                XMDM_LOG("XMDM_RX_DATA_EVT %x, %x, %x\n",g_xmdm_ctx.xmit_len, g_xmdm_ctx.xmit_off, count);

                if (count > 0)
                {
                    //get effective data
                    xmdm_cb_param_t param =
                    {
                        .addr = g_xmdm_ctx.xmit_off,
                        .buf = s_xbuff + 3,
                        .len = count,
                    };
                    ret = g_xmdm_ctx.xmdm_cb(xmdm_cb_write, &param);

                    if(! ret)
                    {
                        g_xmdm_ctx.xmdm_cb(xmdm_cb_err_io, NULL);
                    }
                    else
                    {
                        g_xmdm_ctx.xmit_off += count;
                    }
                }

                ++g_xmdm_ctx.block_id;
                g_xmdm_ctx.retrans = XMDM_MAXRETRANS+1;
            }

            if (--g_xmdm_ctx.retrans <= 0)
            {
                port_outbyte(CAN);
                port_outbyte(CAN);
                port_outbyte(CAN);
                g_xmdm_ctx.xmdm_cb(xmdm_cb_err_retry, NULL);
            }
            else
            {
                g_xmdm_ctx.st = XMDM_ST_RX_HEAD;
                XMDM_LOG("ACK\n");
                port_outbyte(ACK);
            }
        }

        return ( events ^ XMDM_RX_DATA_EVT);
    }

    if ( events & XMDM_CANCEL_RECV_EVT )
    {
        port_outbyte(ACK);
        g_xmdm_ctx.xmdm_cb(xmdm_cb_rsp_cancel, NULL);
        osal_memset(&g_xmdm_ctx, 0, sizeof(g_xmdm_ctx));
        return ( events ^ XMDM_CANCEL_RECV_EVT );
    }

    if ( events & XMDM_FINISH_RECV_EVT )
    {
        port_outbyte(ACK);
        g_xmdm_ctx.xmdm_cb(xmdm_cb_rsp_finished, NULL);
        osal_memset(&g_xmdm_ctx, 0, sizeof(g_xmdm_ctx));
        return ( events ^ XMDM_FINISH_RECV_EVT );
    }

    return 0;
}




