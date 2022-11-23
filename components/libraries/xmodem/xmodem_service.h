
#ifndef __XMDM_SV_H
#define __XMDM_SV_H

#include "types.h"


typedef struct
{
    uint32_t addr;
    uint32_t len;
    uint8_t* buf;
} xmdm_cb_param_t;

typedef bool (*xmdm_cb_t)(int, xmdm_cb_param_t*);

bool xmdm_data_callback_default(int rsp_val, xmdm_cb_param_t* param);

int xmodem_service_xmit(UART_INDEX_e port, int boardrate,
                        gpio_pin_e tx, gpio_pin_e rx,
                        uint32_t xmit_size, xmdm_cb_t cb);

void xmodem_service_init( uint8 task_id );
uint16_t xmodem_service_processevent( uint8 task_id, uint16 events );

#endif /*__XMDM_SV_H*/
