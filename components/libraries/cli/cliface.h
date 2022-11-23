
/**
    \file cliface.h

    This file contains definitions for command line interface (CLI) framework.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_CLIFACE_
#define _H_CLIFACE_

/* --------------------------------------------- Header File Inclusion */
#include "types.h"
#include "error.h"
#include "string.h"
#include "log.h"


/* --------------------------------------------- Macros */

/* Debug Macros */
/* TBD: Mapped with debug sub-system */
#define CLI_OUT(...)       {LOG(__VA_ARGS__);LOG("\r\n");}
#define CLI_OUTc(...)       LOG(__VA_ARGS__)
#define CLI_ERR(...)       /* printf(__VA_ARGS__) */
#define CLI_TRC(...)       /* printf(__VA_ARGS__) */
#define CLI_INF(...)       /* printf(__VA_ARGS__) */

#define CLI_STR_COMPARE(s0, s1)    strcmp((const char *)(s0), (const char *)(s1))

#define CLI_NULL_CHECK(ptr) \
    if (NULL == (ptr)) \
    {\
        CLI_ERR( \
                 "[CLI] NULL Pointer\n"); \
        \
        return PPlus_ERR_INVALID_DATA; \
    }

#define CLI_IS_WHITE_SPACE(ch)   ((' ' == (ch)) || ('\t' == (ch)))
#define CLI_IS_CMD_SEPARATOR(ch) ((' ' == (ch)) || ('\t' == (ch)) || ('\r' == (ch)) || ('\n' == (ch)))

/** TBD: Move to limits/configuration header file */
#define CLI_MAX_ARGS          16
#define CLI_MAX_PROMPT_LEN    16

#define CLI_strlen(s)   EM_str_len(s)

enum

{
    CLI_CMD_MODE_TASK = 0,    //run cli command in osal task
    CLI_CMD_MODE_LOOP = 1,    //run cli command in main loop
    CLI_CMD_MODE_INTERRUPT = 2 //run cli command just in irq
};
#define CLI_OSAL_MSG_EVENT    0xf3

/* --------------------------------------------- Data Types/ Structures */
/**
    CLI command handler.

    CLI will call the handler for the received command.

    \param [in] argc    Number of arguments.
    \param [in] argv    List of arguments.
*/
typedef uint16_t (* CLI_CMD_HANDLER)
(
    uint32_t        argc,
    uint8_t*        argv[]
) ;

/** This data structure represents a CLI command */
typedef struct _cli_command
{
    /** Command name */
    const char*            cmd;

    /* Command description */
    const char*            desc;

    /** Command handler */
    const CLI_CMD_HANDLER   cmd_hdlr;

} CLI_COMMAND;

/* --------------------------------------------- Global Definitions */
extern CLI_COMMAND* g_cli_cmd_list;
extern uint8_t g_cli_cmd_len;

/* --------------------------------------------- Functions */
int CLI_help(CLI_COMMAND* cmd_list, uint16_t cmd_num);
int CLI_init(CLI_COMMAND* cmd_list, uint16_t cmd_num);


int CLI_process_line(void);

int CLI_process_line_manual(uint8_t* buffer,uint32_t buffer_len);

int CLI_strtoi
(
    /* IN */ uint8_t* data,
    /* IN */ uint16_t data_length,
    /* IN */ uint8_t base
);

int CLI_strtoarray
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
);

int CLI_strtoarray_le
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
);
char*  CLIT_preamble(const uint8_t* preamble);

int CLI_uart_init(
    UART_INDEX_e port,
    gpio_pin_e tx,
    gpio_pin_e rx,
    uint32_t baudrate,
    const char* prompt
);

#define CLIT_output(...) {_uart_putc(CLIT_preamble(NULL),4); CLI_OUT(__VA_ARGS__);}
#define CLIT_outputs(...) {_uart_putc(CLIT_preamble(NULL),4); CLI_OUTc(__VA_ARGS__);}
#define CLIT_outputc(...) CLI_OUTc(__VA_ARGS__)
#define CLIT_outpute(...) CLI_OUT(__VA_ARGS__)

#define CLI_UART_INIT(prompt) CLI_uart_init(UART0, P9, P10, 115200, prompt)

#endif /* _H_CLIFACE_ */


