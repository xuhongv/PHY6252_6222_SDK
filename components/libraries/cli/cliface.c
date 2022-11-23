/**
    \file cli.c

    This file implements command line interface (CLI) framework.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "cliface.h"
#include "uart.h"
#include "OSAL.h"



#ifndef CLI_CMD_MODE
    #define CLI_CMD_MODE CLI_CMD_MODE_TASK

#endif
CLI_COMMAND* g_cli_cmd_list;
uint8_t g_cli_cmd_len;

//#ifdef HAVE_CLI
#define CLI_CMD_BUF_LEN 1
typedef struct
{
    CLI_COMMAND*    cmd_list;
    uint16_t        cmd_num;
    uint32_t        buf_off;
    uint8_t         buf[CLI_CMD_BUF_LEN];
    char            prompt[CLI_MAX_PROMPT_LEN];

    CLI_CMD_HANDLER cmd;
    uint8_t*        argv[CLI_MAX_ARGS];

    //CLIT
    uint8_t         preamble[4];//4 bytes preamble for auto test output

} cli_ctx_t;

cli_ctx_t g_cli_ctx =
{
    .cmd_list = NULL,
    .cmd_num = 0,
    .buf_off = 0,
    .preamble = {0xdb, 0x74, 0x53, 0x74}
};


/* --------------------------------------------- Global Definitions */

void CLI_cmd_exec(CLI_CMD_HANDLER cmd, uint32_t argc, uint8_t** argv)
{
    cli_ctx_t* p_ctx = & g_cli_ctx;
    cmd(argc, argv);
    //output prompt
    CLI_OUT("\n%s",p_ctx->prompt);
}

#if(CLI_CMD_MODE == CLI_CMD_MODE_TASK)
typedef struct
{
    osal_event_hdr_t  hdr;
    CLI_CMD_HANDLER   cmd;
    uint32_t          argc;
    uint8_t**         argv;
} CLI_cmd_msg_t;

void __attribute__((weak)) CLI_msg_send(uint8_t* pmsg)
{
}

void __attribute__((weak)) CLI_cmd_post_task(
    CLI_CMD_HANDLER cmd,
    uint32_t argc,
    uint8_t** argv
)
{
    CLI_cmd_msg_t* pmsg = (CLI_cmd_msg_t*)osal_msg_allocate(sizeof(CLI_cmd_msg_t));

    if(!pmsg)
    {
        CLI_ERR("[CLI] cli command msg alloc failed\n");
        return;
    }

    pmsg->cmd = cmd;
    pmsg->argc = argc;
    pmsg->argv = argv;
    pmsg->hdr.event = CLI_OSAL_MSG_EVENT;
    pmsg->hdr.status = 0;
    CLI_msg_send((uint8_t*)pmsg);
}


void CLI_cmd_msg_handler(osal_event_hdr_t* pmsg)
{
    CLI_cmd_msg_t* pclimsg = (CLI_cmd_msg_t*)pmsg;
    CLI_cmd_exec(pclimsg->cmd, pclimsg->argc, pclimsg->argv);
}

#elif (CLI_CMD_MODE == CLI_CMD_MODE_INTERRUPT)

void __attribute__((weak)) CLI_cmd_post_task(
    CLI_CMD_HANDLER cmd,
    uint32_t argc,
    uint8_t** argv
)
{
    CLI_cmd_exec(cmd, argc, argv);
}

#endif
void CLI_uart_process_data(uart_Evt_t* pev)//uint8_t* buf, uint32_t len)
{
    cli_ctx_t* p_ctx = & g_cli_ctx;
    uint32_t i;

    for(i = 0; i< pev->len; i++)
    {
        if(p_ctx->buf_off >= CLI_CMD_BUF_LEN)
        {
            p_ctx->buf_off = 0;
            return;
        }

        if(pev->data[i] == '\r' || pev->data[i] == '\n')
        {
            //command ready
            p_ctx->buf[p_ctx->buf_off] = '\0';
            CLI_process_line();
            p_ctx->buf_off = 0;
            return;
        }

        if(pev->data[i] < 0x20 || pev->data[i] > 0x7d)
        {
            LOG("Parse failed, cmd has illegal character\n");
            return;
        }

        p_ctx->buf[p_ctx->buf_off ++] = pev->data[i];
    }
}

int CLI_uart_init(
    UART_INDEX_e port,
    gpio_pin_e tx,
    gpio_pin_e rx,
    uint32_t baudrate,
    const char* prompt
)
{
    cli_ctx_t* p_ctx = & g_cli_ctx;
    uart_Cfg_t cfg =
    {
        .tx_pin = tx,
        .rx_pin = rx,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = CLI_uart_process_data,
    };

    if(strlen(prompt) >= CLI_MAX_PROMPT_LEN)
        return PPlus_ERR_INVALID_PARAM;

    strcpy(p_ctx->prompt, prompt);
    return hal_uart_init(cfg, port);//uart init
}


int CLI_help(CLI_COMMAND* cmd_list, uint16_t cmd_num)
{
    uint16_t i;

    for(i = 0; i< cmd_num; i++)
    {
        CLI_OUT("cmd: %s		%s", cmd_list[i].cmd,cmd_list[i].desc);
    }

    return 0;
}

/* --------------------------------------------- Function */
/**
    \fn CLI_init

    \brief Initialize CLI

    \Description
    This routine intializes CLI.

    \return EM_SUCCESS or an error code indicating reason for failure

*/
int CLI_init(CLI_COMMAND* cmd_list, uint16_t cmd_num)
{
    cli_ctx_t* p_ctx = & g_cli_ctx;
    //osal_memset((void*)p_ctx, 0, sizeof(cli_ctx_t));
    p_ctx->cmd_list = cmd_list;
    p_ctx->cmd_num = cmd_num;
    g_cli_cmd_list = cmd_list;
    g_cli_cmd_len = cmd_num;
    p_ctx->buf_off = 0;
    CLI_OUT("\n%s",p_ctx->prompt);
    return PPlus_SUCCESS;
}

/**
    \brief Process a command line instruction

    \Description
    This routine processes a command line instruction.

    \param [in] buffer        Buffer containing a command
    \param [in] buffer_len    Length of command in buffer
    \param [in] cmd_list      Command List
    \param [in] cmd_count     Number of command in the list

    \return EM_SUCCESS or an error code indicating reason for failure
*/
int CLI_process_line(void)
{
    cli_ctx_t* p_ctx = & g_cli_ctx;
    uint8_t* buffer = p_ctx->buf;
    uint32_t buffer_len = p_ctx->buf_off;
    uint32_t  argc;
    uint8_t*   cmd;
    uint16_t   index;

    /* Skip initial white spaces */
    for (; CLI_IS_WHITE_SPACE(*buffer) && (0 != buffer_len); buffer++, buffer_len--);

    /* Check this is not an empty command line */
    if (0 == buffer_len)
    {
        CLI_ERR("[CLI] Empty command line\n");
        return PPlus_ERR_INVALID_LENGTH;
    }

    /**
        Got the initial command.
        Parse the remaining command line to get the arguments.
    */
    argc = 0;

    for (cmd = buffer + 1; cmd < (buffer + buffer_len); cmd++)
    {
        /**
            If command argument separator is detected, replace with '\0'
            to create each as separate strings.
        */
        if (CLI_IS_CMD_SEPARATOR(*cmd))
        {
            *cmd = '\0';
        }
        /* Check if this is start of a new argument */
        else if ('\0' == (*(cmd - 1)))
        {
            p_ctx->argv[argc++] = cmd;
        }
        else
        {
            /* Nothing to do */
        }
    }

    CLI_TRC("[CLI] Command %s, Number of arguments %d\n", buffer, argc);
    {
        uint8_t ai;

        for (ai = 0; ai < argc; ai++)
        {
            CLI_TRC("Arg [%02X] %s\n", ai, argv[ai]);
        }
    }
    /* Identified command name */
    cmd = buffer;

    /* Search command and call associated callback */
    for (index = 0; index < p_ctx->cmd_num; index++)
    {
        if (0 == CLI_STR_COMPARE(buffer, p_ctx->cmd_list[index].cmd))
        {
            CLI_cmd_post_task(p_ctx->cmd_list[index].cmd_hdlr, argc, p_ctx->argv);
            break;
        }
    }

    return PPlus_SUCCESS;
}

int CLI_process_line_manual(uint8_t* buffer,uint32_t buffer_len)
{
    uint32_t  argc;
    uint8_t*   argv[CLI_MAX_ARGS];
    uint8_t*   cmd;
    uint32_t   index;
    /* TBD: Parameter Validation */
    CLI_NULL_CHECK(buffer);

    /* Skip initial white spaces */
    for (; CLI_IS_WHITE_SPACE(*buffer) && (0 != buffer_len); buffer++, buffer_len--);

    /* Check this is not an empty command line */
    if (0 == buffer_len)
    {
        CLI_ERR("[CLI] Empty command line\n");
        return PPlus_ERR_INVALID_LENGTH;
    }

    /**
        Got the initial command.
        Parse the remaining command line to get the arguments.
    */
    argc = 0;

    for (cmd = buffer + 1; cmd < (buffer + buffer_len); cmd++)
    {
        /**
            If command argument separator is detected, replace with '\0'
            to create each as separate strings.
        */
        if (CLI_IS_CMD_SEPARATOR(*cmd))
        {
            *cmd = '\0';
        }
        /* Check if this is start of a new argument */
        else if ('\0' == (*(cmd - 1)))
        {
            argv[argc++] = cmd;
        }
        else
        {
            /* Nothing to do */
        }
    }

    CLI_TRC("[CLI] Command %s, Number of arguments %d\n", buffer, argc);
    {
        uint8_t ai;

        for (ai = 0; ai < argc; ai++)
        {
            CLI_TRC("Arg [%02X] %s\n", ai, argv[ai]);
        }
    }
    /* Identified command name */
    cmd = buffer;

    /* Search command and call associated callback */
    for (index = 0; index < g_cli_cmd_len; index++)
    {
        if (0 == CLI_STR_COMPARE(buffer, g_cli_cmd_list[index].cmd))
        {
            g_cli_cmd_list[index].cmd_hdlr(argc, argv);
            break;
        }
    }

    return PPlus_SUCCESS;
}


/* TODO: Create a separe utility module or move to a common utility module */
/* Supporting Macros */
#define IS_SPACE(c) ((' ' == (c)) || ('\t' == (c)))
#define IS_DIGIT(c) (('0' <= (c)) && ('9' >= (c)))
#define IS_UPPER(c) (('A' <= (c)) && ('F' >= (c)))
#define IS_LOWER(c) (('a' <= (c)) && ('f' >= (c)))
#define IS_ALPHA(c) IS_LOWER(c) || IS_UPPER(c)

/* Convert string to Integer */
int CLI_strtoi(
    /* IN */ uint8_t* data,
    /* IN */ uint16_t data_length,
    /* IN */ uint8_t base
)
{
    int32_t  value;
    uint16_t index;
    int8_t   sign_adj;
    uint8_t  c;
    c = 0;

    /* Skip Whitespaces */
    for (index = 0; index < data_length; index++)
    {
        c = data[index];

        if (IS_SPACE(c))
        {
            continue;
        }
        else
        {
            break;
        }
    }

    value = 0;
    sign_adj = 1;

    /* Check Sign */
    if ('-' == c)
    {
        sign_adj = (int8_t)-1;
        index++;
    }

    /* Not handling spaces after '-' or '0x' etc. */
    for (; index < data_length; index++)
    {
        c = data[index];

        /* Check if Digit */
        if (IS_DIGIT(c))
        {
            value *= base;
            value += (c - '0');
        }
        else if (IS_LOWER(c))
        {
            value *= base;
            value += (c - 'a' + 10);
        }
        else if (IS_UPPER(c))
        {
            value *= base;
            value += (c - 'A' + 10);
        }
        else
        {
            break;
        }
    }

    return (sign_adj * value);
}

/* Convert string to Integer Array */
int CLI_strtoarray
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
)
{
    int32_t  index;
    uint8_t  c0, c1;
    uint8_t  base;
    uint16_t output_index;
    /* HEX */
    base = 16;
    c0 = 0;
    c1 = 0;
    /* Fill with Zeros */
    memset(output_array, 0, output_array_len);

    /* Check the length */
    if (data_length > (2 * output_array_len))
    {
        return PPlus_ERR_INVALID_LENGTH;
    }

    /* Process from end */
    output_index = output_array_len - 1;

    for (index = data_length - 1; index >= 0; index -= 2)
    {
        if (0 != index)
        {
            c1 = data[index];
            c0 = data[index - 1];
        }
        else
        {
            c1 = data[index];
            c0 = '0';
        }

        /* Check if Digit */
        if (IS_DIGIT(c0))
        {
            c0 = (c0 - '0');
        }
        else if (IS_LOWER(c0))
        {
            c0 = (c0 - 'a' + 10);
        }
        else if (IS_UPPER(c0))
        {
            c0 = (c0 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        /* Check if Digit */
        if (IS_DIGIT(c1))
        {
            c1 = (c1 - '0');
        }
        else if (IS_LOWER(c1))
        {
            c1 = (c1 - 'a' + 10);
        }
        else if (IS_UPPER(c1))
        {
            c1 = (c1 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        output_array[output_index] = c0 * base + c1;
        output_index--;
    }

    return PPlus_SUCCESS;
}

/* Convert string to Integer Array in Little Endian Packing */
int CLI_strtoarray_le
(
    /* IN */  uint8_t*   data,
    /* IN */  uint16_t   data_length,
    /* OUT */ uint8_t*   output_array,
    /* IN */  uint16_t   output_array_len
)
{
    int32_t  index;
    uint8_t  c0, c1;
    uint8_t  base;
    uint16_t output_index;
    /* HEX */
    base = 16;
    c0 = 0;
    c1 = 0;
    /* Fill with Zeros */
    memset(output_array, 0, output_array_len);

    /* Check the length */
    if (data_length > (2 * output_array_len))
    {
        return PPlus_ERR_INVALID_LENGTH;
    }

    /* Process from end */
    output_index = 0;

    for (index = data_length - 1; index >= 0; index -= 2)
    {
        if (0 != index)
        {
            c1 = data[index];
            c0 = data[index - 1];
        }
        else
        {
            c1 = data[index];
            c0 = '0';
        }

        /* Check if Digit */
        if (IS_DIGIT(c0))
        {
            c0 = (c0 - '0');
        }
        else if (IS_LOWER(c0))
        {
            c0 = (c0 - 'a' + 10);
        }
        else if (IS_UPPER(c0))
        {
            c0 = (c0 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        /* Check if Digit */
        if (IS_DIGIT(c1))
        {
            c1 = (c1 - '0');
        }
        else if (IS_LOWER(c1))
        {
            c1 = (c1 - 'a' + 10);
        }
        else if (IS_UPPER(c1))
        {
            c1 = (c1 - 'A' + 10);
        }
        else
        {
            return PPlus_ERR_INVALID_PARAM;
        }

        output_array[output_index] = c0 * base + c1;
        output_index++;
    }

    return PPlus_SUCCESS;
}



/**************************************************************/
/*                                                            */
/*                CLI for auto-test                           */
/*                                                            */
/**************************************************************/
char* CLIT_preamble(const uint8_t* preamble)
{
    cli_ctx_t* p_ctx = & g_cli_ctx;

    if(preamble)
        osal_memcpy(p_ctx->preamble, preamble, 4);

    return (char*)p_ctx->preamble;
}



//#endif /* HAVE_CLI */

