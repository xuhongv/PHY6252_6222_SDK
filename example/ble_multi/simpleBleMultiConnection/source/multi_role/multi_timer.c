/*
    All rights reserved
*/
#include "types.h"
#include "multi_timer.h"
#include "osal.h"
#include "osal_memory.h"
// #include "log.h"
#include "multi.h"
//timer handle list head. master role use head_handle list, slave role use slave_head_handle list.
struct multiTimer* head_handle = NULL;
struct multiTimer* slave_head_handle = NULL;
//Timer ticks
static uint32 _timer_ticks = 0;
///
extern multiTimer* g_peri_conn_update_timer[MAX_CONNECTION_SLAVE_NUM];
extern multiTimer* g_pcu_no_success_timer[MAX_CONNECTION_SLAVE_NUM];
/**
    @brief  Initializes the timer struct handle.
    @param  handle: the timer handle strcut.
    @param  timeout_cb: timeout callback.
    @param  repeat: repeat interval time.
    @retval None
*/
void multitimer_init(struct multiTimer* handle, void(*timeout_cb)(uint16 idx), uint32 timeout, uint32 repeat,uint32 id)
{
    osal_memset(handle, sizeof(struct multiTimer), 0);
    handle->timeout_cb = timeout_cb;
    handle->timeout = _timer_ticks + timeout;
    handle->repeat = repeat;
    handle->id = id;
}

/**
    @brief  Start the timer work, add the handle into work list.
    @param  btn: target handle strcut.
    @retval 0: succeed. -1: already exist.
*/
int multitimer_start(struct multiTimer* handle)
{
    struct multiTimer* target = head_handle;

    while(target)
    {
        if(target == handle) return -1; //already exist.

        target = target->next;
    }

    handle->next = head_handle;
    head_handle = handle;
    return 0;
}

int multitimer_start_slave(struct multiTimer* handle)
{
    struct multiTimer* target = slave_head_handle;

    while(target)
    {
        if(target == handle) return -1; //already exist.

        target = target->next;
    }

    handle->next = slave_head_handle;
    slave_head_handle = handle;
    return 0;
}

void multitimer_slave_del(struct multiTimer* handle)
{
    multiTimer* conn_update_entry = g_peri_conn_update_timer[0];
    multiTimer* pcu_no_success_entry = g_pcu_no_success_timer[0];

    if(conn_update_entry != NULL || pcu_no_success_entry != NULL)
    {
        for(uint8 i = 0 ; i < MAX_CONNECTION_SLAVE_NUM; i++)
        {
            if(conn_update_entry == handle)
            {
                // AT_LOG("del g_peri_conn_update_timer timer\n");
                g_peri_conn_update_timer[i] = NULL;
            }
            else if (pcu_no_success_entry == handle)
            {
                // AT_LOG("del g_pcu_no_success_timer timer\n");
                g_pcu_no_success_timer[i] = NULL;
            }

            conn_update_entry = conn_update_entry->next;
            pcu_no_success_entry = pcu_no_success_entry->next;
        }
    }
}


void multitimer_stop_slave(struct multiTimer* handle)
{
    struct multiTimer** curr;

    for(curr = &slave_head_handle; *curr; )
    {
        struct multiTimer* entry = *curr;

        if (entry == handle)
        {
            // AT_LOG("timer stop : %p id %d  repeat:%d\n",entry,entry->id,entry->repeat);
            *curr = entry->next;
            /// The use of osal_mem_free causes hard fault
            /// think about dynamic alloc multi timer buffer
            osal_mem_free(entry);
            multitimer_slave_del(entry);
        }
        else
            curr = &entry->next;
    }
}

void multitimer_loop_slave()
{
    struct multiTimer* target;

    for(target=slave_head_handle; target; target=target->next)
    {
        if(_timer_ticks >= target->timeout)
        {
            target->timeout_cb(target->id);

            if(target->repeat == 0)
            {
                multitimer_stop_slave(target);
            }
            else
            {
                target->timeout = _timer_ticks + target->repeat;
            }
        }
    }
}


/**
    @brief  Stop the timer work, remove the handle off work list.
    @param  handle: target handle strcut.
    @retval None
*/
void multitimer_stop(struct multiTimer* handle)
{
    struct multiTimer** curr;

    for(curr = &head_handle; *curr; )
    {
        struct multiTimer* entry = *curr;

        if (entry == handle)
        {
            // AT_LOG("timer stop : %p id %d  repeat:%d\n",entry,entry->id,entry->repeat);
            *curr = entry->next;
            /// The use of osal_mem_free causes hard fault
            /// think about dynamic alloc multi timer buffer
            osal_mem_free(entry);
        }
        else
            curr = &entry->next;
    }
}

/**
    @brief  main loop.
    @param  None.
    @retval None
*/
void multitimer_loop()
{
    struct multiTimer* target;

    for(target=head_handle; target; target=target->next)
    {
        if(_timer_ticks >= target->timeout)
        {
            target->timeout_cb(target->id);

            if(target->repeat == 0)
            {
                multitimer_stop(target);
            }
            else
            {
                target->timeout = _timer_ticks + target->repeat;
            }
        }
    }
}

/**
    @brief  background ticks, timer repeat invoking interval 1ms.
    @param  None.
    @retval None.
*/
void multitimer_ticks( uint32 tick )
{
    _timer_ticks += tick ;
}

