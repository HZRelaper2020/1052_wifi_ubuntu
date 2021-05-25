/*
 * sys_arch.c
 *
 *  Created on: 2021Äê5ÔÂ8ÈÕ
 *      Author: nwz
 */

#include "sys_arch.h"
#include "lwip/err.h"

u32_t sys_now(void) {
	return xTaskGetTickCount();
}

/*-----------------------------------------------------------------------------------
 * Creates and returns a new semaphore. The "count" argument specifies
 * the initial state of the semaphore. TBD finish and test
 */
err_t sys_sem_new( /*@special@*/ /*@out@*/ sys_sem_t *sem, u8_t count) /*@allocates *sem@*/
{
    if ( sem == NULL )
    {
        /*@-mustdefine@*/ /*@-compdef@*/ /* do not need to allocate or set *sem */
        return ERR_VAL;
        /*@+mustdefine@*/ /*@+compdef@*/
    }

    portENTER_CRITICAL();
    vSemaphoreCreateBinary( *sem )
    if ( *sem == NULL )
    {
        portEXIT_CRITICAL();
        /*@-mustdefine@*/ /*@-compdef@*/ /* allocation failed - dont allocate or set *sem */
        return ERR_MEM;
        /*@+mustdefine@*/ /*@+compdef@*/
    }

    if ( count == (u8_t) 0 ) /* Means it can't be taken */
    {
        if ( pdTRUE != xSemaphoreTake( *sem, (TickType_t) 1 ) )
        {
            vQueueDelete( *sem );
            portEXIT_CRITICAL();
            /*@-compdef@*/ /* take failed - dont allocate or set *sem */
            return ERR_VAL;
            /*@+compdef@*/
        }
    }

    portEXIT_CRITICAL();

    /*@-compdef@*/ /* Lint doesnt realise allocation has occurred */
    return ERR_OK;
    /*@+compdef@*/
}

/*-----------------------------------------------------------------------------------
 * Blocks the thread while waiting for the semaphore to be
 * signaled. If the "timeout" argument is non-zero, the thread should
 * only be blocked for the specified time (measured in
 * milliseconds).
 *
 * If the timeout argument is non-zero, the return value is the number of
 * milliseconds spent waiting for the semaphore to be signaled. If the
 * semaphore wasn't signaled within the specified time, the return value is
 * SYS_ARCH_TIMEOUT. If the thread didn't have to wait for the semaphore
 * (i.e., it was already signaled), the function may return zero.
 *
 * Notice that lwIP implements a function with a similar name,
 * sys_sem_wait(), that uses the sys_arch_sem_wait() function.
 */
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    TickType_t start_time, end_time, elapsed_time;

    start_time = xTaskGetTickCount( );

    if ( timeout != 0 )
    {
        if ( xSemaphoreTake( *sem, timeout ) == (long) pdTRUE )
        {
            end_time = xTaskGetTickCount( );
            elapsed_time = end_time - start_time;
            if ( elapsed_time == 0 )
            {
                elapsed_time = (TickType_t) 1;
            }
            return ( elapsed_time ); /* return time blocked TBD test */
        }
        else
        {
            return SYS_ARCH_TIMEOUT;
        }
    }
    else /* must block without a timeout */
    {
        while ( xSemaphoreTake( *sem, (TickType_t) 10000 ) != (long) pdTRUE )
        {
            /* Do nothing */
        }
        end_time = xTaskGetTickCount( );
        elapsed_time = end_time - start_time;
        if ( elapsed_time == 0 )
        {
            elapsed_time = (TickType_t) 1;
        }

        return ( elapsed_time ); /* return time blocked */

    }
}

/*-----------------------------------------------------------------------------------
 * Signals a semaphore
 */
void sys_sem_signal( sys_sem_t *sem )
{
    signed portBASE_TYPE result;

    result = xSemaphoreGive( *sem );
    /*@-noeffect@*/
    (void) result;  /* unused in release build */
    /*@+noeffect@*/

//    LWIP_ASSERT( "FreeRTOS failed to set semaphore for LwIP", result == pdTRUE );
}

/*-----------------------------------------------------------------------------------
 * Deallocates a semaphore
 */
void sys_sem_free( /*@special@*/ sys_sem_t *sem ) /*@releases *sem@*/
{
    vQueueDelete( *sem );
}

/*-----------------------------------------------------------------------------------
 * Initialize sys arch
 */
void sys_init( void )
{
}

void sys_deinit( void )
{
}

int sys_mbox_valid( sys_mbox_t *mbox )
{
    return (int)( *mbox != 0 );
}

/*-----------------------------------------------------------------------------------
 * Try to post the "msg" to the mailbox.
 */
err_t sys_mbox_trypost( sys_mbox_t *mbox, void *msg )
{
    err_t result;

    if ( xQueueSend( *mbox, &msg, 0 ) == pdPASS )
    {
        result = ERR_OK;
    }
    else
    {
        /* could not post, queue must be full */
        result = ERR_MEM;


    }

    return result;
}

