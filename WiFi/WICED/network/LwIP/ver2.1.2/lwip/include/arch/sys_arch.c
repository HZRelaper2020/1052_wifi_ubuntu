/*
 * sys_arch.c
 *
 *  Created on: 2021Äê5ÔÂ8ÈÕ
 *      Author: nwz
 */

#include "sys_arch.h"
#include "lwip/err.h"

#define archMESG_QUEUE_LENGTH     ( (unsigned long) 6 )
#define archPOST_BLOCK_TIME_MS    ( ( unsigned long ) 10000 )


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

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, /*@null@*/ /*@out@*/ void **msg, u32_t timeout)
{
    void *dummyptr;
    void ** tmp_ptr;
    TickType_t start_time, end_time, elapsed_time;

    start_time = xTaskGetTickCount( );

    if ( msg == NULL )
    {
        tmp_ptr = &dummyptr;
    }
    else
    {
        tmp_ptr = msg;
    }

    if ( timeout != 0 )
    {
        if ( pdTRUE == xQueueReceive( *mbox, tmp_ptr, timeout ) )
        {
            end_time = xTaskGetTickCount( );
            elapsed_time = end_time - start_time;
            if ( elapsed_time == 0 )
            {
                elapsed_time = (TickType_t) 1;
            }
            return ( elapsed_time );
        }
        else /* timed out blocking for message */
        {
            if ( msg != NULL )
            {
                *msg = NULL;
            }
            return SYS_ARCH_TIMEOUT;
        }
    }
    else /* block forever for a message. */
    {
        if ( xQueueReceive( *mbox, &(*tmp_ptr), (TickType_t) 0xFFFFFFFF ) == errQUEUE_EMPTY)
        {
            *tmp_ptr = NULL;
            return SYS_ARCH_TIMEOUT;
        }

        end_time = xTaskGetTickCount( );
        elapsed_time = end_time - start_time;
        if ( elapsed_time == 0 )
        {
            elapsed_time = (TickType_t) 1;
        }
        return ( elapsed_time ); /* return time blocked TBD test */
    }
}

err_t sys_mbox_new( /*@special@*/ /*@out@*/ sys_mbox_t *mbox, /*@unused@*/int size ) /*@allocates *mbox@*/  /*@defines **mbox@*/
{
    /*@-noeffect@*/
    (void) size; /* unused parameter */
    /*@+noeffect@*/

    *mbox = xQueueCreate( archMESG_QUEUE_LENGTH, (unsigned long) sizeof(void *) );

    /*@-compdef@*/ /* Lint doesnt realise allocation has occurred */
    return ERR_OK;
    /*@+compdef@*/
}

typedef void (*lwip_thread_fn)(void *arg);
sys_thread_t sys_thread_new( const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio )
{
    signed portBASE_TYPE result;
    sys_thread_t thread_out;
    /* The first time this is called we are creating the lwIP handler. */
    result = xTaskCreate( thread, name, (unsigned short) (stacksize / sizeof( portSTACK_TYPE )), arg, (unsigned portBASE_TYPE) prio, &thread_out );

    /* LwIP requires that this function always succeed */
    LWIP_ASSERT("Error creating thread", result == pdTRUE );

#if LWIP_SYS_ARCH_TIMEOUTS
    /* For each task created, store the task handle (pid) in the timers array.
     * This scheme doesn't allow for threads to be deleted
     */
    timeout_list[next_thread++].pid = created_task;
#endif /* if LWIP_SYS_ARCH_TIMEOUTS */

    return thread_out;
}

int sys_sem_valid( sys_sem_t *sem )
{
    return (int)( *sem != 0 );
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    void *dummy_ptr;
    void ** tmp_ptr = msg;

    if ( msg == NULL )
    {
        tmp_ptr = &dummy_ptr;
    }

    if ( pdTRUE == xQueueReceive( *mbox, tmp_ptr, 0 ) )
    {
        return ERR_OK;
    }
    else
    {
        return SYS_MBOX_EMPTY;
    }
}

void sys_mbox_free( /*@special@*/ sys_mbox_t *mbox ) /*@releases *mbox@*/
{
    if ( uxQueueMessagesWaiting( *mbox ) != 0 )
    {
        /* Line for breakpoint.  Should never break here! */
#ifdef __GNUC__
        __asm volatile ( "NOP" );
#elif defined( __IAR_SYSTEMS_ICC__ )
        asm( "NOP" );
#endif
    }

    vQueueDelete( *mbox );
}
void sys_mbox_set_invalid( sys_mbox_t *mbox )
{
    ( *(int*) mbox ) = 0;
}
void sys_sem_set_invalid( sys_sem_t *sem )
{
    ( *(int*) sem ) = 0;
}
