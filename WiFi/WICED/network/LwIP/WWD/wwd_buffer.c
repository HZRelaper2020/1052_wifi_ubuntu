/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "network/wwd_buffer_interface.h"
#include "platform/wwd_bus_interface.h"
#include "lwip/netbuf.h"
#include "lwip/memp.h"
#include <string.h> /* for NULL */
#include "wwd_assert.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wiced_utilities.h"
#include "wwd_wifi.h"

//#define PBUF_POOL_RX PBUF_POOL
//#define PBUF_POOL_TX PBUF_RAW
#define PBUF_POOL_RX 0x182
#define PBUF_POOL_TX 0x280
static struct memp_desc *application_defined_rx_pool = NULL;
static struct memp_desc *application_defined_tx_pool = NULL;

void memp_free_notify( unsigned int type );

#define BUF_NUMS 30
static struct pbuf g_pbuf[BUF_NUMS];
static uint8_t g_buf[4096*BUF_NUMS];
static int g_curnum=0;

static wiced_buffer_t my_pbuf_alloc(int type,int size,int nouse1)
{
	wiced_assert("malloc size is too big",size < 4096);

	g_curnum++;
	if (g_curnum >= BUF_NUMS)  g_curnum = 0;

	struct pbuf* p = &g_pbuf[g_curnum];
	
	memset(p,0,sizeof(struct pbuf));
	p->payload = g_buf+4096*g_curnum;
	p->tot_len = size;
	p->len = size;

	return p;
}

static void my_pbuf_free(wiced_buffer_t buf)
{
    UNUSED_PARAMETER( buf );
}

static int my_pbuf_header(wiced_buffer_t header,int add_remove_amount)
{
	header->payload +=  add_remove_amount;
	return 0;
}

wwd_result_t wwd_buffer_init( /*@null@*/ /*@unused@*/ void * native_arg )
{
    UNUSED_PARAMETER( native_arg );

    return WWD_SUCCESS;
}


wwd_result_t host_buffer_check_leaked( void )
{
    return WWD_SUCCESS;
}

wwd_result_t internal_host_buffer_get( wiced_buffer_t * buffer, wwd_buffer_dir_t direction, unsigned short size, unsigned long timeout_ms )
{
    UNUSED_PARAMETER( direction );

    wiced_assert("Error: Invalid buffer size\n", size != 0);

    *buffer = NULL;

    if ( size > (unsigned short) WICED_LINK_MTU )
    {
        WPRINT_NETWORK_DEBUG(("Attempt to allocate a buffer larger than the MTU of the link\n"));
        /*@-compdef@*/ /* Lint: buffer is not allocated in error case */
        return WWD_BUFFER_UNAVAILABLE_PERMANENT;
        /*@+compdef@*/
    }

    do
    {
        void *custom_mem;
        struct memp_desc *application_defined_pool = ( direction == WWD_NETWORK_RX ) ? application_defined_rx_pool : application_defined_tx_pool;

        *buffer = pbuf_alloc( PBUF_RAW, size, ( direction == WWD_NETWORK_RX ) ? PBUF_POOL_RX : PBUF_POOL_TX );
	static int anum=0;
	anum++;
	//PRINTF("%03d alloc 0x%08x 0x%08x %d %x\n",anum,*buffer,(*buffer)->payload,size,( direction == WWD_NETWORK_RX ) ? PBUF_POOL_RX : PBUF_POOL_TX);
	if ((*buffer)->tot_len != (*buffer)->len){
	    ERROR("tot_len:%d len:%d\n",(*buffer)->tot_len , (*buffer)->len);
	}
	wiced_assert("tot_len != len",(*buffer)->tot_len == (*buffer)->len);

        if ( *buffer != NULL )
        {
            break;
        }else{
		ERROR("pbuf alloc failed type:0x%x\n",( direction == WWD_NETWORK_RX ) ? PBUF_POOL_RX : PBUF_POOL_TX );
	}

        /* Attempt to allocate via application supplied pools */

        host_rtos_delay_milliseconds( 1 );

    } while ( timeout_ms-- > 0);

    if ( *buffer == NULL )
    {
#if 0
        WPRINT_NETWORK_DEBUG(("Failed to allocate packet buffer\n"));
#endif /* if 0 */
	PRINTF("buffer alloc failed\n");
        WWD_STATS_CONDITIONAL_INCREMENT_VARIABLE( direction == WWD_NETWORK_TX,  tx_no_mem );
        WWD_STATS_CONDITIONAL_INCREMENT_VARIABLE( direction == WWD_NETWORK_RX,  rx_no_mem );
        /*@-compdef@*/ /* Lint: buffer is not allocated in this case */
        return WWD_BUFFER_UNAVAILABLE_TEMPORARY;
        /*@+compdef@*/
    }

    /*@-compdef@*/ /* Lint does not realise allocation has occurred */
    return WWD_SUCCESS;
    /*@+compdef@*/
}

wwd_result_t host_buffer_get( /*@special@*/ /*@out@*/ wiced_buffer_t* buffer, wwd_buffer_dir_t direction, unsigned short size, wiced_bool_t wait ) /*@allocates *buffer@*/  /*@defines **buffer@*/
{
    unsigned long wait_option = ( wait == WICED_TRUE ) ? (unsigned long) WICED_NEVER_TIMEOUT : 0;
    return internal_host_buffer_get(buffer, direction, size, wait_option);
}

void host_buffer_release( /*@only@*/ wiced_buffer_t buffer, wwd_buffer_dir_t direction )
{
    UNUSED_PARAMETER( direction );

    wiced_assert("Error: Invalid buffer\n", buffer != NULL);
    static int fnum = 0;
    fnum++;
    (void) pbuf_free( buffer ); /* Ignore returned number of freed segments since TCP packets will still be referenced by LWIP after relase by WICED */
}

/*@exposed@*/ uint8_t* host_buffer_get_current_piece_data_pointer( /*@temp@*/ wiced_buffer_t buffer )
{
    wiced_assert("Error: Invalid buffer\n", buffer != NULL);
    return (uint8_t*) buffer->payload;
}

uint16_t host_buffer_get_current_piece_size( /*@temp@*/ wiced_buffer_t buffer )
{
    wiced_assert("Error: Invalid buffer\n", buffer != NULL);
    return (uint16_t) buffer->len;
}

/*@exposed@*/ /*@dependent@*/ /*@null@*/ wiced_buffer_t host_buffer_get_next_piece( /*@dependent@*/ wiced_buffer_t buffer )
{
    wiced_assert("Error: Invalid buffer\n", buffer != NULL);
    return buffer->next;
}

wwd_result_t host_buffer_add_remove_at_front( wiced_buffer_t * buffer, int32_t add_remove_amount )
{
    wiced_assert("Error: Invalid buffer\n", buffer != NULL);
    if ( (u8_t) 0 != pbuf_header( *buffer, (s16_t) ( -add_remove_amount ) ) )
    {
        ERROR("0x%08x 0x%08x tlen:%d len:%d add_remove:%d  \n",*buffer,(*buffer)->payload,(*buffer)->tot_len,(*buffer)->len,add_remove_amount);
//        WPRINT_NETWORK_ERROR(("0x%08x 0x%08x tlen:%d len:%d add_remove:%d  ",*buffer,(*buffer)->payload,(*buffer)->tot_len,(*buffer)->len,add_remove_amount));
	PRINTF("failed\n");
        WPRINT_NETWORK_DEBUG(("Failed to move pointer - usually because not enough space at front of buffer\n"));
        return WWD_BUFFER_POINTER_MOVE_ERROR;
    }

    return WWD_SUCCESS;
}

wwd_result_t host_buffer_set_size( /*@temp@*/ wiced_buffer_t buffer, unsigned short size )
{
     if ( size > (unsigned short) WICED_LINK_MTU )
        {
            WPRINT_NETWORK_ERROR(("Attempt to set a length larger than the MTU of the link\n"));
            /*@-unreachable@*/ /* Reachable after hitting assert */
            return WWD_BUFFER_SIZE_SET_ERROR;
            /*@+unreachable@*/
        }

          buffer->tot_len = size;
         buffer->len = size;

         return WWD_SUCCESS;
}
#if 0
wiced_buffer_t host_buffer_pop_from_fifo( wiced_buffer_fifo_t* fifos, wwd_interface_t* interface_return )
{
    wiced_buffer_t buffer = NULL;
    int interface_index               = 0;
    wwd_interface_t current_interface = WWD_INDEX_TO_INTERFACE( interface_index );
    for (  ; interface_index < WWD_INTERFACE_MAX ; interface_index++, current_interface = WWD_INDEX_TO_INTERFACE( interface_index ) )
    {
        wiced_buffer_interface_fifo_t *fifo = &( fifos->per_interface_fifos[ WWD_INTERFACE_INDEX( current_interface ) ] );

        if ( fifo->first != NULL )
        {
            buffer         = fifo->first;
            if ( interface_return != NULL )
            {
                *interface_return = current_interface;
            }
            fifo->first    = buffer->next;
        }
    }
    return buffer;
}

void host_buffer_push_to_fifo( wiced_buffer_fifo_t* fifos, wiced_buffer_t buffer, wwd_interface_t interface )
{

    wiced_buffer_interface_fifo_t *fifo = &( fifos->per_interface_fifos[ WWD_INTERFACE_INDEX( interface ) ] );
    buffer->next = NULL;

    if ( fifo->first == NULL )
    {
        fifo->first = buffer;
        fifo->last  = buffer;
    }
    else
    {
        fifo->last->next = buffer;
        fifo->last       = buffer;
    }
}


wiced_bool_t host_buffer_pool_is_full( wwd_buffer_dir_t direction )
{
    wiced_bool_t full = WICED_TRUE;
    uint16_t one_byte = 1;
    wiced_buffer_t buffer;

    /* not full if we can pull even one byte long out of the pool */
    if ( WWD_SUCCESS == host_buffer_get( &buffer, direction, one_byte, WICED_NO_WAIT ) )
    {
        full = WICED_FALSE;
        host_buffer_release( buffer, direction );
    }

    return full;
}

#if 0
void memp_free_notify( unsigned int type )
{
    if ( type == MEMP_PBUF_POOL_TX )
    {
        host_platform_bus_buffer_freed( WWD_NETWORK_TX );
    }
    else if ( type == MEMP_PBUF_POOL_RX )
    {
        host_platform_bus_buffer_freed( WWD_NETWORK_RX );
    }
}
#endif
#endif
