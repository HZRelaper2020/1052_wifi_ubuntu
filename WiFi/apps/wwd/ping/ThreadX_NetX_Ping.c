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

/**
 * @file
 *
 * ICMP Ping Application using ThreadX/NetX
 *
 * This is an example of a simple 'ping' (ICMP request-reply)
 * application for ThreadX/Netx
 *
 * By default, the app connects using DHCP and pings the gateway
 *
 */

#include "tx_api.h"
#include "tx_thread.h"
#include "nx_api.h"
#include "wwd_management.h"
#include "netx_applications/dhcp/nx_dhcp.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "network/wwd_buffer_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_network.h"
#include "wwd_network_constants.h"

/******************************************************
 *                      Macros
 ******************************************************/
/** @cond */
#define MAKE_IPV4_ADDRESS   IP_ADDRESS
#define NUM_BUFFERS_POOL_SIZE(x)       ((WICED_LINK_MTU+sizeof(NX_PACKET)+1)*(x))
/** @endcond */

/******************************************************
 *                    Constants
 ******************************************************/

#define AP_SSID                  "YOUR_AP_SSID"
#define AP_PASS                  "YOUR_AP_PASSPHRASE"
#define AP_SEC                   WICED_SECURITY_WPA2_MIXED_PSK

#define COUNTRY                  WICED_COUNTRY_AUSTRALIA
#define USE_DHCP                 WICED_TRUE
#define IP_ADDR                  MAKE_IPV4_ADDRESS( 192, 168,   1,  95 )  /* Not required if USE_DHCP is WICED_TRUE */
#define GW_ADDR                  MAKE_IPV4_ADDRESS( 192, 168,   1,   1 )  /* Not required if USE_DHCP is WICED_TRUE */
#define NETMASK                  MAKE_IPV4_ADDRESS( 255, 255, 255,   0 )  /* Not required if USE_DHCP is WICED_TRUE */
/* #define PING_TARGET              MAKE_IPV4_ADDRESS( 192, 168,   1, 2 ) */  /* Uncomment if you want to ping a specific IP instead of the gateway*/

#define PING_RCV_TIMEO           (1000)    /** ping receive timeout - in milliseconds */
#define PING_DELAY               (1000)    /** Delay between ping response/timeout and the next ping send - in milliseconds */
#define JOIN_TIMEOUT             (10000)   /** timeout for joining the wireless network in milliseconds  = 10 seconds */

#define APP_STACK_SIZE           (3072)
#define APP_TX_BUFFER_POOL_SIZE  NUM_BUFFERS_POOL_SIZE(3)
#define APP_RX_BUFFER_POOL_SIZE  NUM_BUFFERS_POOL_SIZE(3)
#define IP_STACK_SIZE            (1024*2)
#define SIZE_OF_ARP_ENTRY        sizeof(NX_ARP)
#define ARP_CACHE_SIZE           (3*SIZE_OF_ARP_ENTRY)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static TX_THREAD      app_main_thread_handle;
static char           app_main_thread_stack [APP_STACK_SIZE];
static NX_IP          ip_handle;
static char           ip_stack              [IP_STACK_SIZE];
static char           arp_cache             [ARP_CACHE_SIZE];
static char           tx_buffer_pool_memory [APP_TX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES];
static char           rx_buffer_pool_memory [APP_RX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES];
static NX_PACKET_POOL nx_pools[2]; /* 0=TX, 1=RX */
static NX_DHCP        dhcp_handle;

static const wiced_ssid_t ap_ssid =
{
    .length = sizeof(AP_SSID)-1,
    .value  = AP_SSID,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * Main Ping app
 *
 * Initialises NetX, Wiced, joins a wireless network, then periodically pings the specified IP address forever.
 *
 * @param thread_input : Unused parameter - required to match thread prototype
 */

static void app_main_thread( ULONG thread_input )
{
    UINT status;
    NX_PACKET *response_ptr;
    wwd_result_t result;

    WPRINT_APP_INFO(("\nPlatform " PLATFORM " initialised\n"));
    WPRINT_APP_INFO(("Started ThreadX " ThreadX_VERSION "\n"));

    /* Compile-time checks */
    wiced_static_assert(packet_header_not_cache_aligned, sizeof(NX_PACKET) == PLATFORM_L1_CACHE_ROUND_UP(sizeof(NX_PACKET)));

    /* Initialize the NetX system.  */
    WPRINT_APP_INFO(("Initialising NetX " NetX_VERSION "\n"));
    nx_system_initialize( );

    /* Create packet pools for transmit and receive */
    WPRINT_APP_INFO(("Creating Packet pools\n"));
    if ( NX_SUCCESS != nx_packet_pool_create( &nx_pools[0], (char*)"",
        WICED_LINK_MTU_ALIGNED, PLATFORM_L1_CACHE_PTR_ROUND_UP(tx_buffer_pool_memory), APP_TX_BUFFER_POOL_SIZE ) )
    {
        WPRINT_APP_ERROR(("Couldn't create TX packet pool\n"));
        return;
    }
    if ( NX_SUCCESS != nx_packet_pool_create( &nx_pools[1], (char*)"",
        WICED_LINK_MTU_ALIGNED, PLATFORM_L1_CACHE_PTR_ROUND_UP(rx_buffer_pool_memory), APP_RX_BUFFER_POOL_SIZE ) )
    {
        WPRINT_APP_ERROR(("Couldn't create RX packet pool\n"));
        return;
    }

    /* Initialise Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init( &nx_pools );
    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

    /* Attempt to join the Wi-Fi network */
    WPRINT_APP_INFO(("Joining : " AP_SSID "\n"));
    while ( wwd_wifi_join( &ap_ssid, AP_SEC, (uint8_t*) AP_PASS, sizeof( AP_PASS ) - 1, NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to join  : " AP_SSID "   .. retrying\n"));
    }
    WPRINT_APP_INFO(("Successfully joined : " AP_SSID "\n"));

    /* Enable the network interface  */
    uint8_t ip_create_result;
    if ( USE_DHCP == WICED_TRUE )
    {
        ip_create_result = ( status = nx_ip_create( &ip_handle, (char*)"NetX IP", IP_ADDRESS(0, 0, 0, 0), 0xFFFFF000UL, &nx_pools[0], wiced_sta_netx_driver_entry, ip_stack, IP_STACK_SIZE, 2 ) );
    }
    else
    {
        ip_create_result = ( status = nx_ip_create( &ip_handle, (char*)"NetX IP", IP_ADDR, NETMASK, &nx_pools[0], wiced_sta_netx_driver_entry, ip_stack, IP_STACK_SIZE, 2 ) );
    }

    if ( NX_SUCCESS != ip_create_result )
    {
        WPRINT_APP_ERROR(("Failed to create IP\n"));
        return;
    }

    if ( NX_SUCCESS != ( status = nx_arp_enable( &ip_handle, (void *) arp_cache, ARP_CACHE_SIZE ) ) )
    {
        WPRINT_APP_ERROR(("Failed to enable ARP\n"));
        nx_ip_delete( &ip_handle );
        return;
    }

    if ( NX_SUCCESS != ( status = nx_tcp_enable( &ip_handle ) ) )
    {
        WPRINT_APP_ERROR(("Failed to enable TCP\n"));
        nx_ip_delete( &ip_handle );
        return;
    }

    if ( NX_SUCCESS != ( status = nx_udp_enable( &ip_handle ) ) )
    {
        WPRINT_APP_ERROR(("Failed to enable UDP\n"));
        nx_ip_delete( &ip_handle );
        return;
    }

    if ( NX_SUCCESS != ( status = nx_icmp_enable( &ip_handle ) ) )
    {
        WPRINT_APP_ERROR(("Failed to enable ICMP\n"));
        nx_ip_delete( &ip_handle );
        return;
    }

    /* Obtain an IP address via DHCP if required */
    if ( USE_DHCP == WICED_TRUE )
    {
        WPRINT_APP_INFO(("Obtaining IP address via DHCP\n"));
        do
        {
            /* Create the DHCP instance. */
            status = nx_dhcp_create( &dhcp_handle, &ip_handle, (char*)"WICED Web Server" );
            if ( status )
            {
                WPRINT_APP_ERROR(("Failed to Create DHCP thread\n"));
                return;
            }

            /* Start DHCP. */
            status = nx_dhcp_start( &dhcp_handle );
            if ( status )
            {
                WPRINT_APP_ERROR(("Failed to initiate DHCP transaction\n"));
                return;
            }

            /* Check for address resolution. */
            status = nx_ip_status_check( &ip_handle, NX_IP_ADDRESS_RESOLVED, (ULONG *) &status, 100000 );

            if ( status != NX_SUCCESS )
            {
                nx_dhcp_stop( &dhcp_handle );
                nx_dhcp_delete( &dhcp_handle );
                WPRINT_APP_INFO(("DHCP failed - retrying...\n"));
            }
        } while ( status != NX_SUCCESS );
    } /* USE_DHCP == WICED_TRUE */

    ULONG ip_address, network_mask, ping_target;
    nx_ip_address_get( &ip_handle, &ip_address, &network_mask );
    WPRINT_APP_INFO(( "Network ready IP: %u.%u.%u.%u\n", (unsigned char) ( ( ip_address >> 24 ) & 0xff ),
                                                     (unsigned char) ( ( ip_address >> 16 ) & 0xff ),
                                                     (unsigned char) ( ( ip_address >>  8 ) & 0xff ),
                                                     (unsigned char) ( ( ip_address >>  0 ) & 0xff ) ) );

#ifdef PING_TARGET
    ping_target = PING_TARGET;
#else /* ifdef PING_TARGET */
    if (USE_DHCP == WICED_TRUE)
    {
        ping_target = ip_handle.nx_ip_gateway_address;
    }
    else
    {
        ping_target = GW_ADDR;
    }
#endif /* ifdef PING_TARGET */

    /* Ping Loop */
    WPRINT_APP_INFO(( "Pinging: %u.%u.%u.%u (%.2fms time accuracy)\n", (unsigned char) ( ( ping_target >> 24 ) & 0xff ),
                                                                 (unsigned char) ( ( ping_target >> 16 ) & 0xff ),
                                                                 (unsigned char) ( ( ping_target >>  8 ) & 0xff ),
                                                                 (unsigned char) ( ( ping_target >>  0 ) & 0xff ),
                                                                 1000.0 / (double) SYSTICK_FREQUENCY ));

    /* Do an initial ping to force an ARP request */
    if (nx_icmp_ping( &ip_handle, ping_target, NULL, 0, &response_ptr, PING_RCV_TIMEO * SYSTICK_FREQUENCY / 1000 ) == NX_SUCCESS)
    {
        nx_packet_release( response_ptr );
    }

    while ( 1 )
    {
        wwd_time_t send_time;
        wwd_time_t reply_time;
        UINT status;

        /* Send ping and wait for reply */
        send_time = host_rtos_get_time( );
        status = nx_icmp_ping( &ip_handle, ping_target, NULL, 0, &response_ptr, PING_RCV_TIMEO * SYSTICK_FREQUENCY / 1000 );
        reply_time = host_rtos_get_time( );

        /* Print result */
        if ( status == NX_SUCCESS )
        {
            WPRINT_APP_INFO(("Ping Reply %dms\n", (int)( reply_time - send_time ) ));
            nx_packet_release( response_ptr );
        }
        else if ( status == NX_NO_RESPONSE )
        {
            WPRINT_APP_INFO(("Ping timeout\n"));
        }
        else
        {
            WPRINT_APP_INFO(("Ping error\n"));
        }

        /* Sleep until time for next ping */
        tx_thread_sleep( PING_DELAY * SYSTICK_FREQUENCY / 1000 );

    }

    /* Shutdown code - not used due to infinite loop */
#if 0
    WPRINT_APP_INFO(("Closing down\n"));

    if ( USE_DHCP == WICED_TRUE )
    {
        nx_dhcp_stop(&dhcp_handle);
        nx_dhcp_delete(&dhcp_handle);
    }

    if ( NX_SUCCESS != nx_ip_delete( &ip_handle ) )
    {
        WPRINT_APP_ERROR(("Failed to delete IP\n"));
    }

    wiced_wifi_leave( );
    if ( WICED_SUCCESS != wiced_management_deinit( ) )
    {
        WPRINT_APP_ERROR(("WICED de-initialization failed\n"));
    }
#endif /* if 0 */
}




/**
 *  Main function - starts ThreadX
 *  Called from the crt0 _start function
 *
 */

int main( )
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter( );
    return 0;
}


/**
 *  Application Define function - creates and starts the application thread
 *  Called by ThreadX whilst it is initialising
 *
 *  @param first_unused_memory: unused parameter - required to match prototype
 */

void tx_application_define( void *first_unused_memory )
{

    /* Create the application thread.  */
    tx_thread_create( &app_main_thread_handle, (char*)"app thread", app_main_thread, 0, app_main_thread_stack, APP_STACK_SIZE, 4, 4, TX_NO_TIME_SLICE, TX_AUTO_START );

}
