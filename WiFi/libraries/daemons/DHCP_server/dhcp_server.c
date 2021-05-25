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


/** @file
 *  Implementation of a simple DHCP server
 */

#include "wiced.h"
#include "dhcp_server.h"
#include "wwd_network_constants.h"
#include "wiced_network.h"

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef UNIT_TESTER
#define DHCP_CHECK_PARAMS_NO_RETURN_VAL( expr )  { if (expr) { return; }}
#define DHCP_CHECK_PARAMS( expr, retval )        { if (expr) { return retval; }}
#elif defined( DEBUG )
#define DHCP_CHECK_PARAMS_NO_RETURN_VAL( expr )  { if (expr) { wiced_assert( "", 0==1 ); return; }}
#define DHCP_CHECK_PARAMS( expr, retval )        { if (expr) { wiced_assert( "", 0==1 ); return retval; }}
#else /* ifdef DEBUG */
#define DHCP_CHECK_PARAMS_NO_RETURN_VAL( expr )
#define DHCP_CHECK_PARAMS( expr, retval )
#endif /* ifdef DEBUG */


/******************************************************
 *                    Constants
 ******************************************************/
#ifndef DHCP_IP_ADDRESS_CACHE_MAX
#define DHCP_IP_ADDRESS_CACHE_MAX       (5)
#endif

#define DHCP_THREAD_PRIORITY (WICED_DEFAULT_LIBRARY_PRIORITY)
#define DHCP_SERVER_RECEIVE_TIMEOUT (500)
/* BOOTP operations */
#define BOOTP_OP_REQUEST                (1)
#define BOOTP_OP_REPLY                  (2)

/* DHCP options */
#define DHCP_SUBNETMASK_OPTION_CODE             (1)
#define DHCP_MTU_OPTION_CODE                    (26)
#define DHCP_REQUESTED_IP_ADDRESS_OPTION_CODE   (50)
#define DHCP_LEASETIME_OPTION_CODE              (51)
#define DHCP_MESSAGETYPE_OPTION_CODE            (53)
#define DHCP_SERVER_IDENTIFIER_OPTION_CODE      (54)
#define DHCP_WPAD_OPTION_CODE                   (252)
#define DHCP_END_OPTION_CODE                    (255)



/* DHCP commands */
#define DHCPDISCOVER                    (1)
#define DHCPOFFER                       (2)
#define DHCPREQUEST                     (3)
#define DHCPDECLINE                     (4)
#define DHCPACK                         (5)
#define DHCPNAK                         (6)
#define DHCPRELEASE                     (7)
#define DHCPINFORM                      (8)

/* UDP port numbers for DHCP server and client */
#define IPPORT_DHCPS                   (67)
#define IPPORT_DHCPC                   (68)

#define WPAD_SAMPLE_URL "http://xxx.xxx.xxx.xxx/wpad.dat"

static const uint8_t mtu_option_buff[]             = { DHCP_MTU_OPTION_CODE, 2, WICED_PAYLOAD_MTU>>8, WICED_PAYLOAD_MTU&0xff };
static const uint8_t dhcp_offer_option_buff[]      = { DHCP_MESSAGETYPE_OPTION_CODE, 1, DHCPOFFER };
static const uint8_t dhcp_ack_option_buff[]        = { DHCP_MESSAGETYPE_OPTION_CODE, 1, DHCPACK };
static const uint8_t dhcp_nak_option_buff[]        = { DHCP_MESSAGETYPE_OPTION_CODE, 1, DHCPNAK };
static const uint8_t lease_time_option_buff[]      = { DHCP_LEASETIME_OPTION_CODE, 4, 0x00, 0x01, 0x51, 0x80 }; /* 1 day lease */
static const uint8_t dhcp_magic_cookie[]           = { 0x63, 0x82, 0x53, 0x63 };
static const wiced_mac_t empty_cache               = { .octet = {0} };

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/* DHCP data structure */
typedef struct
{
    uint8_t  opcode;                     /* packet opcode type */
    uint8_t  hardware_type;              /* hardware addr type */
    uint8_t  hardware_addr_len;          /* hardware addr length */
    uint8_t  hops;                       /* gateway hops */
    uint32_t transaction_id;             /* transaction ID */
    uint16_t second_elapsed;             /* seconds since boot began */
    uint16_t flags;
    uint8_t  client_ip_addr[4];          /* client IP address */
    uint8_t  your_ip_addr[4];            /* 'your' IP address */
    uint8_t  server_ip_addr[4];          /* server IP address */
    uint8_t  gateway_ip_addr[4];         /* gateway IP address */
    uint8_t  client_hardware_addr[16];   /* client hardware address */
    uint8_t  legacy[192];
    uint8_t  magic[4];
    uint8_t  options[275];               /* options area */
    /* as of RFC2131 it is variable length */
} dhcp_header_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void           dhcp_thread                      ( wiced_thread_arg_t thread_input );
static const uint8_t* find_option                      ( const dhcp_header_t* request, uint8_t option_num );
static wiced_result_t get_client_ip_address_from_cache ( const wiced_mac_t* client_mac_address, wiced_ip_address_t* client_ip_address );
static wiced_result_t add_client_to_cache              ( const wiced_mac_t* client_mac_address, const wiced_ip_address_t* client_ip_address );
static void           ipv4_to_string                   ( char* buffer, uint32_t ipv4_address );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_mac_t             cached_mac_addresses[DHCP_IP_ADDRESS_CACHE_MAX];
static wiced_ip_address_t      cached_ip_addresses [DHCP_IP_ADDRESS_CACHE_MAX];

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_start_dhcp_server(wiced_dhcp_server_t* server, wiced_interface_t interface)
{
    DHCP_CHECK_PARAMS( (server == NULL) || ( (interface != WICED_STA_INTERFACE) && (interface != WICED_AP_INTERFACE) && (interface != WICED_P2P_INTERFACE) && (interface != WICED_CONFIG_INTERFACE) ), WICED_BADARG );

    server->interface = interface;

    /* Create DHCP socket */
    WICED_VERIFY(wiced_udp_create_socket(&server->socket, IPPORT_DHCPS, interface));

    /* Clear cache */
    memset( cached_mac_addresses, 0, sizeof( cached_mac_addresses ) );
    memset( cached_ip_addresses,  0, sizeof( cached_ip_addresses  ) );

    /* Initialise the server quit flag - done here in case quit is requested before thread runs */
    server->quit = WICED_FALSE;

    /* Start DHCP server */
    wiced_rtos_create_thread(&server->thread, DHCP_THREAD_PRIORITY, "DHCPserver", dhcp_thread, DHCP_STACK_SIZE, server);
    return WICED_SUCCESS;
}

wiced_result_t wiced_stop_dhcp_server(wiced_dhcp_server_t* server)
{
    DHCP_CHECK_PARAMS( (server == NULL), WICED_BADARG );

    server->quit = WICED_TRUE;
    if ( wiced_rtos_is_current_thread( &server->thread ) != WICED_SUCCESS )
    {
        wiced_rtos_thread_force_awake( &server->thread );
        wiced_rtos_thread_join( &server->thread );
        wiced_rtos_delete_thread( &server->thread );
    }
    /* Delete DHCP socket */
    wiced_udp_delete_socket( &server->socket );

    return WICED_SUCCESS;
}

wiced_result_t wiced_get_clients_ip_address_list_dhcp_server( wiced_dhcp_server_t* server, void* client_ip_address_list )
{
    wiced_ip_address_list_t* ip_addr_list_ptr;

    uint32_t iterator;
    wiced_result_t result;
    struct {
        uint32_t count;
        wiced_mac_t maclist[DHCP_IP_ADDRESS_CACHE_MAX];
    } client_mac_list;

    DHCP_CHECK_PARAMS( (server == NULL), WICED_BADARG );

    UNUSED_PARAMETER(server);

    client_mac_list.count = DHCP_IP_ADDRESS_CACHE_MAX;

    result = wiced_wifi_get_associated_client_list( &client_mac_list, sizeof(client_mac_list) );
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    ip_addr_list_ptr = (wiced_ip_address_list_t*) client_ip_address_list;

    if( ip_addr_list_ptr->count < client_mac_list.count )
    {
        return WICED_ERROR;
    }

    if(client_mac_list.count == 0)
    {
        ip_addr_list_ptr->count = 0;
        return WICED_SUCCESS;
    }

    ip_addr_list_ptr->count = client_mac_list.count;

    for ( iterator = 0; iterator < client_mac_list.count; iterator++ )
    {
        result = get_client_ip_address_from_cache( (const wiced_mac_t*) &client_mac_list.maclist[iterator], &ip_addr_list_ptr->ip_address_list[iterator] );
        if( result != WICED_SUCCESS ) // WICED_NOT_FOUND? No IP-address for an associated client's mac-address, huh?
        {
            return result;
        }
    }

    return WICED_SUCCESS;
}

/**
 *  Implements a very simple DHCP server.
 *
 *  Server will always offer next available address to a DISCOVER command
 *  Server will NAK any REQUEST command which is not requesting the next available address
 *  Server will ACK any REQUEST command which is for the next available address, and then increment the next available address
 *
 * @param my_addr : local IP address for binding of server port.
 */
static void dhcp_thread( wiced_thread_arg_t thread_input )
{
    wiced_packet_t*      received_packet;
    wiced_packet_t*      transmit_packet;
    wiced_ip_address_t   local_ip_address;
    wiced_ip_address_t   netmask;
    uint32_t             next_available_ip_addr;
    uint32_t             ip_mask;
    uint32_t             subnet;
    uint32_t             netmask_htobe;
    char*                option_ptr;
    wiced_dhcp_server_t* server                       = (wiced_dhcp_server_t*)thread_input;
    uint8_t              subnet_mask_option_buff[]    = { DHCP_SUBNETMASK_OPTION_CODE, 4, 0, 0, 0, 0 };
    uint8_t              server_ip_addr_option_buff[] = { DHCP_SERVER_IDENTIFIER_OPTION_CODE, 4, 0, 0, 0, 0 };
    uint32_t*            server_ip_addr_ptr           = (uint32_t*)&server_ip_addr_option_buff[2];
    uint8_t              wpad_option_buff[ 2 + sizeof(WPAD_SAMPLE_URL)-1 ] = { DHCP_WPAD_OPTION_CODE, sizeof(WPAD_SAMPLE_URL)-1 };

    /* Save local IP address to be sent in DHCP packets */
    wiced_ip_get_ipv4_address(server->interface, &local_ip_address);
    *server_ip_addr_ptr = htobe32( GET_IPV4_ADDRESS( local_ip_address ) );

    /* Save the current netmask to be sent in DHCP packets as the 'subnet mask option' */
    wiced_ip_get_netmask(server->interface, &netmask);
    netmask_htobe = htobe32( GET_IPV4_ADDRESS(netmask) );
    memcpy(&subnet_mask_option_buff[2], &netmask_htobe, 4);

    /* Calculate the first available IP address which will be served - based on the netmask and the local IP */
    ip_mask = ~( GET_IPV4_ADDRESS( netmask ) );
    subnet = GET_IPV4_ADDRESS( local_ip_address ) & GET_IPV4_ADDRESS( netmask );
    next_available_ip_addr = subnet | ( ( GET_IPV4_ADDRESS( local_ip_address ) + 1 ) & ip_mask );

    /* Prepare the Web proxy auto discovery URL */
    memcpy(&wpad_option_buff[2], WPAD_SAMPLE_URL, sizeof(WPAD_SAMPLE_URL)-1);
    ipv4_to_string( (char*)&wpad_option_buff[2 + 7], *server_ip_addr_ptr);

    /* Loop endlessly */
    while ( server->quit == WICED_FALSE )
    {
        uint16_t       data_length;
        uint16_t       available_data_length;
        dhcp_header_t* request_header;

        /* Sleep until data is received from socket. */
        if ( wiced_udp_receive( &server->socket, &received_packet, WICED_WAIT_FOREVER ) != WICED_SUCCESS )
        {
            continue;
        }

        /* Get a pointer to the data in the packet */
        wiced_packet_get_data( received_packet, 0, (uint8_t**) &request_header, &data_length, &available_data_length );

        if (data_length != available_data_length)
        {
            /* We don't support fragmented packets */
            wiced_packet_delete( received_packet );
            continue;
        }

        /* Check if received data length is at least the size of  dhcp_header_t. */
        /* Options field in DHCP header is variable length. We are looking for option "DHCP Message Type" that is 3 octets in size (code, length and type) */
        if (data_length < (sizeof(dhcp_header_t) - sizeof(request_header->options) + 3))
        {
            wiced_packet_delete( received_packet );
            continue;
        }

        /* Check if the option in the dhcp header is "DHCP Message Type", code value for option "DHCP Message Type" is 53 as per rfc2132 */
        if (request_header->options[0] != DHCP_MESSAGETYPE_OPTION_CODE)
        {
            wiced_packet_delete( received_packet );
            continue;
        }

        /* Check DHCP command */
        switch ( request_header->options[2] )
        {
            case DHCPDISCOVER:
            {
                dhcp_header_t*     reply_header;
                uint16_t           available_space;
                wiced_mac_t        client_mac_address;
                wiced_ip_address_t client_ip_address;
                uint32_t           temp;

                /* Create reply packet */
                if ( wiced_packet_create_udp( &server->socket, sizeof(dhcp_header_t), &transmit_packet, (uint8_t**) &reply_header, &available_space ) != WICED_SUCCESS )
                {
                    /* Cannot reply - release incoming packet */
                    wiced_packet_delete( received_packet );
                    break;
                }

                /* Copy in the DHCP header content from the received discover packet into the reply packet */
                memcpy( reply_header, request_header, sizeof(dhcp_header_t) - sizeof(reply_header->options) );

                /* Finished with the received discover packet - release it */
                wiced_packet_delete( received_packet );

                /* Now construct the OFFER response */
                reply_header->opcode = BOOTP_OP_REPLY;

                /* Clear the DHCP options list */
                memset( &reply_header->options, 0, sizeof( reply_header->options ) );

                /* Record client MAC address */
                memcpy( &client_mac_address, request_header->client_hardware_addr, sizeof( client_mac_address ) );

                /* Check whether device is already cached */
                if ( get_client_ip_address_from_cache( &client_mac_address, &client_ip_address ) != WICED_SUCCESS )
                {
                    /* Address not found in cache. Use next available IP address */
                    client_ip_address.version = WICED_IPV4;
                    client_ip_address.ip.v4   = next_available_ip_addr;
                }

                /* Create the IP address for the Offer */
                temp = htonl(client_ip_address.ip.v4);
                memcpy( reply_header->your_ip_addr, &temp, sizeof( temp ) );

                /* Copy the magic DHCP number */
                memcpy( reply_header->magic, dhcp_magic_cookie, 4 );

                /* Add options */
                option_ptr     = (char *) &reply_header->options;
                option_ptr     = MEMCAT( option_ptr, dhcp_offer_option_buff, 3 );                                   /* DHCP message type            */
                option_ptr     = MEMCAT( option_ptr, server_ip_addr_option_buff, 6 );                               /* Server identifier            */
                option_ptr     = MEMCAT( option_ptr, lease_time_option_buff, 6 );                                   /* Lease Time                   */
                option_ptr     = MEMCAT( option_ptr, subnet_mask_option_buff, 6 );                                  /* Subnet Mask                  */
                option_ptr     = (char*)MEMCAT( option_ptr, wpad_option_buff, sizeof(wpad_option_buff) );           /* Web proxy auto discovery URL */
                /* Copy the local IP into the Router & DNS server Options */
                memcpy( option_ptr, server_ip_addr_option_buff, 6 );                                                /* Router (gateway)             */
                option_ptr[0]  = 3;                                                                                 /* Router id                    */
                option_ptr    += 6;
                memcpy( option_ptr, server_ip_addr_option_buff, 6 );                                                /* DNS server                   */
                option_ptr[0]  = 6;                                                                                 /* DNS server id                */
                option_ptr    += 6;
                option_ptr     = MEMCAT( option_ptr, mtu_option_buff, 4 );                                          /* Interface MTU                */
                option_ptr[0]  = (char) DHCP_END_OPTION_CODE;                                                       /* end options                  */
                option_ptr++;

                /* Send OFFER reply packet */
                wiced_packet_set_data_end( transmit_packet, (uint8_t*) option_ptr );
                if ( wiced_udp_send( &server->socket, WICED_IP_BROADCAST, IPPORT_DHCPC, transmit_packet ) != WICED_SUCCESS )
                {
                    wiced_packet_delete( transmit_packet );
                }
            }
                break;

            case DHCPREQUEST:
            {
                /* REQUEST command - send back ACK or NAK */
                uint32_t           temp;
                uint32_t*          server_id_req;
                dhcp_header_t*     reply_header;
                uint16_t           available_space;
                wiced_mac_t        client_mac_address;
                wiced_ip_address_t given_ip_address;
                wiced_ip_address_t requested_ip_address;
                wiced_bool_t       next_avail_ip_address_used = WICED_FALSE;

                /* Check that the REQUEST is for this server */
                server_id_req = (uint32_t*) find_option( request_header, DHCP_SERVER_IDENTIFIER_OPTION_CODE );
                if ( ( server_id_req != NULL ) && ( GET_IPV4_ADDRESS( local_ip_address ) != htobe32(*server_id_req) ) )
                {
                    break; /* Server ID does not match local IP address */
                }

                /* Create reply packet */
                if ( wiced_packet_create_udp( &server->socket, sizeof(dhcp_header_t), &transmit_packet, (uint8_t**) &reply_header, &available_space ) != WICED_SUCCESS )
                {
                    /* Cannot reply - release incoming packet */
                    wiced_packet_delete( received_packet );
                    break;
                }

                /* Copy in the DHCP header content from the received request packet into the reply packet */
                memcpy( reply_header, request_header, sizeof(dhcp_header_t) - sizeof(reply_header->options) );

                /* Record client MAC address */
                memcpy( &client_mac_address, request_header->client_hardware_addr, sizeof( client_mac_address ) );

                /* Locate the requested address in the options and keep requested address */
                requested_ip_address.version = WICED_IPV4;
                requested_ip_address.ip.v4   = ntohl(*(uint32_t*)find_option( request_header, DHCP_REQUESTED_IP_ADDRESS_OPTION_CODE ));

                /* Delete received packet. We don't need it anymore */
                wiced_packet_delete( received_packet );

                reply_header->opcode = BOOTP_OP_REPLY;

                /* Blank options list */
                memset( &reply_header->options, 0, sizeof( reply_header->options ) );

                /* Copy DHCP magic number into packet */
                memcpy( reply_header->magic, dhcp_magic_cookie, 4 );

                option_ptr = (char *) &reply_header->options;

                /* Check if device is cache. If it is, give the previous IP address. Otherwise give the next available IP address */
                if ( get_client_ip_address_from_cache( &client_mac_address, &given_ip_address ) != WICED_SUCCESS )
                {
                    /* Address not found in cache. Use next available IP address */
                    next_avail_ip_address_used = WICED_TRUE;
                    given_ip_address.version   = WICED_IPV4;
                    given_ip_address.ip.v4     = next_available_ip_addr;
                }

                /* Check if the requested IP address matches one we have assigned */
                if ( memcmp( &requested_ip_address.ip.v4, &given_ip_address.ip.v4, sizeof( requested_ip_address.ip.v4 ) ) != 0 )
                {
                    /* Request is not for the assigned IP - force client to take next available IP by sending NAK */
                    /* Add appropriate options */
                    option_ptr = (char*)MEMCAT( option_ptr, dhcp_nak_option_buff, 3 );             /* DHCP message type */
                    option_ptr = (char*)MEMCAT( option_ptr, server_ip_addr_option_buff, 6 );       /* Server identifier */
                    memset( reply_header->your_ip_addr, 0, sizeof( reply_header->your_ip_addr ) ); /* Clear IP addr     */
                }
                else
                {
                    /* Request is for next available IP */
                    /* Add appropriate options */
                    option_ptr     = (char*)MEMCAT( option_ptr, dhcp_ack_option_buff, 3 );                              /* DHCP message type            */
                    option_ptr     = (char*)MEMCAT( option_ptr, server_ip_addr_option_buff, 6 );                        /* Server identifier            */
                    option_ptr     = (char*)MEMCAT( option_ptr, lease_time_option_buff, 6 );                            /* Lease Time                   */
                    option_ptr     = (char*)MEMCAT( option_ptr, subnet_mask_option_buff, 6 );                           /* Subnet Mask                  */
                    option_ptr     = (char*)MEMCAT( option_ptr, wpad_option_buff, sizeof(wpad_option_buff) );           /* Web proxy auto discovery URL */
                    /* Copy the local IP into the Router & DNS server Options */
                    memcpy( option_ptr, server_ip_addr_option_buff, 6 );                                                /* Router (gateway)             */
                    option_ptr[0]  = 3;                                                                                 /* Router id                    */
                    option_ptr    += 6;
                    memcpy( option_ptr, server_ip_addr_option_buff, 6 );                                                /* DNS server                   */
                    option_ptr[0]  = 6;                                                                                 /* DNS server id                */
                    option_ptr    += 6;
                    option_ptr     = (char*)MEMCAT( option_ptr, mtu_option_buff, 4 );                                   /* Interface MTU                */

                    /* Create the IP address for the Offer */
                    temp = htonl(given_ip_address.ip.v4);
                    memcpy( reply_header->your_ip_addr, &temp, sizeof( temp ) );

                    /* Increment next available IP address only if not found in cache */
                    if ( next_avail_ip_address_used == WICED_TRUE )
                    {
                        do
                        {
                            next_available_ip_addr = subnet | ( ( next_available_ip_addr + 1 ) & ip_mask );
                        } while ( next_available_ip_addr == GET_IPV4_ADDRESS(local_ip_address) );
                    }

                    /* Cache client */
                    add_client_to_cache( &client_mac_address, &given_ip_address );

                }

                option_ptr[0] = (char) DHCP_END_OPTION_CODE; /* end options */
                option_ptr++;

                /* Send reply packet */
                wiced_packet_set_data_end( transmit_packet, (uint8_t*) option_ptr );
                if ( wiced_udp_send( &server->socket, WICED_IP_BROADCAST, IPPORT_DHCPC, transmit_packet ) != WICED_SUCCESS )
                {
                    wiced_packet_delete( transmit_packet );
                }

            }
                break;

            default:
                /* Unknown packet type - release received packet */
                wiced_packet_delete( received_packet );
                break;
        }
    }

    /* Server loop has exited due to quit flag */

    WICED_END_OF_CURRENT_THREAD( );
}

/**
 *  Finds a specified DHCP option
 *
 *  Searches the given DHCP request and returns a pointer to the
 *  specified DHCP option data, or NULL if not found
 *
 * @param[out] request    : The DHCP request structure
 * @param[in]  option_num : Which DHCP option number to find
 *
 * @return Pointer to the DHCP option data, or NULL if not found
 */

static const uint8_t* find_option( const dhcp_header_t* request, uint8_t option_num )
{
    const uint8_t* option_ptr = request->options;
    while ( ( option_ptr[0] != DHCP_END_OPTION_CODE ) &&                               /* Check for end-of-options flag */
            ( option_ptr[0] != option_num ) &&                                         /* Check for matching option number */
            ( option_ptr < ( (const uint8_t*) request ) + sizeof( dhcp_header_t ) ) )  /* Check for buffer overrun */
    {
        option_ptr += option_ptr[1] + 2;
    }

    /* Was the option found? */
    if ( option_ptr[0] == option_num )
    {
        return &option_ptr[2];
    }
    return NULL;

}

/**
 *  Searches the cache for a given MAC address to find a matching IP address
 *
 * @param[in]  client_mac_address : MAC address to search for
 * @param[out] client_ip_address  : Receives any IP address which is found
 *
 * @return WICED_SUCCESS if found, WICED_NOT_FOUND otherwise
 */
static wiced_result_t get_client_ip_address_from_cache( const wiced_mac_t* client_mac_address, wiced_ip_address_t* client_ip_address )
{
    uint32_t a;

    /* Check whether device is already cached */
    for ( a = 0; a < DHCP_IP_ADDRESS_CACHE_MAX; a++ )
    {
        if ( memcmp( &cached_mac_addresses[ a ], client_mac_address, sizeof( *client_mac_address ) ) == 0 )
        {
            *client_ip_address = cached_ip_addresses[ a ];
            return WICED_SUCCESS;
        }
    }

    return WICED_NOT_FOUND;
}

/**
 *  Adds the MAC & IP addresses of a client to the cache
 *
 * @param[in] client_mac_address : MAC address of the client to store
 * @param[in] client_ip_address  : IP address of the client to store
 *
 * @return WICED_SUCCESS
 */
static wiced_result_t add_client_to_cache( const wiced_mac_t* client_mac_address, const wiced_ip_address_t* client_ip_address )
{
    uint32_t a;
    uint32_t first_empty_slot;
    uint32_t cached_slot;

    /* Search for empty slot in cache */
    for ( a = 0, first_empty_slot = DHCP_IP_ADDRESS_CACHE_MAX, cached_slot = DHCP_IP_ADDRESS_CACHE_MAX; a < DHCP_IP_ADDRESS_CACHE_MAX; a++ )
    {
        /* Check for matching MAC address */
        if ( memcmp( &cached_mac_addresses[ a ], client_mac_address, sizeof( *client_mac_address ) ) == 0 )
        {
            /* Cached device found */
            cached_slot = a;
            break;
        }
        else if ( first_empty_slot == DHCP_IP_ADDRESS_CACHE_MAX && memcmp( &cached_mac_addresses[ a ], &empty_cache, sizeof(wiced_mac_t) ) == 0 )
        {
            /* Device not found in cache. Return the first empty slot */
            first_empty_slot = a;
        }
    }

    if ( cached_slot != DHCP_IP_ADDRESS_CACHE_MAX )
    {
        /* Update IP address of cached device */
        cached_ip_addresses[cached_slot] = *client_ip_address;
    }
    else if ( first_empty_slot != DHCP_IP_ADDRESS_CACHE_MAX )
    {
        /* Add device to the first empty slot */
        cached_mac_addresses[ first_empty_slot ] = *client_mac_address;
        cached_ip_addresses [ first_empty_slot ] = *client_ip_address;
    }
    else
    {
        /* Cache is full. Add device to slot 0 */
        cached_mac_addresses[ 0 ] = *client_mac_address;
        cached_ip_addresses [ 0 ] = *client_ip_address;
    }

    return WICED_SUCCESS;
}


/**
 *  Convert a IPv4 address to a string
 *
 *  @note: String is 16 bytes including terminating null
 *
 * @param[out] buffer       : Buffer which will recieve the IPv4 string
 * @param[in]  ipv4_address : IPv4 address to convert
 */
static void ipv4_to_string( char buffer[16], uint32_t ipv4_address )
{
    uint8_t* ip = (uint8_t*)&ipv4_address;
    unsigned_to_decimal_string(ip[0], &buffer[0], 3, 3);
    buffer[3] = '.';
    unsigned_to_decimal_string(ip[1], &buffer[4], 3, 3);
    buffer[7] = '.';
    unsigned_to_decimal_string(ip[2], &buffer[8], 3, 3);
    buffer[11] = '.';
    unsigned_to_decimal_string(ip[3], &buffer[12], 3, 3);
}
