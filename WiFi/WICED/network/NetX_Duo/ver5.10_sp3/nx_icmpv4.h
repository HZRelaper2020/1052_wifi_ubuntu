/**************************************************************************/
/*                                                                        */
/*            Copyright (c) 1996-2017 by Express Logic Inc.               */
/*                                                                        */
/*  This software is copyrighted by and is the sole property of Express   */
/*  Logic, Inc.  All rights, title, ownership, or other interests         */
/*  in the software remain the property of Express Logic, Inc.  This      */
/*  software may only be used in accordance with the corresponding        */
/*  license agreement.  Any unauthorized use, duplication, transmission,  */
/*  distribution, or disclosure of this software is expressly forbidden.  */
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */
/*  written consent of Express Logic, Inc.                                */
/*                                                                        */
/*  Express Logic, Inc. reserves the right to modify this software        */
/*  without notice.                                                       */
/*                                                                        */
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   Internet Control Message Protocol (ICMP)                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    nx_icmp.h                                           PORTABLE C      */
/*                                                           5.10 SP3     */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Express Logic, Inc.                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file defines the NetX Internet Control Message Protocol (ICMP) */
/*    component, including all data types and external references.  It is */
/*    assumed that nx_api.h and nx_port.h have already been included.     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */
/*  08-09-2007     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.1    */
/*  12-30-2007     Yuxin Zhou               Modified comment(s), and      */
/*                                            added support for IPv6,     */
/*                                            resulting in version 5.2    */
/*  08-03-2009     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.3    */
/*  11-23-2009     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.4    */
/*  06-01-2010     Yuxin Zhou               Removed internal debug logic, */
/*                                            resulting in version 5.5    */
/*  10-10-2011     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.6    */
/*  01-31-2013     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.7    */
/*  01-12-2015     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.8    */
/*  02-22-2016     Yuxin Zhou               Modified comment(s), added    */
/*                                            ICMP error message support, */
/*                                            resulting in version 5.9    */
/*  05-10-2016     Yuxin Zhou               Modified comment(s), and      */
/*                                            passed the PC-Lint check,   */
/*                                            removed deprecated function */
/*                                            declaration, removed unused */
/*                                            code,                       */
/*                                            resulting in version 5.10   */
/*  06-30-2017     Yuxin Zhou               Modified comment(s), and      */
/*                                            modified to be compatible   */
/*                                            with ThreadX 5.8 or later,  */
/*                                            resulting in version 5.10SP3*/
/*                                                                        */
/**************************************************************************/

#ifndef NX_ICMPV4_H
#define NX_ICMPV4_H


/* Define ICMP types and codes.  According to RFC792.  */

/* Define ICMP types.  */
#define NX_ICMP_ECHO_REPLY_TYPE       0
#define NX_ICMP_DEST_UNREACHABLE_TYPE 3
/*
#define NX_ICMP_SOURCE_QUENCH_TYPE    4
#define NX_ICMP_REDIRECT_TYPE         5
 */
#define NX_ICMP_ECHO_REQUEST_TYPE     8
#define NX_ICMP_TIME_EXCEEDED_TYPE    11
#define NX_ICMP_PARAMETER_PROB_TYPE   12
#define NX_ICMP_TIMESTAMP_REQ_TYPE    13
/*
#define NX_ICMP_TIMESTAMP_REP_TYPE    14
#define NX_ICMP_INFORMATION_REQ_TYPE  15
#define NX_ICMP_INFORMATION_REP_TYPE  16
#define NX_ICMP_ADDRESS_MASK_REQ_TYPE 17
#define NX_ICMP_ADDRESS_MASK_REP_TYPE 18
 */

/* Define the ICMP Destination Unreachable Message codes.  */
/*
#define NX_ICMP_NETWORK_UNREACH_CODE  0
#define NX_ICMP_HOST_UNREACH_CODE     1
 */
#define NX_ICMP_PROTOCOL_UNREACH_CODE 2
#define NX_ICMP_PORT_UNREACH_CODE     3
/*
#define NX_ICMP_FRAMENT_NEEDED_CODE   4
#define NX_ICMP_SOURCE_ROUTE_CODE     5
 */

/* Define the TIME Exceeded codes.  */
/*
#define NX_ICMP_TTL_EXCEEDED_CODE     0
 */
#define NX_ICMP_FRT_EXCEEDED_CODE     1

/* Define the Zero Code for Parameter Problem Message, Source Quench Message, Redirect Message,
   Echo Request/Reply, Timestamp Request/Reply and Information Request/Reply Message .  */
#define NX_ICMP_ZERO_CODE             0

/* Define Basic ICMP packet header data type.  This will be used to
   build new ICMP packets and to examine incoming packets into NetX.  */

typedef  struct NX_ICMP_HEADER_STRUCT
{
    /* Define the first 32-bit word of the ICMP header.  This word contains
       the following information:

            bits 31-24  ICMP 8-bit type defined as follows:

                                        Type Field      ICMP Message Type

                                            0           Echo Reply
                                            3           Destination Unreachable
                                            4           Source Quench
                                            5           Redirect (change a route)
                                            8           Echo Request
                                            11          Time exceeded for Datagram
                                            12          Parameter Problem on a Datagram
                                            13          Timestamp Request
                                            14          Timestamp Reply
                                            17          Address Mask Request
                                            18          Address Mask Reply

            bits 23-16  ICMP 8-bit code defined as follows:

                                        Code Field      ICMP Code Meaning

                                            0           Network unreachable
                                            1           Host unreachable
                                            2           Protocol unreachable
                                            3           Port unreachable
                                            4           Fragmentation needed and DF is set
                                            5           Source route failed
                                            6           Destination network unknown
                                            7           Destination host unknown
                                            8           Source host isolated
                                            9           Communication with destination network
                                                            administratively prohibited
                                            10          Communication with destination host
                                                            administratively prohibited
                                            11          Network unreachable for type of service
                                            12          Host unreachable for type of service

            bits 15-0   ICMP 16-bit checksum

     */

    ULONG nx_icmp_header_word_0;

    /* Define the second and final word of the ICMP header.  This word contains
       the following information:

            bits 31-16  ICMP 16-bit Identification
            bits 15-0   ICMP 16-bit Sequence Number
     */
    ULONG nx_icmp_header_word_1;
} NX_ICMP_HEADER;

/* Define the ICMP echo request header message size.  */

#define NX_ICMP_HEADER_SIZE sizeof(NX_ICMP_HEADER)


/* Define basic ICMPv4 packet header data type. */

typedef  struct NX_ICMPV4_HEADER_STRUCT
{

    /*  Header field for ICMPv4 message type */
    UCHAR   nx_icmpv4_header_type;

    /*  Header field for ICMPv4 message code (type context specific) */
    UCHAR   nx_icmpv4_header_code;

    /*  Header field for ICMPv4 header checksum */
    USHORT  nx_icmpv4_header_checksum;
} NX_ICMPV4_HEADER;

/* Define ICMPv4 error message type. */

typedef struct NX_ICMPV4_ERROR_STRUCT
{
    /* General ICMP header. */
    NX_ICMPV4_HEADER nx_icmpv4_error_header;

    /* Pointer to the original IPv4 packet where error is detected. */
    ULONG            nx_icmpv4_error_pointer;
} NX_ICMPV4_ERROR;

/* ICMP echo request message type. */

typedef struct NX_ICMPV4_ECHO_STRUCT
{

    /* General ICMP header. */
    /*lint -esym(768,NX_ICMPV4_ECHO_STRUCT::nx_icmpv4_echo_header) suppress member not referenced. It is used before casting from NX_ICMPV4_HEADER. */
    NX_ICMPV4_HEADER nx_icmpv4_echo_header;

    /* Echo request message ID */
    /*lint -esym(768,NX_ICMPV4_ECHO_STRUCT::nx_icmpv4_echo_identifier) suppress member not referenced. It is not used as a host. */
    USHORT           nx_icmpv4_echo_identifier;

    /* Echo request message sequence number */
    USHORT           nx_icmpv4_echo_sequence_num;
} NX_ICMPV4_ECHO;


#ifndef NX_DISABLE_ICMPV4_ERROR_MESSAGE
/* Define macros for sending out ICMPv4 error messages. */
#define NX_ICMPV4_SEND_DEST_UNREACHABLE(ip_ptr, packet, code) \
    _nx_icmpv4_send_error_message((ip_ptr), (packet), (ULONG)((NX_ICMP_DEST_UNREACHABLE_TYPE << 24) | ((code) << 16)), 0)

#define NX_ICMPV4_SEND_TIME_EXCEED(ip_ptr, packet, code) \
    _nx_icmpv4_send_error_message((ip_ptr), (packet), (ULONG)((NX_ICMP_TIME_EXCEEDED_TYPE << 24) | ((code) << 16)), 0)

#define NX_ICMPV4_SEND_PARAMETER_PROBLEM(ip_ptr, packet, code, offset) \
    _nx_icmpv4_send_error_message((ip_ptr), (packet), (ULONG)((NX_ICMP_PARAMETER_PROB_TYPE << 24) | ((code) << 16)), (offset))

#endif /* NX_DISABLE_ICMPV4_ERROR_MESSAGE */


/* Define service for sending ICMPv4 error messages. */

#ifndef NX_DISABLE_ICMPV4_ERROR_MESSAGE
VOID _nx_icmpv4_send_error_message(NX_IP *ip_ptr, NX_PACKET *packet, ULONG word1, ULONG pointer);
#endif /* NX_DISABLE_ICMPV4_ERROR_MESSAGE */


/* Define ICMPv4 API function prototypes. */

UINT _nx_icmp_enable(NX_IP *ip_ptr);
UINT _nx_icmp_info_get(NX_IP *ip_ptr, ULONG *pings_sent, ULONG *ping_timeouts,
                       ULONG *ping_threads_suspended, ULONG *ping_responses_received,
                       ULONG *icmp_checksum_errors, ULONG *icmp_unhandled_messages);
UINT _nx_icmp_ping(NX_IP *ip_ptr, ULONG ip_address, CHAR *data, ULONG data_size,
                   NX_PACKET **response_ptr, ULONG wait_option);
VOID _nx_icmp_cleanup(TX_THREAD *thread_ptr NX_CLEANUP_PARAMETER);
VOID _nx_icmpv4_packet_process(NX_IP *ip_ptr, NX_PACKET *packet_ptr);

/* Define internal ICMPv4 handling functions. */

VOID _nx_icmpv4_process_echo_reply(NX_IP *ip_ptr, NX_PACKET *packet_ptr);
VOID _nx_icmpv4_process_echo_request(NX_IP *ip_ptr, NX_PACKET *packet_ptr);

/* Define error checking shells for API services.  These are only referenced by the
   application.  */

UINT _nxe_icmp_enable(NX_IP *ip_ptr);
UINT _nxe_icmp_info_get(NX_IP *ip_ptr, ULONG *pings_sent, ULONG *ping_timeouts,
                        ULONG *ping_threads_suspended, ULONG *ping_responses_received,
                        ULONG *icmp_checksum_errors, ULONG *icmp_unhandled_messages);
UINT _nxe_icmp_ping(NX_IP *ip_ptr, ULONG ip_address, CHAR *data, ULONG data_size,
                    NX_PACKET **response_ptr, ULONG wait_option);
UINT _nx_icmp_interface_ping(NX_IP *ip_ptr, ULONG ip_address,
                             NX_INTERFACE *interface_ptr, ULONG next_hop_address,
                             CHAR *data_ptr, ULONG data_size,
                             NX_PACKET **response_ptr, ULONG wait_option);


#endif

