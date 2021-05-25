/*--------------------------------------------------------------- 
 * Copyright (c) 1999,2000,2001,2002,2003                              
 * The Board of Trustees of the University of Illinois            
 * All Rights Reserved.                                           
 *--------------------------------------------------------------- 
 * Permission is hereby granted, free of charge, to any person    
 * obtaining a copy of this software (Iperf) and associated       
 * documentation files (the "Software"), to deal in the Software  
 * without restriction, including without limitation the          
 * rights to use, copy, modify, merge, publish, distribute,        
 * sublicense, and/or sell copies of the Software, and to permit     
 * persons to whom the Software is furnished to do
 * so, subject to the following conditions: 
 *
 *     
 * Redistributions of source code must retain the above 
 * copyright notice, this list of conditions and 
 * the following disclaimers. 
 *
 *     
 * Redistributions in binary form must reproduce the above 
 * copyright notice, this list of conditions and the following 
 * disclaimers in the documentation and/or other materials 
 * provided with the distribution. 
 * 
 *     
 * Neither the names of the University of Illinois, NCSA, 
 * nor the names of its contributors may be used to endorse 
 * or promote products derived from this Software without
 * specific prior written permission. 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE CONTIBUTORS OR COPYRIGHT 
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * ________________________________________________________________
 * National Laboratory for Applied Network Research 
 * National Center for Supercomputing Applications 
 * University of Illinois at Urbana-Champaign 
 * http://www.ncsa.uiuc.edu
 * ________________________________________________________________ 
 *
 * headers.h
 * by Mark Gates <mgates@nlanr.net>
 * -------------------------------------------------------------------
 * All system headers required by iperf.
 * This could be processed to form a single precompiled header,
 * to avoid overhead of compiling it multiple times.
 * This also verifies a few things are defined, for portability.
 * ------------------------------------------------------------------- */

#ifndef HEADERS_H
#define HEADERS_H

#ifdef HAVE_CONFIG_H
    /* NOTE: config.h doesn't have guard includes! */
    #define INCLUDED_CONFIG_H_
    #include "config.h"

    /* OSF1 (at least the system I use) needs extern C
     * around the <netdb.h> and <arpa/inet.h> files. */
    #if defined( SPECIAL_OSF1_EXTERN ) && defined( __cplusplus )
        #define SPECIAL_OSF1_EXTERN_C_START extern "C" {
        #define SPECIAL_OSF1_EXTERN_C_STOP  }
    #else
        #define SPECIAL_OSF1_EXTERN_C_START
        #define SPECIAL_OSF1_EXTERN_C_STOP
    #endif /* defined( SPECIAL_OSF1_EXTERN ) && defined( __cplusplus ) */
#endif /* defined(HAVE_CONFIG_H) && !defined(INCLUDED_CONFIG_H_) */

/* turn off assert debugging if we aren't debugging */
#if !defined(NDEBUG) && !defined(DEBUG)
    #define NDEBUG
#endif /* !defined(NDEBUG) && !defined(DEBUG) */

/*
 * For WICED we must include these later to avoid conflicts, but for Windows we
 * must include them here.
 */
#ifndef WICED
    /* standard C headers */
    #include <stdlib.h>
    #include <stdio.h>
    #include <assert.h>
    #include <ctype.h>
    #include <errno.h>
    #include <string.h>
    #include <time.h>
    #include <math.h>
#endif /* WICED */

#if defined(WIN32)
    /* Windows config file */
    #include "config.win32.h"

    /* Windows headers */
	#undef _WIN32_WINNT
    #define _WIN32_WINNT 0x0501
    #define WIN32_LEAN_AND_MEAN /* exclude unnecessary headers */
    #include <windows.h>
    #include <winsock2.h>
    #include <ws2tcpip.h>

    /* define EINTR, just to help compile; it isn't useful */
    #ifndef EINTR
        #define EINTR  WSAEINTR
    #endif /* EINTR */

    #ifndef ENOBUFS
        #define ENOBUFS WSAENOBUFS
    #endif /* ENOBUFS */

    /* Visual C++ has INT64, but not 'long long'.
     * Metrowerks has 'long long', but INT64 doesn't work. */
    #ifdef __MWERKS__
        #define int64_t  long long 
    #else
        #define int64_t  INT64
    #endif /* __MWERKS__ */

    /* Visual C++ has _snprintf instead of snprintf */
    #ifndef __MWERKS__
        #define snprintf _snprintf
    #endif /* __MWERKS__ */

    /* close, read, and write only work on files in Windows.
     * I get away with #defining them because I don't read files. */
    #define close( s )       closesocket( s )
    #define read( s, b, l )  recv( s, (char*) b, l, 0 )
    #define write( s, b, l ) send( s, (char*) b, l, 0 )

    /* Visual C++ does not have setitimer */
    #include "setitimer.h"

#elif defined(WICED)

    /* WICED configuration file */
	#include "config.wiced.h"
	
    /* WICED header files */
	#include <RTOS/wwd_rtos_interface.h>
	#include <wwd_debug.h>
	#include <wwd_assert.h>
	#include <wwd_constants.h>
	#include <wwd_wifi.h>
	#include <wiced_rtos.h>

    /* Disable some unneeded/unimplemented features */
    #ifndef NO_SIGNALS
        #define NO_SIGNALS
    #endif /* NO_SIGNALS */
    #ifndef NO_FILE_IO
        #define NO_FILE_IO
    #endif /* NO_FILE_IO */
    #ifndef NO_DAEMON
        #define NO_DAEMON
    #endif /* NO_DAEMON */
    #ifndef NO_ENVIRONMENT
        #define NO_ENVIRONMENT
    #endif /* NO_ENVIRONMENT */
    #ifndef NO_EXIT
        #define NO_EXIT
    #endif /* NO_EXIT */
    #ifndef NO_INTERRUPTS
        #define NO_INTERRUPTS
    #endif /* NO_INTERRUPTS */
    #ifndef NO_SERVICE
        #define NO_SERVICE
    #endif /* NO_SERVICE */
    #ifndef NO_ITIMER
        #define NO_ITIMER
    #endif /* NO_ITIMER */

    /* WICED threads not yet implemented */
    #ifndef NO_THREADS
        #define NO_THREADS
    #endif /* NO_THREADS */

    #if !defined(RTOS_NuttX)
        /* Override the unimplemented gettimeofday function */
        #define gettimeofday(tv, timezone) wiced_gettimeofday(tv, timezone)
        /* Custom assertion function */
        #define assert(assertion) wiced_assert(TOSTRING(assertion), assertion)
    #endif

    #ifndef SOCKET_ERROR
	    #define SOCKET_ERROR   -1
    #endif /* SOCKET_ERROR */
    #ifndef INVALID_SOCKET
        #define INVALID_SOCKET -1
    #endif /* INVALID_SOCKET */
	
	#if defined(RTOS_FreeRTOS)
		#define HAVE_FREERTOS_THREAD
		#include <FreeRTOS.h>
		#include <task.h>
        #include <semphr.h>
	#elif defined(RTOS_ThreadX)
		#define HAVE_THREADX_THREAD
		#include <tx_api.h>
	#endif

	#if defined(NETWORK_LwIP)
		#include <lwip/netdb.h>
		#include <lwip/sockets.h>
		#include <lwip/sys.h>
	#elif defined(NETWORK_NetX) || defined(NETWORK_NetX_Duo)
        #include "WICED/compat.h"

        //typedef int socklen_t;

        #define close(socket)               soc_close(socket)
        #define write(socket, msg, length)  send(socket, msg, length, 0)
        #define read(socket, msg, length)   recv(socket, msg, length, 0)
        #elif defined(NETWORK_NuttX_NS)
                #include "WICED/NuttX_NS/sockets.h"
                #include "WICED/netdb.h"
                #include <sys/select.h>
                #include <netinet/in.h>
                #include <arpa/inet.h>
	#endif

    /* standard C headers */
    #include <stdlib.h>
    #include <stdio.h>
    #include <assert.h>
    #include <ctype.h>
    #include <errno.h>
    #include <string.h>
    #include <time.h>
    #include <math.h>
    #include "compat_gnudefs.h"

    #define wiced_ntohs(x) ( (((x)&0xff00)>>8) | \
                   (((x)&0x00ff)<<8) )
    #define wiced_ntohl(x) ( (((x)&0xff000000)>>24) | \
                   (((x)&0x00ff0000)>> 8) | \
                   (((x)&0x0000ff00)<< 8) | \
                   (((x)&0x000000ff)<<24) )

    #define wiced_htons(x)  wiced_ntohs(x)
    #define wiced_htonl(x)  wiced_ntohl(x)

#else
    /* required on AIX for FD_SET (requires bzero).
     * often this is the same as <string.h> */
    #ifdef HAVE_STRINGS_H
        #include <strings.h>
    #endif /* HAVE_STRINGS_H */

    /* Unix headers */
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <sys/time.h>
    #include <signal.h>
    #include <unistd.h>

    /* Added for daemonizing the process */
    #include <syslog.h>

    SPECIAL_OSF1_EXTERN_C_START
    #include <netdb.h>
    SPECIAL_OSF1_EXTERN_C_STOP
    #include <netinet/in.h>
    #include <netinet/tcp.h>

    SPECIAL_OSF1_EXTERN_C_START
    #include <arpa/inet.h>   /* netinet/in.h must be before this on SunOS */
    SPECIAL_OSF1_EXTERN_C_STOP

    #ifdef HAVE_POSIX_THREAD
        #include <pthread.h>
    #endif /* HAVE_POSIX_THREAD */

    /* used in Win32 for error checking,
     * rather than checking rc < 0 as unix usually does */
    #define SOCKET_ERROR   -1
    #define INVALID_SOCKET -1

#endif /* not defined WIN32 */

#ifndef INET6_ADDRSTRLEN
    #define INET6_ADDRSTRLEN 40
#endif /* INET6_ADDRSTRLEN */
#ifndef INET_ADDRSTRLEN
    #define INET_ADDRSTRLEN 15
#endif /* INET_ADDRSTRLEN */

//#ifdef __cplusplus
    #ifdef HAVE_IPV6
        #define REPORT_ADDRLEN (INET6_ADDRSTRLEN + 1)
typedef struct sockaddr_storage iperf_sockaddr;
    #else
        #define REPORT_ADDRLEN (INET_ADDRSTRLEN + 1)
typedef struct sockaddr_in iperf_sockaddr;
    #endif /* HAVE_IPV6 */
//#endif

/* On WICED, we don't want the main function to be called "main" */
#ifndef WICED
	#define IPERF_MAIN main
#else
	#define IPERF_MAIN iperf
#endif /* WICED */

// Rationalize stdint definitions and sizeof, thanks to ac_create_stdint_h.m4
// from the gnu archive
#ifndef WICED
    #include <iperf-int.h>
#else
    #include <stdint.h>
#endif /* WICED */
typedef uint64_t max_size_t;

/* in case the OS doesn't have these, we provide our own implementations */
#include "gettimeofday.h"
#include "inet_aton.h"
#include "snprintf.h"

#ifndef SHUT_RD
    #define SHUT_RD   0
    #define SHUT_WR   1
    #define SHUT_RDWR 2
#endif /* SHUT_RD */

#if defined(NETWORK_NuttX_NS)
#define IPERF_BUFFERLEN 	        (32 * 1024)
#define NUM_REPORT_STRUCTS 	        (100)
#define NUM_MULTI_SLOTS             (5)
#elif defined(WICED)
#define IPERF_BUFFERLEN 	        (2 * 1024)
#define NUM_REPORT_STRUCTS 	        (10)
#define NUM_MULTI_SLOTS             (5)
#else
/* Default values */
#define IPERF_BUFFERLEN 	        (128 * 1024)
#define NUM_REPORT_STRUCTS 	        (700)
#define NUM_MULTI_SLOTS             (5)
#endif

#include "header_version.h"
#define HEADER_VERSION              HEADER_VERSION1

#include "iperf_debug.h"

/*============================================================================*/
/* Sanity checks                                                              */
/*============================================================================*/

/* Make sure that we only have one thread type defined */
#define __NUM_THREAD_TYPES_0 0
#ifdef HAVE_POSIX_THREAD
    #define __NUM_THREAD_TYPES_1 __NUM_THREAD_TYPES_0 + 1
#else
    #define __NUM_THREAD_TYPES_1 __NUM_THREAD_TYPES_0
#endif /* HAVE_POSIX_THREAD */
#ifdef HAVE_WIN32_THREAD
    #define __NUM_THREAD_TYPES_2 __NUM_THREAD_TYPES_1 + 1
#else
    #define __NUM_THREAD_TYPES_2 __NUM_THREAD_TYPES
#endif /* HAVE_WIN32_THREAD */
#ifdef HAVE_FREERTOS_THREAD
    #define __NUM_THREAD_TYPES_3 __NUM_THREAD_TYPES_2 + 1
#else
    #define __NUM_THREAD_TYPES_3 __NUM_THREAD_TYPES_2
#endif /* HAVE_FREERTOS_THREAD */
#ifdef HAVE_THREADX_THREAD
    #define __NUM_THREAD_TYPES_4 __NUM_THREAD_TYPES_3 + 1
#else
    #define __NUM_THREAD_TYPES_4 __NUM_THREAD_TYPES_3
#endif /* HAVE_THREADX_THREAD */
#define __NUM_THREAD_TYPES __NUM_THREAD_TYPES_4

#if __NUM_THREAD_TYPES > 1
#error "Too many thread types defined."
#endif /* __NUM_THREAD_TYPES > 1 */

#endif /* HEADERS_H */
