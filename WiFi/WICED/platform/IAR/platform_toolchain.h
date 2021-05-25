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
#ifndef INCLUDED_WWD_TOOLCHAIN_H
#define INCLUDED_WWD_TOOLCHAIN_H

#include <stddef.h>
#include "intrinsics.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define WEAK __weak

#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE_PRE     _Pragma( "inline=forced" )
#define ALWAYS_INLINE
#endif

#ifndef MAY_BE_UNUSED
#define MAY_BE_UNUSED_PRE    __root
#define MAY_BE_UNUSED
#endif

#ifndef NORETURN
#define NORETURN_PRE         __noreturn
#define NORETURN
#endif

#ifndef ALIGNED
#define PRAGMA_ALIGN_4096       _Pragma( "data_alignment=4096" )
#define PRAGMA_ALIGN_2048       _Pragma( "data_alignment=2048" )
#define PRAGMA_ALIGN_256        _Pragma( "data_alignment=256" )
#define PRAGMA_ALIGN_128        _Pragma( "data_alignment=128" )
#define PRAGMA_ALIGN_64         _Pragma( "data_alignment=64" )
#define PRAGMA_ALIGN_48         _Pragma( "data_alignment=48" )
#define PRAGMA_ALIGN_32         _Pragma( "data_alignment=32" )
#define PRAGMA_ALIGN_4          _Pragma( "data_alignment=4" )
#define ALIGNED_PRE(size)       PRAGMA_ALIGN_ ## size
#define ALIGNED(size)
#define IAR_AUTO_VAR_MAX_ALIGN  8
#endif

#ifndef STATIC_ALWAYS_INLINE
#define STATIC_ALWAYS_INLINE _Pragma( "inline=forced" ) static inline
#endif

#ifndef INLINE_ASM
#define INLINE_ASM  __asm
#endif

/******************************************************
 *                    Constants
 ******************************************************/

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
void *memmem( const void *haystack, size_t haystacklen, const void *needle, size_t needlelen );
void *memrchr( const void *s, int c, size_t n );
size_t strlcpy( char *dest, const char *src, size_t size );

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* #ifndef INCLUDED_WWD_TOOLCHAIN_H */
