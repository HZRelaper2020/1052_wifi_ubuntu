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

/* libFLAC - Free Lossless Audio Codec library
 * Copyright (C) 2000-2009  Josh Coalson
 * Copyright (C) 2011-2014  Xiph.Org Foundation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * - Neither the name of the Xiph.org Foundation nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
    #  include <config.h>
#endif

#ifndef FLAC__INTEGER_ONLY_LIBRARY
    #ifndef FLAC__NO_ASM
        #if ( defined FLAC__CPU_IA32 || defined FLAC__CPU_X86_64 ) && defined FLAC__HAS_X86INTRIN
            #include "private/lpc.h"
            #ifdef FLAC__SSE_SUPPORTED

                #include "FLAC/assert.h"
                #include "FLAC/format.h"

                #include <xmmintrin.h> /* SSE */

                #if 1
/* Faster on current Intel (starting from Core i aka Nehalem) and all AMD CPUs */

FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_4( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    int    i;
    int    limit = data_len - 4;
    __m128 sum0;

    (void) lag;
    FLAC__ASSERT( lag <= 4 );
    FLAC__ASSERT( lag <= data_len );

    sum0 = _mm_setzero_ps( );

    for ( i = 0; i <= limit; i++ )
    {
        __m128 d, d0;
        d0 = _mm_loadu_ps( data + i );
        d = d0;
        d = _mm_shuffle_ps( d, d, 0 );
        sum0 = _mm_add_ps( sum0, _mm_mul_ps( d0, d ) );
    }

    {
        __m128 d0 = _mm_setzero_ps( );
        limit++;
        if ( limit < 0 )
        {
            limit = 0;
        }

        for ( i = data_len - 1; i >= limit; i-- )
        {
            __m128 d;
            d = _mm_load_ss( data + i );
            d = _mm_shuffle_ps( d, d, 0 );
            d0 = _mm_shuffle_ps( d0, d0, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d0 = _mm_move_ss( d0, d );
            sum0 = _mm_add_ps( sum0, _mm_mul_ps( d, d0 ) );
        }
    }

    _mm_storeu_ps( autoc, sum0 );
}


FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_8( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    int    i;
    int    limit = data_len - 8;
    __m128 sum0, sum1;

    (void) lag;
    FLAC__ASSERT( lag <= 8 );
    FLAC__ASSERT( lag <= data_len );

    sum0 = _mm_setzero_ps( );
    sum1 = _mm_setzero_ps( );

    for ( i = 0; i <= limit; i++ )
    {
        __m128 d, d0, d1;
        d0 = _mm_loadu_ps( data + i );
        d1 = _mm_loadu_ps( data + i + 4 );
        d = d0;
        d = _mm_shuffle_ps( d, d, 0 );
        sum0 = _mm_add_ps( sum0, _mm_mul_ps( d0, d ) );
        sum1 = _mm_add_ps( sum1, _mm_mul_ps( d1, d ) );
    }

    {
        __m128 d0 = _mm_setzero_ps( );
        __m128 d1 = _mm_setzero_ps( );
        limit++;
        if ( limit < 0 )
        {
            limit = 0;
        }

        for ( i = data_len - 1; i >= limit; i-- )
        {
            __m128 d;
            d = _mm_load_ss( data + i );
            d = _mm_shuffle_ps( d, d, 0 );
            d1 = _mm_shuffle_ps( d1, d1, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d0 = _mm_shuffle_ps( d0, d0, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d1 = _mm_move_ss( d1, d0 );
            d0 = _mm_move_ss( d0, d );
            sum1 = _mm_add_ps( sum1, _mm_mul_ps( d, d1 ) );
            sum0 = _mm_add_ps( sum0, _mm_mul_ps( d, d0 ) );
        }
    }

    _mm_storeu_ps( autoc,     sum0 );
    _mm_storeu_ps( autoc + 4, sum1 );
}


FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_12( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    int    i;
    int    limit = data_len - 12;
    __m128 sum0, sum1, sum2;

    (void) lag;
    FLAC__ASSERT( lag <= 12 );
    FLAC__ASSERT( lag <= data_len );

    sum0 = _mm_setzero_ps( );
    sum1 = _mm_setzero_ps( );
    sum2 = _mm_setzero_ps( );

    for ( i = 0; i <= limit; i++ )
    {
        __m128 d, d0, d1, d2;
        d0 = _mm_loadu_ps( data + i );
        d1 = _mm_loadu_ps( data + i + 4 );
        d2 = _mm_loadu_ps( data + i + 8 );
        d = d0;
        d = _mm_shuffle_ps( d, d, 0 );
        sum0 = _mm_add_ps( sum0, _mm_mul_ps( d0, d ) );
        sum1 = _mm_add_ps( sum1, _mm_mul_ps( d1, d ) );
        sum2 = _mm_add_ps( sum2, _mm_mul_ps( d2, d ) );
    }

    {
        __m128 d0 = _mm_setzero_ps( );
        __m128 d1 = _mm_setzero_ps( );
        __m128 d2 = _mm_setzero_ps( );
        limit++;
        if ( limit < 0 )
        {
            limit = 0;
        }

        for ( i = data_len - 1; i >= limit; i-- )
        {
            __m128 d;
            d = _mm_load_ss( data + i );
            d = _mm_shuffle_ps( d, d, 0 );
            d2 = _mm_shuffle_ps( d2, d2, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d1 = _mm_shuffle_ps( d1, d1, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d0 = _mm_shuffle_ps( d0, d0, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d2 = _mm_move_ss( d2, d1 );
            d1 = _mm_move_ss( d1, d0 );
            d0 = _mm_move_ss( d0, d );
            sum2 = _mm_add_ps( sum2, _mm_mul_ps( d, d2 ) );
            sum1 = _mm_add_ps( sum1, _mm_mul_ps( d, d1 ) );
            sum0 = _mm_add_ps( sum0, _mm_mul_ps( d, d0 ) );
        }
    }

    _mm_storeu_ps( autoc,     sum0 );
    _mm_storeu_ps( autoc + 4, sum1 );
    _mm_storeu_ps( autoc + 8, sum2 );
}


FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_16( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    int    i;
    int    limit = data_len - 16;
    __m128 sum0, sum1, sum2, sum3;

    (void) lag;
    FLAC__ASSERT( lag <= 16 );
    FLAC__ASSERT( lag <= data_len );

    sum0 = _mm_setzero_ps( );
    sum1 = _mm_setzero_ps( );
    sum2 = _mm_setzero_ps( );
    sum3 = _mm_setzero_ps( );

    for ( i = 0; i <= limit; i++ )
    {
        __m128 d, d0, d1, d2, d3;
        d0 = _mm_loadu_ps( data + i );
        d1 = _mm_loadu_ps( data + i + 4 );
        d2 = _mm_loadu_ps( data + i + 8 );
        d3 = _mm_loadu_ps( data + i + 12 );
        d = d0;
        d = _mm_shuffle_ps( d, d, 0 );
        sum0 = _mm_add_ps( sum0, _mm_mul_ps( d0, d ) );
        sum1 = _mm_add_ps( sum1, _mm_mul_ps( d1, d ) );
        sum2 = _mm_add_ps( sum2, _mm_mul_ps( d2, d ) );
        sum3 = _mm_add_ps( sum3, _mm_mul_ps( d3, d ) );
    }

    {
        __m128 d0 = _mm_setzero_ps( );
        __m128 d1 = _mm_setzero_ps( );
        __m128 d2 = _mm_setzero_ps( );
        __m128 d3 = _mm_setzero_ps( );
        limit++;
        if ( limit < 0 )
        {
            limit = 0;
        }

        for ( i = data_len - 1; i >= limit; i-- )
        {
            __m128 d;
            d = _mm_load_ss( data + i );
            d = _mm_shuffle_ps( d, d, 0 );
            d3 = _mm_shuffle_ps( d3, d3, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d2 = _mm_shuffle_ps( d2, d2, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d1 = _mm_shuffle_ps( d1, d1, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d0 = _mm_shuffle_ps( d0, d0, _MM_SHUFFLE( 2, 1, 0, 3 ) );
            d3 = _mm_move_ss( d3, d2 );
            d2 = _mm_move_ss( d2, d1 );
            d1 = _mm_move_ss( d1, d0 );
            d0 = _mm_move_ss( d0, d );
            sum3 = _mm_add_ps( sum3, _mm_mul_ps( d, d3 ) );
            sum2 = _mm_add_ps( sum2, _mm_mul_ps( d, d2 ) );
            sum1 = _mm_add_ps( sum1, _mm_mul_ps( d, d1 ) );
            sum0 = _mm_add_ps( sum0, _mm_mul_ps( d, d0 ) );
        }
    }

    _mm_storeu_ps( autoc,      sum0 );
    _mm_storeu_ps( autoc + 4,  sum1 );
    _mm_storeu_ps( autoc + 8,  sum2 );
    _mm_storeu_ps( autoc + 12, sum3 );
}


                #else
/* Faster on older Intel CPUs (up to Core 2) */

FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_4( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    __m128 xmm0, xmm2, xmm5;

    (void) lag;
    FLAC__ASSERT( lag > 0 );
    FLAC__ASSERT( lag <= 4 );
    FLAC__ASSERT( lag <= data_len );
    FLAC__ASSERT( data_len > 0 );

    xmm5 = _mm_setzero_ps( );

    xmm0 = _mm_load_ss( data++ );
    xmm2 = xmm0;
    xmm0 = _mm_shuffle_ps( xmm0, xmm0, 0 );

    xmm0 = _mm_mul_ps( xmm0, xmm2 );
    xmm5 = _mm_add_ps( xmm5, xmm0 );

    data_len--;

    while ( data_len )
    {
        xmm0 = _mm_load1_ps( data++ );

        xmm2 = _mm_shuffle_ps( xmm2, xmm2, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm2 = _mm_move_ss( xmm2, xmm0 );
        xmm0 = _mm_mul_ps( xmm0, xmm2 );
        xmm5 = _mm_add_ps( xmm5, xmm0 );

        data_len--;
    }

    _mm_storeu_ps( autoc, xmm5 );
}


FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_8( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    __m128 xmm0, xmm1, xmm2, xmm3, xmm5, xmm6;

    (void) lag;
    FLAC__ASSERT( lag > 0 );
    FLAC__ASSERT( lag <= 8 );
    FLAC__ASSERT( lag <= data_len );
    FLAC__ASSERT( data_len > 0 );

    xmm5 = _mm_setzero_ps( );
    xmm6 = _mm_setzero_ps( );

    xmm0 = _mm_load_ss( data++ );
    xmm2 = xmm0;
    xmm0 = _mm_shuffle_ps( xmm0, xmm0, 0 );
    xmm3 = _mm_setzero_ps( );

    xmm0 = _mm_mul_ps( xmm0, xmm2 );
    xmm5 = _mm_add_ps( xmm5, xmm0 );

    data_len--;

    while ( data_len )
    {
        xmm0 = _mm_load1_ps( data++ );

        xmm2 = _mm_shuffle_ps( xmm2, xmm2, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm3 = _mm_shuffle_ps( xmm3, xmm3, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm3 = _mm_move_ss( xmm3, xmm2 );
        xmm2 = _mm_move_ss( xmm2, xmm0 );

        xmm1 = xmm0;
        xmm1 = _mm_mul_ps( xmm1, xmm3 );
        xmm0 = _mm_mul_ps( xmm0, xmm2 );
        xmm6 = _mm_add_ps( xmm6, xmm1 );
        xmm5 = _mm_add_ps( xmm5, xmm0 );

        data_len--;
    }

    _mm_storeu_ps( autoc,     xmm5 );
    _mm_storeu_ps( autoc + 4, xmm6 );
}


FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_12( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    __m128 xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7;

    (void) lag;
    FLAC__ASSERT( lag > 0 );
    FLAC__ASSERT( lag <= 12 );
    FLAC__ASSERT( lag <= data_len );
    FLAC__ASSERT( data_len > 0 );

    xmm5 = _mm_setzero_ps( );
    xmm6 = _mm_setzero_ps( );
    xmm7 = _mm_setzero_ps( );

    xmm0 = _mm_load_ss( data++ );
    xmm2 = xmm0;
    xmm0 = _mm_shuffle_ps( xmm0, xmm0, 0 );
    xmm3 = _mm_setzero_ps( );
    xmm4 = _mm_setzero_ps( );

    xmm0 = _mm_mul_ps( xmm0, xmm2 );
    xmm5 = _mm_add_ps( xmm5, xmm0 );

    data_len--;

    while ( data_len )
    {
        xmm0 = _mm_load1_ps( data++ );

        xmm2 = _mm_shuffle_ps( xmm2, xmm2, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm3 = _mm_shuffle_ps( xmm3, xmm3, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm4 = _mm_shuffle_ps( xmm4, xmm4, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm4 = _mm_move_ss( xmm4, xmm3 );
        xmm3 = _mm_move_ss( xmm3, xmm2 );
        xmm2 = _mm_move_ss( xmm2, xmm0 );

        xmm1 = xmm0;
        xmm1 = _mm_mul_ps( xmm1, xmm2 );
        xmm5 = _mm_add_ps( xmm5, xmm1 );
        xmm1 = xmm0;
        xmm1 = _mm_mul_ps( xmm1, xmm3 );
        xmm6 = _mm_add_ps( xmm6, xmm1 );
        xmm0 = _mm_mul_ps( xmm0, xmm4 );
        xmm7 = _mm_add_ps( xmm7, xmm0 );

        data_len--;
    }

    _mm_storeu_ps( autoc,     xmm5 );
    _mm_storeu_ps( autoc + 4, xmm6 );
    _mm_storeu_ps( autoc + 8, xmm7 );
}


FLAC__SSE_TARGET( "sse" )
void FLAC__lpc_compute_autocorrelation_intrin_sse_lag_16( const FLAC__real data[], unsigned data_len, unsigned lag, FLAC__real autoc[] )
{
    __m128 xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7, xmm8, xmm9;

    (void) lag;
    FLAC__ASSERT( lag > 0 );
    FLAC__ASSERT( lag <= 16 );
    FLAC__ASSERT( lag <= data_len );
    FLAC__ASSERT( data_len > 0 );

    xmm6 = _mm_setzero_ps( );
    xmm7 = _mm_setzero_ps( );
    xmm8 = _mm_setzero_ps( );
    xmm9 = _mm_setzero_ps( );

    xmm0 = _mm_load_ss( data++ );
    xmm2 = xmm0;
    xmm0 = _mm_shuffle_ps( xmm0, xmm0, 0 );
    xmm3 = _mm_setzero_ps( );
    xmm4 = _mm_setzero_ps( );
    xmm5 = _mm_setzero_ps( );

    xmm0 = _mm_mul_ps( xmm0, xmm2 );
    xmm6 = _mm_add_ps( xmm6, xmm0 );

    data_len--;

    while ( data_len )
    {
        xmm0 = _mm_load1_ps( data++ );

        /* shift xmm5:xmm4:xmm3:xmm2 left by one float */
        xmm5 = _mm_shuffle_ps( xmm5, xmm5, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm4 = _mm_shuffle_ps( xmm4, xmm4, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm3 = _mm_shuffle_ps( xmm3, xmm3, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm2 = _mm_shuffle_ps( xmm2, xmm2, _MM_SHUFFLE( 2, 1, 0, 3 ) );
        xmm5 = _mm_move_ss( xmm5, xmm4 );
        xmm4 = _mm_move_ss( xmm4, xmm3 );
        xmm3 = _mm_move_ss( xmm3, xmm2 );
        xmm2 = _mm_move_ss( xmm2, xmm0 );

        /* xmm9|xmm8|xmm7|xmm6 += xmm0|xmm0|xmm0|xmm0 * xmm5|xmm4|xmm3|xmm2 */
        xmm1 = xmm0;
        xmm1 = _mm_mul_ps( xmm1, xmm5 );
        xmm9 = _mm_add_ps( xmm9, xmm1 );
        xmm1 = xmm0;
        xmm1 = _mm_mul_ps( xmm1, xmm4 );
        xmm8 = _mm_add_ps( xmm8, xmm1 );
        xmm1 = xmm0;
        xmm1 = _mm_mul_ps( xmm1, xmm3 );
        xmm7 = _mm_add_ps( xmm7, xmm1 );
        xmm0 = _mm_mul_ps( xmm0, xmm2 );
        xmm6 = _mm_add_ps( xmm6, xmm0 );

        data_len--;
    }

    _mm_storeu_ps( autoc,      xmm6 );
    _mm_storeu_ps( autoc + 4,  xmm7 );
    _mm_storeu_ps( autoc + 8,  xmm8 );
    _mm_storeu_ps( autoc + 12, xmm9 );
}
                #endif
            #endif /* FLAC__SSE_SUPPORTED */
        #endif /* (FLAC__CPU_IA32 || FLAC__CPU_X86_64) && FLAC__HAS_X86INTRIN */
    #endif /* FLAC__NO_ASM */
#endif /* FLAC__INTEGER_ONLY_LIBRARY */
