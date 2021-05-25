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

#ifndef FLAC__NO_ASM
    #if ( defined FLAC__CPU_IA32 || defined FLAC__CPU_X86_64 ) && defined FLAC__HAS_X86INTRIN
        #include "private/stream_encoder.h"
        #include "private/bitmath.h"
        #ifdef FLAC__SSSE3_SUPPORTED

            #include <stdlib.h>    /* for abs() */
            #include <tmmintrin.h> /* SSSE3 */
            #include "FLAC/assert.h"

FLAC__SSE_TARGET( "ssse3" )
void FLAC__precompute_partition_info_sums_intrin_ssse3( const FLAC__int32 residual[], FLAC__uint64 abs_residual_partition_sums[],
                                                        unsigned residual_samples, unsigned predictor_order, unsigned min_partition_order, unsigned max_partition_order, unsigned bps )
{
    const unsigned default_partition_samples = ( residual_samples + predictor_order ) >> max_partition_order;
    unsigned       partitions = 1u << max_partition_order;

    FLAC__ASSERT( default_partition_samples > predictor_order );

    /* first do max_partition_order */
    {
        unsigned partition, residual_sample, end = (unsigned) ( -(int) predictor_order );
        unsigned e1, e3;
        __m128i  mm_res, mm_sum;

        if ( FLAC__bitmath_ilog2( default_partition_samples ) + bps + FLAC__MAX_EXTRA_RESIDUAL_BPS < 32 )
        {
            for ( partition = residual_sample = 0; partition < partitions; partition++ )
            {
                end += default_partition_samples;
                mm_sum = _mm_setzero_si128( );

                e1 = ( residual_sample + 3 ) & ~3;
                e3 = end & ~3;
                if ( e1 > end )
                {
                    e1 = end; /* try flac -l 1 -b 16 and you'll be here */
                }
                /* assumption: residual[] is properly aligned so (residual + e1) is properly aligned too and _mm_loadu_si128() is fast */
                for ( ; residual_sample < e1; residual_sample++ )
                {
                    mm_res = _mm_cvtsi32_si128( residual[ residual_sample ] );
                    mm_res = _mm_abs_epi32( mm_res ); /* abs(INT_MIN) is undefined, but if the residual is INT_MIN we have bigger problems */
                    mm_sum = _mm_add_epi32( mm_sum, mm_res );
                }

                for ( ; residual_sample < e3; residual_sample += 4 )
                {
                    mm_res = _mm_loadu_si128( (const __m128i*) ( residual + residual_sample ) );
                    mm_res = _mm_abs_epi32( mm_res );
                    mm_sum = _mm_add_epi32( mm_sum, mm_res );
                }

                for ( ; residual_sample < end; residual_sample++ )
                {
                    mm_res = _mm_cvtsi32_si128( residual[ residual_sample ] );
                    mm_res = _mm_abs_epi32( mm_res );
                    mm_sum = _mm_add_epi32( mm_sum, mm_res );
                }

                mm_sum = _mm_hadd_epi32( mm_sum, mm_sum );
                mm_sum = _mm_hadd_epi32( mm_sum, mm_sum );
                abs_residual_partition_sums[ partition ] = (FLAC__uint32) _mm_cvtsi128_si32( mm_sum );
            }
        }
        else   /* have to pessimistically use 64 bits for accumulator */
        {
            for ( partition = residual_sample = 0; partition < partitions; partition++ )
            {
                end += default_partition_samples;
                mm_sum = _mm_setzero_si128( );

                e1 = ( residual_sample + 1 ) & ~1;
                e3 = end & ~1;
                FLAC__ASSERT( e1 <= end );

                for ( ; residual_sample < e1; residual_sample++ )
                {
                    mm_res = _mm_cvtsi32_si128( residual[ residual_sample ] ); /*  0   0   0   r0 */
                    mm_res = _mm_abs_epi32( mm_res );                          /*  0   0   0  |r0|  ==   00   |r0_64| */
                    mm_sum = _mm_add_epi64( mm_sum, mm_res );
                }

                for ( ; residual_sample < e3; residual_sample += 2 )
                {
                    mm_res = _mm_loadl_epi64( (const __m128i*) ( residual + residual_sample ) ); /*  0   0   r1  r0 */
                    mm_res = _mm_abs_epi32( mm_res );                                            /*  0   0  |r1|   |r0| */
                    mm_res = _mm_shuffle_epi32( mm_res, _MM_SHUFFLE( 3, 1, 2, 0 ) );             /* 0  |r1|  0  |r0|  ==  |r1_64|  |r0_64|  */
                    mm_sum = _mm_add_epi64( mm_sum, mm_res );
                }

                for ( ; residual_sample < end; residual_sample++ )
                {
                    mm_res = _mm_cvtsi32_si128( residual[ residual_sample ] );
                    mm_res = _mm_abs_epi32( mm_res );
                    mm_sum = _mm_add_epi64( mm_sum, mm_res );
                }

                mm_sum = _mm_add_epi64( mm_sum, _mm_srli_si128( mm_sum, 8 ) );
                _mm_storel_epi64( (__m128i*) ( abs_residual_partition_sums + partition ), mm_sum );
            }
        }
    }

    /* now merge partitions for lower orders */
    {
        unsigned from_partition = 0, to_partition = partitions;
        int      partition_order;
        for ( partition_order = (int) max_partition_order - 1; partition_order >= (int) min_partition_order; partition_order-- )
        {
            unsigned i;
            partitions >>= 1;
            for ( i = 0; i < partitions; i++ )
            {
                abs_residual_partition_sums[ to_partition++ ] =
                    abs_residual_partition_sums[ from_partition  ] +
                    abs_residual_partition_sums[ from_partition + 1 ];
                from_partition += 2;
            }
        }
    }
}
        #endif /* FLAC__SSSE3_SUPPORTED */
    #endif /* (FLAC__CPU_IA32 || FLAC__CPU_X86_64) && FLAC__HAS_X86INTRIN */
#endif /* FLAC__NO_ASM */
