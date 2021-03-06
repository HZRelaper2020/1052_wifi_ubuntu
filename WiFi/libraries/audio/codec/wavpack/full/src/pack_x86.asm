;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;                           **** WAVPACK ****                            ;;
;;                  Hybrid Lossless Wavefile Compressor                   ;;
;;              Copyright (c) 1998 - 2015 Conifer Software.               ;;
;;                          All Rights Reserved.                          ;;
;;      Distributed under the BSD Software License (see license.txt)      ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        .686
        .mmx
        .model  flat
asmcode segment page 'CODE'
        public  _pack_decorr_stereo_pass_x86
        public  _pack_decorr_stereo_pass_cont_rev_x86
        public  _pack_decorr_stereo_pass_cont_x86
        public  _pack_decorr_mono_buffer_x86
        public  _pack_decorr_mono_pass_cont_x86
        public  _pack_cpu_has_feature_x86
        public  _scan_max_magnitude_x86
        public  _log2buffer_x86

; This module contains X86 assembly optimized versions of functions required
; to encode WavPack files.

; This is an assembly optimized version of the following WavPack function:
;
; void pack_decorr_stereo_pass (
;   struct decorr_pass *dpp,
;   int32_t *buffer,
;   int32_t sample_count);
;
; It performs a single pass of stereo decorrelation, in place, as specified
; by the decorr_pass structure. Note that this function does NOT return the
; dpp->samples_X[] values in the "normalized" positions for terms 1-8, so if
; the number of samples is not a multiple of MAX_TERM, these must be moved if
; they are to be used somewhere else.
;
; This is written to work on an IA-32 processor and uses the MMX extensions
; to improve the performance by processing both stereo channels together.
; It is based on the original MMX code written by Joachim Henke that used
; MMX intrinsics called from C. Many thanks to Joachim for that!
;
; An issue with using MMX for this is that the sample history array in the
; decorr_pass structure contains separate arrays for each channel while the
; MMX code wants there to be a single array of dual samples. The fix for
; this is to convert the data in the arrays on entry and exit, and this is
; made easy by the fact that the 8 MMX regsiters hold exactly the required
; amount of data (64 bytes)!
;
; This is written to work on an IA-32 processor. The arguments are on the
; stack at these locations (after 4 pushes, we do not use ebp as a base
; pointer):
;
;   struct decorr_pass *dpp   [esp+20]
;   int32_t *buffer           [esp+24]
;   int32_t sample_count      [esp+28]
;
; During the processing loops, the following registers are used:
;
;   edi         buffer pointer
;   esi         termination buffer pointer
;   eax,ebx,edx used in default term to reduce calculation         
;   ebp         decorr_pass pointer
;   mm0, mm1    scratch
;   mm2         original sample values
;   mm3         correlation samples
;   mm4         0 (for pcmpeqd)
;   mm5         weights
;   mm6         delta
;   mm7         512 (for rounding)
;

_pack_decorr_stereo_pass_x86:
        push    ebp
        push    ebx
        push    edi
        push    esi

        mov     ebp, [esp+20]               ; ebp = *dpp
        mov     edi, [esp+24]               ; edi = buffer
        mov     esi, [esp+28]
        sal     esi, 3
        jz      bdone
        add     esi, edi                    ; esi = termination buffer pointer

        ; convert samples_A and samples_B array into samples_AB array for MMX
        ; (the MMX registers provide exactly enough storage to do this easily)

        movq        mm0, [ebp+16]
        punpckldq   mm0, [ebp+48]
        movq        mm1, [ebp+16]
        punpckhdq   mm1, [ebp+48]
        movq        mm2, [ebp+24]
        punpckldq   mm2, [ebp+56]
        movq        mm3, [ebp+24]
        punpckhdq   mm3, [ebp+56]
        movq        mm4, [ebp+32]
        punpckldq   mm4, [ebp+64]
        movq        mm5, [ebp+32]
        punpckhdq   mm5, [ebp+64]
        movq        mm6, [ebp+40]
        punpckldq   mm6, [ebp+72]
        movq        mm7, [ebp+40]
        punpckhdq   mm7, [ebp+72]

        movq    [ebp+16], mm0
        movq    [ebp+24], mm1
        movq    [ebp+32], mm2
        movq    [ebp+40], mm3
        movq    [ebp+48], mm4
        movq    [ebp+56], mm5
        movq    [ebp+64], mm6
        movq    [ebp+72], mm7

        mov     eax, 512
        movd    mm7, eax
        punpckldq mm7, mm7                  ; mm7 = round (512)

        mov     eax, [ebp+4]
        movd    mm6, eax
        punpckldq mm6, mm6                  ; mm6 = delta (0-7)

        mov     eax, 0FFFFh                 ; mask high weights to zero for PMADDWD
        movd    mm5, eax
        punpckldq mm5, mm5                  ; mm5 = weight mask 0x0000FFFF0000FFFF
        pand    mm5, [ebp+8]                ; mm5 = weight_AB masked to 16-bit

        movq    mm4, [ebp+16]               ; preload samples_AB[0]

        mov     al, [ebp]                   ; get term and vector to correct loop
        cmp     al, 17
        je      buff_term_17_loop
        cmp     al, 18
        je      buff_term_18_loop
        cmp     al, -1
        je      buff_term_minus_1_loop
        cmp     al, -2
        je      buff_term_minus_2_loop
        cmp     al, -3
        je      buff_term_minus_3_loop

        pxor    mm4, mm4                    ; mm4 = 0 (for pcmpeqd)
        xor     eax, eax
        xor     ebx, ebx
        add     bl, [ebp]
        mov     ecx, 7
        and     ebx, ecx
        jmp     buff_default_term_loop

        align  64

buff_default_term_loop:
        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm3, [ebp+16+eax*8]
        inc     eax
        and     eax, ecx
        movq    [ebp+16+ebx*8], mm2
        inc     ebx
        and     ebx, ecx

        movq    mm1, mm3
        paddd   mm1, mm1
        psrlw   mm1, 1
        pmaddwd mm1, mm5

        movq    mm0, mm3
        psrld   mm0, 15
        pmaddwd mm0, mm5

        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi], mm2                  ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        add     edi, 8
        pcmpeqd mm2, mm4                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm4                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pxor    mm5, mm0
        paddw   mm5, mm2                    ; and add to weight_AB
        pxor    mm5, mm0
        cmp     edi, esi
        jnz     buff_default_term_loop

        jmp     bdone

        align  64

buff_term_17_loop:
        movq    mm3, mm4                    ; get previous calculated value
        paddd   mm3, mm4
        psubd   mm3, [ebp+24]
        movq    [ebp+24], mm4

        movq    mm1, mm3
        paddd   mm1, mm1
        psrlw   mm1, 1
        pmaddwd mm1, mm5

        movq    mm0, mm3
        psrld   mm0, 15
        pmaddwd mm0, mm5

        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm4, mm2
        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi], mm2                  ; store result
        pxor    mm1, mm1
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        add     edi, 8
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pxor    mm5, mm0
        paddw   mm5, mm2                    ; and add to weight_AB
        pxor    mm5, mm0
        cmp     edi, esi
        jnz     buff_term_17_loop

        movq    [ebp+16], mm4               ; post-store samples_AB[0]
        jmp     bdone

        align  64

buff_term_18_loop:
        movq    mm3, mm4                    ; get previous calculated value
        psubd   mm3, [ebp+24]
        psrad   mm3, 1
        paddd   mm3, mm4                    ; mm3 = sam_AB
        movq    [ebp+24], mm4

        movq    mm1, mm3
        paddd   mm1, mm1
        psrlw   mm1, 1
        pmaddwd mm1, mm5

        movq    mm0, mm3
        psrld   mm0, 15
        pmaddwd mm0, mm5

        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm4, mm2
        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi], mm2                  ; store result
        pxor    mm1, mm1
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        add     edi, 8
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pxor    mm5, mm0
        paddw   mm5, mm2                    ; and add to weight_AB
        pxor    mm5, mm0
        cmp     edi, esi
        jnz     buff_term_18_loop

        movq    [ebp+16], mm4               ; post-store samples_AB[0]
        jmp     bdone

        align  64

buff_term_minus_1_loop:
        movq    mm3, mm4                    ; mm3 = previous calculated value
        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm4, mm2
        psrlq   mm4, 32
        punpckldq mm3, mm2                  ; mm3 = sam_AB

        movq    mm1, mm3
        paddd   mm1, mm1
        psrlw   mm1, 1
        pmaddwd mm1, mm5

        movq    mm0, mm3
        psrld   mm0, 15
        pmaddwd mm0, mm5

        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi], mm2                  ; store result
        pxor    mm1, mm1
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        add     edi, 8
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pcmpeqd mm1, mm1
        psubd   mm1, mm7
        psubd   mm1, mm7
        psubd   mm1, mm0
        pxor    mm5, mm0
        paddw   mm5, mm1
        paddusw mm5, mm2                    ; and add to weight_AB
        psubw   mm5, mm1
        pxor    mm5, mm0
        cmp     edi, esi
        jnz     buff_term_minus_1_loop

        movq    [ebp+16], mm4               ; post-store samples_AB[0]
        jmp     bdone

        align  64

buff_term_minus_2_loop:
        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm3, mm2
        psrlq   mm3, 32
        por     mm3, mm4
        punpckldq mm4, mm2

        movq    mm1, mm3
        paddd   mm1, mm1
        psrlw   mm1, 1
        pmaddwd mm1, mm5

        movq    mm0, mm3
        psrld   mm0, 15
        pmaddwd mm0, mm5

        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi], mm2                  ; store result
        pxor    mm1, mm1
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        add     edi, 8
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pcmpeqd mm1, mm1
        psubd   mm1, mm7
        psubd   mm1, mm7
        psubd   mm1, mm0
        pxor    mm5, mm0
        paddw   mm5, mm1
        paddusw mm5, mm2                    ; and add to weight_AB
        psubw   mm5, mm1
        pxor    mm5, mm0
        cmp     edi, esi
        jnz     buff_term_minus_2_loop

        movq    [ebp+16], mm4               ; post-store samples_AB[0]
        jmp     bdone

        align  64

buff_term_minus_3_loop:
        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm3, mm4                    ; mm3 = previous calculated value
        movq    mm4, mm2                    ; mm0 = swap dwords of new data
        psrlq   mm4, 32
        punpckldq mm4, mm2                  ; mm3 = sam_AB

        movq    mm1, mm3
        paddd   mm1, mm1
        psrlw   mm1, 1
        pmaddwd mm1, mm5

        movq    mm0, mm3
        psrld   mm0, 15
        pmaddwd mm0, mm5

        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi], mm2                  ; store result
        pxor    mm1, mm1
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        add     edi, 8
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pcmpeqd mm1, mm1
        psubd   mm1, mm7
        psubd   mm1, mm7
        psubd   mm1, mm0
        pxor    mm5, mm0
        paddw   mm5, mm1
        paddusw mm5, mm2                    ; and add to weight_AB
        psubw   mm5, mm1
        pxor    mm5, mm0
        cmp     edi, esi
        jnz     buff_term_minus_3_loop

        movq    [ebp+16], mm4               ; post-store samples_AB[0]

bdone:  pslld   mm5, 16                     ; sign-extend 16-bit weights back to dwords
        psrad   mm5, 16
        movq    [ebp+8], mm5                ; put weight_AB back

        ; convert samples_AB array back into samples_A and samples_B

        movq    mm0, [ebp+16]
        movq    mm1, [ebp+24]
        movq    mm2, [ebp+32]
        movq    mm3, [ebp+40]
        movq    mm4, [ebp+48]
        movq    mm5, [ebp+56]
        movq    mm6, [ebp+64]
        movq    mm7, [ebp+72]

        movd    DWORD PTR [ebp+16], mm0
        movd    DWORD PTR [ebp+20], mm1
        movd    DWORD PTR [ebp+24], mm2
        movd    DWORD PTR [ebp+28], mm3
        movd    DWORD PTR [ebp+32], mm4
        movd    DWORD PTR [ebp+36], mm5
        movd    DWORD PTR [ebp+40], mm6
        movd    DWORD PTR [ebp+44], mm7

        punpckhdq   mm0, mm0
        punpckhdq   mm1, mm1
        punpckhdq   mm2, mm2
        punpckhdq   mm3, mm3
        punpckhdq   mm4, mm4
        punpckhdq   mm5, mm5
        punpckhdq   mm6, mm6
        punpckhdq   mm7, mm7

        movd    DWORD PTR [ebp+48], mm0
        movd    DWORD PTR [ebp+52], mm1
        movd    DWORD PTR [ebp+56], mm2
        movd    DWORD PTR [ebp+60], mm3
        movd    DWORD PTR [ebp+64], mm4
        movd    DWORD PTR [ebp+68], mm5
        movd    DWORD PTR [ebp+72], mm6
        movd    DWORD PTR [ebp+76], mm7

        emms

        pop     esi
        pop     edi
        pop     ebx
        pop     ebp
        ret

; These are assembly optimized version of the following WavPack functions:
;
; void pack_decorr_stereo_pass_cont (
;   struct decorr_pass *dpp,
;   int32_t *in_buffer,
;   int32_t *out_buffer,
;   int32_t sample_count);
;
; void pack_decorr_stereo_pass_cont_rev (
;   struct decorr_pass *dpp,
;   int32_t *in_buffer,
;   int32_t *out_buffer,
;   int32_t sample_count);
;
; It performs a single pass of stereo decorrelation, transfering from the
; input buffer to the output buffer. Note that this version of the function
; requires that the up to 8 previous (depending on dpp->term) stereo samples
; are visible and correct. In other words, it ignores the "samples_*"
; fields in the decorr_pass structure and gets the history data directly
; from the source buffer. It does, however, return the appropriate history
; samples to the decorr_pass structure before returning.
;
; This is written to work on an IA-32 processor and uses the MMX extensions
; to improve the performance by processing both stereo channels together.
; It is based on the original MMX code written by Joachim Henke that used
; MMX intrinsics called from C. Many thanks to Joachim for that!
;
; No additional stack space is used; all storage is done in registers. The
; arguments on entry:
;
;   struct decorr_pass *dpp     [ebp+8]
;   int32_t *in_buffer          [ebp+12]
;   int32_t *out_buffer         [ebp+16]
;   int32_t sample_count        [ebp+20]
;
; During the processing loops, the following registers are used:
;
;   edi         input buffer pointer
;   esi         direction (-8 forward, +8 reverse)
;   ebx         delta from input to output buffer
;   ecx         sample count
;   edx         sign (dir) * term * -8 (terms 1-8 only)
;   mm0, mm1    scratch
;   mm2         original sample values
;   mm3         correlation samples
;   mm4         weight sums
;   mm5         weights
;   mm6         delta
;   mm7         512 (for rounding)
;

_pack_decorr_stereo_pass_cont_rev_x86:
        push    ebp
        mov     ebp, esp
        push    ebx                         ; save the registers that we need to
        push    esi
        push    edi

        mov     esi, 8                      ; esi indicates direction (inverted)
        jmp     start

_pack_decorr_stereo_pass_cont_x86:
        push    ebp
        mov     ebp, esp
        push    ebx                         ; save the registers that we need to
        push    esi
        push    edi

        mov     esi, -8                     ; esi indicates direction (inverted)

start:  mov     eax, 512
        movd    mm7, eax
        punpckldq mm7, mm7                  ; mm7 = round (512)

        mov     eax, [ebp+8]                ; access dpp
        mov     eax, [eax+4]
        movd    mm6, eax
        punpckldq mm6, mm6                  ; mm6 = delta (0-7)

        mov     eax, [ebp+8]                ; access dpp
        movq    mm5, [eax+8]                ; mm5 = weight_AB
        movq    mm4, [eax+88]               ; mm4 = sum_AB

        mov     edi, [ebp+12]               ; edi = in_buffer
        mov     ebx, [ebp+16]
        sub     ebx, edi                    ; ebx = delta to output buffer

        mov     ecx, [ebp+20]               ; ecx = sample_count
        test    ecx, ecx
        jz      done

        mov     eax, [ebp+8]                ; *eax = dpp
        mov     eax, [eax]                  ; get term and vector to correct loop
        cmp     eax, 17
        je      term_17_loop
        cmp     eax, 18
        je      term_18_loop
        cmp     eax, -1
        je      term_minus_1_loop
        cmp     eax, -2
        je      term_minus_2_loop
        cmp     eax, -3
        je      term_minus_3_loop

        sal     eax, 3
        mov     edx, eax                    ; edx = term * 8 to index correlation sample
        test    esi, esi                    ; test direction
        jns     default_term_loop
        neg     edx
        jmp     default_term_loop

        align  64

default_term_loop:
        movq    mm3, [edi+edx]              ; mm3 = sam_AB

        movq    mm1, mm3
        pslld   mm1, 17
        psrld   mm1, 17
        pmaddwd mm1, mm5

        movq    mm0, mm3
        pslld   mm0, 1
        psrld   mm0, 16
        pmaddwd mm0, mm5

        movq    mm2, [edi]                  ; mm2 = left_right
        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi+ebx], mm2              ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        sub     edi, esi
        pxor    mm1, mm1                    ; mm1 = zero
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pxor    mm5, mm0
        paddd   mm5, mm2                    ; and add to weight_AB
        pxor    mm5, mm0
        paddd   mm4, mm5                    ; add weights to sum
        dec     ecx
        jnz     default_term_loop

        mov     eax, [ebp+8]                ; access dpp
        movq    [eax+8], mm5                ; put weight_AB back
        movq    [eax+88], mm4               ; put sum_AB back
        emms

        mov     edx, [ebp+8]                ; access dpp with edx
        mov     ecx, [edx]                  ; ecx = dpp->term

default_store_samples:
        dec     ecx
        add     edi, esi                    ; back up one full sample
        mov     eax, [edi+4]
        mov     [edx+ecx*4+48], eax         ; store samples_B [ecx]
        mov     eax, [edi]
        mov     [edx+ecx*4+16], eax         ; store samples_A [ecx]
        test    ecx, ecx
        jnz     default_store_samples
        jmp     done

        align  64

term_17_loop:
        movq    mm3, [edi+esi]              ; get previous calculated value
        paddd   mm3, mm3
        psubd   mm3, [edi+esi*2]

        movq    mm1, mm3
        pslld   mm1, 17
        psrld   mm1, 17
        pmaddwd mm1, mm5

        movq    mm0, mm3
        pslld   mm0, 1
        psrld   mm0, 16
        pmaddwd mm0, mm5

        movq    mm2, [edi]                  ; mm2 = left_right
        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi+ebx], mm2              ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        sub     edi, esi
        pxor    mm1, mm1                    ; mm1 = zero
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pxor    mm5, mm0
        paddd   mm5, mm2                    ; and add to weight_AB
        pxor    mm5, mm0
        paddd   mm4, mm5                    ; add weights to sum
        dec     ecx
        jnz     term_17_loop

        mov     eax, [ebp+8]                ; access dpp
        movq    [eax+8], mm5                ; put weight_AB back
        movq    [eax+88], mm4               ; put sum_AB back
        emms
        jmp     term_1718_common_store

        align  64

term_18_loop:
        movq    mm3, [edi+esi]              ; get previous calculated value
        movq    mm0, mm3
        psubd   mm3, [edi+esi*2]
        psrad   mm3, 1
        paddd   mm3, mm0                    ; mm3 = sam_AB

        movq    mm1, mm3
        pslld   mm1, 17
        psrld   mm1, 17
        pmaddwd mm1, mm5

        movq    mm0, mm3
        pslld   mm0, 1
        psrld   mm0, 16
        pmaddwd mm0, mm5

        movq    mm2, [edi]                  ; mm2 = left_right
        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi+ebx], mm2              ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        sub     edi, esi
        pxor    mm1, mm1                    ; mm1 = zero
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pxor    mm5, mm0
        paddd   mm5, mm2                    ; and add to weight_AB
        pxor    mm5, mm0
        dec     ecx
        paddd   mm4, mm5                    ; add weights to sum
        jnz     term_18_loop

        mov     eax, [ebp+8]                ; access dpp
        movq    [eax+8], mm5                ; put weight_AB back
        movq    [eax+88], mm4               ; put sum_AB back
        emms

term_1718_common_store:

        mov     eax, [ebp+8]                ; access dpp
        add     edi, esi                    ; back up a full sample
        mov     edx, [edi+4]                ; dpp->samples_B [0] = iptr [-1];
        mov     [eax+48], edx
        mov     edx, [edi]                  ; dpp->samples_A [0] = iptr [-2];
        mov     [eax+16], edx
        add     edi, esi                    ; back up another sample
        mov     edx, [edi+4]                ; dpp->samples_B [1] = iptr [-3];
        mov     [eax+52], edx
        mov     edx, [edi]                  ; dpp->samples_A [1] = iptr [-4];
        mov     [eax+20], edx
        jmp     done

        align  64

term_minus_1_loop:
        movq    mm3, [edi+esi]              ; mm3 = previous calculated value
        movq    mm2, [edi]                  ; mm2 = left_right
        psrlq   mm3, 32
        punpckldq mm3, mm2                  ; mm3 = sam_AB

        movq    mm1, mm3
        pslld   mm1, 17
        psrld   mm1, 17
        pmaddwd mm1, mm5

        movq    mm0, mm3
        pslld   mm0, 1
        psrld   mm0, 16
        pmaddwd mm0, mm5

        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi+ebx], mm2              ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        sub     edi, esi
        pxor    mm1, mm1                    ; mm1 = zero
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pcmpeqd mm1, mm1
        psubd   mm1, mm7
        psubd   mm1, mm7
        psubd   mm1, mm0
        pxor    mm5, mm0
        paddd   mm5, mm1
        paddusw mm5, mm2                    ; and add to weight_AB
        psubd   mm5, mm1
        pxor    mm5, mm0
        paddd   mm4, mm5                    ; add weights to sum
        dec     ecx
        jnz     term_minus_1_loop

        mov     eax, [ebp+8]                ; access dpp
        movq    [eax+8], mm5                ; put weight_AB back
        movq    [eax+88], mm4               ; put sum_AB back
        emms

        add     edi, esi                    ; back up a full sample
        mov     edx, [edi+4]                ; dpp->samples_A [0] = iptr [-1];
        mov     eax, [ebp+8]
        mov     [eax+16], edx
        jmp     done

        align  64

term_minus_2_loop:
        movq    mm2, [edi]                  ; mm2 = left_right
        movq    mm3, mm2                    ; mm3 = swap dwords
        psrlq   mm3, 32
        punpckldq mm3, [edi+esi]            ; mm3 = sam_AB

        movq    mm1, mm3
        pslld   mm1, 17
        psrld   mm1, 17
        pmaddwd mm1, mm5

        movq    mm0, mm3
        pslld   mm0, 1
        psrld   mm0, 16
        pmaddwd mm0, mm5

        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi+ebx], mm2              ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        sub     edi, esi
        pxor    mm1, mm1                    ; mm1 = zero
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pcmpeqd mm1, mm1
        psubd   mm1, mm7
        psubd   mm1, mm7
        psubd   mm1, mm0
        pxor    mm5, mm0
        paddd   mm5, mm1
        paddusw mm5, mm2                    ; and add to weight_AB
        psubd   mm5, mm1
        pxor    mm5, mm0
        paddd   mm4, mm5                    ; add weights to sum
        dec     ecx
        jnz     term_minus_2_loop

        mov     eax, [ebp+8]                ; access dpp
        movq    [eax+8], mm5                ; put weight_AB back
        movq    [eax+88], mm4               ; put sum_AB back
        emms

        add     edi, esi                    ; back up a full sample
        mov     edx, [edi]                  ; dpp->samples_B [0] = iptr [-2];
        mov     eax, [ebp+8]
        mov     [eax+48], edx
        jmp     done

        align  64

term_minus_3_loop:
        movq    mm0, [edi+esi]              ; mm0 = previous calculated value
        movq    mm3, mm0                    ; mm3 = swap dwords
        psrlq   mm3, 32
        punpckldq mm3, mm0                  ; mm3 = sam_AB

        movq    mm1, mm3
        pslld   mm1, 17
        psrld   mm1, 17
        pmaddwd mm1, mm5

        movq    mm0, mm3
        pslld   mm0, 1
        psrld   mm0, 16
        pmaddwd mm0, mm5

        movq    mm2, [edi]                  ; mm2 = left_right
        pslld   mm0, 5
        paddd   mm1, mm7                    ; add 512 for rounding
        psrad   mm1, 10
        psubd   mm2, mm0
        psubd   mm2, mm1                    ; add shifted sums
        movq    mm0, mm3
        movq    [edi+ebx], mm2              ; store result
        pxor    mm0, mm2
        psrad   mm0, 31                     ; mm0 = sign (sam_AB ^ left_right)
        sub     edi, esi
        pxor    mm1, mm1                    ; mm1 = zero
        pcmpeqd mm2, mm1                    ; mm2 = 1s if left_right was zero
        pcmpeqd mm3, mm1                    ; mm3 = 1s if sam_AB was zero
        por     mm2, mm3                    ; mm2 = 1s if either was zero
        pandn   mm2, mm6                    ; mask delta with zeros check
        pcmpeqd mm1, mm1
        psubd   mm1, mm7
        psubd   mm1, mm7
        psubd   mm1, mm0
        pxor    mm5, mm0
        paddd   mm5, mm1
        paddusw mm5, mm2                    ; and add to weight_AB
        psubd   mm5, mm1
        pxor    mm5, mm0
        paddd   mm4, mm5                    ; add weights to sum
        dec     ecx
        jnz     term_minus_3_loop

        mov     eax, [ebp+8]                ; access dpp
        movq    [eax+8], mm5                ; put weight_AB back
        movq    [eax+88], mm4               ; put sum_AB back
        emms

        add     edi, esi                    ; back up a full sample
        mov     edx, [edi+4]                ; dpp->samples_A [0] = iptr [-1];
        mov     eax, [ebp+8]
        mov     [eax+16], edx
        mov     edx, [edi]                  ; dpp->samples_B [0] = iptr [-2];
        mov     [eax+48], edx

done:   pop     edi
        pop     esi
        pop     ebx
        leave
        ret


; This is an assembly optimized version of the following WavPack function:
;
; uint32_t decorr_mono_buffer (int32_t *buffer,
;                              struct decorr_pass *decorr_passes,
;                              int32_t num_terms,
;                              int32_t sample_count)
;
; Decorrelate a buffer of mono samples, in place, as specified by the array
; of decorr_pass structures. Note that this function does NOT return the
; dpp->samples_X[] values in the "normalized" positions for terms 1-8, so if
; the number of samples is not a multiple of MAX_TERM, these must be moved if
; they are to be used somewhere else. The magnitude of the output samples is
; accumulated and returned (see scan_max_magnitude() for more details). By
; using the overflow detection of the multiply instruction, this detects
; when the "long_math" varient is required.
;
; For the fastest possible operation with the four "common" decorrelation
; filters (i.e, fast, normal, high and very high) this function can be
; configured to include hardcoded versions of these filters that are created
; using macros. In that case, the passed filter is checked to make sure that
; it matches one of the four. If it doesn't, or if the hardcoded flters are
; not enabled, a "general" version of the decorrelation loop is used. This
; variable enables the hardcoded filters and can be disabled if there are
; problems with the code or macros:

        HARDCODED_FILTERS = 1

; This is written to work on an IA-32 processor. The arguments are on the
; stack at these locations (after 6 pushes, we do not use ebp as a base
; pointer):
;
;   int32_t *buffer             [esp+28]
;   struct decorr_pass *dpp     [esp+32]
;   int32_t num_terms           [esp+36]
;   int32_t sample_count        [esp+40]
;
; register usage:
;
; ecx = sample being decorrelated
; esi = sample up counter
; edi = *buffer
; ebp = *dpp
;
; stack usage:
;
; [esp+0] = dpp end ptr (unused in hardcoded filter case)
; [esp+4] = magnitude accumulator
;
        if      HARDCODED_FILTERS
;
; This macro is used for checking the decorr_passes array to make sure that the terms match
; the hardcoded terms. The terms of these filters are the first element in the tables defined
; in decorr_tables.h (with the negative terms replaced with 1).
;

chkterm macro   term, ebp_offset
        cmp     BYTE PTR [ebp], term
        jnz     use_general_version
        add     ebp, ebp_offset
        endm

;
; This macro processes the single specified term (with a fixed delta of 2) and updates the
; term pointer (rbp) with the specified offset when done. It assumes the following registers:
;
; ecx = sample being decorrelated
; esi = sample up counter (used for terms 1-8)
; rbp = decorr_pass pointer for this term (updated with "rbp_offset" when done)
; rax, rbx, rdx = scratch
;

exeterm macro   term, ebp_offset
        local   over, cont, done

        if      term le 8
        mov     eax, esi
        and     eax, 7
        mov     ebx, [ebp+16+eax*4]
        if      term ne 8
        add     eax, term
        and     eax, 7
        endif
        mov     [ebp+16+eax*4], ecx

        elseif  term eq 17

        mov     edx, [ebp+16]               ; handle term 17
        mov     [ebp+16], ecx
        lea     ebx, [edx+edx]
        sub     ebx, [ebp+20]
        mov     [ebp+20], edx

        else

        mov     edx, [ebp+16]               ; handle term 18
        mov     [ebp+16], ecx
        lea     ebx, [edx+edx*2]
        sub     ebx, [ebp+20]
        sar     ebx, 1
        mov     [ebp+20], edx

        endif

        mov     eax, [ebp+8]
        imul    eax, ebx                    ; 32-bit multiply is almost always enough
        jo      over                        ; but handle overflow if it happens
        sar     eax, 10
        sbb     ecx, eax                    ; borrow flag provides rounding
        jmp     cont
over:   mov     eax, [ebp+8]                ; perform 64-bit multiply on overflow
        imul    ebx
        shr     eax, 10
        sbb     ecx, eax
        shl     edx, 22
        sub     ecx, edx
cont:   je      done
        test    ebx, ebx
        je      done
        xor     ebx, ecx
        sar     ebx, 30
        or      ebx, 1                      ; this generates delta of 1
        sal     ebx, 1                      ; this generates delta of 2
        add     [ebp+8], ebx
done:   add     ebp, ebp_offset

        endm

        endif                               ; end of macro definitions

; entry point of function

_pack_decorr_mono_buffer_x86:
        push    ebp                         ; save the resgister that we need to
        push    ebx
        push    esi
        push    edi
        xor     eax, eax
        push    eax                         ; this is magnitude accumulator
        push    eax                         ; this will be dpp end ptr

        mov     edi, [esp+28]               ; edi is buffer pointer
        xor     esi, esi                    ; up counter = 0

        cmp     DWORD PTR [esp+40], 0       ; test & handle zero sample count & zero term count
        jz      mexit
        cmp     DWORD PTR [esp+36], 0
        jz      mexit

        if      HARDCODED_FILTERS

; first check to make sure all the "deltas" are 2

        mov     ebp, [esp+32]               ; ebp is decorr_pass pointer
        mov     ebx, [esp+36]               ; get term count
deltas: cmp     BYTE PTR [ebp+4], 2         ; make sure all the deltas are 2
        jnz     use_general_version         ; if any aren't, use general case
        add     ebp, 96
        dec     ebx
        jnz     deltas

        mov     ebp, [esp+32]               ; ebp is decorr_pass pointer
        mov     edx, [esp+36]               ; get term count
        cmp     dl, 2                       ; 2 terms is "fast"
        jnz     nfast
        chkterm 18,  96                     ; check "fast" terms
        chkterm 17, -96
        jmp     mono_fast_loop

nfast:  cmp     dl, 5                       ; 5 terms is "normal"
        jnz     nnorm
        chkterm 18, 96                      ; check "normal" terms
        chkterm 18, 96
        chkterm 2,  96
        chkterm 17, 96
        chkterm 3,  96*-4
        jmp     mono_normal_loop

nnorm:  cmp     dl, 10                      ; 10 terms is "high"
        jnz     nhigh
        chkterm 18, 96                      ; check "high" terms
        chkterm 18, 96
        chkterm 18, 96
        chkterm 1,  96
        chkterm 2,  96
        chkterm 3,  96
        chkterm 5,  96
        chkterm 1,  96
        chkterm 17, 96
        chkterm 4,  96*-9
        jmp     mono_high_loop

nhigh:  cmp     dl, 16                      ; 16 terms is "very high"
        jnz     use_general_version         ; if none of these, use general version
        chkterm 18, 96                      ; else check "very high" terms
        chkterm 18, 96
        chkterm 2,  96
        chkterm 3,  96
        chkterm 1,  96
        chkterm 18, 96
        chkterm 2,  96
        chkterm 4,  96
        chkterm 7,  96
        chkterm 5,  96
        chkterm 3,  96
        chkterm 6,  96
        chkterm 8,  96
        chkterm 1,  96
        chkterm 18, 96
        chkterm 2,  96*-15
        jmp     mono_vhigh_loop

        align   64

mono_fast_loop:
        mov     ecx, [edi+esi*4]            ; ecx is the sample we're decorrelating

        exeterm 18,  96
        exeterm 17, -96

        mov     [edi+esi*4], ecx            ; store completed sample
        mov     eax, ecx                    ; magnitude accumulator |= (sample < 0) ? ~sample : sample
        cdq
        xor     eax, edx
        or      [esp+4], eax
        inc     esi                         ; increment sample index
        cmp     esi, [esp+40]
        jnz     mono_fast_loop              ; loop back for all samples
        jmp     mexit

        align   64

mono_normal_loop:
        mov     ecx, [edi+esi*4]            ; ecx is the sample we're decorrelating

        exeterm 18, 96
        exeterm 18, 96
        exeterm 2,  96
        exeterm 17, 96
        exeterm 3,  96*-4

        mov     [edi+esi*4], ecx            ; store completed sample
        mov     eax, ecx                    ; magnitude accumulator |= (sample < 0) ? ~sample : sample
        cdq
        xor     eax, edx
        or      [esp+4], eax
        inc     esi                         ; increment sample index
        cmp     esi, [esp+40]
        jnz     mono_normal_loop            ; loop back for all samples
        jmp     mexit

        align   64

mono_high_loop:
        mov     ecx, [edi+esi*4]             ; ecx is the sample we're decorrelating

        exeterm 18, 96
        exeterm 18, 96
        exeterm 18, 96
        exeterm 1,  96
        exeterm 2,  96
        exeterm 3,  96
        exeterm 5,  96
        exeterm 1,  96
        exeterm 17, 96
        exeterm 4,  96*-9

        mov     [edi+esi*4], ecx            ; store completed sample
        mov     eax, ecx                    ; magnitude accumulator |= (sample < 0) ? ~sample : sample
        cdq
        xor     eax, edx
        or      [esp+4], eax
        inc     esi                         ; increment sample index
        cmp     esi, [esp+40]
        jnz     mono_high_loop              ; loop back for all samples
        jmp     mexit

        align   64

mono_vhigh_loop:
        mov     ecx, [edi+esi*4]             ; ecx is the sample we're decorrelating

        exeterm 18, 96
        exeterm 18, 96
        exeterm 2,  96
        exeterm 3,  96
        exeterm 1,  96
        exeterm 18, 96
        exeterm 2,  96
        exeterm 4,  96
        exeterm 7,  96
        exeterm 5,  96
        exeterm 3,  96
        exeterm 6,  96
        exeterm 8,  96
        exeterm 1,  96
        exeterm 18, 96
        exeterm 2,  96*-15

        mov     [edi+esi*4], ecx            ; store completed sample
        mov     eax, ecx                    ; magnitude accumulator |= (sample < 0) ? ~sample : sample
        cdq
        xor     eax, edx
        or      [esp+4], eax
        inc     esi                         ; increment sample index
        cmp     esi, [esp+40]
        jnz     mono_vhigh_loop             ; loop back for all samples
        jmp     mexit

        endif                               ; end of HARDCODED_FILTERS

use_general_version:
        mov     ebp, [esp+32]
        mov     edx, [esp+36]               ; get number of terms
        imul    eax, edx, 96                ; calculate & store termination check ptr
        add     eax, [esp+32]
        mov     [esp], eax
        jmp     decorrelate_loop

        align   64

decorrelate_loop:
        mov     ecx, [edi+esi*4]             ; ecx is the sample we're decorrelating
nxterm: mov     edx, [ebp]
        cmp     dl, 17
        jge     @f

        mov     eax, esi
        and     eax, 7
        mov     ebx, [ebp+16+eax*4]
        add     eax, edx
        and     eax, 7
        mov     [ebp+16+eax*4], ecx
        jmp     domult

        align   4
@@:     mov     edx, [ebp+16]
        mov     [ebp+16], ecx
        je      @f
        lea     ebx, [edx+edx*2]
        sub     ebx, [ebp+20]
        sar     ebx, 1
        mov     [ebp+20], edx
        jmp     domult

        align   4
@@:     lea     ebx, [edx+edx]
        sub     ebx, [ebp+20]
        mov     [ebp+20], edx

domult: mov     eax, [ebp+8]
        mov     edx, eax
        imul    eax, ebx
        jo      multov                      ; on overflow, jump to use 64-bit imul varient
        sar     eax, 10
        sbb     ecx, eax
        je      @f
        test    ebx, ebx
        je      @f
        xor     ebx, ecx
        sar     ebx, 31
        xor     edx, ebx
        add     edx, [ebp+4]
        xor     edx, ebx
        mov     [ebp+8], edx
@@:     add     ebp, 96
        cmp     ebp, [esp]
        jnz     nxterm

        mov     [edi+esi*4], ecx            ; store completed sample
        mov     eax, ecx                    ; magnitude accumulator |= (sample < 0) ? ~sample : sample
        cdq
        xor     eax, edx
        or      [esp+4], eax
        mov     ebp, [esp+32]               ; reload decorr_passes pointer to first term
        inc     esi                         ; increment sample index
        cmp     esi, [esp+40]
        jnz     decorrelate_loop
        jmp     mexit

        align   4
multov: mov     eax, [ebp+8]
        imul    ebx
        shr     eax, 10
        sbb     ecx, eax
        shl     edx, 22
        sub     ecx, edx
        je      @f
        test    ebx, ebx
        je      @f
        xor     ebx, ecx
        sar     ebx, 31
        mov     eax, [ebp+8]
        xor     eax, ebx
        add     eax, [ebp+4]
        xor     eax, ebx
        mov     [ebp+8], eax
@@:     add     ebp, 96
        cmp     ebp, [esp]
        jnz     nxterm

        mov     [edi+esi*4], ecx            ; store completed sample
        mov     eax, ecx                    ; magnitude accumulator |= (sample < 0) ? ~sample : sample
        cdq
        xor     eax, edx
        or      [esp+4], eax
        mov     ebp, [esp+32]               ; reload decorr_passes pointer to first term
        inc     esi                         ; increment sample index
        cmp     esi, [esp+40]
        jnz     decorrelate_loop            ; loop all the way back this time

mexit:  pop     eax
        pop     eax                         ; pop magnitude accumulator
        pop     edi
        pop     esi
        pop     ebx
        pop     ebp
        ret


; This is an assembly optimized version of the following WavPack function:
;
; void decorr_mono_pass_cont (int32_t *out_buffer,
;                             int32_t *in_buffer,
;                             struct decorr_pass *dpp,
;                             int32_t sample_count);
;
; It performs a single pass of mono decorrelation, transfering from the
; input buffer to the output buffer. Note that this version of the function
; requires that the up to 8 previous (depending on dpp->term) mono samples
; are visible and correct. In other words, it ignores the "samples_*"
; fields in the decorr_pass structure and gets the history data directly
; from the source buffer. It does, however, return the appropriate history
; samples to the decorr_pass structure before returning.
;
; By using the overflow detection of the multiply instruction, it detects
; when the "long_math" varient is required and automatically does it.
;
; This is written to work on an IA-32 processor. The arguments on entry:
;
;   int32_t *out_buffer         [ebp+8]
;   int32_t *in_buffer          [ebp+12]
;   struct decorr_pass *dpp     [ebp+16]
;   int32_t sample_count        [ebp+20]
;
; Register / stack usage:
;
; esi = source ptr
; edi = destination ptr
; ecx = term * -4 (default terms)
; ecx = previous sample (terms 17 & 18)
; ebp = weight
; [esp] = delta
; [esp+4] = weight sum
; [esp+8] = eptr
;

_pack_decorr_mono_pass_cont_x86:
        push    ebp
        mov     ebp, esp
        push    ebx                         ; save the registers that we need to
        push    esi
        push    edi
        cld

        mov     esi, [ebp+12]
        mov     edi, [ebp+8]
        mov     edx, [ebp+16]               ; edx = *dpp
        mov     ecx, [ebp+20]               ; ecx = sample count
        mov     ebp, [edx+8]                ; ebp = weight
        lea     eax, [esi+ecx*4]            ; calc & push eptr (access with [esp+8])
        push    eax
        mov     eax, [edx+88]               ; push dpp->sum_A (access with [esp+4])
        push    eax
        mov     eax, [edx+4]                ; push delta (access with [esp])
        push    eax
        test    ecx, ecx                    ; test for and handle zero count
        jz      mono_done

        cld                                 ; we use lodsd/stosd
        mov     ecx, [esi-4]                ; preload last sample
        mov     eax, [edx]                  ; get term & branch for terms 17 & 18
        cmp     eax, 17
        je      mono_term_17_loop
        cmp     eax, 18
        je      mono_term_18_loop

        imul    ecx, eax, -4                ; ecx is index to correlation sample now
        jmp     mono_default_term_loop

        align  64

mono_default_term_loop:
        mov     edx, [esi+ecx]
        mov     ebx, edx
        imul    edx, ebp
        jo      over
        lodsd
        sar     edx, 10
        sbb     eax, edx
        jmp     @f
over:   mov     eax, ebx
        imul    ebp
        shl     edx, 22
        shr     eax, 10
        adc     edx, eax                    ; edx = apply_weight (sam_A)
        lodsd
        sub     eax, edx
@@:     stosd
        je      @f
        test    ebx, ebx
        je      @f
        xor     eax, ebx
        cdq
        xor     ebp, edx
        add     ebp, [esp]
        xor     ebp, edx
@@:     add     [esp+4], ebp
        cmp     esi, [esp+8]
        jnz     mono_default_term_loop

        mov     ecx, ebp                    ; ecx = weight
        mov     eax, [esp+4]                ; eax = weight sum
        lea     ebp, [esp+24]               ; restore ebp (we've pushed 6 DWORDS)
        mov     edx, [ebp+16]               ; edx = *dpp
        mov     [edx+8], ecx                ; put weight back
        mov     [edx+88], eax               ; put dpp->sum_A back
        mov     ecx, [edx]                  ; ecx = dpp->term

mono_default_store_samples:
        dec     ecx
        sub     esi, 4                      ; back up one sample
        mov     eax, [esi]
        mov     [edx+ecx*4+16], eax         ; store samples_A [ecx]
        test    ecx, ecx
        jnz     mono_default_store_samples
        jmp     mono_done

        align  64

mono_term_17_loop:
        lea     edx, [ecx+ecx]
        sub     edx, [esi-8]                ; ebx = sam_A
        mov     ebx, edx
        imul    edx, ebp
        jo      over17
        sar     edx, 10
        lodsd
        mov     ecx, eax
        sbb     eax, edx
        jmp     @f
over17: mov     eax, ebx
        imul    ebp
        shl     edx, 22
        shr     eax, 10
        adc     edx, eax                    ; edx = apply_weight (sam_A)
        lodsd
        mov     ecx, eax
        sub     eax, edx
@@:     stosd
        je      @f
        test    ebx, ebx
        je      @f
        xor     eax, ebx
        cdq
        xor     ebp, edx
        add     ebp, [esp]
        xor     ebp, edx
@@:     add     [esp+4], ebp
        cmp     esi, [esp+8]
        jnz     mono_term_17_loop
        jmp     mono_term_1718_exit

        align  64

mono_term_18_loop:
        lea     edx, [ecx+ecx*2]
        sub     edx, [esi-8]
        sar     edx, 1
        mov     ebx, edx                    ; ebx = sam_A
        imul    edx, ebp
        jo      over18
        sar     edx, 10
        lodsd
        mov     ecx, eax
        sbb     eax, edx
        jmp     @f
over18: mov     eax, ebx
        imul    ebp
        shl     edx, 22
        shr     eax, 10
        adc     edx, eax                    ; edx = apply_weight (sam_A)
        lodsd
        mov     ecx, eax
        sub     eax, edx
@@:     stosd
        je      @f
        test    ebx, ebx
        je      @f
        xor     eax, ebx
        cdq
        xor     ebp, edx
        add     ebp, [esp]
        xor     ebp, edx
@@:     add     [esp+4], ebp
        cmp     esi, [esp+8]
        jnz     mono_term_18_loop

mono_term_1718_exit:
        mov     ecx, ebp                    ; ecx = weight
        mov     eax, [esp+4]                ; eax = weight sum
        lea     ebp, [esp+24]               ; restore ebp (we've pushed 6 DWORDS)
        mov     edx, [ebp+16]               ; edx = *dpp
        mov     [edx+8], ecx                ; put weight back
        mov     [edx+88], eax               ; put dpp->sum_A back
        mov     eax, [esi-4]                ; dpp->samples_A [0] = bptr [-1]
        mov     [edx+16], eax
        mov     eax, [esi-8]                ; dpp->samples_A [1] = bptr [-2]
        mov     [edx+20], eax

mono_done:
        add     esp, 12                     ; deallocate stack space
        pop     edi                         ; pop saved registers & return
        pop     esi
        pop     ebx
        pop     ebp
        ret


; This is an assembly optimized version of the following WavPack function:
;
; uint32_t scan_max_magnitude (int32_t *buffer, int32_t sample_count);
;
; This function scans a buffer of signed 32-bit ints and returns the magnitude
; of the largest sample, with a power-of-two resolution. It might be more
; useful to return the actual maximum absolute value, but that implementation
; would be slower. Instead, this simply returns the "or" of all the values
; "xor"d with their own sign, like so:
;
;     while (sample_count--)
;         magnitude |= (*buffer < 0) ? ~*buffer++ : *buffer++;
;
; This is written to work on an IA-32 processor and uses the MMX extensions
; to improve the performance by processing two samples together. The arguments
; are on the stack at these locations (after 4 pushes, we do not use ebp as a
; base pointer):
;
;   int32_t *buffer             [esp+20]
;   uint32_t sample_count       [esp+24]
;
; During the processing loops, the following registers are used:
;
;   edi         buffer pointer
;   esi         termination buffer pointer
;   ebx         single magnitude accumulator
;   mm0         dual magnitude accumulator
;   mm1, mm2    scratch
;

_scan_max_magnitude_x86:
        push    ebp
        push    ebx
        push    esi
        push    edi

        xor     ebx, ebx                    ; clear magnitude accumulator
        mov     edi, [esp+20]               ; edi = buffer pointer

        mov     eax, [esp+24]               ; eax = count
        and     eax, 7
        mov     ecx, eax                    ; ecx = leftover samples to "manually" scan at end

        mov     eax, [esp+24]               ; eax = count
        shr     eax, 3                      ; eax = num of loops to process mmx (8 samples/loop)
        shl     eax, 5                      ; eax = num of bytes to process mmx (32 bytes/loop)
        jz      nommx                       ; jump around if no mmx loops to do (< 8 samples)

        pxor    mm0, mm0                    ; clear dual magnitude accumulator
        add     eax, edi                    ; esi = termination buffer pointer for mmx loop
        mov     esi, eax
        jmp     mmxlp

        align  64

mmxlp:  movq    mm1, [edi]                  ; get stereo samples in mm1 & mm2
        movq    mm2, mm1
        psrad   mm1, 31                     ; mm1 = sign (mm2)
        pxor    mm1, mm2                    ; mm1 = absolute magnitude, or into result
        por     mm0, mm1

        movq    mm1, [edi+8]                ; do it again with 6 more samples
        movq    mm2, mm1
        psrad   mm1, 31
        pxor    mm1, mm2
        por     mm0, mm1

        movq    mm1, [edi+16]
        movq    mm2, mm1
        psrad   mm1, 31
        pxor    mm1, mm2
        por     mm0, mm1

        movq    mm1, [edi+24]
        movq    mm2, mm1
        psrad   mm1, 31
        pxor    mm1, mm2
        por     mm0, mm1

        add     edi, 32
        cmp     edi, esi
        jnz     mmxlp

        movd    eax, mm0                    ; ebx = "or" of high and low mm0
        punpckhdq mm0, mm0
        movd    ebx, mm0
        or      ebx, eax
        emms

nommx:  and     ecx, ecx                    ; any leftover samples to do?
        jz      noleft

leftlp: mov     eax, [edi]
        cdq
        xor     eax, edx
        or      ebx, eax
        add     edi, 4
        loop    leftlp

noleft: mov     eax, ebx                    ; move magnitude to eax for return
        pop     edi
        pop     esi
        pop     ebx
        pop     ebp
        ret


; This is an assembly optimized version of the following WavPack function:
;
; uint32_t log2buffer (int32_t *samples, uint32_t num_samples, int limit);
;
; This function scans a buffer of 32-bit ints and accumulates the total
; log2 value of all the samples. This is useful for determining maximum
; compression because the bitstream storage required for entropy coding
; is proportional to the base 2 log of the samples.
;
; This is written to work on an IA-32 processor. The arguments are on the
; stack at these locations (after 4 pushes, we do not use ebp as a base
; pointer):
;
;   int32_t *samples            [esp+20]
;   uint32_t num_samples        [esp+24]
;   int limit                   [esp+28]
;
; During the processing loops, the following registers are used:
;
;   esi             input buffer pointer
;   edi             sum accumulator
;   ebx             sample count
;   ebp             log2_table pointer
;   eax,ecx,edx     scratch
;

        align  256
        .radix 16

log2_table:
        byte   000, 001, 003, 004, 006, 007, 009, 00a, 00b, 00d, 00e, 010, 011, 012, 014, 015
        byte   016, 018, 019, 01a, 01c, 01d, 01e, 020, 021, 022, 024, 025, 026, 028, 029, 02a
        byte   02c, 02d, 02e, 02f, 031, 032, 033, 034, 036, 037, 038, 039, 03b, 03c, 03d, 03e
        byte   03f, 041, 042, 043, 044, 045, 047, 048, 049, 04a, 04b, 04d, 04e, 04f, 050, 051
        byte   052, 054, 055, 056, 057, 058, 059, 05a, 05c, 05d, 05e, 05f, 060, 061, 062, 063
        byte   064, 066, 067, 068, 069, 06a, 06b, 06c, 06d, 06e, 06f, 070, 071, 072, 074, 075
        byte   076, 077, 078, 079, 07a, 07b, 07c, 07d, 07e, 07f, 080, 081, 082, 083, 084, 085
        byte   086, 087, 088, 089, 08a, 08b, 08c, 08d, 08e, 08f, 090, 091, 092, 093, 094, 095
        byte   096, 097, 098, 099, 09a, 09b, 09b, 09c, 09d, 09e, 09f, 0a0, 0a1, 0a2, 0a3, 0a4
        byte   0a5, 0a6, 0a7, 0a8, 0a9, 0a9, 0aa, 0ab, 0ac, 0ad, 0ae, 0af, 0b0, 0b1, 0b2, 0b2
        byte   0b3, 0b4, 0b5, 0b6, 0b7, 0b8, 0b9, 0b9, 0ba, 0bb, 0bc, 0bd, 0be, 0bf, 0c0, 0c0
        byte   0c1, 0c2, 0c3, 0c4, 0c5, 0c6, 0c6, 0c7, 0c8, 0c9, 0ca, 0cb, 0cb, 0cc, 0cd, 0ce
        byte   0cf, 0d0, 0d0, 0d1, 0d2, 0d3, 0d4, 0d4, 0d5, 0d6, 0d7, 0d8, 0d8, 0d9, 0da, 0db
        byte   0dc, 0dc, 0dd, 0de, 0df, 0e0, 0e0, 0e1, 0e2, 0e3, 0e4, 0e4, 0e5, 0e6, 0e7, 0e7
        byte   0e8, 0e9, 0ea, 0ea, 0eb, 0ec, 0ed, 0ee, 0ee, 0ef, 0f0, 0f1, 0f1, 0f2, 0f3, 0f4
        byte   0f4, 0f5, 0f6, 0f7, 0f7, 0f8, 0f9, 0f9, 0fa, 0fb, 0fc, 0fc, 0fd, 0fe, 0ff, 0ff

        .radix  10

_log2buffer_x86:
        push    ebp
        push    ebx
        push    esi
        push    edi
        cld

        mov     esi, [esp+20]               ; esi = sample source pointer
        xor     edi, edi                    ; edi = 0 (accumulator)
        mov     ebx, [esp+24]               ; ebx = num_samples
        test    ebx, ebx                    ; exit now if none, sum = 0
        jz      normal_exit

; These three instructions allow this to be PIC (position independent code). The purpose is to
; load the address of the log2_table into ebp regardless of where this is all loaded in memory.

        call    nexti                       ; push address of nexti (return address)
nexti:  pop     ebp                         ; pop address of nexti into ebp
        sub     ebp, nexti - log2_table     ; offset to log2_table

        mov     eax, [esp+28]               ; eax = limit
        test    eax, eax                    ; we have separate loops for limit and no limit
        jz      no_limit_loop
        jmp     limit_loop

        align  64

limit_loop:
        mov     eax, [esi]                  ; get next sample into eax
        cdq                                 ; edx = sign of sample (for abs)
        add     esi, 4
        xor     eax, edx
        sub     eax, edx
        je      L40                         ; skip if sample was zero
        mov     edx, eax                    ; move to edx and apply rounding
        shr     eax, 9
        add     edx, eax
        bsr     ecx, edx                    ; ecx = MSB set in sample (0 - 31)
        lea     eax, [ecx+1]                ; eax = number used bits in sample (1 - 32)
        sub     ecx, 8                      ; ecx = shift right amount (-8 to 23)
        ror     edx, cl                     ; use rotate to do "signed" shift 
        sal     eax, 8                      ; move nbits to integer portion of log
        movzx   edx, dl                     ; dl = mantissa, look up log fraction in table 
        mov     al, BYTE PTR [ebp+edx]      ; eax = combined integer and fraction for full log
        add     edi, eax                    ; add to running sum and compare to limit
        cmp     eax, [esp+28]
        jge     limit_exceeded
L40:    sub     ebx, 1                      ; loop back if more samples
        jne     limit_loop
        jmp     normal_exit

        align  64

no_limit_loop:
        mov     eax, [esi]                  ; get next sample into eax
        cdq                                 ; edx = sign of sample (for abs)
        add     esi, 4
        xor     eax, edx
        sub     eax, edx
        je      L45                         ; skip if sample was zero
        mov     edx, eax                    ; move to edx and apply rounding
        shr     eax, 9
        add     edx, eax
        bsr     ecx, edx                    ; ecx = MSB set in sample (0 - 31)
        lea     eax, [ecx+1]                ; eax = number used bits in sample (1 - 32)
        sub     ecx, 8                      ; ecx = shift right amount (-8 to 23)
        ror     edx, cl                     ; use rotate to do "signed" shift 
        sal     eax, 8                      ; move nbits to integer portion of log
        movzx   edx, dl                     ; dl = mantissa, look up log fraction in table 
        mov     al, BYTE PTR [ebp+edx]      ; eax = combined integer and fraction for full log
        add     edi, eax                    ; add to running sum
L45:    sub     ebx, 1                      ; loop back if more samples
        jne     no_limit_loop
        jmp     normal_exit

limit_exceeded:
        mov     edi, -1                     ; -1 return means log limit exceeded
normal_exit:
        mov     eax, edi                    ; move sum accumulator into eax for return
        pop     edi
        pop     esi
        pop     ebx
        pop     ebp
        ret

; Helper function to determine if specified CPU feature is available (used here for MMX).
; Input parameter is index of feature to be checked (EDX from CPUID(1) only, MMX = 23).
; Return value is the specified bit (0 or 1) or 0 if CPUID is not supported.

_pack_cpu_has_feature_x86:
        pushfd                              ; save eflags
        pushfd                              ; push another copy
        xor     dword ptr [esp], 200000h    ; toggle ID bit on stack & pop it back into eflags
        popfd
        pushfd                              ; store possibly modified eflags
        pop     eax                         ; and pop back into eax
        xor     eax, [esp]                  ; compare to original pushed eflags
        popfd                               ; restore original eflags
        and     eax, 200000h                ; eax = 1 if eflags ID bit was changable
        jz      oldcpu                      ; return zero if CPUID is not available (wow!)

        push    ebx                         ; we must save ebx
        mov     eax, 1                      ; do cpuid (1) to get features into edx
        cpuid
        mov     eax, edx                    ; copy into eax for shift
        mov     cl, [esp+8]                 ; get parameter and shift that bit index into LSB
        sar     eax, cl
        and     eax, 1
        pop     ebx                         ; restore ebx and return 0 or 1

oldcpu: ret                                 ; return value in eax

asmcode ends

        end

