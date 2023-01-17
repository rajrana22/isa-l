;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Copyright(c) 2011-2015 Intel Corporation All rights reserved.
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;    * Redistributions of source code must retain the above copyright
;      notice, this list of conditions and the following disclaimer.
;    * Redistributions in binary form must reproduce the above copyright
;      notice, this list of conditions and the following disclaimer in
;      the documentation and/or other materials provided with the
;      distribution.
;    * Neither the name of Intel Corporation nor the names of its
;      contributors may be used to endorse or promote products derived
;      from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;
;;; gf_2vect_dot_prod_avx512(len_n, vec_n, *g_tbls_n, **buffs_n, **dests_n, len_l, vec_l, *g_tbls_l, **buffs_l, **dests_l);
;;;

%include "reg_sizes.asm"

%ifdef HAVE_AS_KNOWS_AVX512

; Linux/Unix
%ifidn __OUTPUT_FORMAT__, elf64
 %define arg0 rdi
 %define arg1 rsi
 %define arg2 rdx
 %define arg3 rcx
 %define arg4 r8
 %define arg5 r9
 %define arg6 r10 ; New registers used to store function args
 %define arg7 r11
 %define arg8 r12
 %define arg9 r13

 %define tmp r15
 %define tmp2 r14
 %define tmp3 rbx ; must be saved and restored
 %define PS     8
 %define LOG_PS 3

; Save and restore more registers
 %define func(x) x: endbranch
 %macro FUNC_SAVE 0
	push	rbx
    push r12
    push r13
    push r14
    push r15
 %endmacro
 %macro FUNC_RESTORE 0
    pop r15
    pop r14
    pop r13
    pop r12
	pop	rbx
 %endmacro
%endif

; Windows
%ifidn __OUTPUT_FORMAT__, win64
 %define arg0   rcx
 %define arg1   rdx
 %define arg2   r8
 %define arg3   r9

 %define arg4   r12 		; must be saved, loaded and restored
 %define arg5   r15 		; must be saved and restored
 %define arg6 r13           ; must be saved and restored
 %define arg7 r14           ; must be saved and restored
 %define tmp    r11
 %define tmp2   r10
 %define tmp3   r13		; must be saved and restored
 %define return rax
 %define PS     8
 %define LOG_PS 3
 %define stack_size  9*16 + 5*8		; must be an odd multiple of 8
 %define arg(x)      [rsp + stack_size + PS + PS*x]

 %define func(x) proc_frame x
 %macro FUNC_SAVE 0
	alloc_stack	stack_size
	vmovdqa	[rsp + 0*16], xmm6
	vmovdqa	[rsp + 1*16], xmm7
	vmovdqa	[rsp + 2*16], xmm8
	vmovdqa	[rsp + 3*16], xmm9
	vmovdqa	[rsp + 4*16], xmm10
	vmovdqa	[rsp + 5*16], xmm11
	vmovdqa	[rsp + 6*16], xmm12
	vmovdqa	[rsp + 7*16], xmm13
	vmovdqa	[rsp + 8*16], xmm14
	save_reg	r12,  9*16 + 0*8
	save_reg	r13,  9*16 + 1*8
	save_reg	r14,  9*16 + 2*8
	save_reg	r15,  9*16 + 3*8
	end_prolog
	mov arg4, arg(4)
    mov arg5, arg(5)
    mov arg6, arg(6)
    mov arg7, arg(7)
 %endmacro

 %macro FUNC_RESTORE 0
	vmovdqa	xmm6, [rsp + 0*16]
	vmovdqa	xmm7, [rsp + 1*16]
	vmovdqa	xmm8, [rsp + 2*16]
	vmovdqa	xmm9, [rsp + 3*16]
	vmovdqa	xmm10, [rsp + 4*16]
	vmovdqa	xmm11, [rsp + 5*16]
	vmovdqa	xmm12, [rsp + 6*16]
	vmovdqa	xmm13, [rsp + 7*16]
	vmovdqa	xmm14, [rsp + 8*16]
	mov	r12,  [rsp + 9*16 + 0*8]
	mov	r13,  [rsp + 9*16 + 1*8]
	mov	r14,  [rsp + 9*16 + 2*8]
	mov	r15,  [rsp + 9*16 + 3*8]
	add	rsp, stack_size
    mov arg5, r15
    mov arg4, r12
 %endmacro
%endif

; Include both network and local portions of MLEC
%define len_n       arg0 ; network chunksize
%define vec_n       arg1 ; network data chunks k
%define g_tbls_n    arg2 ; network g_tbls
%define buffs_n     arg3 ; network data buffer
%define dests_n     arg4 ; network parity buffer
%define len_l       arg5 ; local chunksize
%define vec_l       arg6 ; local data chunks k
%define g_tbls_l    arg7 ; local g_tbls
%define buffs_l     arg8 ; local data buffer
%define dests_l     arg9 ; local parity buffer
%define ptr         tmp2
%define vec_i       tmp3
%define pos         return ; pos is the return value


%ifndef EC_ALIGNED_ADDR
;;; Use Un-aligned load/store
 %define XLDR vmovdqu8
 %define XSTR vmovdqu8
%else
;;; Use Non-temporal load/stor
 %ifdef NO_NT_LDST
  %define XLDR vmovdqa
  %define XSTR vmovdqa
 %else
  %define XLDR vmovntdqa
  %define XSTR vmovntdq
 %endif
%endif

; Define xmm and ymm registers for use in storing intermediate results
%define xmask0f   zmm8
%define xgft1_lo  zmm7
%define xgft1_loy ymm7
%define xgft1_hi  zmm6
%define xgft2_lo  zmm5
%define xgft2_loy ymm5
%define xgft2_hi  zmm4

%define x0        zmm0
%define xtmpa     zmm1
%define xp1       zmm2
%define xp2       zmm3

; Assembler directives
default rel
[bits 64]

section .text

align 16 ; Align instruction to 16 bytes
mk_global gf_2vect_dot_prod_avx512, function ; Make function visible to the linker

; Begin function
func(gf_2vect_dot_prod_avx512)
	FUNC_SAVE ; Save function to stack

    ; ec_highlevel_func.c requirement
	sub	len_n, 64 ; Subtract 64 from chunk size
	jl	.return_fail ; Jump to return_fail if chunk size < 64
    sub	len_l, 64 ; Subtract 64 from chunk size
	jl	.return_fail ; Jump to return_fail if chunk size < 64

	xor	pos_n, pos_n ; pos = 0
    xor	pos_l, pos_l ; pos = 0
	mov	tmp_n, 0x0f ; move 15 to tmp
    mov	tmp_l, 0x0f ; move 15 to tmp
	vpbroadcastb xmask0f_n, tmp_n	;Construct mask 0x0f0f0f...
    vpbroadcastb xmask0f_l, tmp_l	;Construct mask 0x0f0f0f...
	sal	vec_n, LOG_PS		;vec *= PS. Make vec_i count by PS
    sal	vec_l, LOG_PS		;vec *= PS. Make vec_i count by PS
	mov	dest2_n, [dest1_n+PS] ; move value at dest1 + PS to dest2
    mov	dest2_l, [dest1_l+PS] ; move value at dest1 + PS to dest2
	mov	dest1_n, [dest1_n] ; NOP (align instruction)
    mov	dest1_l, [dest1_l] ; NOP (align instruction)

; pos moves on 64 bytes at a time (this is the slice size)
; vec_i moves on 8 bytes at a time (size of each data chunk)

; Begin outer loop
.loop64:
    ; Resets for next inner loop.
	vpxorq	xp1_n, xp1_n, xp1_n ; Accumulator xp1 = 0
    vpxorq	xp1_l, xp1_l, xp1_l ; Accumulator xp1 = 0
	vpxorq	xp2_n, xp2_n, xp2_n ; Accumulator xp2 = 0
    vpxorq	xp2_l, xp2_l, xp2_l ; Accumulator xp2 = 0
	mov	tmp_n, mul_array_n ; Moves the mul_array into tmp
    mov	tmp_l, mul_array_l ; Moves the mul_array into tmp
	xor	vec_i_n, vec_i_n ; vec_i = 0
    xor	vec_i_l, vec_i_l ; vec_i = 0

; Begin inner loop
.next_vect:
    ; Goes through each coefficient and source to multiply and accumulate.
    mov    ptr_n, [src_n+vec_i_n] ; Ptr_n becomes data buffer + vec_i_n
    XLDR   x0_n, [ptr_n+pos_n] ; Get next source vector_n
    add    vec_i_n, PS ; Add 8 to vec_i_n
    mov    ptr_l, [src_l+vec_i_l] ; Ptr_l becomes data buffer + vec_i_l
    XLDR   x0_l, [ptr_l+pos_l] ; Get next source vector_l
    add    vec_i_l, PS ; Add 8 to vec_i_l

    ; Begin erasure computation for N layer
    vpandq   xtmpa_n, x0_n, xmask0f ; Mask low src nibble in bits 4-0
    vpsraw   x0_n, x0_n, 4 ; Shift to put high nibble into bits 4-0
    vpandq   x0_n, x0_n, xmask0f ; Mask high src nibble in bits 4-0

    vmovdqu8 xgft1_loy_n, [tmp_n] ; Load array Ax{00}..{0f}, Ax{00}..{f0}
    vmovdqu8 xgft2_loy_n, [tmp_n+vec_n*(32/PS)] ; Load array Bx{00}..{0f}, Bx{00}..{f0}
    add     tmp_n, 32

    vshufi64x2 xgft1_hi_n, xgft1_lo_n, xgft1_lo_n, 0x55
    vshufi64x2 xgft1_lo_n, xgft1_lo_n, xgft1_lo_n, 0x00
    vshufi64x2 xgft2_hi_n, xgft2_lo_n, xgft2_lo_n, 0x55
    vshufi64x2 xgft2_lo_n, xgft2_lo_n, xgft2_lo_n, 0x00

    vpshufb xgft2_hi_n, xgft2_hi_n, x0_n ; Lookup mul table of high nibble
	vpshufb xgft2_lo_n, xgft2_lo_n, xtmpa_n ; Lookup mul table of low nibble
	vpxorq xgft2_hi_n, xgft2_hi_n, xgft2_lo_n ; GF add high and low partials
	vpxorq xp2_n, xp2_n, xgft2_hi_n ; xp2_n += partial

	cmp vec_i_n, vec
	jl .next_vect_n

	add pos, 64
	cmp pos, len
	jl .loop64
	
	; Accumulate xp1 and xp2 into dest1 and dest2
	vpxorq	[dest1], xp1_n, [dest1]
	vpxorq	[dest2], xp2_n, [dest2]
	
	FUNC_RESTORE
	ret

.return_fail:
	xor	eax, eax
	FUNC_RESTORE
	ret

endproc_frame

%else
%ifidn __OUTPUT_FORMAT__, win64
global no_gf_2vect_dot_prod_avx512
no_gf_2vect_dot_prod_avx512:
%endif
%endif  ; ifdef HAVE_AS_KNOWS_AVX512
