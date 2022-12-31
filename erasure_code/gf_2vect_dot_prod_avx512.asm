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
;;; gf_2vect_dot_prod_avx512(len, vec, *g_tbls, **buffs, **dests);
;;;

%include "reg_sizes.asm"

%ifdef HAVE_AS_KNOWS_AVX512

; Linux/Unix
%ifidn __OUTPUT_FORMAT__, elf64
 %define arg0  rdi
 %define arg1  rsi
 %define arg2  rdx
 %define arg3  rcx
 %define arg4  r8
 %define arg5  r9

 %define tmp   r11
 %define tmp2  r10
 %define tmp3  r12		; must be saved and restored
 %define return rax
 %define PS     8
 %define LOG_PS 3

 %define func(x) x: endbranch
 %macro FUNC_SAVE 0
	push	r12
 %endmacro
 %macro FUNC_RESTORE 0
	pop	r12
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
	mov	arg4, arg(4)
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
 %endmacro
%endif


%define len    arg0 ; chunksize
%define vec    arg1 ; data chunks k
%define mul_array arg2 ; g_tbls
%define src    arg3 ; data buffer
%define dest1  arg4 ; parity buffer
%define ptr    arg5 
%define vec_i  tmp2
%define dest2  tmp3
%define pos    return ; pos is the return value


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

default rel
[bits 64]

section .text

align 16 ; Align instruction to 16 bytes
mk_global gf_2vect_dot_prod_avx512, function ; Make function visible to the linker

; Begin function
func(gf_2vect_dot_prod_avx512)
	FUNC_SAVE ; Save function to stack

    ; ec_highlevel_func.c requirement
	sub	len, 64 ; Subtract 64 from chunk size
	jl	.return_fail ; Jump to return_fail if chunk size < 64

	xor	pos, pos ; pos = 0
	mov	tmp, 0x0f ; move 15 to tmp
	vpbroadcastb xmask0f, tmp	;Construct mask 0x0f0f0f...
	sal	vec, LOG_PS		;vec *= PS. Make vec_i count by PS
	mov	dest2, [dest1+PS] ; move value at dest1 + PS to dest2
	mov	dest1, [dest1] ; NOP (align instruction)

; pos moves on 64 bytes at a time (this is the slice size)
; vec_i moves on 8 bytes at a time (size of each data chunk)

; Begin outer loop
.loop64:
    ; Resets for next inner loop.
	vpxorq	xp1, xp1, xp1 ; Accumulator xp1 = 0
	vpxorq	xp2, xp2, xp2 ; Accumulator xp2 = 0
	mov	tmp, mul_array ; Moves the mul_array into tmp
	xor	vec_i, vec_i ; vec_i = 0

; Begin inner loop
.next_vect:
    ; Goes through each coefficient and source to multiply and accumulate.
	mov	ptr, [src+vec_i] ; Ptr becomes data buffer + vec_i
	XLDR	x0, [ptr+pos] ; Get next source vector
	add	vec_i, PS ; Add 8 to vec_i

    ; Begin erasure computation
	vpandq	xtmpa, x0, xmask0f ; Mask low src nibble in bits 4-0
	vpsraw	x0, x0, 4 ; Shift to put high nibble into bits 4-0
	vpandq	x0, x0, xmask0f ; Mask high src nibble in bits 4-0

	vmovdqu8 xgft1_loy, [tmp] ; Load array Ax{00}..{0f}, Ax{00}..{f0}
	vmovdqu8 xgft2_loy, [tmp+vec*(32/PS)] ; Load array Bx{00}..{0f}, Bx{00}..{f0}
	add	tmp, 32

	vshufi64x2 xgft1_hi, xgft1_lo, xgft1_lo, 0x55
	vshufi64x2 xgft1_lo, xgft1_lo, xgft1_lo, 0x00
	vshufi64x2 xgft2_hi, xgft2_lo, xgft2_lo, 0x55
	vshufi64x2 xgft2_lo, xgft2_lo, xgft2_lo, 0x00

	vpshufb	xgft1_hi, xgft1_hi, x0 ; Lookup mul table of high nibble
	vpshufb	xgft1_lo, xgft1_lo, xtmpa ; Lookup mul table of low nibble
	vpxorq	xgft1_hi, xgft1_hi, xgft1_lo ; GF add high and low partials
	vpxorq	xp1, xp1, xgft1_hi ; xp1 += partial

	vpshufb	xgft2_hi, xgft2_hi, x0 ; Lookup mul table of high nibble
	vpshufb	xgft2_lo, xgft2_lo, xtmpa ; Lookup mul table of low nibble
	vpxorq	xgft2_hi, xgft2_hi, xgft2_lo ; GF add high and low partials
	vpxorq	xp2, xp2, xgft2_hi ; xp2 += partial
    ; End erasure computation

    ; Loop through all k data chunks (vec = k)
	cmp	vec_i, vec
	jl	.next_vect ; Keep looping through while vec_i < vec
    ; End inner loop.

    ; Write out parities to dest buffers.
	XSTR	[dest1+pos], xp1 ; Move value from accumulator xp1 to dest1+pos
	XSTR	[dest2+pos], xp2 ; Move value from accumulator xp2 to dest2+pos

	add	pos, 64	; Loop on 64 bytes at a time
	cmp	pos, len ; Compare pos to chunk size
	jle	.loop64 ; Jump to loop64 if pos <= chunk size
    ; End outer loop

	lea	tmp, [len + 64] ; Load address of len + 64 into temp
	cmp	pos, tmp
	je	.return_pass ; Jump to return pass if pos = address of len + 64

	;; Tail len
	mov	pos, len ; Overlapped offset length-64
	jmp	.loop64 ; Do one more overlap pass

; Return successfully
.return_pass:
	mov	return, 0
	FUNC_RESTORE
	ret

; Return with error code
.return_fail:
	mov	return, 1
	FUNC_RESTORE
	ret

endproc_frame

%else
%ifidn __OUTPUT_FORMAT__, win64
global no_gf_2vect_dot_prod_avx512
no_gf_2vect_dot_prod_avx512:
%endif
%endif  ; ifdef HAVE_AS_KNOWS_AVX512
