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
;;; gf_2vect_dot_prod_avx512(len_n, k_n, *g_tbls, **buffs_n, **dests_n, len_l, k_l, **buffs_l, **dests_l);
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
 %define arg6 r10
 %define arg7 r11
 %define arg8 r12

 %define tmp r15
 %define tmp2 r14
 %define tmp3 rbx ; must be saved and restored
 %define tmp4 r13
 %define tmp5 r12
 %define tmp6 r11
 %define tmp7 r10
 %define return rax
 %define PS     8
 %define LOG_PS 3

; Save and restore function arguments that won't fit into registers on the stack
 %define func(x) x: endbranch
 %macro FUNC_SAVE 0
	push rbx
    push arg6
    push arg7
    push arg8
 %endmacro
 %macro FUNC_RESTORE 0
    pop arg8
    pop arg7
    pop arg6
	pop	rbx
 %endmacro
%endif

; Begin ignore
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
; End ignore

; Include both network and local portions of MLEC
%define len_n       arg0 ; network chunksize
%define k_n         arg1 ; network data chunks k_n
%define g_tbls      arg2 ; g_tbls
%define buffs_n     arg3 ; network data buffer
%define dest_n      arg4 ; network parity buffer
%define len_l       arg5 ; local chunksize
%define k_l         arg6 ; local data chunks k_l
%define buffs_l     arg7 ; local data buffer
%define dest_l      arg8 ; local parity buffer
%define ptr         tmp3 ; pointer to next chunk
%define n_i         tmp4 ; network counter
%define l_i         tmp5 ; local counter
%define dest2_n     tmp6 ; network offset parity buffer
%define dest2_l     tmp7 ; local offset parity buffer
%define pos         return ; pos is the return value

;;; gf_2vect_dot_prod_avx512(len_n, k_n, *g_tbls, **buffs_n, **dests_n, len_l, k_l, **buffs_l, **dests_l);


%ifndef EC_ALIGNED_ADDR
;;; Use Un-aligned load/store
 %define XLDR vmovdqu8
 %define XSTR vmovdqu8
%else
;;; Use Non-temporal load/store
 %ifdef NO_NT_LDST
  %define XLDR vmovdqa
  %define XSTR vmovdqa
 %else
  %define XLDR vmovntdqa
  %define XSTR vmovntdq
 %endif
%endif

; Define xmm and ymm registers for use in storing intermediate results

; Reusable for network and local computation
%define xmask0f   zmm10
%define xgft1_lo  zmm9
%define xgft1_loy ymm7
%define xgft1_hi  zmm8
%define xgft2_lo  zmm7
%define xgft2_loy ymm5
%define xgft2_hi  zmm6
%define x0        zmm0
%define xtmpa     zmm1

; Accumulators not reusable: assign network and local separately
%define xp1_net   zmm2
%define xp2_net   zmm3
%define xp1_loc   zmm4
%define xp2_loc   zmm5

; Assembler directives
default rel
[bits 64]

section .text

align 16 ; Align instruction to 16 bytes
mk_global gf_2vect_dot_prod_avx512, function ; Make function visible to the linker

; Begin function
func(gf_2vect_dot_prod_avx512)
    FUNC_SAVE ; Save function to stack

	xor	pos, pos ; pos = 0

    ; This is for erasure computation
	mov	tmp, 0x0f ; move 15 to tmp
	vpbroadcastb xmask0f, tmp	;Construct mask 0x0f0f0f...

    ; Set up network looping and data loading conditions
	sal	k_n, LOG_PS		; k_n *= PS. Make n_i count by PS (8)
	mov	dest2_n, [dest_n + PS] ; move value at dest_n + PS to dest2_n
	mov	dest_n, [dest_n] ; NOP (align instruction)

    ; Set up local looping and data loading conditions
    sal k_l, LOG_PS      ; k_l *= PS. Make l_i count by PS (8)
	mov	dest2_l, [dest_l + PS] ; move value at dest_l + PS to dest2_l
	mov	dest_l, [dest_l] ; NOP (align instruction)

    ; Begin outer loop for network chunks
    .loop64:
        ; Reset network temporary variables
        vpxorq	xp1_net, xp1_net, xp1_net ; Accumulator xp1_net = 0
        vpxorq	xp2_net, xp2_net, xp2_net ; Accumulator xp2_net = 0
        mov	tmp, g_tbls ; Moves g_tbls into tmp
        xor	n_i, n_i ; n_i = 0
        
        ; Begin inner loop for network chunks
        .next_net_vect:
            ; Load next network data chunk
            mov	ptr, [buffs_n + n_i] ; Pointer to next array -> net_data[n_i]
            XLDR x0, [ptr + pos] ; Load data from specific offset -> net_data[n_i][pos]
            add	n_i, PS ; Add 8 to network counter
            
            ; Begin network erasure computation
            vpandq	xtmpa, x0, xmask0f ; Mask low src nibble in bits 4-0
            vpsraw	x0, x0, 4 ; Shift to put high nibble into bits 4-0
            vpandq	x0, x0, xmask0f ; Mask high src nibble in bits 4-0

            vmovdqu8 xgft1_loy, [tmp] ; Load array Ax{00}..{0f}, Ax{00}..{f0}
            vmovdqu8 xgft2_loy, [tmp+k_n*(32/PS)] ; Load array Bx{00}..{0f}, Bx{00}..{f0}
            add	tmp, 32

            vshufi64x2 xgft1_hi, xgft1_lo, xgft1_lo, 0x55
            vshufi64x2 xgft1_lo, xgft1_lo, xgft1_lo, 0x00
            vshufi64x2 xgft2_hi, xgft2_lo, xgft2_lo, 0x55
            vshufi64x2 xgft2_lo, xgft2_lo, xgft2_lo, 0x00

            vpshufb	xgft1_hi, xgft1_hi, x0 ; Lookup mul table of high nibble
            vpshufb	xgft1_lo, xgft1_lo, xtmpa ; Lookup mul table of low nibble
            vpxorq	xgft1_hi, xgft1_hi, xgft1_lo ; GF add high and low partials
            vpxorq	xp1_net, xp1_net, xgft1_hi ; xp1_net += partial

            vpshufb	xgft2_hi, xgft2_hi, x0 ; Lookup mul table of high nibble
            vpshufb	xgft2_lo, xgft2_lo, xtmpa ; Lookup mul table of low nibble
            vpxorq	xgft2_hi, xgft2_hi, xgft2_lo ; GF add high and low partials
            vpxorq	xp2_net, xp2_net, xgft2_hi ; xp2_net += partial
            ; End network erasure computation

            ; Reset local temporary variables
            vpxorq	xp1_loc, xp1_loc, xp1_loc ; Accumulator xp1_loc = 0
            vpxorq	xp2_loc, xp2_loc, xp2_loc ; Accumulator xp2_loc = 0
            mov	tmp2, g_tbls ; Moves g_tbls into tmp2
            xor	l_i, l_i ; l_i = 0

            ; Begin inner loop for local chunks
            .next_loc_vect:
                ; Load next local data chunk
                mov ptr, [buffs_l + l_i] ; Pointer to next array -> loc_data[l_i]
                XLDR x0, [ptr + pos] ; Load local data from specific offset -> loc_data[l_i][pos]
                add	l_i, PS ; Add 8 to l_i

                ; Begin local erasure computations
                vpandq	xtmpa, x0, xmask0f ; Mask low src nibble in bits 4-0
                vpsraw	x0, x0, 4 ; Shift to put high nibble into bits 4-0
                vpandq	x0, x0, xmask0f ; Mask high src nibble in bits 4-0

                vmovdqu8 xgft1_loy, [tmp2] ; Load array Ax{00}..{0f}, Ax{00}..{f0}
                vmovdqu8 xgft2_loy, [tmp2+k_l*(32/PS)] ; Load array Bx{00}..{0f}, Bx{00}..{f0}
                add	tmp2, 32

                vshufi64x2 xgft1_hi, xgft1_lo, xgft1_lo, 0x55
                vshufi64x2 xgft1_lo, xgft1_lo, xgft1_lo, 0x00
                vshufi64x2 xgft2_hi, xgft2_lo, xgft2_lo, 0x55
                vshufi64x2 xgft2_lo, xgft2_lo, xgft2_lo, 0x00

                vpshufb	xgft1_hi, xgft1_hi, x0 ; Lookup mul table of high nibble
                vpshufb	xgft1_lo, xgft1_lo, xtmpa ; Lookup mul table of low nibble
                vpxorq	xgft1_hi, xgft1_hi, xgft1_lo ; GF add high and low partials
                vpxorq	xp1_loc, xp1_loc, xgft1_hi ; xp1_loc += partial

                vpshufb	xgft2_hi, xgft2_hi, x0 ; Lookup mul table of high nibble
                vpshufb	xgft2_lo, xgft2_lo, xtmpa ; Lookup mul table of low nibble
                vpxorq	xgft2_hi, xgft2_hi, xgft2_lo ; GF add high and low partials
                vpxorq	xp2_loc, xp2_loc, xgft2_hi ; xp2_loc += partial
                ; End local erasure computation

                ; Local loop condition
                cmp	l_i, k_l ; Loop through all k local data chunks
                jl	.next_loc_vect ; Keep looping through while l_i < k_n

                ; End inner loop for local chunks

                ; Write out local parities in loc_temp buffer to loc_dest buffer
                XSTR [dest_l + pos], xp1_loc ; Move value from accumulator xp1_loc to dest_l + pos
                XSTR [dest2_l + pos], xp2_loc ; Move value from accumulator xp2_loc to dest2_l + pos

            ; Network loop condition
            cmp	n_i, k_n ; Loop through all k network data chunks
            jl	.next_net_vect ; Keep looping through while n_i < k_n

            ; End inner loop for network chunks

            ; Write out network parities in network accumulator to network parity buffer
            XSTR [dest_n + pos], xp1_net ; Move value from accumulator xp1_net to dest_n + pos
            XSTR [dest2_n + pos], xp2_net ; Move value from accumulator xp2_net to dest2_n + pos

        ; Outer loop condition
        add	pos, 64	; Loop on 64 bytes at a time
        cmp	pos, len_n ; Compare pos to network chunk size
        jle	.loop64 ; Jump to loop64 if pos <= network chunk size

        ; End outer loop
        lea	tmp, [len_n + 64] ; Load address of len + 64 into temp
        cmp	pos, tmp
        je	.return_pass ; Jump to return pass if pos = address of len + 64

        ; If not, there is a tailing 64 bytes
        ; NOTE: This might be a hard-coded safety condition for the loop
        mov	pos, len_n ; Overlapped offset length-64
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
