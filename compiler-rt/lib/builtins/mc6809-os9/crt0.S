//===-- crt0.S - OS-9 / NitrOS-9 program-module CRT entry ----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// OS-9 / NitrOS-9 program-module entry stub.
//
// On entry, the OS-9 kernel hands us (llvm/docs/MC6809-OS9.md):
//
//   U  = process data-area base   (M$Mem bytes, 256-byte aligned)
//   S  = U + M$Mem                (stack, grows towards U)
//   Y  = U + M$Mem                (end-of-area convenience pointer)
//   X  = parameter string pointer (from F$Fork or shell)
//   D  = parameter byte count
//   DP = high byte of U           (the first page of the data area is the
//                                   direct page)
//   PC = module_base + M$Exec     (which lands here at _start)
//
// The kernel does NOT copy initialised data and does NOT zero BSS —
// the CRT must do both.  The data-area layout is a set of U-relative
// offsets defined by mc6809-os9.lds (see its header); in order:
//
//   1. Copy the .dp.data image from the module body to U + __dp_data_start
//      (inside the direct page) and the .data image to U + __data_start.
//
//   2. Rebase pointers held in the copied .data.  ld.lld emits the
//      .os9_reloc table for address relocations whose storage lives in
//      .data or .dp.data and has already turned each such word into an
//      offset; every entry is (kind, U-relative offset of the word):
//        kind 1: word is a module-body offset; add the module base
//        kind 2: word is a data-area offset;   add U
//      A zero kind ends the table.
//
//   3. Zero .dp.bss (at U + __dp_bss_start) and .bss (at U + __bss_start).
//
//   4. Hand over to the C half (crt1.c): store the entry registers (U, X,
//      D, Y, the module base) and the linker script's numbers (the heap
//      request and the static pool) in its variables and call __os9_crt1(),
//      which claims the heap (F$Mem), builds argc/argv from the parameter
//      string, calls main() and exits via F$Exit ($06).
//
// The copy and zero loops are plain byte-by-byte MC6809 code with the
// 16-bit count held temporarily in U; an HD6309 TFM variant can live in a
// later CPU-tier file once the rest is stable.
//
//===----------------------------------------------------------------------===//

	.section .text.startup,"ax",@progbits

	.globl	_start
	.type	_start,@function
_start:
	// --- 0. Keep the parameter string (X), its length (D) and the top of
	//        the area (Y) while the data area is set up; U survives on its
	//        own (nothing below writes it).
	pshs	d,x,y
	// --- 1. Copy the initialised-data images from the module body into
	//        the data area: .dp.data to U + __dp_data_start (the direct
	//        page), .data to U + __data_start.
	ldd	#__dp_data_start
	leay	d,u			// Y = U + __dp_data_start
	leax	__dp_data_image_start,pc
	ldd	#__dp_data_size
	lbsr	.Lcopy
	ldd	#__data_start
	leay	d,u			// Y = U + __data_start
	leax	__data_image_start,pc
	ldd	#__data_size
	lbsr	.Lcopy
	// --- 2. Rebase pointers held in the copied .data ---------------------
	//
	// ld.lld has already turned each such word into an offset: into the
	// module body (kind 1) or into the data area (kind 2). Each entry is
	// (kind, U-relative offset of the word); a zero kind ends the table.
	ldd	#__os9_reloc_size
	beq	.Lreloc_done
	leax	__os9_reloc_start,pc
.Lreloc_loop:
	ldb	,x+			// B = relocation kind, 0 = sentinel
	beq	.Lreloc_done
	pshs	b			// keep the kind while D is used for the offset
	ldd	,x++			// D = U-relative offset of the word
	leay	d,u			// Y = address of the copied word
	puls	b			// B = kind
	pshs	x			// keep the table pointer
	cmpb	#1
	beq	.Lreloc_module
	ldd	,y			// kind 2: word += U
	leax	d,u
	bra	.Lreloc_store
.Lreloc_module:
	ldd	,y			// kind 1: word += module body base
	leax	_start,pc
	leax	d,x
.Lreloc_store:
	tfr	x,d
	std	,y
	puls	x
	bra	.Lreloc_loop
.Lreloc_done:
	// --- 3. Zero .dp.bss (in the direct page) and .bss ---------------------
	ldd	#__dp_bss_start
	leay	d,u			// Y = U + __dp_bss_start
	ldd	#__dp_bss_size
	lbsr	.Lzero
	ldd	#__bss_start
	leay	d,u			// Y = U + __bss_start
	ldd	#__bss_size
	lbsr	.Lzero
	bra	.Lcall_main

	// Copy D bytes from X to Y. Clobbers D, X, Y; U preserved.
.Lcopy:
	cmpd	#0
	beq	.Lcopy_done
	pshs	u
	tfr	d,u			// U = byte count
.Lcopy_loop:
	lda	,x+
	sta	,y+
	leau	-1,u
	cmpu	#0
	bne	.Lcopy_loop
	puls	u
.Lcopy_done:
	rts

	// Zero D bytes at Y. Clobbers D, Y; U preserved.
.Lzero:
	cmpd	#0
	beq	.Lzero_done
	pshs	u
	tfr	d,u			// U = byte count
.Lzero_loop:
	clr	,y+
	leau	-1,u
	cmpu	#0
	bne	.Lzero_loop
	puls	u
.Lzero_done:
	rts

.Lcall_main:
	// --- 4. Hand over to the C half ------------------------------------
	//
	// The variables live in the data area (U-relative, the data model);
	// the linker-script numbers are plain immediates.
	stu	mc6809_os9_data(__os9_data_base),u
	puls	d,x,y			// D = parameter length, X = parameter string,
					// Y = top of the area
	std	mc6809_os9_data(__os9_param_len),u
	stx	mc6809_os9_data(__os9_param),u
	sty	mc6809_os9_data(__os9_top),u
	leax	_start,pc		// the module body starts at its header, 13
	leax	-13,x			// bytes before _start (M$Exec = 13)
	stx	mc6809_os9_data(__os9_module),u
	ldd	#__os9_mem_request
	std	mc6809_os9_data(__os9_mem_want),u
	ldd	#__heap_start
	std	mc6809_os9_data(__os9_pool_off),u
	ldd	#__heap_size
	std	mc6809_os9_data(__os9_pool_len),u
	ldd	#__stack_size
	std	mc6809_os9_data(__os9_stack_size),u
	lbsr	__os9_crt1		// does not return (F$Exit)
	.size	_start, .-_start
