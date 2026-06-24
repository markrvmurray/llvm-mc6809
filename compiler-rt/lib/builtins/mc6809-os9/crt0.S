//===-- crt0.S - OS-9 / NitrOS-9 program-module CRT entry ----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Bug #163 Phase 2: program-module entry stub.
//
// On entry, the OS-9 kernel hands us (per llvm/docs/MC6809-OS9.md §A.3):
//
//   U  = process data-area base   (M$Mem bytes were allocated for us)
//   S  = U + M$Mem - 1            (stack top, grows down)
//   Y  = U + M$Mem - 1            (top-of-area convenience pointer)
//   X  = parameter string pointer (from F$Fork or shell)
//   D  = parameter byte count
//   DP = high byte of U           (so direct-mode addressing into data
//                                   area works without explicit setup)
//   PC = module_base + M$Exec     (which lands here at _start)
//
// The kernel does NOT copy initialised data and does NOT zero BSS —
// the CRT must do both.
//
// Our job, in order:
//
//   1. Copy the .data image from the module body into U+0
//        src: PCR-relative __data_image_start (in module body)
//        dst: U+0 (process data area base)
//        n  : __data_size (link-time constant)
//
//   2. Relocate initialized pointers inside the copied .data image.
//
//      The .os9_reloc table is emitted by lld for address relocations
//      whose storage lives in .data.  Each entry patches one copied word:
//        kind 1: word is a module-body offset; add actual _start address
//        kind 2: word is a .data image offset; convert to process .data
//        kind 3: word is a .bss image offset; convert to process .bss
//
//   3. Zero .bss at U + __data_size
//        dst: U + __data_size (= __bss_start)
//        n  : __bss_size (link-time constant)
//
//   4. Save the param-string descriptor across the main() call.  At
//      this minimal Phase 2 PoC, main() takes (void) — param parsing
//      will land with the picolibc syscall layer.
//
//   5. Call main().  Return value comes back in X; low byte is
//      the exit status per the C `int main(void)` convention.
//
//   6. Exit via F$Exit ($06) — does not return.
//
// Both the .data copy and .bss zero loops use simple byte-by-byte
// operations with a 16-bit count held temporarily in U.  No HD6309-only
// TFM optimisation here — Phase 2 stays plain MC6809; a TFM variant can
// land as a later compiler-rt CPU-tier file (mc6809-os9/hd6309/crt0.S)
// once the rest is stable.
//
//===----------------------------------------------------------------------===//

	.section .text.startup,"ax",@progbits

	.globl	_start
	.type	_start,@function
_start:

	// --- 1. Copy .data image from module body to U+0 -------------------

	ldd	#__data_size
	beq	.Ldata_done		// zero-sized .data: skip the loop entirely
	leax	__data_image_start,pc	// X = PCR address of .data image in body
	leay	0,u			// Y = U + 0 (destination = data-area base)
	pshs	u			// Save process data base.
	tfr	d,u			// U = byte count
.Ldata_loop:
	lda	,x+
	sta	,y+
	leau	-1,u
	cmpu	#0
	bne	.Ldata_loop
	puls	u
.Ldata_done:

	// --- 2. Relocate initialized pointers in copied .data ----------------

	ldd	#__os9_reloc_size
	beq	.Lreloc_done
	leax	__os9_reloc_start,pc
.Lreloc_loop:
	ldb	,x+			// B = relocation kind, 0 = sentinel
	beq	.Lreloc_done
	pshs	b
	ldd	,x++			// D = U-relative offset of word to patch
	leay	d,u			// Y = address of copied word in process data
	puls	b
	cmpb	#1
	beq	.Lreloc_module
	cmpb	#2
	beq	.Lreloc_data
	cmpb	#3
	beq	.Lreloc_bss
	bra	.Lreloc_loop

.Lreloc_module:
	ldd	,y			// D = module-body offset
	pshs	x
	leax	_start,pc		// X = actual module body base
	leax	d,x
	tfr	x,d
	puls	x
	std	,y
	bra	.Lreloc_loop

.Lreloc_data:
	ldd	,y			// D = .data image offset
	subd	#__data_image_start	// D = process-data offset
	pshs	x
	leax	d,u
	tfr	x,d
	puls	x
	std	,y
	bra	.Lreloc_loop

.Lreloc_bss:
	ldd	,y			// D = .bss image offset
	subd	#__bss_image_start
	addd	#__data_size		// D = process-bss offset
	pshs	x
	leax	d,u
	tfr	x,d
	puls	x
	std	,y
	bra	.Lreloc_loop
.Lreloc_done:

	// --- 3. Zero .bss at U + __data_size --------------------------------

	ldd	#__bss_size
	beq	.Lbss_done
	pshs	d			// Save .bss byte count.
	ldd	#__data_size
	leay	d,u			// Y = U + __data_size = bss start
	puls	d			// Restore .bss byte count.
	pshs	u			// Save process data base.
	tfr	d,u			// U = byte count
.Lbss_loop:
	clr	,y+
	leau	-1,u
	cmpu	#0
	bne	.Lbss_loop
	puls	u
.Lbss_done:

	// --- 4-5. Call main() ----------------------------------------------
	//
	// Param-string pointer (X) and byte count (D) on entry are NOT
	// preserved across this call.  Once the picolibc syscall layer
	// lands, we'll tokenize X+D into argc/argv on the stack before
	// jsr'ing main; for the Phase 2 PoC main() takes (void).

	lbsr	main

	// --- 6. Exit via F$Exit --------------------------------------------
	//
	// main() returned with the exit status in X per the LLVM-MC6809
	// ABI for `int`.  F$Exit consumes B as the status and never
	// returns to us.

	tfr	x,d
	swi2
	.byte	0x06			// F$Exit
	.size	_start, .-_start
