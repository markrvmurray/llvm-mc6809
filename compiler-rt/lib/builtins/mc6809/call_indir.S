//===-- call_indir.S - Indirect function call thunk for MC6809 -*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// void __call_indir(void)
//
// Indirect function call helper. The MC6809 backend
// (MC6809CallLowering.cpp) saves the target function pointer in RS7
// (the last 16-bit imaginary direct-page register, at __rs7) and emits
// LBSR __call_indir. This thunk jumps through that pointer.
//
// We route via RS7 rather than a real register (e.g. a JSR [,X] at the
// call site) because the indirect call is lowered early in GlobalISel
// before argument placement, and pinning the function pointer to X
// would conflict with the first-arg convention. Using an imaginary
// direct-page slot sidesteps that contention and mirrors MOS's RS9
// approach.
//
// LBSR pushed the return address before arriving here, so JMP [__rs7]
// transfers control to the target function. When the target executes
// RTS, it returns directly to the original caller (not to __call_indir).
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__call_indir
	.type	__call_indir,@function
__call_indir:
	// Workaround for MC6809 assembler bug: `jmp [__rs7]` (extended
	// indirect with relocated symbol) silently miscompiles to
	// `jmp $0,x` -- the 16-bit relocation gets written into a
	// 2-byte instruction.
	// Indirect-indexed via X works correctly:
	ldx	#__rs7
	jmp	[,x]
