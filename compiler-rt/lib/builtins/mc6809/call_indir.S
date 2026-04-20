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
// Indirect function call helper. The MC6809 backend (MC6809CallLowering.cpp)
// emits an LBSR to this thunk with the target function pointer in IX,
// rather than a direct JSR [,X] -- the 6809 does support JSR [,X], but
// CallLowering uses the thunk approach to keep the call-site encoding
// uniform with direct calls.
//
// LBSR pushed the return address onto the stack before arriving here,
// so JMP [,X] transfers control to the target function. When the target
// executes RTS, it returns directly to the original caller (not to
// __call_indir).
//
// gcc6809 ABI v1: first pointer arg in X. This function clobbers nothing
// beyond what the target function itself may clobber.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__call_indir
	.type	__call_indir,@function
__call_indir:
	jmp	[,x]