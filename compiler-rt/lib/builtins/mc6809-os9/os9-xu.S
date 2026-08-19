//===-- os9-xu.S - OS-9 calls that carry a 32-bit position in X:U -*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// I$Seek, and SS.Size / SS.Pos through I$GetStt / I$SetStt, pass a 32-bit
// file position as X (high word) : U (low word).  U is the process data-area
// base, which compiled code can never give up, so these three are the
// hand-written exceptions to "a system call is one inline instruction":
// each saves U, loads the registers from the caller's struct __os9_xu
// (os9.h: path at 0, function at 1, position at 2..5 big-endian), makes the
// call, stores the position back for the get, restores U, and returns 0 or
// the error code in X (the C `int` return register; the C side puts it in
// errno).  Y is callee-saved and untouched.
//
//   int __os9_seek(struct __os9_xu *p);        I$Seek   A = path, X:U = pos
//   int __os9_getstt_xu(struct __os9_xu *p);   I$GetStt A = path, B = fn
//                                                       -> X:U = pos
//   int __os9_setstt_xu(struct __os9_xu *p);   I$SetStt A = path, B = fn,
//                                                       X:U = pos
//
//===----------------------------------------------------------------------===//

	.text

	.globl	__os9_seek
	.type	__os9_seek,@function
__os9_seek:
	pshs	u,x			// [0,s] = struct pointer, [2,s] = data base
	lda	,x			// A = path
	ldu	4,x			// U = low word of the position
	ldx	2,x			// X = high word (loads X last)
	os9	0x88			// I$Seek
	bra	.Lreturn
	.size	__os9_seek, .-__os9_seek

	.globl	__os9_setstt_xu
	.type	__os9_setstt_xu,@function
__os9_setstt_xu:
	pshs	u,x
	ldd	,x			// A = path, B = function
	ldu	4,x
	ldx	2,x
	os9	0x8E			// I$SetStt
	bra	.Lreturn
	.size	__os9_setstt_xu, .-__os9_setstt_xu

	.globl	__os9_getstt_xu
	.type	__os9_getstt_xu,@function
__os9_getstt_xu:
	pshs	u,x			// [0,s] = struct pointer, [2,s] = data base
	ldd	,x			// A = path, B = function
	os9	0x8D			// I$GetStt -> X:U
	bcs	.Lreturn
	pshs	u			// [0,s] = low word, [2,s] = struct, [4,s] = base
	ldu	2,s			// U = struct pointer for the stores
	stx	2,u			// high word
	puls	x			// X = low word
	stx	4,u			// low word
	puls	x,u			// X = struct pointer (dropped), U = data base
	ldx	#0
	rts
	.size	__os9_getstt_xu, .-__os9_getstt_xu

	// Common return: carry set = failure with the code in B.
.Lreturn:
	puls	u,x			// X = struct pointer (dropped), U = data base
	bcs	.Lfail
	ldx	#0
	rts
.Lfail:
	clra
	tfr	d,x			// return the OS-9 error code
	rts
