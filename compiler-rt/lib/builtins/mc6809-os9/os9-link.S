//===-- os9-link.S - OS-9 module calls, which answer in U ---*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// F$Link and F$Load hand back the module's header in U and its entry point
// in Y; F$UnLink is given the header in U.  U is the process data-area
// base, which compiled code can never give up, so -- like the X:U file
// positions next door -- these are hand-written.
//
//   int __os9_link(struct __os9_module *m);     name, type -> header, entry
//   int __os9_load(struct __os9_module *m);     the same, from a file
//   int __os9_unlink(void *header);
//
// The struct is os9.h's: name at 0, type at 2, header at 3, entry at 5.
// Each returns 0 or the OS-9 error code in X (the C `int` register).
//
//===----------------------------------------------------------------------===//

	.text

	.globl	__os9_link
	.type	__os9_link,@function
__os9_link:
	pshs	u,x			// [0,s] = the struct, [2,s] = data base
	lda	2,x			// A = type/language
	ldx	,x			// X = the module name
	os9	0x00			// F$Link
	bra	.Lfound
	.size	__os9_link, .-__os9_link

	.globl	__os9_load
	.type	__os9_load,@function
__os9_load:
	pshs	u,x
	lda	2,x
	ldx	,x			// X = the pathlist
	os9	0x01			// F$Load
	bra	.Lfound
	.size	__os9_load, .-__os9_load

	// Common return: carry set = failure with the code in B.
.Lfound:
	bcs	.Lfail
	ldx	,s			// X = the struct again
	stu	3,x			// the module header
	sty	5,x			// its entry point
	puls	x,u			// X = the struct (dropped), U = data base
	ldx	#0
	rts
.Lfail:
	puls	x,u
	clra
	tfr	d,x			// return the OS-9 error code
	rts

	.globl	__os9_unlink
	.type	__os9_unlink,@function
__os9_unlink:
	pshs	u			// save the data base
	tfr	x,u			// U = the module header
	os9	0x02			// F$UnLink
	puls	u			// (PULS without CC leaves the carry alone)
	bcs	.Lunlink_fail
	ldx	#0
	rts
.Lunlink_fail:
	clra
	tfr	d,x
	rts
	.size	__os9_unlink, .-__os9_unlink
