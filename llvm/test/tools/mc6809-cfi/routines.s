# scripts/mc6809-cfi follows the stack through every path of each routine and
# compares it with the CFI.  It passes routines whose CFI is right, however
# they branch and return, and names each way of getting it wrong.
#
# RUN: llvm-mc -triple=mc6809 -filetype=obj -o %t.good.o %s
# RUN: %python %S/../../../../scripts/mc6809-cfi check --require %t.good.o | FileCheck %s --check-prefix=GOOD
# GOOD: mc6809-cfi: 1 object(s), 2 FDE(s), {{[0-9]+}} instruction(s) followed, 0 problem(s)
#
# RUN: llvm-mc -triple=mc6809 -filetype=obj --defsym=NOADJUST=1 -o %t.noadjust.o %s
# RUN: not %python %S/../../../../scripts/mc6809-cfi check %t.noadjust.o | FileCheck %s --check-prefix=NOADJUST
# NOADJUST: early+0x2: CFI says CFA=SS+2, the stack says SS+4
#
# RUN: llvm-mc -triple=mc6809 -filetype=obj --defsym=NORESTATE=1 -o %t.norestate.o %s
# RUN: not %python %S/../../../../scripts/mc6809-cfi check %t.norestate.o | FileCheck %s --check-prefix=NORESTATE
# NORESTATE: early+0x{{[0-9a-f]+}}: CFI says CFA=SS+2, the stack says SS+4
#
# RUN: llvm-mc -triple=mc6809 -filetype=obj --defsym=UNBALANCED=1 -o %t.unbalanced.o %s
# RUN: not %python %S/../../../../scripts/mc6809-cfi check %t.unbalanced.o | FileCheck %s --check-prefix=UNBALANCED
# UNBALANCED: leaky+0x{{[0-9a-f]+}}: return with CFA-S = 4, not 2
#
# RUN: llvm-mc -triple=mc6809 -filetype=obj --defsym=NOCFI=1 -o %t.nocfi.o %s
# RUN: not %python %S/../../../../scripts/mc6809-cfi check --require %t.nocfi.o | FileCheck %s --check-prefix=NOCFI
# NOCFI: bare: no CFI

	.cfi_sections .debug_frame
	.text

# A push, a branch to an early return, and code after it that is reached with
# the push still on the stack: the CFA must be restated there.
	.globl	early
	.type	early,@function
early:
	.cfi_startproc
	pshs	x
.ifndef NOADJUST
	.cfi_adjust_cfa_offset 2
.endif
	tstb
	beq	1f
	puls	x
	.cfi_adjust_cfa_offset -2
	rts
1:
.ifndef NORESTATE
	.cfi_def_cfa_offset 4
.endif
	leas	2,s
	.cfi_adjust_cfa_offset -2
	rts
	.cfi_endproc
	.size	early, .-early

	.globl	leaky
	.type	leaky,@function
leaky:
	.cfi_startproc
	pshs	d
	.cfi_adjust_cfa_offset 2
.ifndef UNBALANCED
	puls	d
	.cfi_adjust_cfa_offset -2
.endif
	rts
	.cfi_endproc
	.size	leaky, .-leaky

.ifdef NOCFI
	.globl	bare
	.type	bare,@function
bare:
	rts
	.size	bare, .-bare
.endif
