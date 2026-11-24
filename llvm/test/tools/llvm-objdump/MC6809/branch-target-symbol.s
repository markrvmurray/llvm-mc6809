# llvm-objdump names a branch target after the symbol that holds it, for
# backward branches too.  A backward displacement is negative; read as an
# unsigned field it put the target 256 or 64K bytes past the real one, and
# the target was named after whatever symbol lay below that -- here `after`,
# or an absolute symbol near the end of memory -- while the address printed
# beside it was right.

# RUN: llvm-mc -triple=mc6809 -filetype=obj %s -o %t.o
# RUN: llvm-objdump -d %t.o | FileCheck %s

	.text
leaf:
	rts
	nop

caller:
	bra	leaf
	beq	leaf
	bsr	leaf
	lbra	leaf
	lbeq	leaf
	lbsr	leaf
	bra	after
	lbsr	after
after:
	rts

	.globl	high
	.set	high, 0xffcc

# CHECK-LABEL: <caller>:
# CHECK-NEXT:  bra	$0 <leaf>
# CHECK-NEXT:  beq	$0 <leaf>
# CHECK-NEXT:  bsr	$0 <leaf>
# CHECK-NEXT:  lbra	$0 <leaf>
# CHECK-NEXT:  lbeq	$0 <leaf>
# CHECK-NEXT:  lbsr	$0 <leaf>
# CHECK-NEXT:  bra	$17 <after>
# CHECK-NEXT:  lbsr	$17 <after>
