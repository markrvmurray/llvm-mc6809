# `n,pcr` is Motorola's spelling of program-counter-relative indexing, which
# lwasm and gcc6809's assembler share: n is an address, and the assembler
# encodes its distance from the next instruction.  With a symbol it is the
# same instruction as `n,pc`; with a plain number the number is an address
# too, so it becomes a PC-relative reference to that absolute address,
# while `n,pc` with a number stays the offset itself.

# RUN: llvm-mc -triple=mc6809 -show-encoding %s | FileCheck %s --check-prefix=ENC
# RUN: llvm-mc -triple=mc6809 -filetype=obj %s -o %t.o
# RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=REL
# RUN: ld.lld --image-base=0 -Ttext=0x4000 -e start %t.o -o %t.elf
# RUN: llvm-objdump -d %t.elf | FileCheck %s --check-prefix=LINK
# RUN: not llvm-mc -triple=mc6809 -show-encoding --defsym ERR=1 %s 2>&1 | FileCheck %s --check-prefix=ERR

	.text
	.globl	start
start:
# ENC: leax foo,pc ; encoding: [0x30,0x8d,A,A]
	leax	foo,pcr
# ENC: leax foo,pc ; encoding: [0x30,0x8d,A,A]
	leax	foo,pc
# ENC: ldd foo,pc ; encoding: [0xec,0x8d,A,A]
	ldd	foo,PCR
# ENC: sty foo,pc ; encoding: [0x10,0xaf,0x8d,A,A]
	sty	foo,Pcr
# ENC: jsr [foo,pc] ; encoding: [0xad,0x9d,A,A]
	jsr	[foo,pcr]
# ENC: leay {{.*}},pc ; encoding: [0x31,0x8d,A,A]
	leay	$1234,pcr
# ENC: leay 5,pc ; encoding: [0x31,0x8c,0x05]
	leay	5,pc
foo:
	rts

# REL: R_MC6809_PCREL_16 *ABS*+0x1234

# The number after `,pcr` is where Y points, wherever the code is linked.
# LINK: 4015: 31 8d d2 1b leay $1234,pc
# LINK: 4019: 31 8c 05 leay $4021,pc

.ifdef ERR
# ERR: error: expected an address before ',pcr'
	leax	,pcr
.endif
