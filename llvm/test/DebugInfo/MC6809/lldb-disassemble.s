; lldb disassembles MC6809 and HD6309 instructions of every length.
;
; lldb reads the bytes for a disassembly as the instruction count times the
; longest instruction its core table records.  The table said 2 bytes for
; the MC6809, which is 1 to 5, so any instruction that ran past twice the
; count came out <unknown> and the decoding lost its place.  These run 1, 2,
; 3, 4 and 5 bytes long, with the 5-byte ones last, past where a 2-byte
; reckoning ends.
;
; REQUIRES: lldb
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 --filetype=obj -o %t.o %s
; RUN: ld.lld --image-base=0 -Ttext=0x4000 -e start %t.o -o %t.elf
; RUN: %lldb -b -o "target create %t.elf" \
; RUN:   -o "disassemble -b -s 0x4000 -c 9" 2>&1 | FileCheck %s

; CHECK-NOT:  <unknown>
; CHECK:      [0x4000] <+0>:  4f              clra
; CHECK-NEXT: [0x4001] <+1>:  1f 8b           tfr a,dp
; CHECK-NEXT: [0x4003] <+3>:  8e 00 21        ldx #$21
; CHECK-NEXT: [0x4006] <+6>:  8c 00 23        cmpx #$23
; CHECK-NEXT: [0x4009] <+9>:  10 ce 40 00     lds #$4000
; CHECK-NEXT: [0x400d] <+13>: a6 9f 12 34     lda [$1234]
; CHECK-NEXT: [0x4011] <+17>: 10 ae 89 12 34  ldy $1234,x
; CHECK-NEXT: [0x4016] <+22>: 10 ae 9f 12 34  ldy [$1234]
; CHECK-NEXT: [0x401b] <+27>: cd 12 34 56 78  ldq #$12345678
; CHECK-NOT:  <unknown>

	.text
	.globl	start
	.type	start,@function
start:
	clra
	tfr	a,dp
	ldx	#0x21
	cmpx	#0x23
	lds	#0x4000
	lda	[0x1234]
	ldy	0x1234,x
	ldy	[0x1234]
	ldq	#0x12345678
	.size	start, .-start
