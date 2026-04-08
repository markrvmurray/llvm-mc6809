; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=100000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Hello-world smoke test — exercises the runtime, ACIA write path, and halt.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	ldx	#message
	jsr	putstr
	jsr	putnl
	rts

message:
	.asciz	"Hello, 6809!"

; CHECK: Hello, 6809!
