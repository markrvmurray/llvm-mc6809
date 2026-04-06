; RUN: llvm-mc -triple=mc6809 --filetype=obj %s -o %t.o
; RUN: llvm-objcopy -O binary %t.o %t.bin
; RUN: od -A x -t x1 -N 14 %t.bin | FileCheck %s
;
; Verify .word and .long emit big-endian bytes (6809 is big-endian).

; 16-bit words
.word 0x1234
.word 0xBEEF
.word 0x0001

; 32-bit longs
.long 0xDEADBEEF
.long 0x00010002

; CHECK: 000000 12 34 be ef 00 01 de ad be ef 00 01 00 02
