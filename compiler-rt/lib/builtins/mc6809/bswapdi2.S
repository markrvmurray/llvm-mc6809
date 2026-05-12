// __bswapdi2 — 64-bit byte swap for MC6809
//
// di_int __bswapdi2(di_int u)
//
// Calling convention (gcc6809 ABI, sret for i64 return):
//   X  = pointer to caller-allocated 8-byte result buffer
//   Stack (after return address):
//     S+2..S+9   i64 input 'u' (big-endian, MSB at S+2)
//
// Companion to bswapsi2.S — same self-recursion bug, same shape
// of fix. The compiler-rt C source `bswapdi2.c` uses the
// canonical shift-and-mask idiom that LLVM recognises as
// llvm.bswap.i64; without a hand-asm override the backend would
// legalise G_BSWAP s64 to a __bswapdi2 libcall, creating
// infinite self-recursion.
//
// Algorithm: 8 LDA/STA pairs from stack to sret buffer with
// reversed byte indices. Byte 0 (MSB) ↔ byte 7 (LSB), etc.
//
// Registers used: A (caller-saved). B, X, Y, U untouched other
// than the sret pointer in X (already supplied by caller).

	.section .text,"ax",@progbits
	.globl __bswapdi2
	.type  __bswapdi2,@function
__bswapdi2:
	lda	2,s
	sta	7,x
	lda	3,s
	sta	6,x
	lda	4,s
	sta	5,x
	lda	5,s
	sta	4,x
	lda	6,s
	sta	3,x
	lda	7,s
	sta	2,x
	lda	8,s
	sta	1,x
	lda	9,s
	sta	,x
	rts
	.size __bswapdi2, . - __bswapdi2
