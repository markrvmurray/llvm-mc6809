// __bswapsi2 — 32-bit byte swap for MC6809
//
// si_int __bswapsi2(si_int u)
//
// Calling convention (gcc6809 ABI, sret for i32 return):
//   X  = pointer to caller-allocated 4-byte result buffer
//   Stack (after return address):
//     S+2..S+5   i32 input 'u' (big-endian, MSB at S+2)
//
// The compiler-rt C source `bswapsi2.c` uses the canonical
// shift-and-mask idiom that LLVM's middle-end recognises as
// llvm.bswap.i32. On a target without a single-instruction i32
// byte swap (like MC6809), the backend legalises G_BSWAP s32 to
// a libcall — to __bswapsi2 itself, creating an infinite
// self-recursion. The asm version below sidesteps the recogniser
// entirely. Surface: picolibc test-rand at Os-fp / Os-lto-fp
// where arc4random's `_rs_init` calls __bswapsi2 9 times to
// seed-byte-swap; without this fix, each call recursed until
// stack overflow → PC corruption → "invalid instruction".
//
// Algorithm: 4 LDA/STA pairs from stack to sret buffer with
// reversed byte indices. Big-endian byte 0 (MSB) → result byte
// 3 (LSB), and vice versa.
//
// Registers used: A (caller-saved). B, X, Y, U untouched
// other than the sret pointer in X (already supplied by caller).

	.section .text,"ax",@progbits
	.globl __bswapsi2
	.type  __bswapsi2,@function
__bswapsi2:
	lda	2,s		// input byte 0 (MSB)
	sta	3,x		// → result byte 3 (LSB)
	lda	3,s		// input byte 1
	sta	2,x		// → result byte 2
	lda	4,s		// input byte 2
	sta	1,x		// → result byte 1
	lda	5,s		// input byte 3 (LSB)
	sta	,x		// → result byte 0 (MSB)
	rts
	.size __bswapsi2, . - __bswapsi2
