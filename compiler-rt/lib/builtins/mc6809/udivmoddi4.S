// 64-bit division and modulo for MC6809
//
// All six functions in one file (shared core algorithm):
//   __udivdi3, __umoddi3, __udivmoddi4  (unsigned)
//   __divdi3, __moddi3, __divmoddi4     (signed wrappers)
//
// Calling convention (sret for i64 return):
//   X  = pointer to 8-byte result buffer
//   Stack: S+2..S+9 = a, S+10..S+17 = b
//   For divmod variants: S+18..S+19 = rem pointer (or NULL)
//
// The shift amount for i64 shift libcalls is i16 on the stack
// (see shiftRotateLibcall in MC6809LegalizerInfo.cpp).

	.section .text,"ax",@progbits

// ============================================================
// __udivdi3: unsigned 64-bit division (quotient only)
// ============================================================
	.globl __udivdi3
	.type  __udivdi3,@function
__udivdi3:
	pshs	y
	tfr	x,y		// Y = sret pointer
	leas	-17,s		// workspace: 8 quot + 8 rem + 1 counter
	// Copy dividend (a) to quotient workspace
	ldd	21,s		// a+0 (was S+2, now S+21 after -17 and pshs y)
	std	,s
	ldd	23,s
	std	2,s
	ldd	25,s
	std	4,s
	ldd	27,s
	std	6,s
	// Clear remainder
	ldd	#0
	std	8,s
	std	10,s
	std	12,s
	std	14,s
	// Counter = 64
	ldb	#64
	stb	16,s
	// Divisor is at 29,s..36,s
	bsr	.Ludiv64_loop
	// Copy quotient to [Y]
	ldd	,s
	std	,y
	ldd	2,s
	std	2,y
	ldd	4,s
	std	4,y
	ldd	6,s
	std	6,y
	leas	17,s
	puls	y,pc
	.size __udivdi3, . - __udivdi3

// ============================================================
// __umoddi3: unsigned 64-bit modulo (remainder only)
// ============================================================
	.globl __umoddi3
	.type  __umoddi3,@function
__umoddi3:
	pshs	y
	tfr	x,y
	leas	-17,s
	ldd	21,s
	std	,s
	ldd	23,s
	std	2,s
	ldd	25,s
	std	4,s
	ldd	27,s
	std	6,s
	ldd	#0
	std	8,s
	std	10,s
	std	12,s
	std	14,s
	ldb	#64
	stb	16,s
	bsr	.Ludiv64_loop
	// Copy remainder to [Y]
	ldd	8,s
	std	,y
	ldd	10,s
	std	2,y
	ldd	12,s
	std	4,y
	ldd	14,s
	std	6,y
	leas	17,s
	puls	y,pc
	.size __umoddi3, . - __umoddi3

// ============================================================
// __udivmoddi4: unsigned 64-bit divmod (quotient + remainder)
// du_int __udivmoddi4(du_int a, du_int b, du_int *rem)
// ============================================================
	.globl __udivmoddi4
	.type  __udivmoddi4,@function
__udivmoddi4:
	pshs	y
	tfr	x,y
	leas	-17,s
	ldd	21,s
	std	,s
	ldd	23,s
	std	2,s
	ldd	25,s
	std	4,s
	ldd	27,s
	std	6,s
	ldd	#0
	std	8,s
	std	10,s
	std	12,s
	std	14,s
	ldb	#64
	stb	16,s
	// rem pointer at 37,s..38,s
	bsr	.Ludiv64_loop
	// Copy quotient to [Y]
	ldd	,s
	std	,y
	ldd	2,s
	std	2,y
	ldd	4,s
	std	4,y
	ldd	6,s
	std	6,y
	// Copy remainder to [rem_ptr] if non-NULL
	ldx	37,s
	beq	.Ludivmod_done
	ldd	8,s
	std	,x
	ldd	10,s
	std	2,x
	ldd	12,s
	std	4,x
	ldd	14,s
	std	6,x
.Ludivmod_done:
	leas	17,s
	puls	y,pc
	.size __udivmoddi4, . - __udivmoddi4

// ============================================================
// Shared core: 64-bit shift-and-subtract division loop
//
// On entry (called via BSR, so return address at S+0..S+1):
//   S+2..S+9:   quotient/dividend workspace
//   S+10..S+17: remainder workspace (zeroed)
//   S+18:       counter (64)
//   Divisor at S+31..S+38 (accounting for BSR return addr)
// On exit: quotient in S+2..S+9, remainder in S+10..S+17
// ============================================================
.Ludiv64_loop:
	// Shift quotient left by 1 (LSB to MSB)
	asl	9,s		// quot byte 7 (LSB)
	rol	8,s
	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s
	rol	3,s
	rol	2,s		// quot byte 0 (MSB), carry out

	// Rotate remainder left by 1 (carry in from quotient MSB)
	rol	17,s		// rem byte 7 (LSB)
	rol	16,s
	rol	15,s
	rol	14,s
	rol	13,s
	rol	12,s
	rol	11,s
	rol	10,s		// rem byte 0 (MSB)

	// Compare remainder >= divisor (unsigned, high word to low)
	ldd	10,s		// rem word 0
	cmpd	31,s		// div word 0
	bhi	.Ludiv64_sub
	blo	.Ludiv64_next
	ldd	12,s		// rem word 1
	cmpd	33,s
	bhi	.Ludiv64_sub
	blo	.Ludiv64_next
	ldd	14,s		// rem word 2
	cmpd	35,s
	bhi	.Ludiv64_sub
	blo	.Ludiv64_next
	ldd	16,s		// rem word 3
	cmpd	37,s
	blo	.Ludiv64_next
	// Fall through: remainder >= divisor

.Ludiv64_sub:
	// Subtract divisor from remainder (LSW first, borrow upward)
	ldd	16,s		// rem word 3
	subd	37,s		// - div word 3
	std	16,s
	lda	15,s
	sbca	36,s
	sta	15,s
	lda	14,s
	sbca	35,s
	sta	14,s
	lda	13,s
	sbca	34,s
	sta	13,s
	lda	12,s
	sbca	33,s
	sta	12,s
	lda	11,s
	sbca	32,s
	sta	11,s
	lda	10,s
	sbca	31,s
	sta	10,s
	// Set quotient LSB = 1
	lda	9,s
	ora	#1
	sta	9,s

.Ludiv64_next:
	dec	18,s		// decrement counter
	bne	.Ludiv64_loop
	rts

// ============================================================
// Signed wrappers: negate to unsigned, call, fix sign
//
// The negate-and-call pattern:
//   sign = a >> 63 (0 or -1)
//   a_unsigned = (a ^ sign) - sign
//   ... same for b ...
//   call unsigned version
//   result = (result ^ sign) - sign
// ============================================================

// Helper: negate i64 at [X] in place
// Clobbers A, B
.Lneg64:
	ldd	6,x
	coma
	comb
	addd	#1
	std	6,x
	ldd	4,x
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	4,x
	ldd	2,x
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	2,x
	ldd	,x
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,x
	rts

// ============================================================
// __divdi3: signed 64-bit division (quotient only)
// ============================================================
	.globl __divdi3
	.type  __divdi3,@function
__divdi3:
	pshs	y,u
	tfr	x,y		// Y = sret pointer
	// Copy a and b to local workspace (need to negate in place)
	leas	-17,s		// 8 for a_copy + 8 for b_copy + 1 sign byte
	// a at 23,s..30,s; b at 31,s..38,s (after pshs y,u + leas -17)
	ldd	23,s
	std	,s
	ldd	25,s
	std	2,s
	ldd	27,s
	std	4,s
	ldd	29,s
	std	6,s
	ldd	31,s
	std	8,s
	ldd	33,s
	std	10,s
	ldd	35,s
	std	12,s
	ldd	37,s
	std	14,s
	// Compute sign of result: negative if signs of a and b differ
	lda	,s		// MSB of a
	eora	8,s		// XOR with MSB of b
	sta	16,s		// save result sign (bit 7 = negative)
	// Negate a if negative
	tst	,s
	bpl	.Ldivdi3_apos
	leax	,s
	bsr	.Lneg64
.Ldivdi3_apos:
	// Negate b if negative
	tst	8,s
	bpl	.Ldivdi3_bpos
	leax	8,s
	bsr	.Lneg64
.Ldivdi3_bpos:
	// Push args for __udivdi3: a(8) + b(8) on stack, X = sret
	leas	-16,s
	ldd	16,s		// a_copy+0
	std	,s
	ldd	18,s
	std	2,s
	ldd	20,s
	std	4,s
	ldd	22,s
	std	6,s
	ldd	24,s		// b_copy+0
	std	8,s
	ldd	26,s
	std	10,s
	ldd	28,s
	std	12,s
	ldd	30,s
	std	14,s
	tfr	y,x
	lbsr	__udivdi3
	leas	16,s
	// Negate result if sign byte bit 7 is set
	tst	16,s
	bpl	.Ldivdi3_done
	tfr	y,x
	bsr	.Lneg64
.Ldivdi3_done:
	leas	17,s
	puls	y,u,pc
	.size __divdi3, . - __divdi3

// ============================================================
// __moddi3: signed 64-bit modulo (remainder only)
// Remainder sign follows dividend sign (sign of a).
// ============================================================
	.globl __moddi3
	.type  __moddi3,@function
__moddi3:
	pshs	y,u
	tfr	x,y
	leas	-17,s
	ldd	23,s
	std	,s
	ldd	25,s
	std	2,s
	ldd	27,s
	std	4,s
	ldd	29,s
	std	6,s
	ldd	31,s
	std	8,s
	ldd	33,s
	std	10,s
	ldd	35,s
	std	12,s
	ldd	37,s
	std	14,s
	// Remainder sign = sign of a (not XOR of a and b)
	lda	,s
	sta	16,s
	// Negate a if negative
	tst	,s
	bpl	.Lmoddi3_apos
	leax	,s
	bsr	.Lneg64
.Lmoddi3_apos:
	// Negate b if negative
	tst	8,s
	bpl	.Lmoddi3_bpos
	leax	8,s
	bsr	.Lneg64
.Lmoddi3_bpos:
	leas	-16,s
	ldd	16,s
	std	,s
	ldd	18,s
	std	2,s
	ldd	20,s
	std	4,s
	ldd	22,s
	std	6,s
	ldd	24,s
	std	8,s
	ldd	26,s
	std	10,s
	ldd	28,s
	std	12,s
	ldd	30,s
	std	14,s
	tfr	y,x
	lbsr	__umoddi3
	leas	16,s
	// Negate result if sign of a was negative
	tst	16,s
	bpl	.Lmoddi3_done
	tfr	y,x
	bsr	.Lneg64
.Lmoddi3_done:
	leas	17,s
	puls	y,u,pc
	.size __moddi3, . - __moddi3

// ============================================================
// __divmoddi4: signed 64-bit divmod
// di_int __divmoddi4(di_int a, di_int b, di_int *rem)
// Quotient sign: negative if signs differ
// Remainder sign: follows dividend (sign of a)
// ============================================================
	.globl __divmoddi4
	.type  __divmoddi4,@function
__divmoddi4:
	pshs	y,u
	tfr	x,y
	leas	-18,s		// 8 a + 8 b + 1 quot_sign + 1 rem_sign
	// a at 24,s..31,s; b at 32,s..39,s; rem_ptr at 40,s..41,s
	ldd	24,s
	std	,s
	ldd	26,s
	std	2,s
	ldd	28,s
	std	4,s
	ldd	30,s
	std	6,s
	ldd	32,s
	std	8,s
	ldd	34,s
	std	10,s
	ldd	36,s
	std	12,s
	ldd	38,s
	std	14,s
	// Quotient sign: a^b
	lda	,s
	eora	8,s
	sta	16,s
	// Remainder sign: a
	lda	,s
	sta	17,s
	// Negate a if negative
	tst	,s
	bpl	.Ldivmod_apos
	leax	,s
	lbsr	.Lneg64
.Ldivmod_apos:
	tst	8,s
	bpl	.Ldivmod_bpos
	leax	8,s
	lbsr	.Lneg64
.Ldivmod_bpos:
	// Call __udivmoddi4(a, b, rem_ptr)
	// Need 18 bytes: a(8) + b(8) + rem_ptr(2)
	leas	-18,s
	ldd	18,s
	std	,s
	ldd	20,s
	std	2,s
	ldd	22,s
	std	4,s
	ldd	24,s
	std	6,s
	ldd	26,s
	std	8,s
	ldd	28,s
	std	10,s
	ldd	30,s
	std	12,s
	ldd	32,s
	std	14,s
	ldd	58,s		// rem_ptr (was 40,s + 18 for leas)
	std	16,s
	tfr	y,x
	lbsr	__udivmoddi4
	leas	18,s
	// Negate quotient if signs differed
	tst	16,s
	bpl	.Ldivmod_qok
	tfr	y,x
	lbsr	.Lneg64
.Ldivmod_qok:
	// Negate remainder if dividend was negative
	ldx	40,s		// rem_ptr
	beq	.Ldivmod_done
	tst	17,s
	bpl	.Ldivmod_done
	lbsr	.Lneg64
.Ldivmod_done:
	leas	18,s
	puls	y,u,pc
	.size __divmoddi4, . - __divmoddi4
