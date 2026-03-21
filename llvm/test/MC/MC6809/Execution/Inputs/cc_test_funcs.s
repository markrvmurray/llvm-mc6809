;; Test functions for calling convention validation.
;; These simulate what LLVM codegen should produce.
;;
;; CMOC calling convention:
;;   - Args pushed as 16-bit words on stack (right to left)
;;   - Frame pointer in U: pshs u / leau ,s
;;   - After frame setup: U+0=saved_U, U+2=retaddr,
;;     U+4=arg1(16-bit), U+6=arg2(16-bit), U+8=arg3(16-bit), ...
;;   - 8-bit args: high byte at even offset, low byte at odd offset
;;   - Return: 8-bit in B, 16-bit in D
;;   - Caller cleans up stack

	.section .rom,"ax",@progbits

;; Test 1: identity8(a) → a
	.globl	_test_identity8
_test_identity8:
	pshs	u
	leau	,s
	ldb	5,u		; arg1 low byte
	leas	,u
	puls	u,pc

;; Test 2: add8(a, b) → a + b
	.globl	_test_add8
_test_add8:
	pshs	u
	leau	,s
	ldb	5,u		; arg1 low byte
	addb	7,u		; arg2 low byte
	leas	,u
	puls	u,pc

;; Test 3: add16(a, b) → a + b (16-bit)
	.globl	_test_add16
_test_add16:
	pshs	u
	leau	,s
	ldd	4,u		; arg1 (16-bit)
	addd	6,u		; arg2 (16-bit)
	leas	,u
	puls	u,pc

;; Test 4: add3(a, b, c) → a + b + c
	.globl	_test_add3
_test_add3:
	pshs	u
	leau	,s
	ldb	5,u		; arg1
	addb	7,u		; arg2
	addb	9,u		; arg3
	leas	,u
	puls	u,pc

;; Test 5: nested(x) → add8(x, 1)
;; Exercises nested function calls with the same convention.
	.globl	_test_nested
_test_nested:
	pshs	u
	leau	,s
	; Push args for add8: arg2=1, arg1=x
	ldd	#1		; arg2 = 1
	pshs	b,a
	ldd	4,u		; arg1 = x (as 16-bit)
	pshs	b,a
	lbsr	_test_add8
	leas	4,s		; clean up args
	; result in B
	leas	,u
	puls	u,pc

;; Test 6: factorial(n) → n! (recursive)
	.globl	_test_factorial
_test_factorial:
	pshs	u
	leau	,s
	ldb	5,u		; n
	cmpb	#1
	bhi	.Lfact_recurse
	; base case: return 1
	ldb	#1
	leas	,u
	puls	u,pc
.Lfact_recurse:
	; recursive case: n * factorial(n-1)
	pshs	b		; save n
	decb			; n-1
	clra
	pshs	b,a		; push arg: n-1 as 16-bit
	lbsr	_test_factorial
	leas	2,s		; clean up arg
	; B = factorial(n-1)
	puls	a		; A = saved n
	mul			; D = A * B = n * factorial(n-1)
	; result in B (low byte of product, sufficient for small n)
	leas	,u
	puls	u,pc
