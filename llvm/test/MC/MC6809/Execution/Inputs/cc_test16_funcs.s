;; 16-bit test functions for calling convention validation.
;;
;; CMOC calling convention:
;;   16-bit args at U+4, U+6, ... (after pshs u / leau ,s)
;;   16-bit return in D (A=high, B=low)

	.section .rom,"ax",@progbits

;; identity16(a) → a
	.globl	_test_identity16
_test_identity16:
	pshs	u
	leau	,s
	ldd	4,u		; arg1 (16-bit)
	leas	,u
	puls	u,pc

;; add16(a, b) → a + b
	.globl	_test_add16
_test_add16:
	pshs	u
	leau	,s
	ldd	4,u		; arg1
	addd	6,u		; arg2
	leas	,u
	puls	u,pc

;; sub16(a, b) → a - b
	.globl	_test_sub16
_test_sub16:
	pshs	u
	leau	,s
	ldd	4,u		; arg1
	subd	6,u		; arg2
	leas	,u
	puls	u,pc
