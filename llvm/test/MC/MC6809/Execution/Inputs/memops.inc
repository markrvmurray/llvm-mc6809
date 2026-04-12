;; memops.inc — memcpy, memmove, memset for MC6809
;;
;; Calling convention: IX = first arg (dst/s), stack = remaining args.
;; Return value in IX (pointer to dst/s).
;;
;; Stack layout on entry (after return address):
;;   0,S = return address (2 bytes)
;;   2,S = second arg (2 bytes)
;;   4,S = third arg (2 bytes)

	.text

;; ===== memcpy =====
;; void *memcpy(void *dst, const void *src, size_t n)
;; IX=dst, 2,S=src, 4,S=n. Returns IX=dst.

	.globl	memcpy
memcpy:
	pshs	x		; save dst for return
	ldd	6,s		; n (+2 saved X, +2 retaddr, +2 src)
	beq	.Lmemcpy_done
	pshs	d		; push counter onto stack
	ldy	6,s		; src (+2 counter, +2 saved X, +2 retaddr)
.Lmemcpy_loop:
	lda	,y+
	sta	,x+
	ldd	,s		; reload counter
	subd	#1
	std	,s		; store counter
	bne	.Lmemcpy_loop
	leas	2,s		; pop counter
.Lmemcpy_done:
	puls	x,pc

;; ===== memmove =====
;; void *memmove(void *dst, const void *src, size_t n)
;; IX=dst, 2,S=src, 4,S=n. Returns IX=dst.

	.globl	memmove
memmove:
	pshs	x		; save dst
	ldd	6,s		; n
	beq	.Lmemmove_done
	ldy	4,s		; src (+2 saved X, +2 retaddr)
	; If dst <= src, copy forwards
	pshs	y
	cmpx	,s++
	bls	.Lmemmove_fwd
	; dst > src: copy backwards
	pshs	d		; push counter
	leax	d,x		; X = dst + n
	leay	d,y		; Y = src + n
.Lmemmove_bwd:
	lda	,-y
	sta	,-x
	ldd	,s
	subd	#1
	std	,s
	bne	.Lmemmove_bwd
	leas	2,s
	bra	.Lmemmove_done
.Lmemmove_fwd:
	pshs	d		; push counter
.Lmemmove_fwd_loop:
	lda	,y+
	sta	,x+
	ldd	,s
	subd	#1
	std	,s
	bne	.Lmemmove_fwd_loop
	leas	2,s
.Lmemmove_done:
	puls	x,pc

;; ===== memset =====
;; void *memset(void *s, int c, size_t n)
;; IX=s, 2,S=c (low byte), 4,S=n. Returns IX=s.

	.globl	memset
memset:
	pshs	x		; save s
	ldb	5,s		; c low byte (+2 saved X, +2 retaddr, +1 hi byte)
	ldy	6,s		; n (+2 saved X, +2 retaddr, +2 c)
	beq	.Lmemset_done
.Lmemset_loop:
	stb	,x+
	leay	-1,y
	bne	.Lmemset_loop
.Lmemset_done:
	puls	x,pc
