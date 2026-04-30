;;
;; runtime6309.inc — HD6309-native variant of runtime.inc.
;;
;; Identical to runtime.inc except `_start` issues `LDMD #1` immediately
;; after stack setup, engaging HD6309 native mode (bit 0 of MD = 1):
;;
;;   * TFR/EXG of E, F, V, W become legal.
;;   * PSHSW / PULSW / PSHUW / PULUW become available.
;;   * DIVQ, MULD, LDQ, STQ, LDW, STW, etc. become legal.
;;   * Some 6809 instruction encodings shift subtly under native mode
;;     (e.g. illegal-instruction trap sites differ); regression-test
;;     against the spec rather than memorising opcodes.
;;
;; Implementation: defines `__runtime_skip_start` *before* including
;; runtime.inc. The guard in runtime.inc then suppresses its own
;; `_start` body, so this file owns the prologue and the helpers
;; (_acia_init, putchar, puthex, putstr, putnl, halt) come from
;; runtime.inc unchanged. No source duplication.
;;
;; Use with `; REQUIRES: mame, hd6309` in the lit RUN line — usim
;; cannot execute LDMD or any subsequent native-mode instruction.
;;

	.section .rom,"ax",@progbits
	.globl	_start
_start:
	lds	#0x4000		; stack at top of low-RAM data region
	ldmd	#1		; HD6309: engage native mode
	clra
	tfr	a,dp		; DP=0 (direct page at $0000)

	;; Zero .bss
	ldx	#__bss_start
.Lbss_zero_6309:
	cmpx	#__bss_end
	beq	.Lbss_done_6309
	clr	,x+
	bra	.Lbss_zero_6309
.Lbss_done_6309:

	jsr	_acia_init
	jsr	test_main
	jsr	halt
_hang_6309:
	bra	_hang_6309

;; Suppress runtime.inc's own _start emission and pull in the helpers
;; (_acia_init, putchar, puthex, putstr, putnl, halt, vector table).
__runtime_skip_start = 1
.include "runtime.inc"
