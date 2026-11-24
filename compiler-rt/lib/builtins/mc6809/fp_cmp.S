; MC6839 FP — comparison wrappers
;
; FPOP_FCMP (0x8A) has the M (mixed-precision) bit set, so X is a
; "parameter word" specifying the precisions of arg1 and arg2:
;
;   bits 4-6 : arg1 precision (0 = single, 1 = double)
;   bits 0-2 : arg2 precision
;   bit  7   : reserved (0)
;
; For our wrappers:
;   f32-vs-f32 compare → X = 0x0000
;   f64-vs-f64 compare → X = 0x0011
;
; CMPST2 in compare.s encodes the comparison result into the saved-CC
; byte at [CCREG,U]. RDYXIT runs PULS CC,…,PC which restores the modified
; CC; PC is restored to FPOP+1 (the dispatch overwrote the lbsr ret addr
; slot via STY SIZREG,S).  At FPOP+1 the CC bits are:
;
;   Z=1  : a == b      (CCEQ)
;   N=1  : a < b       (CCLT)
;   C=1  : unordered   (CCUN, NaN operand)
;   none : a > b       (CCGT)
;
; The wrappers return -1/0/+1 for less/equal/greater, and LLVM tests
; the result against zero per predicate: a > b is __gtXf2(a, b) > 0,
; a >= b is __geXf2(a, b) >= 0, a < b is __ltXf2(a, b) < 0, and so on.
; Every ordered predicate must be false on a NaN, so the unordered
; result depends on the entry, as in libgcc and compiler-rt: +1 from
; __cmp/__lt/__le/__eq/__ne, -1 from __gt/__ge.  (+1 from __gt/__ge
; made x > NaN and x >= NaN true.)
;
; The wrapper saves the caller's U, Y, AND the result-buffer X via
; pshs u,y,x — we cannot pass X = result buffer to FPREG because FCMP
; uses X as the parameter word.  After PULS lands at FPOP+1, we capture
; CC, reload X from the saved slot, write the i32 result, then unwind.

; Each entry group sits in its own section, so a program linked with
; --gc-sections carries only the groups it calls plus the shared result
; code.  Entries reach that code with LBRA (BRA cannot cross sections).

; Bug #287: opcodes + parameter-word constants from shared headers.
.include "mc6839_fpops.inc"
#include "mc6839_call.inc"
.include "mc6839_abi.inc"

; ---- f32 comparison --------------------------------------------------------

        .section .text.__cmpsf2,"ax",@progbits
        .global __cmpsf2
        .global __ltsf2
        .global __lesf2
        .global __eqsf2
        .global __nesf2
__cmpsf2:
__ltsf2:
__lesf2:
__eqsf2:
	.cfi_sections .debug_frame
__nesf2:
	.cfi_startproc
        pshs    u,y,x               ; save callee-saved U,Y + result buffer X
	.cfi_adjust_cfa_offset 6
	.cfi_offset u, -4
	.cfi_offset y, -6
        FP_ENTRY __mc6839_fpcb_f32
        leau    8+FP_SLOT,s                 ; U = &arg1
        leay    12+FP_SLOT,s                ; Y = &arg2
        ldx     #FCMP_S_S           ; parameter word: single, single
        FP_CALL FPOP_FCMP
        lbra    __mc6839_cmp_result ; (LBRA preserves CC)
	.cfi_endproc

        .section .text.__gtsf2,"ax",@progbits
        .global __gtsf2
        .global __gesf2
__gtsf2:
	.cfi_startproc
__gesf2:
        pshs    u,y,x
	.cfi_adjust_cfa_offset 6
	.cfi_offset u, -4
	.cfi_offset y, -6
        FP_ENTRY __mc6839_fpcb_f32
        leau    8+FP_SLOT,s
        leay    12+FP_SLOT,s
        ldx     #FCMP_S_S
        FP_CALL FPOP_FCMP
        lbra    __mc6839_cmp_result_gt ; (LBRA preserves CC)
	.cfi_endproc

; ---- f64 comparison --------------------------------------------------------

        .section .text.__cmpdf2,"ax",@progbits
        .global __cmpdf2
        .global __ltdf2
        .global __ledf2
        .global __eqdf2
        .global __nedf2
__cmpdf2:
__ltdf2:
__ledf2:
__eqdf2:
__nedf2:
	.cfi_startproc
        pshs    u,y,x
	.cfi_adjust_cfa_offset 6
	.cfi_offset u, -4
	.cfi_offset y, -6
        FP_ENTRY __mc6839_fpcb_f64
        leau    8+FP_SLOT,s
        leay    16+FP_SLOT,s                ; Y = &arg2 (f64 arg2 starts 8 bytes higher)
        ldx     #FCMP_D_D           ; parameter word: double, double
        FP_CALL FPOP_FCMP
        lbra    __mc6839_cmp_result ; (LBRA preserves CC)
	.cfi_endproc

        .section .text.__gtdf2,"ax",@progbits
        .global __gtdf2
        .global __gedf2
__gtdf2:
	.cfi_startproc
__gedf2:
        pshs    u,y,x
	.cfi_adjust_cfa_offset 6
	.cfi_offset u, -4
	.cfi_offset y, -6
        FP_ENTRY __mc6839_fpcb_f64
        leau    8+FP_SLOT,s
        leay    16+FP_SLOT,s
        ldx     #FCMP_D_D
        FP_CALL FPOP_FCMP
        lbra    __mc6839_cmp_result_gt ; (LBRA preserves CC)
	.cfi_endproc

; ---- shared result ---------------------------------------------------------
; On entry: CC = comparison result; X = parameter word (= 0 or 0x0011).
; Saved X (result buffer) at [S+0..1]; saved Y at [S+2..3]; saved U at [S+4..5].
; Capture CC into A first, then reload X from the saved slot.

        .section .text.__mc6839_cmp_result,"ax",@progbits
        .global __mc6839_cmp_result
        .global __mc6839_cmp_result_gt
        .hidden __mc6839_cmp_result
        .hidden __mc6839_cmp_result_gt

; __gtXf2/__geXf2: unordered (C=1) gives -1, so a > b and a >= b are
; false on a NaN; anything else as below.
__mc6839_cmp_result_gt:
	.cfi_startproc
	.cfi_def_cfa_offset 8
	.cfi_offset u, -4
	.cfi_offset y, -6
        bcs     .Lcmp_uo_gt         ; unordered: -1
	.cfi_endproc
__mc6839_cmp_result:
	.cfi_startproc
	.cfi_def_cfa_offset 8
	.cfi_offset u, -4
	.cfi_offset y, -6
        tfr     cc,a                ; A = CC byte (TFR doesn't modify CC)
        ldx     ,s                  ; X = saved result buffer (LDX modifies CC; OK)
        bita    #0x04               ; Z?
        bne     .Lcmp_eq
        bita    #0x01               ; C?
        bne     .Lcmp_gt            ; unordered: +1, as a > b
        bita    #0x08               ; N?
        bne     .Lcmp_lt
.Lcmp_gt:
        clra
        clrb
        std     ,x
        incb
        std     2,x                 ; i32 = +1
        bra     .Lcmp_ret
.Lcmp_uo_gt:
        ldx     ,s                  ; X = saved result buffer
.Lcmp_lt:
        ldd     #0xFFFF
        std     ,x
        std     2,x                 ; i32 = -1
        bra     .Lcmp_ret
.Lcmp_eq:
        clra
        clrb
        std     ,x
        std     2,x                 ; i32 = 0
.Lcmp_ret:
        leas    2,s                 ; pop the saved-X slot
	.cfi_adjust_cfa_offset -2
        puls    y,u,pc              ; restore caller's U and Y, return
	.cfi_endproc

; ---- unordered predicate (Bug #337) ----------------------------------------
; int __unordsf2(float, float) / int __unorddf2(double, double): return
; nonzero iff either operand is NaN.  The MC6839 FCMP already reports this
; via C=1 (CCUN) at FPOP+1 (see the header above).  These reuse the same
; FCMP setup as __cmpXf2 but a cleanup that yields 1 for unordered, else 0.
; Same i32-via-X-sret return convention as the compare wrappers.

        .section .text.__unordsf2,"ax",@progbits
        .global __unordsf2
__unordsf2:
	.cfi_startproc
        pshs    u,y,x               ; save callee-saved U,Y + result buffer X
	.cfi_adjust_cfa_offset 6
	.cfi_offset u, -4
	.cfi_offset y, -6
        FP_ENTRY __mc6839_fpcb_f32
        leau    8+FP_SLOT,s                 ; U = &arg1
        leay    12+FP_SLOT,s                ; Y = &arg2
        ldx     #FCMP_S_S           ; parameter word: single, single
        FP_CALL FPOP_FCMP
        lbra    __mc6839_unord_result ; (LBRA preserves CC)
	.cfi_endproc

        .section .text.__unorddf2,"ax",@progbits
        .global __unorddf2
__unorddf2:
	.cfi_startproc
        pshs    u,y,x
	.cfi_adjust_cfa_offset 6
	.cfi_offset u, -4
	.cfi_offset y, -6
        FP_ENTRY __mc6839_fpcb_f64
        leau    8+FP_SLOT,s
        leay    16+FP_SLOT,s                ; Y = &arg2 (f64 arg2 starts 8 bytes higher)
        ldx     #FCMP_D_D           ; parameter word: double, double
        FP_CALL FPOP_FCMP
        lbra    __mc6839_unord_result ; (LBRA preserves CC)
	.cfi_endproc

; On entry: CC = comparison result; X = parameter word. Saved result
; buffer X at [S+0..1]; saved Y at [S+2..3]; saved U at [S+4..5].
        .section .text.__mc6839_unord_result,"ax",@progbits
        .global __mc6839_unord_result
        .hidden __mc6839_unord_result
__mc6839_unord_result:
	.cfi_startproc
	.cfi_def_cfa_offset 8
	.cfi_offset u, -4
	.cfi_offset y, -6
        tfr     cc,b                ; B = CC byte (TFR doesn't modify CC)
        andb    #0x01               ; C = unordered (NaN): B = 1, else 0
        clra
        ldx     ,s                  ; X = saved result buffer
        std     2,x                 ; low 16 bits = 1 or 0
        clrb
        std     ,x                  ; high 16 bits = 0
        leas    2,s                 ; pop the saved-X slot
	.cfi_adjust_cfa_offset -2
        puls    y,u,pc              ; restore caller's U and Y, return
	.cfi_endproc
