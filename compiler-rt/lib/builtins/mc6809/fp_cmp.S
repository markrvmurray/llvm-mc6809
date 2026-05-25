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
; All six predicate wrappers share one body returning -1/0/+1/+1
; (less/equal/greater/unordered) per libgcc convention.  LLVM applies
; the predicate-specific ICMP_<pred>(result, 0) in createFCMPLibcall.
; Unordered → +1 makes OLT/OLE/OEQ correctly false; OGT/OGE/ONE inherit
; libgcc's known NaN imprecision.
;
; The wrapper saves the caller's U, Y, AND the result-buffer X via
; pshs u,y,x — we cannot pass X = result buffer to FPREG because FCMP
; uses X as the parameter word.  After PULS lands at FPOP+1, we capture
; CC, reload X from the saved slot, write the i32 result, then unwind.

        .text

; Bug #287: opcodes + parameter-word constants from shared headers.
.include "mc6839_fpops.inc"
.include "mc6839_abi.inc"

; ---- f32 comparison --------------------------------------------------------

        .global __cmpsf2
        .global __ltsf2
        .global __lesf2
        .global __gtsf2
        .global __gesf2
        .global __eqsf2
        .global __nesf2
__cmpsf2:
__ltsf2:
__lesf2:
__gtsf2:
__gesf2:
__eqsf2:
__nesf2:
        pshs    u,y,x               ; save callee-saved U,Y + result buffer X
        leau    8,s                 ; U = &arg1
        leay    __mc6839_fpcb_f32,pc ; Bug #295: PCR-load &fpcb (X is sret buffer)
        tfr     y,d                 ; D = &fpcb (ROM ABI)
        leay    12,s                ; Y = &arg2
        ldx     #FCMP_S_S           ; parameter word: single, single
        lbsr    __fpreg
        .byte   FPOP_FCMP
        bra     .Lcmp_cleanup       ; (BRA preserves CC)

; ---- f64 comparison --------------------------------------------------------

        .global __cmpdf2
        .global __ltdf2
        .global __ledf2
        .global __gtdf2
        .global __gedf2
        .global __eqdf2
        .global __nedf2
__cmpdf2:
__ltdf2:
__ledf2:
__gtdf2:
__gedf2:
__eqdf2:
__nedf2:
        pshs    u,y,x
        leau    8,s
        leay    __mc6839_fpcb_f64,pc ; Bug #295: PCR-load &fpcb (X is sret buffer)
        tfr     y,d                 ; D = &fpcb (ROM ABI)
        leay    16,s                ; Y = &arg2 (f64 arg2 starts 8 bytes higher)
        ldx     #FCMP_D_D           ; parameter word: double, double
        lbsr    __fpreg
        .byte   FPOP_FCMP
        ; fall through

; ---- shared cleanup --------------------------------------------------------
; On entry: CC = comparison result; X = parameter word (= 0 or 0x0011).
; Saved X (result buffer) at [S+0..1]; saved Y at [S+2..3]; saved U at [S+4..5].
; Capture CC into A first, then reload X from the saved slot.

.Lcmp_cleanup:
        tfr     cc,a                ; A = CC byte (TFR doesn't modify CC)
        ldx     ,s                  ; X = saved result buffer (LDX modifies CC; OK)
        bita    #0x04               ; Z?
        bne     .Lcmp_eq
        bita    #0x01               ; C?
        bne     .Lcmp_uo
        bita    #0x08               ; N?
        bne     .Lcmp_lt
        ; fall through: a > b → +1
        clra
        clrb
        std     ,x
        ldd     #1
        std     2,x
        bra     .Lcmp_ret
.Lcmp_lt:
        ldd     #0xFFFF
        std     ,x
        std     2,x                 ; i32 = -1
        bra     .Lcmp_ret
.Lcmp_uo:
        clra
        clrb
        std     ,x
        ldd     #1
        std     2,x                 ; i32 = +1 (libgcc unordered)
        bra     .Lcmp_ret
.Lcmp_eq:
        clra
        clrb
        std     ,x
        std     2,x                 ; i32 = 0
.Lcmp_ret:
        leas    2,s                 ; pop the saved-X slot
        puls    y,u                 ; restore caller's U and Y
        rts

; ---- unordered predicate (Bug #337) ----------------------------------------
; int __unordsf2(float, float) / int __unorddf2(double, double): return
; nonzero iff either operand is NaN.  The MC6839 FCMP already reports this
; via C=1 (CCUN) at FPOP+1 (see the header above).  These reuse the same
; FCMP setup as __cmpXf2 but a cleanup that yields 1 for unordered, else 0.
; Same i32-via-X-sret return convention as the compare wrappers.

        .global __unordsf2
__unordsf2:
        pshs    u,y,x               ; save callee-saved U,Y + result buffer X
        leau    8,s                 ; U = &arg1
        leay    __mc6839_fpcb_f32,pc ; &fpcb (X is sret buffer)
        tfr     y,d                 ; D = &fpcb (ROM ABI)
        leay    12,s                ; Y = &arg2
        ldx     #FCMP_S_S           ; parameter word: single, single
        lbsr    __fpreg
        .byte   FPOP_FCMP
        bra     .Lunord_cleanup     ; (BRA preserves CC)

        .global __unorddf2
__unorddf2:
        pshs    u,y,x
        leau    8,s
        leay    __mc6839_fpcb_f64,pc ; &fpcb (X is sret buffer)
        tfr     y,d                 ; D = &fpcb (ROM ABI)
        leay    16,s                ; Y = &arg2 (f64 arg2 starts 8 bytes higher)
        ldx     #FCMP_D_D           ; parameter word: double, double
        lbsr    __fpreg
        .byte   FPOP_FCMP
        ; fall through

; On entry: CC = comparison result; X = parameter word. Saved result
; buffer X at [S+0..1]; saved Y at [S+2..3]; saved U at [S+4..5].
.Lunord_cleanup:
        tfr     cc,a                ; A = CC byte (TFR doesn't modify CC)
        ldx     ,s                  ; X = saved result buffer (LDX modifies CC; OK)
        bita    #0x01               ; C = unordered (NaN)?  (A still holds CC)
        bne     .Lunord_yes
        clra                        ; ordered → i32 = 0
        clrb
        std     ,x
        std     2,x
        bra     .Lunord_ret
.Lunord_yes:
        clra
        clrb
        std     ,x                  ; high 16 bits = 0
        ldd     #1
        std     2,x                 ; unordered → i32 = 1
.Lunord_ret:
        leas    2,s                 ; pop the saved-X slot
        puls    y,u                 ; restore caller's U and Y
        rts
