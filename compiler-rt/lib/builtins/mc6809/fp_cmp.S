; MC6839 FP — comparison wrappers
;
; FCMP (opcode 0x8A) sets the MC6809 CC register:
;   Z=1        : arg1 == arg2
;   N≠V (BLT)  : arg1 < arg2
;   Z=0, N=V (BGT): arg1 > arg2
;   otherwise  : unordered (NaN operand)
;
; GNU soft-float comparison convention: each predicate wrapper returns an i32
; result through *X. The backend then compares that int to 0 to test the
; predicate. All predicate wrappers return the same -1/0/+1 value:
;   a <  b → -1      a == b → 0      a > b → +1      unordered → 1
;
; Since all six predicate wrappers (__eqsf2, __ltsf2, etc.) produce the same
; value and the backend checks the correct condition, they share one body.
;
; Calling convention:
;   X  = i32 result buffer (4 bytes), set by caller
;   [S+2..5]  = f32 arg1   [S+6..9]  = f32 arg2
;   [S+2..9]  = f64 arg1   [S+10..17]= f64 arg2
;
; FCMP does not write to X — result is only in CC. We save X, call FCMP,
; then convert CC to -1/0/+1 and write through the saved X.

        .text

FPOP_FCMP       = 0x8A          ; FPOP_FREM+2+FPOP_M (mixed-precision flag)

; ---- f32 comparison ---------------------------------------------------------
; All six predicate wrappers share __cmpsf2's body.

        .global __eqsf2
        .global __nesf2
        .global __ltsf2
        .global __lesf2
        .global __gtsf2
        .global __gesf2
        .global __cmpsf2
__eqsf2:
__nesf2:
__ltsf2:
__lesf2:
__gtsf2:
__gesf2:
__cmpsf2:
        ; Entry: X=i32 result(4B), [S+2..5]=arg1(f32), [S+6..9]=arg2(f32)
        leau    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        pshs    x                   ; FCMP doesn't use X; save for result write
        lbsr    __fpreg
        .byte   FPOP_FCMP           ; sets CC: N/Z/V reflect arg1 vs arg2
        beq     .Lcmpsf2_equal      ; Z=1 → equal
        blt     .Lcmpsf2_less       ; N≠V → less than
        bgt     .Lcmpsf2_greater    ; Z=0, N=V → greater
        ; unordered (NaN): return 1 (non-zero, ≠ the "equal" sentinel)
        puls    x
        clra
        clrb
        std     ,x                  ; result[0..1] = 0x0000
        ldd     #1
        std     2,x                 ; result[2..3] = 0x0001  → i32 = +1
        rts
.Lcmpsf2_equal:
        puls    x
        clra
        clrb
        std     ,x
        std     2,x                 ; i32 = 0
        rts
.Lcmpsf2_less:
        puls    x
        ldd     #0xFFFF
        std     ,x
        std     2,x                 ; i32 = -1 = 0xFFFFFFFF
        rts
.Lcmpsf2_greater:
        puls    x
        clra
        clrb
        std     ,x
        ldd     #1
        std     2,x                 ; i32 = +1
        rts

; ---- f64 comparison ---------------------------------------------------------

        .global __eqdf2
        .global __nedf2
        .global __ltdf2
        .global __ledf2
        .global __gtdf2
        .global __gedf2
        .global __cmpdf2
__eqdf2:
__nedf2:
__ltdf2:
__ledf2:
__gtdf2:
__gedf2:
__cmpdf2:
        ; Entry: X=i32 result(4B), [S+2..9]=arg1(f64), [S+10..17]=arg2(f64)
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        pshs    x
        lbsr    __fpreg
        .byte   FPOP_FCMP
        beq     .Lcmpdf2_equal
        blt     .Lcmpdf2_less
        bgt     .Lcmpdf2_greater
        ; unordered
        puls    x
        clra
        clrb
        std     ,x
        ldd     #1
        std     2,x
        rts
.Lcmpdf2_equal:
        puls    x
        clra
        clrb
        std     ,x
        std     2,x
        rts
.Lcmpdf2_less:
        puls    x
        ldd     #0xFFFF
        std     ,x
        std     2,x
        rts
.Lcmpdf2_greater:
        puls    x
        clra
        clrb
        std     ,x
        ldd     #1
        std     2,x
        rts
