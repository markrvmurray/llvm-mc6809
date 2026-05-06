; MC6839 FP — arithmetic wrappers
;
; Implements the LLVM soft-float arithmetic libcall names by delegating to
; the MC6839 ROM via its register ABI.
;
; Calling convention (MC6809 soft-float, verified via llc):
;   X  = caller-allocated result buffer (f32: 4 bytes, f64: 8 bytes)
;   [S+2..5]   = f32 arg1    [S+6..9]   = f32 arg2
;   [S+2..9]   = f64 arg1    [S+10..17] = f64 arg2
;
; MC6839 register ABI: U→arg1, Y→arg2, D→fpcb, X→result (already set).
; Args are addressed in-place on the caller's stack — no copying.
;
; All calls use lbsr __fpreg (PC-relative, PIE-compatible).

        .text

; FPOP opcodes
FPOP_FADD       = 0x00
FPOP_FSUB       = 0x02
FPOP_FMUL       = 0x04
FPOP_FDIV       = 0x06
FPOP_FSQRT      = 0x12

; ---- f32 dyadic arithmetic --------------------------------------------------
; Entry: X=result(4B), [S+2..5]=arg1(f32), [S+6..9]=arg2(f32)

        .global __addsf3
__addsf3:
        leau    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
        rts

        .global __subsf3
__subsf3:
        leau    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FSUB
        rts

        .global __mulsf3
__mulsf3:
        leau    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FMUL
        rts

        .global __divsf3
__divsf3:
        leau    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FDIV
        rts

; ---- f64 dyadic arithmetic --------------------------------------------------
; Entry: X=result(8B), [S+2..9]=arg1(f64), [S+10..17]=arg2(f64)

        .global __adddf3
__adddf3:
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD
        rts

        .global __subdf3
__subdf3:
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FSUB
        rts

        .global __muldf3
__muldf3:
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FMUL
        rts

        .global __divdf3
__divdf3:
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FDIV
        rts

; ---- square root (monadic: Y→arg) ------------------------------------------
; Entry: X=result, [S+2..N]=arg

        .global __sqrtsf2
__sqrtsf2:
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FSQRT
        rts

        .global __sqrtdf2
__sqrtdf2:
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FSQRT
        rts
