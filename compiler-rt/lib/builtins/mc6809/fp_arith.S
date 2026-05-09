; MC6839 FP — arithmetic wrappers
;
; LLVM soft-float entry conditions:
;   X         = caller-allocated result buffer (4B for f32, 8B for f64)
;   [S+2..5]  = f32 arg1   [S+6..9]   = f32 arg2
;   [S+2..9]  = f64 arg1   [S+10..17] = f64 arg2
;
; MC6839 FPREG register-ABI requirements (nfp09-abi.s):
;   U → arg1   Y → arg2   D → fpcb   X → result
;
; U and Y are callee-saved on MC6809; we save and restore them.
; After pshs u,y (4 bytes pushed), arg offsets shift by 4:
;   f32 arg1 at [S+6..9]   f32 arg2 at [S+10..13]
;   f64 arg1 at [S+6..13]  f64 arg2 at [S+14..21]
;
; The ROM's RDYXIT exit lands PULS PC at FPOP+1 (the byte after the
; FPOP byte, where the dispatch's STY SIZREG,S re-points the saved
; PC).  Our `puls u,y; rts` lives at FPOP+1 and runs as the cleanup.

        .text

FPOP_FADD       = 0x00
FPOP_FSUB       = 0x02
FPOP_FMUL       = 0x04
FPOP_FDIV       = 0x06
FPOP_FSQRT      = 0x12
FPOP_FABS       = 0x1E    ; FCAB = FCBNDC+2 in MC6839 equates.s
FPOP_FNEG       = 0x20    ; FCNEG = FCAB+2

; ---- f32 dyadic -------------------------------------------------------------

        .global __addsf3
__addsf3:
        pshs    u,y
        leau    6,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
        puls    u,y
        rts

        .global __subsf3
__subsf3:
        pshs    u,y
        leau    6,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FSUB
        puls    u,y
        rts

        .global __mulsf3
__mulsf3:
        pshs    u,y
        leau    6,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FMUL
        puls    u,y
        rts

        .global __divsf3
__divsf3:
        pshs    u,y
        leau    6,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FDIV
        puls    u,y
        rts

; ---- f64 dyadic -------------------------------------------------------------

        .global __adddf3
__adddf3:
        pshs    u,y
        leau    6,s
        leay    14,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD
        puls    u,y
        rts

        .global __subdf3
__subdf3:
        pshs    u,y
        leau    6,s
        leay    14,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FSUB
        puls    u,y
        rts

        .global __muldf3
__muldf3:
        pshs    u,y
        leau    6,s
        leay    14,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FMUL
        puls    u,y
        rts

        .global __divdf3
__divdf3:
        pshs    u,y
        leau    6,s
        leay    14,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FDIV
        puls    u,y
        rts

; ---- square root (monadic; Y → arg) ----------------------------------------

        .global __sqrtsf2
__sqrtsf2:
        pshs    u,y
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FSQRT
        puls    u,y
        rts

        .global __sqrtdf2
__sqrtdf2:
        pshs    u,y
        leay    6,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FSQRT
        puls    u,y
        rts

; ---- absolute value (monadic; Y → arg) -------------------------------------
;
; Replaces clang's default lowering of llvm.fabs.f32 / .f64 (bit-AND with
; 0x7FFFFFFF / 0x7FFFFFFFFFFFFFFF) which on MC6809 narrows to an i32/i64
; __andsi3/__anddi3 libcall — ~30 bytes per call site. The MC6839 FABS path
; is ~8 bytes per call, single ROM dispatch.

        .global __abssf2
__abssf2:
        pshs    u,y
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FABS
        puls    u,y
        rts

        .global __absdf2
__absdf2:
        pshs    u,y
        leay    6,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FABS
        puls    u,y
        rts
