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
;
; Bug #295: FPCB pointer load is PIE — `leay __sym,pc; tfr y,d` puts
; the FPCB address in D via Y (X must be preserved as the sret result
; buffer for >2-byte returns).  The Y-as-scratch pattern is placed
; BEFORE the final `leay N,s` so Y ends up holding arg2 at the ROM
; call.

        .text

; Bug #287: opcodes from shared header (single source of truth).
.include "mc6839_fpops.inc"
#include "mc6839_call.inc"

; ---- f32 dyadic -------------------------------------------------------------

        .global __addsf3
__addsf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leau    6+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FADD
        puls    u,y
        rts

        .global __subsf3
__subsf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leau    6+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FSUB
        puls    u,y
        rts

        .global __mulsf3
__mulsf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leau    6+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FMUL
        puls    u,y
        rts

        .global __divsf3
__divsf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leau    6+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FDIV
        puls    u,y
        rts

; ---- f64 dyadic -------------------------------------------------------------

        .global __adddf3
__adddf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leau    6+FP_SLOT,s
        leay    14+FP_SLOT,s
        FP_CALL FPOP_FADD
        puls    u,y
        rts

        .global __subdf3
__subdf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leau    6+FP_SLOT,s
        leay    14+FP_SLOT,s
        FP_CALL FPOP_FSUB
        puls    u,y
        rts

        .global __muldf3
__muldf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leau    6+FP_SLOT,s
        leay    14+FP_SLOT,s
        FP_CALL FPOP_FMUL
        puls    u,y
        rts

        .global __divdf3
__divdf3:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leau    6+FP_SLOT,s
        leay    14+FP_SLOT,s
        FP_CALL FPOP_FDIV
        puls    u,y
        rts

; ---- square root (monadic; Y → arg) ----------------------------------------

        .global __sqrtsf2
__sqrtsf2:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FSQRT
        puls    u,y
        rts

        .global __sqrtdf2
__sqrtdf2:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FSQRT
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
        FP_ENTRY __mc6839_fpcb_f32
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FABS
        puls    u,y
        rts

        .global __absdf2
__absdf2:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FABS
        puls    u,y
        rts

; ---- negation (monadic; Y → arg) -------------------------------------------
;
; Mirrors __abssf2/__absdf2. Replaces clang's default lowering of
; llvm.fneg.f32 / .f64 (bit-XOR with 0x80000000 / 0x8000000000000000) which
; on MC6809 narrows to an i32/i64 __xorsi3/__xordi3 libcall.

        .global __negsf2
__negsf2:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FNEG
        puls    u,y
        rts

        .global __negdf2
__negdf2:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FNEG
        puls    u,y
        rts
