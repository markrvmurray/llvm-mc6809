; MC6839 FP — type conversion wrappers
;
; All wrappers follow the textbook register-ABI pattern: pshs u,y at entry,
; puls u,y; rts after the FPOP byte.  After pshs u,y (4 bytes), the arg
; shifts from [S+2..N] to [S+6..N+4].
;
; FMOV (precision conversion) is the only operation here that uses U as
; data (a precision parameter word), not as an arg pointer.  We still save
; the caller's U via pshs first, then ldu the precision word.
;
; FFLTD/FFIXD are monadic: only Y points to the arg.  U is unused by the
; ROM for these.
;
; Bug #295: FPCB pointer load is PIE — `leay __sym,pc; tfr y,d` puts
; the FPCB address in D via Y (X must be preserved as the sret buffer
; for >2-byte returns).  The Y-as-scratch pattern is placed BEFORE the
; final `leay N,s` so Y ends up holding the arg pointer at the ROM call.

        .text

; Bug #287: opcodes + FMOV/IEEE constants from shared headers.
.include "mc6839_fpops.inc"
#include "mc6839_call.inc"
.include "mc6839_abi.inc"

; ---- precision conversion (FMOV; U = precision word) ----------------------

        .global __extendsfdf2
__extendsfdf2:
        ; f32 → f64.  Precision word: src=single(0), dst=double(1).
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        ldu     #FMOV_S_TO_D
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FMOV
        puls    u,y
        rts

        .global __truncdfsf2
__truncdfsf2:
        ; f64 → f32.  Precision word: src=double(1), dst=single(0).
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        ldu     #FMOV_D_TO_S
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FMOV
        puls    u,y
        rts

; ---- signed integer → float (FFLTD) ---------------------------------------

        .global __floatsisf
__floatsisf:
        ; signed i32 → f32.  Y → i32 arg.
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        puls    u,y
        rts

        .global __floatsidf
__floatsidf:
        ; signed i32 → f64.
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        puls    u,y
        rts

        ; __floatdisf / __floatdidf moved to mc6809/floatdi.S
        ; (Bug #286: full i64 range via hi*2^32 + lo composition.)

; ---- unsigned integer → float ---------------------------------------------
; FFLTD treats input as signed.  For u32 with bit31 set we clear bit31,
; convert, then add 2^31.  Allocates a scratch buffer on the stack.

        .global __floatunsisf
__floatunsisf:
        ; u32 → f32.  Entry: X=f32 result(4B), [S+2..5]=u32.
        pshs    u,y                 ; arg now at [S+6..9]
        tst     6,s
        bpl     .Lfunsisf_simple
        ; bit31 set: scratch slot at [S+0..3]; arg now at [S+10..13]
        leas    -4,s
        ldd     10,s
        anda    #0x7F
        std     ,s
        ldd     12,s
        std     2,s
        FP_ENTRY __mc6839_fpcb_f32
        leay    0+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ; result now in *X.  Add 2^31 = 0x4F000000.
        ldd     #F32_TWOP31_HI
        std     ,s
        clra
        clrb
        std     2,s
        FP_BASE 6
        FP_ENTRY __mc6839_fpcb_f32
        leau    ,x                  ; U → *X (FADD arg1)
        leay    0+FP_SLOT,s                  ; Y → 2^31 constant
        FP_CALL FPOP_FADD
        leas    4,s
        puls    u,y
        rts
.Lfunsisf_simple:
        FP_ENTRY __mc6839_fpcb_f32
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        puls    u,y
        rts

        .global __floatunsidf
__floatunsidf:
        ; u32 → f64.  Entry: X=f64 result(8B), [S+2..5]=u32.
        pshs    u,y
        tst     6,s
        bpl     .Lfunsidf_simple
        ; scratch i32(4) + f64 const(8) = 12 bytes; arg now at [S+18..21]
        leas    -12,s
        ldd     18,s
        anda    #0x7F
        std     ,s
        ldd     20,s
        std     2,s
        FP_ENTRY __mc6839_fpcb_f64
        leay    0+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ; 2^31 as f64 = 0x41E0000000000000
        ldd     #F64_TWOP31_HI
        std     4,s
        clra
        clrb
        std     6,s
        std     8,s
        std     10,s
        FP_BASE 14
        FP_ENTRY __mc6839_fpcb_f64
        leau    ,x
        leay    4+FP_SLOT,s
        FP_CALL FPOP_FADD
        leas    12,s
        puls    u,y
        rts
.Lfunsidf_simple:
        FP_ENTRY __mc6839_fpcb_f64
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        puls    u,y
        rts

        ; __floatundisf / __floatundidf moved to mc6809/floatdi.S
        ; (Bug #286: full u64 range; supersedes the Bug #264 C
        ; wrapper in mc6809/floatundi.c which has been removed.)

; ---- float → signed integer (FFIXD; truncation toward zero) ---------------

        .global __fixsfsi
__fixsfsi:
        ; f32 → signed i32.
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32_trunc
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFIXD
        puls    u,y
        rts

        .global __fixdfsi
__fixdfsi:
        ; f64 → signed i32.
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64_trunc
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFIXD
        puls    u,y
        rts

        ; __fixsfdi / __fixdfdi moved to mc6809/floatdi.S — Bug #286
        ; part 2: full i64 range via hi*2^32 + lo decomposition.
