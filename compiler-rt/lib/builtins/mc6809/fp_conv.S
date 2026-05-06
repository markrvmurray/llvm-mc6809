; MC6839 FP — type conversion wrappers
;
; Precision conversion (FMOV, opcode 0x96):
;   U = precision parameter word:  bits[6:4]=src  bits[2:0]=dst
;   Single=0, Double=1.
;   extendsfdf2: U=(single_src<<4)|double_dst = 0x01
;   truncdfsf2:  U=(double_src<<4)|single_dst = 0x10
;
; Integer conversions:
;   FFLTD (0x26): signed i32 → float.  Y→i32 arg, X→float result.
;   FFIXD (0x18): float → signed i32.  Y→float arg, X→i32 result.
;
; i64 conversions (__floatdisf, __floatdidf, __fixsfdi, __fixdfdi):
;   The MC6839 has no native 64-bit integer path. These are approximations:
;   convert the low 32 bits only (correct for |val| < 2^31, otherwise loses
;   precision). TODO: implement correctly via two-step conversion.

        .text

FPOP_FFIXD      = 0x18
FPOP_FMOV       = 0x96
FPOP_FFLTD      = 0x26

; ---- Precision conversion ---------------------------------------------------

        .global __extendsfdf2
__extendsfdf2:
        ; f32 → f64: src=single(0), dst=double(1)  U=0x01
        ; Entry: X=f64 result(8B), [S+2..5]=f32 arg
        ldu     #0x01
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FMOV
        rts

        .global __truncdfsf2
__truncdfsf2:
        ; f64 → f32: src=double(1), dst=single(0)  U=0x10
        ; Entry: X=f32 result(4B), [S+2..9]=f64 arg
        ldu     #0x10
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FMOV
        rts

; ---- Signed integer → float -------------------------------------------------

        .global __floatsisf
__floatsisf:
        ; signed i32 → f32.  Entry: X=f32 result(4B), [S+2..5]=i32
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

        .global __floatsidf
__floatsidf:
        ; signed i32 → f64.  Entry: X=f64 result(8B), [S+2..5]=i32
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

        .global __floatdisf
__floatdisf:
        ; signed i64 → f32.  Entry: X=f32 result(4B), [S+2..9]=i64
        ; TODO: two-step conversion for |val| >= 2^31. For now uses low 32 bits.
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

        .global __floatdidf
__floatdidf:
        ; signed i64 → f64.  Entry: X=f64 result(8B), [S+2..9]=i64
        ; TODO: two-step conversion for |val| >= 2^31.
        leay    6,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

; ---- Unsigned integer → float -----------------------------------------------
; FFLTD treats its argument as signed. For unsigned i32 values with bit 31 set
; (val ≥ 2^31): clear bit 31, convert to float, then add 2^31.

        .global __floatunsisf
__floatunsisf:
        ; unsigned i32 → f32.  Entry: X=f32 result(4B), [S+2..5]=u32
        tst     2,s                 ; test MSB of u32
        bpl     .Lfunsisf_simple    ; bit 31 clear → same as signed
        ; bit 31 set: copy arg to scratch, clear bit 31, convert, then add 2^31
        leas    -4,s                ; scratch slot at [S+0..3] (arg now at [S+6..9])
        ldd     6,s                 ; high 2 bytes of u32
        anda    #0x7F               ; clear bit 31
        std     ,s
        ldd     8,s                 ; low 2 bytes of u32
        std     2,s
        leay    ,s                  ; Y → scratch (modified i32)
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg             ; FFLTD: scratch → f32 at *X
        .byte   FPOP_FFLTD
        ; Reuse scratch for 2^31 = 0x4F000000 (IEEE 754 single).
        ; Then FADD: *X (current result) + 2^31 → *X.
        ldd     #0x4F00             ; 2^31 f32: high 2 bytes = 0x4F00
        std     ,s
        clra
        clrb
        std     2,s                 ; low 2 bytes = 0x0000
        leau    ,x                  ; U → *X (current result = FADD arg1)
        leay    ,s                  ; Y → 2^31 constant on stack
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   0x00                ; FPOP_FADD
        leas    4,s
        rts
.Lfunsisf_simple:
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

        .global __floatunsidf
__floatunsidf:
        ; unsigned i32 → f64.  Entry: X=f64 result(8B), [S+2..5]=u32
        tst     2,s
        bpl     .Lfunsidf_simple
        ; Allocate 12 bytes: 4 for modified i32 + 8 for f64 constant
        leas    -12,s               ; arg now at [S+14..17]
        ldd     14,s                ; high 2 bytes of u32
        anda    #0x7F               ; clear bit 31
        std     ,s
        ldd     16,s                ; low 2 bytes
        std     2,s
        leay    ,s                  ; Y → modified i32
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD          ; scratch i32 → f64 at *X
        ; Write 2^31 = 0x41E0000000000000 into scratch+4..11
        ldd     #0x41E0
        std     4,s
        clra
        clrb
        std     6,s
        std     8,s
        std     10,s
        leau    ,x                  ; U → *X (FADD arg1)
        leay    4,s                 ; Y → 2^31 f64 constant
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   0x00                ; FPOP_FADD
        leas    12,s
        rts
.Lfunsidf_simple:
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

        .global __floatundisf
__floatundisf:
        ; unsigned i64 → f32.  TODO: implement properly.
        ; Approximation: use low 32 bits only.
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

        .global __floatundidf
__floatundidf:
        ; unsigned i64 → f64.  TODO: implement properly.
        leay    6,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        rts

; ---- Float → signed integer -------------------------------------------------

        .global __fixsfsi
__fixsfsi:
        ; f32 → signed i32.  Entry: X=i32 result(4B), [S+2..5]=f32
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFIXD
        rts

        .global __fixdfsi
__fixdfsi:
        ; f64 → signed i32.  Entry: X=i32 result(4B), [S+2..9]=f64
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFIXD
        rts

        .global __fixsfdi
__fixsfdi:
        ; f32 → signed i64.  Entry: X=i64 result(8B), [S+2..5]=f32
        ; TODO: implement i64 result. For now writes i32 into low 4 bytes of result.
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFIXD
        rts

        .global __fixdfdi
__fixdfdi:
        ; f64 → signed i64.  Entry: X=i64 result(8B), [S+2..9]=f64
        ; TODO: implement i64 result.
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFIXD
        rts
