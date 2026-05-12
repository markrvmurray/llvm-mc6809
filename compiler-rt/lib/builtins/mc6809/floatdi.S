; __float{,un}{di,si}df / __float{,un}{di,si}sf — i64/u64 → f64/f32
; full-range conversion wrappers for MC6809 / MC6839.
;
; Bug #286: the original asm wrappers in fp_conv.S used FFLTD on
; only the low 32 bits of the 8-byte i64 stack slot (leay 10,s
; instead of leay 6,s), interpreting them as signed i32. Anything
; outside the i32 range silently miscompiled. The TODO comments
; "TODO: full range" in fp_conv.S marked the gap.
;
; Strategy: split into high i32 (signed) and low u32 (unsigned).
; Compute hi_fp * 2^32 + lo_fp using the MC6839 ROM directly
; (FFLTD / FMUL / FADD) — no compiler-rt sub-calls.
;
;   __floatdidf(i64)   = (double)(int32_t)(a>>32) * 2^32 + (double)(uint32_t)a
;   __floatundidf(u64) = same but hi is unsigned
;   __floatdisf(i64)   = same shape, f32 result
;   __floatundisf(u64) = same shape, f32 result
;
; Each wrapper handles the signed-i32 conversion limitation of FFLTD
; for the low-half u32: if the bit-31 flag is set, the value must be
; converted as if it were 2^31 smaller, then FADD'd by 2^31. This is
; the same trick __floatunsidf in fp_conv.S already uses.

        .text

FPOP_FADD       = 0x00
FPOP_FMUL       = 0x04
FPOP_FFLTD      = 0x26

; ===========================================================================
;
;   __floatundidf(uint64_t u) → f64 at *X
;
; Entry: X = sret pointer (8B), [S+2..9] = u64 arg (big-endian, MSB at S+2).
;
; Plan:
;   1. *X = (double)(u32)(u >> 32)                   [hi as unsigned u32]
;   2. *X *= 2^32
;   3. lo_f64 = (double)(u32)(u)                     [lo as unsigned u32]
;   4. *X += lo_f64
;
; Stack layout after `pshs u,y; leas -24,s`:
;   [S+0..1]   saved sret pointer (caller's X)
;   [S+2..9]   lo_f64 scratch (reused; first as 2^32 const, then as lo_f64)
;   [S+10..13] modified low u32 (high bit cleared) for FFLTD's signed input
;   [S+14..21] scratch for 2^31 const (only when high u32 has bit 31 set)
;              -- but we'll allocate this on demand to save the common case.
;   arg now at [S+30..37]
;
; Simpler layout actually used:
;   [S+0..1]   saved sret
;   [S+2..9]   lo_f64 (and earlier 2^32 const)
;   [S+10..13] modified u32 scratch
;   [S+14..21] 2^31 const (only when bit31 is set; computed in place)
; ---------------------------------------------------------------------------

        .global __floatundidf
__floatundidf:
        pshs    u,y                  ; arg now at [S+6..13]
        leas    -22,s                ; arg now at [S+28..35]
        stx     ,s                   ; save caller's sret pointer
        ; ----- step 1: hi half u32 → f64 at *X (X already holds sret) -----
        ; Hi u32 is at [S+28..31].  If bit 31 is set, use the
        ; "clear-bit-31 + FADD 2^31" trick; otherwise direct FFLTD.
        lda     28,s
        bmi     .Lfundidf_hi_neg
        leay    28,s                 ; Y → hi u32 (positive-as-signed: OK)
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD           ; *X = (double)hi_i32
        bra     .Lfundidf_hi_mul
.Lfundidf_hi_neg:
        ; modified hi u32 with bit 31 cleared at [S+10..13]
        anda    #0x7F
        sta     10,s
        ldb     29,s
        stb     11,s
        ldd     30,s
        std     12,s
        leay    10,s                 ; Y → modified u32
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD           ; *X = (double)(hi & 0x7FFFFFFF)
        ; add 2^31 = 0x41E0_0000_0000_0000 to *X
        ldd     #0x41E0
        std     14,s
        clra
        clrb
        std     16,s
        std     18,s
        std     20,s
        ldu     ,s                   ; U = caller's sret (= *X target)
        leay    14,s                 ; Y → 2^31 const
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD            ; *sret += 2^31
.Lfundidf_hi_mul:
        ; ----- step 2: *X *= 2^32  (= 0x41F0_0000_0000_0000) -----
        ldd     #0x41F0
        std     2,s
        clra
        clrb
        std     4,s
        std     6,s
        std     8,s
        ldu     ,s                   ; U = sret
        leay    2,s                  ; Y → 2^32 const
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FMUL            ; *sret *= 2^32
        ; ----- step 3: lo_f64 at [S+2..9] via the same FFLTD + maybe-FADD -----
        lda     32,s                 ; lo u32's high byte at [S+32..35]
        bmi     .Lfundidf_lo_neg
        leax    2,s                  ; lo_f64 scratch
        leay    32,s                 ; Y → lo u32 (positive-as-signed: OK)
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        bra     .Lfundidf_combine
.Lfundidf_lo_neg:
        ; modified lo u32 at [S+10..13]; bit 31 cleared
        anda    #0x7F
        sta     10,s
        ldb     33,s
        stb     11,s
        ldd     34,s
        std     12,s
        leax    2,s                  ; lo_f64 scratch
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD           ; lo_f64 = (lo & 0x7FFFFFFF)
        ; 2^31 const at [S+14..21]
        ldd     #0x41E0
        std     14,s
        clra
        clrb
        std     16,s
        std     18,s
        std     20,s
        leau    2,s                  ; U → lo_f64
        leay    14,s                 ; Y → 2^31 const
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD            ; lo_f64 += 2^31
.Lfundidf_combine:
        ; ----- step 4: *sret += lo_f64 -----
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        ldu     ,s                   ; U = sret
        leay    2,s                  ; Y → lo_f64
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD
        leas    22,s
        puls    u,y
        rts
        .size __floatundidf, . - __floatundidf

; ===========================================================================
;
;   __floatdidf(int64_t a) → f64 at *X
;
; Differs from __floatundidf only in step 1: the high half is treated
; as SIGNED int32, so it goes through FFLTD directly without the
; bit-31 dance.  The low half is still treated as unsigned u32
; (after sign-extension is folded into the high half).
; ---------------------------------------------------------------------------

        .global __floatdidf
__floatdidf:
        pshs    u,y                  ; arg now at [S+6..13]
        leas    -22,s                ; arg now at [S+28..35]
        stx     ,s                   ; save caller's sret pointer
        ; ----- step 1: hi i32 (SIGNED) → f64 at *X -----
        leay    28,s                 ; Y → hi i32 (always treated signed)
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        ; ----- step 2: *X *= 2^32 -----
        ldd     #0x41F0
        std     2,s
        clra
        clrb
        std     4,s
        std     6,s
        std     8,s
        ldu     ,s
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FMUL
        ; ----- step 3: lo u32 → lo_f64 at [S+2..9] (unsigned: same as undidf) -----
        lda     32,s
        bmi     .Lfdidf_lo_neg
        leax    2,s
        leay    32,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        bra     .Lfdidf_combine
.Lfdidf_lo_neg:
        anda    #0x7F
        sta     10,s
        ldb     33,s
        stb     11,s
        ldd     34,s
        std     12,s
        leax    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        ldd     #0x41E0
        std     14,s
        clra
        clrb
        std     16,s
        std     18,s
        std     20,s
        leau    2,s
        leay    14,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD
.Lfdidf_combine:
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        ldu     ,s
        leay    2,s
        ldd     #__mc6839_fpcb_f64
        lbsr    __fpreg
        .byte   FPOP_FADD
        leas    22,s
        puls    u,y
        rts
        .size __floatdidf, . - __floatdidf

; ===========================================================================
;
;   __floatundisf(uint64_t u) → f32 at *X
;
; Same algorithm as __floatundidf but with f32 throughout.  The f32
; precision is selected by passing __mc6839_fpcb_f32 to __fpreg.
; Layout:
;   [S+0..1]   saved sret (4B target)
;   [S+2..5]   lo_f32 scratch
;   [S+6..9]   modified-u32 scratch
;   [S+10..13] 2^31 / 2^32 const (4B for f32)
;   arg now at [S+20..27]
;
; (After pshs u,y; leas -14,s.)
; ---------------------------------------------------------------------------

        .global __floatundisf
__floatundisf:
        pshs    u,y                  ; arg [S+6..13]
        leas    -14,s                ; arg now [S+20..27]
        stx     ,s                   ; save caller's sret
        ; ----- step 1: hi u32 → f32 at *X -----
        lda     20,s
        bmi     .Lfundisf_hi_neg
        leay    20,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        bra     .Lfundisf_hi_mul
.Lfundisf_hi_neg:
        anda    #0x7F
        sta     6,s
        ldb     21,s
        stb     7,s
        ldd     22,s
        std     8,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        ; 2^31 as f32 = 0x4F00_0000
        ldd     #0x4F00
        std     10,s
        clra
        clrb
        std     12,s
        ldu     ,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
.Lfundisf_hi_mul:
        ; *X *= 2^32 as f32 = 0x4F80_0000
        ldd     #0x4F80
        std     2,s
        clra
        clrb
        std     4,s
        ldu     ,s
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FMUL
        ; lo u32 → lo_f32 at [S+2..5]
        lda     24,s
        bmi     .Lfundisf_lo_neg
        leax    2,s
        leay    24,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        bra     .Lfundisf_combine
.Lfundisf_lo_neg:
        anda    #0x7F
        sta     6,s
        ldb     25,s
        stb     7,s
        ldd     26,s
        std     8,s
        leax    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        ldd     #0x4F00
        std     10,s
        clra
        clrb
        std     12,s
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
.Lfundisf_combine:
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        ldu     ,s
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
        leas    14,s
        puls    u,y
        rts
        .size __floatundisf, . - __floatundisf

; ===========================================================================
;
;   __floatdisf(int64_t a) → f32 at *X
;
;   Same shape as __floatundisf with signed-hi handling.
; ---------------------------------------------------------------------------

        .global __floatdisf
__floatdisf:
        pshs    u,y
        leas    -14,s
        stx     ,s
        ; hi i32 (SIGNED) → f32 at *X
        leay    20,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        ; *X *= 2^32
        ldd     #0x4F80
        std     2,s
        clra
        clrb
        std     4,s
        ldu     ,s
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FMUL
        ; lo u32 → lo_f32 at [S+2..5]
        lda     24,s
        bmi     .Lfdisf_lo_neg
        leax    2,s
        leay    24,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        bra     .Lfdisf_combine
.Lfdisf_lo_neg:
        anda    #0x7F
        sta     6,s
        ldb     25,s
        stb     7,s
        ldd     26,s
        std     8,s
        leax    2,s
        leay    6,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FFLTD
        ldd     #0x4F00
        std     10,s
        clra
        clrb
        std     12,s
        leau    2,s
        leay    10,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
.Lfdisf_combine:
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        ldu     ,s
        leay    2,s
        ldd     #__mc6839_fpcb_f32
        lbsr    __fpreg
        .byte   FPOP_FADD
        leas    14,s
        puls    u,y
        rts
        .size __floatdisf, . - __floatdisf
