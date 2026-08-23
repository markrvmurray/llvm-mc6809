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

; Bug #287: opcodes + IEEE 754 high-half constants from shared headers.
.include "mc6839_fpops.inc"
.include "mc6839_abi.inc"
#include "mc6839_call.inc"

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

        .section .text.__floatundidf,"ax",@progbits
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
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leay    28+FP_SLOT,s                 ; Y → hi u32 (positive-as-signed: OK)
        FP_CALL FPOP_FFLTD
        bra     .Lfundidf_hi_mul
.Lfundidf_hi_neg:
        ; modified hi u32 with bit 31 cleared at [S+10..13]
        anda    #0x7F
        sta     10,s
        ldb     29,s
        stb     11,s
        ldd     30,s
        std     12,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leay    10+FP_SLOT,s                 ; Y → modified u32
        FP_CALL FPOP_FFLTD
        ; add 2^31 = 0x41E0_0000_0000_0000 to *X
        ldd     #F64_TWOP31_HI
        std     14,s
        clra
        clrb
        std     16,s
        std     18,s
        std     20,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        ldu     0+FP_SLOT,s                   ; U = caller's sret (= *X target)
        leay    14+FP_SLOT,s                 ; Y → 2^31 const
        FP_CALL FPOP_FADD
.Lfundidf_hi_mul:
        ; ----- step 2: *X *= 2^32  (= 0x41F0_0000_0000_0000) -----
        ldd     #F64_TWOP32_HI
        std     2,s
        clra
        clrb
        std     4,s
        std     6,s
        std     8,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        ldu     0+FP_SLOT,s                   ; U = sret
        leay    2+FP_SLOT,s                  ; Y → 2^32 const
        FP_CALL FPOP_FMUL
        ; ----- step 3: lo_f64 at [S+2..9] via the same FFLTD + maybe-FADD -----
        lda     32,s                 ; lo u32's high byte at [S+32..35]
        bmi     .Lfundidf_lo_neg
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leax    2+FP_SLOT,s                  ; lo_f64 scratch
        leay    32+FP_SLOT,s                 ; Y → lo u32 (positive-as-signed: OK)
        FP_CALL FPOP_FFLTD
        bra     .Lfundidf_combine
.Lfundidf_lo_neg:
        ; modified lo u32 at [S+10..13]; bit 31 cleared
        anda    #0x7F
        sta     10,s
        ldb     33,s
        stb     11,s
        ldd     34,s
        std     12,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leax    2+FP_SLOT,s                  ; lo_f64 scratch
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ; 2^31 const at [S+14..21]
        ldd     #F64_TWOP31_HI
        std     14,s
        clra
        clrb
        std     16,s
        std     18,s
        std     20,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leau    2+FP_SLOT,s                  ; U → lo_f64
        leay    14+FP_SLOT,s                 ; Y → 2^31 const
        FP_CALL FPOP_FADD
.Lfundidf_combine:
        ; ----- step 4: *sret += lo_f64 -----
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        ldu     0+FP_SLOT,s                   ; U = sret
        leay    2+FP_SLOT,s                  ; Y → lo_f64
        FP_CALL FPOP_FADD
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

        .section .text.__floatdidf,"ax",@progbits
        .global __floatdidf
__floatdidf:
        pshs    u,y                  ; arg now at [S+6..13]
        leas    -22,s                ; arg now at [S+28..35]
        stx     ,s                   ; save caller's sret pointer
        ; ----- step 1: hi i32 (SIGNED) → f64 at *X -----
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leay    28+FP_SLOT,s                 ; Y → hi i32 (always treated signed)
        FP_CALL FPOP_FFLTD
        ; ----- step 2: *X *= 2^32 -----
        ldd     #F64_TWOP32_HI
        std     2,s
        clra
        clrb
        std     4,s
        std     6,s
        std     8,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        ldu     0+FP_SLOT,s
        leay    2+FP_SLOT,s
        FP_CALL FPOP_FMUL
        ; ----- step 3: lo u32 → lo_f64 at [S+2..9] (unsigned: same as undidf) -----
        lda     32,s
        bmi     .Lfdidf_lo_neg
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leax    2+FP_SLOT,s
        leay    32+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        bra     .Lfdidf_combine
.Lfdidf_lo_neg:
        anda    #0x7F
        sta     10,s
        ldb     33,s
        stb     11,s
        ldd     34,s
        std     12,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leax    2+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ldd     #F64_TWOP31_HI
        std     14,s
        clra
        clrb
        std     16,s
        std     18,s
        std     20,s
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        leau    2+FP_SLOT,s
        leay    14+FP_SLOT,s
        FP_CALL FPOP_FADD
.Lfdidf_combine:
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        FP_BASE 24
        FP_ENTRY __mc6839_fpcb_f64
        ldu     0+FP_SLOT,s
        leay    2+FP_SLOT,s
        FP_CALL FPOP_FADD
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

        .section .text.__floatundisf,"ax",@progbits
        .global __floatundisf
__floatundisf:
        pshs    u,y                  ; arg [S+6..13]
        leas    -14,s                ; arg now [S+20..27]
        stx     ,s                   ; save caller's sret
        ; ----- step 1: hi u32 → f32 at *X -----
        lda     20,s
        bmi     .Lfundisf_hi_neg
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        bra     .Lfundisf_hi_mul
.Lfundisf_hi_neg:
        anda    #0x7F
        sta     6,s
        ldb     21,s
        stb     7,s
        ldd     22,s
        std     8,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ; 2^31 as f32 = 0x4F00_0000
        ldd     #F32_TWOP31_HI
        std     10,s
        clra
        clrb
        std     12,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        ldu     0+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FADD
.Lfundisf_hi_mul:
        ; *X *= 2^32 as f32 = 0x4F80_0000
        ldd     #F32_TWOP32_HI
        std     2,s
        clra
        clrb
        std     4,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        ldu     0+FP_SLOT,s
        leay    2+FP_SLOT,s
        FP_CALL FPOP_FMUL
        ; lo u32 → lo_f32 at [S+2..5]
        lda     24,s
        bmi     .Lfundisf_lo_neg
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leax    2+FP_SLOT,s
        leay    24+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        bra     .Lfundisf_combine
.Lfundisf_lo_neg:
        anda    #0x7F
        sta     6,s
        ldb     25,s
        stb     7,s
        ldd     26,s
        std     8,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leax    2+FP_SLOT,s
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ldd     #F32_TWOP31_HI
        std     10,s
        clra
        clrb
        std     12,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leau    2+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FADD
.Lfundisf_combine:
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        ldu     0+FP_SLOT,s
        leay    2+FP_SLOT,s
        FP_CALL FPOP_FADD
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

        .section .text.__floatdisf,"ax",@progbits
        .global __floatdisf
__floatdisf:
        pshs    u,y
        leas    -14,s
        stx     ,s
        ; hi i32 (SIGNED) → f32 at *X
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ; *X *= 2^32
        ldd     #F32_TWOP32_HI
        std     2,s
        clra
        clrb
        std     4,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        ldu     0+FP_SLOT,s
        leay    2+FP_SLOT,s
        FP_CALL FPOP_FMUL
        ; lo u32 → lo_f32 at [S+2..5]
        lda     24,s
        bmi     .Lfdisf_lo_neg
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leax    2+FP_SLOT,s
        leay    24+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        bra     .Lfdisf_combine
.Lfdisf_lo_neg:
        anda    #0x7F
        sta     6,s
        ldb     25,s
        stb     7,s
        ldd     26,s
        std     8,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leax    2+FP_SLOT,s
        leay    6+FP_SLOT,s
        FP_CALL FPOP_FFLTD
        ldd     #F32_TWOP31_HI
        std     10,s
        clra
        clrb
        std     12,s
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        leau    2+FP_SLOT,s
        leay    10+FP_SLOT,s
        FP_CALL FPOP_FADD
.Lfdisf_combine:
        ldx     ,s                   ; X = sret (step 3 clobbered X)
        FP_BASE 16
        FP_ENTRY __mc6839_fpcb_f32
        ldu     0+FP_SLOT,s
        leay    2+FP_SLOT,s
        FP_CALL FPOP_FADD
        leas    14,s
        puls    u,y
        rts
        .size __floatdisf, . - __floatdisf

; ===========================================================================
;
;   __fixdfdi(double a) → i64 at *X
;
; Entry: X = sret (8B), [S+2..9] = f64 arg.
;
; Plan (mirroring the i64→f64 algorithm in reverse):
;   1. |a| via sign-bit clear; remember sign.
;   2. scratch = |a| * 2^-32           (FMUL by 2^-32 const)
;   3. hi_i32  = FFIXD_trunc(scratch)  (4-byte i32, in [0, 2^31))
;   4. scratch = (double)hi_i32         (FFLTD; reverse hi to f64)
;   5. scratch *= 2^32
;   6. scratch = |a| - scratch         (FSUB; now residual in [0, 2^32))
;   7. If residual >= 2^31:
;        scratch -= 2^31; lo_i32 = FFIXD_trunc(scratch); lo_u32 = lo_i32 | 0x80000000
;      Else:
;        lo_u32 = FFIXD_trunc(scratch)
;   8. Write hi_i32 to sret[0..3], lo_u32 to sret[4..7].
;   9. If sign negative: two's complement negate the 8 bytes.
;
; Layout (40 bytes locals after pshs u,y):
;   [S+0..1]   saved sret X
;   [S+2]      sign flag (1 byte)
;   [S+4..11]  |a| f64
;   [S+12..15] hi_i32 buffer
;   [S+16..19] lo_u32 buffer
;   [S+20..27] scratch f64
;   [S+28..35] const buf (8 bytes; reused for 2^-32 / 2^32 / 2^31)
;
; arg now at [S+46..53] (after leas -40,s).
; ---------------------------------------------------------------------------

        .section .text.__fixdfdi,"ax",@progbits
        .global __fixdfdi
__fixdfdi:
        pshs    u,y
        leas    -40,s                ; arg now at [S+46..53]
        stx     ,s                   ; save sret

        ; ----- step 1: sign flag + |a| at [S+4..11] -----
        clr     2,s
        lda     46,s                 ; high byte of f64
        bpl     .Lfdd_pos
        inc     2,s                  ; sign = 1
        anda    #0x7F                ; clear sign bit
.Lfdd_pos:
        sta     4,s
        ldb     47,s
        stb     5,s
        ldd     48,s
        std     6,s
        ldd     50,s
        std     8,s
        ldd     52,s
        std     10,s

        ; ----- step 2: scratch = |a| * 2^-32 -----
        ldd     #F64_TWOP_NEG_32_HI              ; 2^-32 high word
        std     28,s
        clra
        clrb
        std     30,s
        std     32,s
        std     34,s
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64
        leax    20+FP_SLOT,s                 ; X = scratch
        leau    4+FP_SLOT,s                  ; U = |a|
        leay    28+FP_SLOT,s                 ; Y = 2^-32
        FP_CALL FPOP_FMUL

        ; ----- step 3: hi_i32 = FFIXD_trunc(scratch) -----
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64_trunc
        leax    12+FP_SLOT,s                 ; X = hi_i32 buf
        leay    20+FP_SLOT,s                 ; Y = scratch
        FP_CALL FPOP_FFIXD

        ; ----- step 4: scratch = (double)hi_i32 -----
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64
        leax    20+FP_SLOT,s                 ; X = scratch
        leay    12+FP_SLOT,s                 ; Y = hi_i32 buf
        FP_CALL FPOP_FFLTD

        ; ----- step 5: scratch *= 2^32 -----
        ldd     #F64_TWOP32_HI
        std     28,s
        clra
        clrb
        std     30,s
        std     32,s
        std     34,s
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64
        leax    20+FP_SLOT,s
        leau    20+FP_SLOT,s
        leay    28+FP_SLOT,s
        FP_CALL FPOP_FMUL

        ; ----- step 6: scratch = |a| - scratch  (residual) -----
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64
        leax    20+FP_SLOT,s
        leau    4+FP_SLOT,s                  ; U = |a|
        leay    20+FP_SLOT,s                 ; Y = scratch (current = hi*2^32)
        FP_CALL FPOP_FSUB

        ; ----- step 7: split lo on residual < or >= 2^31 -----
        ldd     20,s                 ; top 2 bytes of residual
        cmpd    #F64_TWOP31_HI
        bhs     .Lfdd_bit31

        ; Simple: lo_u32 = FFIXD_trunc(residual). X = &lo_u32, Y = residual.
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64_trunc
        leax    16+FP_SLOT,s
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFIXD
        bra     .Lfdd_compose

.Lfdd_bit31:
        ; residual -= 2^31, FFIXD, then OR bit 31 of low byte 0.
        ldd     #F64_TWOP31_HI              ; 2^31 const
        std     28,s
        clra
        clrb
        std     30,s
        std     32,s
        std     34,s
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64
        leax    20+FP_SLOT,s
        leau    20+FP_SLOT,s
        leay    28+FP_SLOT,s
        FP_CALL FPOP_FSUB
        FP_BASE 42
        FP_ENTRY __mc6839_fpcb_f64_trunc
        leax    16+FP_SLOT,s                 ; X = lo_u32 buf
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFIXD
        ; OR bit 31 of the high byte: set bit 7 of [S+16].
        lda     16,s
        ora     #0x80
        sta     16,s

.Lfdd_compose:
        ; Write hi_i32 to sret[0..3], lo_u32 to sret[4..7].
        ldx     ,s                   ; X = sret
        ldd     12,s
        std     ,x
        ldd     14,s
        std     2,x
        ldd     16,s
        std     4,x
        ldd     18,s
        std     6,x

        ; ----- step 9: if sign negative, two's complement negate -----
        tst     2,s
        beq     .Lfdd_done
        ; negate 8-byte i64: invert all bytes, then add 1.
        ldd     ,x
        coma
        comb
        std     ,x
        ldd     2,x
        coma
        comb
        std     2,x
        ldd     4,x
        coma
        comb
        std     4,x
        ldd     6,x
        coma
        comb
        std     6,x
        ; add 1 to LSB (with carry propagation)
        ldd     6,x
        addd    #1
        std     6,x
        bcc     .Lfdd_done
        ldd     4,x
        addd    #1
        std     4,x
        bcc     .Lfdd_done
        ldd     2,x
        addd    #1
        std     2,x
        bcc     .Lfdd_done
        ldd     ,x
        addd    #1
        std     ,x

.Lfdd_done:
        leas    40,s
        puls    u,y
        rts
        .size __fixdfdi, . - __fixdfdi

; ===========================================================================
;
;   __fixsfdi(float a) → i64 at *X
;
; Entry: X = sret (8B), [S+2..5] = f32 arg.
;
; Same algorithm as __fixdfdi but with f32 arithmetic.  The f32
; mantissa is only 24 bits, so anything beyond 2^24 has its low
; bits zero — but the arithmetic still produces a correct
; rounded i64 (within f32 precision).
;
; 2^-32 as f32 = 0x2F800000.
; 2^32  as f32 = 0x4F800000.
; 2^31  as f32 = 0x4F000000.
;
; Layout (32 bytes locals after pshs u,y):
;   [S+0..1]   saved sret X
;   [S+2]      sign flag
;   [S+4..7]   |a| f32
;   [S+12..15] hi_i32 buffer
;   [S+16..19] lo_u32 buffer
;   [S+20..23] scratch f32
;   [S+24..27] const f32 (4 bytes; reused)
;
; arg now at [S+34..37] after leas -32,s.
; ---------------------------------------------------------------------------

        .section .text.__fixsfdi,"ax",@progbits
        .global __fixsfdi
__fixsfdi:
        pshs    u,y
        leas    -32,s                ; arg now at [S+38..41]
        stx     ,s

        ; sign flag + |a|
        clr     2,s
        lda     38,s
        bpl     .Lfsd_pos
        inc     2,s
        anda    #0x7F
.Lfsd_pos:
        sta     4,s
        ldb     39,s
        stb     5,s
        ldd     40,s
        std     6,s                  ; |a| at [S+4..7]

        ; scratch = |a| * 2^-32. 2^-32 f32 = 0x2F800000.
        ldd     #F32_TWOP_NEG_32_HI
        std     24,s
        clra
        clrb
        std     26,s
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32
        leax    20+FP_SLOT,s
        leau    4+FP_SLOT,s
        leay    24+FP_SLOT,s
        FP_CALL FPOP_FMUL

        ; hi_i32 = FFIXD_trunc(scratch)
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32_trunc
        leax    12+FP_SLOT,s
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFIXD

        ; scratch = (float)hi_i32
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32
        leax    20+FP_SLOT,s
        leay    12+FP_SLOT,s
        FP_CALL FPOP_FFLTD

        ; scratch *= 2^32. 2^32 f32 = 0x4F800000.
        ldd     #F32_TWOP32_HI
        std     24,s
        clra
        clrb
        std     26,s
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32
        leax    20+FP_SLOT,s
        leau    20+FP_SLOT,s
        leay    24+FP_SLOT,s
        FP_CALL FPOP_FMUL

        ; scratch = |a| - scratch
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32
        leax    20+FP_SLOT,s
        leau    4+FP_SLOT,s
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FSUB

        ; Check residual >= 2^31? Top 2 bytes >= 0x4F00?
        ldd     20,s
        cmpd    #F32_TWOP31_HI
        bhs     .Lfsd_bit31

        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32_trunc
        leax    16+FP_SLOT,s
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFIXD
        bra     .Lfsd_compose

.Lfsd_bit31:
        ; residual -= 2^31. 2^31 f32 = 0x4F000000.
        ldd     #F32_TWOP31_HI
        std     24,s
        clra
        clrb
        std     26,s
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32
        leax    20+FP_SLOT,s
        leau    20+FP_SLOT,s
        leay    24+FP_SLOT,s
        FP_CALL FPOP_FSUB
        FP_BASE 34
        FP_ENTRY __mc6839_fpcb_f32_trunc
        leax    16+FP_SLOT,s
        leay    20+FP_SLOT,s
        FP_CALL FPOP_FFIXD
        lda     16,s
        ora     #0x80
        sta     16,s

.Lfsd_compose:
        ldx     ,s
        ldd     12,s
        std     ,x
        ldd     14,s
        std     2,x
        ldd     16,s
        std     4,x
        ldd     18,s
        std     6,x

        tst     2,s
        beq     .Lfsd_done
        ldd     ,x
        coma
        comb
        std     ,x
        ldd     2,x
        coma
        comb
        std     2,x
        ldd     4,x
        coma
        comb
        std     4,x
        ldd     6,x
        coma
        comb
        std     6,x
        ldd     6,x
        addd    #1
        std     6,x
        bcc     .Lfsd_done
        ldd     4,x
        addd    #1
        std     4,x
        bcc     .Lfsd_done
        ldd     2,x
        addd    #1
        std     2,x
        bcc     .Lfsd_done
        ldd     ,x
        addd    #1
        std     ,x

.Lfsd_done:
        leas    32,s
        puls    u,y
        rts
        .size __fixsfdi, . - __fixsfdi
