; MC6839 FP — global data
;
; Two FPCB (Floating Point Control Block) instances, one per precision.
; The MC6839 reads FPCB on every call to determine precision, rounding
; mode, and exception handling.
;
; CTL byte layout (from equates.s:148-155):
;   bits[7:5] = precision  000=single  001=double
;   bit[3]    = CTLNRM     1=normalize subnormals (IEEE) — REQUIRED for fdlibm
;   bits[2:1] = rounding   00=nearest  01=toward-zero  10=+inf  11=-inf
;   bit[0]    = closure    0=projective (IEEE default for binary ops)
;   All other bits = 0 → no traps.
;
; CTLNRM (BIT3) MUST be set: when clear, the ROM's GETARG path skips
; TSTNRM (getput.s:199) and never normalises subnormal inputs, so any
; arithmetic on a subnormal returns garbage.  Sun fdlibm's logf/log10f/
; log2f/cbrtf hit subnormal inputs in their test vectors; with NRM clear
; they fail with 9999-ulp errors.  Enabling NRM is free for normal
; inputs (TSTNRM is a no-op when type ≠ TYDNRM).
;
; __mc6839_fpcb_f32: CTL=0x08: single/nearest/+normalize.
; __mc6839_fpcb_f64: CTL=0x28: double/nearest/+normalize.
; __mc6839_fpcb_f32_trunc: CTL=0x0A: single/toward-zero/+normalize.
; __mc6839_fpcb_f64_trunc: CTL=0x2A: double/toward-zero/+normalize.

        .data
        .global __mc6839_fpcb_f32
__mc6839_fpcb_f32:
        .byte   0x08, 0, 0, 0, 0, 0   ; CTL=0x08: single/nearest/+NRM

        .global __mc6839_fpcb_f64
__mc6839_fpcb_f64:
        .byte   0x28, 0, 0, 0, 0, 0   ; CTL=0x28: double/nearest/+NRM

        .global __mc6839_fpcb_f32_trunc
__mc6839_fpcb_f32_trunc:
        .byte   0x0A, 0, 0, 0, 0, 0   ; CTL=0x0A: single/toward-zero/+NRM

        .global __mc6839_fpcb_f64_trunc
__mc6839_fpcb_f64_trunc:
        .byte   0x2A, 0, 0, 0, 0, 0   ; CTL=0x2A: double/toward-zero/+NRM

