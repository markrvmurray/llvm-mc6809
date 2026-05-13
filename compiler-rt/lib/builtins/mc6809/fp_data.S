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

; Bug #287: FP_CTRL_* bitfield constants from shared header.
.include "mc6839_abi.inc"

        .data
        .global __mc6839_fpcb_f32
__mc6839_fpcb_f32:
        .byte   FP_CTRL_SINGLE | FP_CTRL_NRM | FP_CTRL_RN, 0, 0, 0, 0, 0

        .global __mc6839_fpcb_f64
__mc6839_fpcb_f64:
        .byte   FP_CTRL_DOUBLE | FP_CTRL_NRM | FP_CTRL_RN, 0, 0, 0, 0, 0

        .global __mc6839_fpcb_f32_trunc
__mc6839_fpcb_f32_trunc:
        .byte   FP_CTRL_SINGLE | FP_CTRL_NRM | FP_CTRL_RZ, 0, 0, 0, 0, 0

        .global __mc6839_fpcb_f64_trunc
__mc6839_fpcb_f64_trunc:
        .byte   FP_CTRL_DOUBLE | FP_CTRL_NRM | FP_CTRL_RZ, 0, 0, 0, 0, 0

