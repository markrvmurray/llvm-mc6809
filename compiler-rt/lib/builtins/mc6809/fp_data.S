; MC6839 FP — global data
;
; Two FPCB (Floating Point Control Block) instances, one per precision.
; The MC6839 reads FPCB on every call to determine precision, rounding
; mode, and exception handling.
;
; CTL byte layout (from nfp09-abi.s):
;   bits[7:5] = precision  000=single  001=double
;   bits[2:1] = rounding   00=nearest  01=toward-zero  10=+inf  11=-inf
;   bit[0]    = closure    0=projective (IEEE default)
;   All other bits = 0 → no traps.
;
; __mc6839_fpcb_f32: CTL=0x00: single/nearest.  .bss (zero-init).
; __mc6839_fpcb_f64: CTL=0x20: double/nearest.  .data.
; __mc6839_fpcb_f32_trunc: CTL=0x02: single/toward-zero.  For FFIXD.
; __mc6839_fpcb_f64_trunc: CTL=0x22: double/toward-zero.  For FFIXD.

        .bss
        .global __mc6839_fpcb_f32
__mc6839_fpcb_f32:
        .space  6           ; CTL=0x00: single/nearest

        .data
        .global __mc6839_fpcb_f64
__mc6839_fpcb_f64:
        .byte   0x20, 0, 0, 0, 0, 0   ; CTL=0x20: double/nearest

        .global __mc6839_fpcb_f32_trunc
__mc6839_fpcb_f32_trunc:
        .byte   0x02, 0, 0, 0, 0, 0   ; CTL=0x02: single/toward-zero

        .global __mc6839_fpcb_f64_trunc
__mc6839_fpcb_f64_trunc:
        .byte   0x22, 0, 0, 0, 0, 0   ; CTL=0x22: double/toward-zero

