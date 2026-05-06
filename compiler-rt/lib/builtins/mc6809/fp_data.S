; MC6839 FP — global data
;
; Two FPCB (Floating Point Control Block) instances, one per precision.
; The MC6839 reads FPCB on every call to determine precision, rounding
; mode, and exception handling.
;
; CTL byte layout:
;   bits[7:5] = precision  000=single  001=double
;   bits[2:1] = rounding   00=nearest (IEEE default)
;   bit[0]    = closure    0=projective (IEEE default)
;   All other bits = 0 → full IEEE defaults, no traps.
;
; __mc6839_fpcb_f32: all-zero → single / nearest / no-traps.  Lives in .bss.
; __mc6839_fpcb_f64: CTL=$20 (double bit set). Lives in .data.
;
; 2^31 constant for unsigned i32→float correction (see fp_conv.S).

        .bss
        .global __mc6839_fpcb_f32
__mc6839_fpcb_f32:
        .space  6           ; 0x00 0x00 0x00 0x00 0x00 0x00

        .data
        .global __mc6839_fpcb_f64
__mc6839_fpcb_f64:
        .byte   0x20, 0, 0, 0, 0, 0   ; CTL=0x20: double/nearest/projective/no-traps

