; MC6839 FP ROM dispatch opcodes — single source of truth.
;
; Bug #287: prior to this header, FPOP_* equates were duplicated across
; fp_arith.S, fp_bcd.S, fp_cmp.S, fp_conv.S, and floatdi.S — five files,
; same constants, opportunity for silent typos.  A wrong FPOP_FMUL byte
; (0x14 instead of 0x04) cost 30+ minutes of debugging during Bug #286.
;
; Canonical source: ~/Documents/MC6839_ROM/abi/nfp09-abi.s line ~847.
; Values below mirror the canonical equates' computed offsets, but
; expressed as absolute hex so each .S file's emitted FPOP byte is
; reviewable inline (and so this header is independent of the canonical
; MC6839 ROM tree being present at build time).
;
; FPOP_M (bit 7) = "mixed precision arguments" — set on FCMP/FMOV
; FPOP_T (bit 6) = "trap on unordered" — not used by our wrappers
;
; Layout (canonical, derived):
;   FPOP_FADD    = 0x00
;   FPOP_FSUB    = FPOP_FADD + 2     = 0x02
;   FPOP_FMUL    = FPOP_FSUB + 2     = 0x04
;   FPOP_FDIV   = FPOP_FMUL + 2     = 0x06
;   FPOP_FREM    = FPOP_FDIV + 2     = 0x08
;   FPOP_FCMP    = FPOP_FREM + 2 + FPOP_M           = 0x8A
;   FPOP_FSQRT   = FPOP_FREM + 10                   = 0x12
;   FPOP_FINT    = FPOP_FSQRT + 2                   = 0x14
;   FPOP_FFIXS   = FPOP_FINT + 2                    = 0x16
;   FPOP_FFIXD   = FPOP_FFIXS + 2                   = 0x18
;   FPOP_FMOV    = FPOP_FFIXD + 2 + FPOP_M          = 0x9A
;   FPOP_BINDEC  = FPOP_FFIXD + 4                   = 0x1C
;   FPOP_FABS    = FPOP_BINDEC + 2                  = 0x1E
;   FPOP_FNEG    = FPOP_FABS + 2                    = 0x20
;   FPOP_DECBIN  = FPOP_FNEG + 2                    = 0x22
;   FPOP_FFLTS   = FPOP_DECBIN + 2                  = 0x24
;   FPOP_FFLTD   = FPOP_FFLTS + 2                   = 0x26
;
; Every wrapper .S file in compiler-rt/lib/builtins/mc6809/ that
; emits MC6839 ROM dispatch bytes (`.byte FPOP_<op>`) MUST
; `.include "mc6839_fpops.inc"` rather than re-declaring equates.

; Modifier bits on FPOP byte
FPOP_M          = 0x80          ; bit 7: mixed-precision arguments
FPOP_T          = 0x40          ; bit 6: trap on unordered

; Arithmetic dyadic
FPOP_FADD       = 0x00          ; FADD
FPOP_FSUB       = 0x02          ; FSUB
FPOP_FMUL       = 0x04          ; FMUL
FPOP_FDIV       = 0x06          ; FDIV
FPOP_FREM       = 0x08          ; FREM

; Compare (mixed-precision dyadic)
FPOP_FCMP       = 0x8A          ; FCMP   (= 0x0A + FPOP_M)
FPOP_FTCMP      = 0xCC          ; FCMP trapping (= 0x0C + FPOP_M + FPOP_T)
FPOP_FPCMP      = 0x8E          ; FPCMP  (= 0x0E + FPOP_M)
FPOP_FTPCMP     = 0xD0          ; FPCMP trapping (= 0x10 + FPOP_M + FPOP_T)

; Arithmetic monadic / sqrt / integer-aware
FPOP_FSQRT      = 0x12          ; FSQRT
FPOP_FINT       = 0x14          ; FINT
FPOP_FFIXS      = 0x16          ; FFIXS   (f → int16)
FPOP_FFIXD      = 0x18          ; FFIXD   (f → int32)

; Move (mixed-precision)
FPOP_FMOV       = 0x9A          ; FMOV    (= 0x1A + FPOP_M)

; BCD / decimal conversion
FPOP_BINDEC     = 0x1C          ; BINDEC  (f → BCD)
FPOP_DECBIN     = 0x22          ; DECBIN  (BCD → f)

; Sign monadic
FPOP_FABS       = 0x1E          ; FABS
FPOP_FNEG       = 0x20          ; FNEG

; Integer → float
FPOP_FFLTS      = 0x24          ; FFLTS   (int16 → f)
FPOP_FFLTD      = 0x26          ; FFLTD   (int32 → f)
