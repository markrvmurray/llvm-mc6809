; MC6839 FP ROM ABI constants — FPCB layout, CTL bitfields, FMOV/FCMP
; parameter words, and IEEE 754 power-of-2 bit patterns used by the
; ROM wrappers.
;
; Bug #287 (widened scope): the original bug filing addressed FPOP_*
; opcode duplication.  Its widened scope captures the OTHER magic
; numbers scattered across fp_data.S, fp_conv.S, fp_cmp.S, and
; floatdi.S: FPCB byte offsets, CTL byte bit fields, FMOV/FCMP
; parameter encodings, and the 16-bit IEEE 754 high-half words for
; 2^31 / 2^32 / 2^-32 in f32 and f64.  Same typo-risk class as the
; FPOP_* values.
;
; Canonical source: ~/Documents/MC6839_ROM/abi/nfp09-abi.s.

; ---- FPCB structure offsets -----------------------------------------
; The Floating Point Control Block is a 6-byte structure in user memory
; that holds the ROM's per-thread state.  D is loaded with its address
; before each `lbsr __fpreg` call.
;
;   offset 0   FP_CTRL    — precision/rounding/closure/NRM mode byte
;   offset 1   FP_TRAPEN  — trap-enable mask (exceptions that vector)
;   offset 2   FP_STAT    — sticky exception status (accumulated)
;   offset 3   FP_SSTAT   — secondary status (per-op)
;   offset 4   FP_TRAPVEC — pointer to trap handler (16-bit)
;   offset 6   — end of FPCB

FPCB_FP_CTRL    = 0
FPCB_FP_TRAPEN  = 1
FPCB_FP_STAT    = 2
FPCB_FP_SSTAT   = 3
FPCB_FP_TRAPVEC = 4
SIZEOF_FPCB     = 6

; ---- FP_STAT byte bitfields (per nfp09-abi.s:516-540) ---------------
; bit 0: IOP — invalid operation
; bit 1: OVF — overflow
; bit 2: UNF — underflow
; bit 3: DZ  — division by zero
; bit 4: UN  — unordered (MC6839 extension; no C99 equivalent)
; bit 5: IOV — integer overflow (MC6839 extension)
; bit 6: INX — inexact result
; bit 7: reserved
;
; Bug #289 (2026-05-24): consumed by compiler-rt/lib/builtins/mc6809/fenv.c
; which exposes these as the standard C99 FE_INVALID / FE_OVERFLOW etc.
; macros via picolibc's machine/fenv.h.  Status is sticky — the ROM
; never auto-clears; user code clears via feclearexcept().

FP_STAT_IOP         = 0x01      ; invalid operation
FP_STAT_OVF         = 0x02      ; overflow
FP_STAT_UNF         = 0x04      ; underflow
FP_STAT_DZ          = 0x08      ; division by zero
FP_STAT_UN          = 0x10      ; unordered
FP_STAT_IOV         = 0x20      ; integer overflow
FP_STAT_INX         = 0x40      ; inexact result

FP_STAT_ALL         = 0x7F      ; OR of all defined bits

; ---- FP_CTRL byte bitfields -----------------------------------------
; bits 7:5   precision    (0=single, 1=double, 2=extended, 3=extfs, 4=extfd)
; bit  4     reserved
; bit  3     NRM          — normalize subnormal operands (recommended on)
; bits 2:1   rounding     (0=nearest, 1=zero, 2=+Inf, 3=-Inf)
; bit  0     closure      (0=projective, 1=affine)

FP_CTRL_PROJ        = 0x00
FP_CTRL_AFF         = 0x01

FP_CTRL_RN          = 0x00      ; round to nearest (ties to even)
FP_CTRL_RZ          = 0x02      ; round toward zero
FP_CTRL_RP          = 0x04      ; round toward +Infinity
FP_CTRL_RM          = 0x06      ; round toward -Infinity

FP_CTRL_NRM         = 0x08      ; normalize subnormals (bit 3)

FP_CTRL_SINGLE      = 0x00      ; precision in bits 7:5
FP_CTRL_DOUBLE      = 0x20
FP_CTRL_EXT         = 0x40
FP_CTRL_EXTFS       = 0x60
FP_CTRL_EXTFD       = 0x80

; ---- FMOV parameter word (passed in U for FPOP_FMOV) ----------------
; Bits 7:4   src precision (FP_PREC_*)
; Bits 3:0   dst precision
; The encoding doesn't use FP_CTRL_* shifts because FMOV's parameter
; word is its own format.
;
; FMOV widens (single→double) or narrows (double→single) per these bytes.

FMOV_SRC_SINGLE     = 0x00
FMOV_SRC_DOUBLE     = 0x10
FMOV_DST_SINGLE     = 0x00
FMOV_DST_DOUBLE     = 0x01

FMOV_S_TO_D         = 0x01      ; FMOV_SRC_SINGLE | FMOV_DST_DOUBLE
FMOV_D_TO_S         = 0x10      ; FMOV_SRC_DOUBLE | FMOV_DST_SINGLE

; ---- FCMP parameter word (passed in X for FPOP_FCMP) ----------------
; Bits 6:4   arg1 precision (0=single, 1=double)
; Bits 2:0   arg2 precision
; (FPOP_FCMP has FPOP_M set; mixed-precision compares are valid but
; the wrappers here only do same-precision compares.)
;
; Result CC after RDYXIT (per fp_cmp.S header):
;   Z=1   equal
;   N=1   less than
;   C=1   unordered (NaN)
;   all 0 greater than

FCMP_S_S            = 0x0000
FCMP_D_D            = 0x0011

; ---- IEEE 754 high-half words for power-of-2 constants --------------
; The MC6809 ROM wrappers materialize these via `ldd #VALUE` and write
; them as the high 16 bits of a 4- or 8-byte FP buffer in stack memory.
; Only the high half is non-zero; lower halves are zeroed by separate
; `clrw`/`std` of the result buffer.
;
; f64 layout: [sign:1][exp:11][mantissa:52]; high-half is bits 63..48.
;   2^31     :  +1.0 * 2^31 → biased exp 1023+31 = 1054 (0x41E); HI = 0x41E0
;   2^32     :  +1.0 * 2^32 → biased exp 1023+32 = 1055 (0x41F); HI = 0x41F0
;   2^-32    :  +1.0 * 2^-32 → biased exp 1023-32 =  991 (0x3DF); HI = 0x3DF0
; f32 layout: [sign:1][exp:8][mantissa:23]; high-half is bits 31..16.
;   2^31     :  biased exp 127+31 = 158 (0x9E); HI = 0x4F00
;   2^32     :  biased exp 127+32 = 159 (0x9F); HI = 0x4F80
;   2^-32    :  biased exp 127-32 =  95 (0x5F)... wait: 127-32=95=0x5F.
;                 Actually 2^-32 in f32 is biased exp 127-32 = 0x5F.
;                 The constant 0x2F80 in floatdi.S encodes 2^-32 as a
;                 *DENORMAL* (sign 0, biased exp 0x5F=95)?  Let me check.
;                   0x2F80_0000 → sign 0, exp 0x5F<<1|MSB=0 → exp=0x5F=95;
;                   95-127=-32 ✓ — biased exp 0x5F = 95, but stored
;                 LEFT-SHIFTED 1 makes mantissa MSB the implicit 1 bit.
;                 0x2F80 = 0010_1111_1000_0000: sign 0, exp 01011111 (0x5F=95),
;                 mantissa 10000000_00000000_0_0_0_0_0_0_0 (high bit set
;                 followed by zeros).  Since IEEE 754 uses an implicit
;                 leading 1, this is +1.0 * 2^(95-127) = 2^-32. ✓

F64_TWOP31_HI       = 0x41E0    ; high 16 bits of (double)2^31
F64_TWOP32_HI       = 0x41F0    ; high 16 bits of (double)2^32
F64_TWOP_NEG_32_HI  = 0x3DF0    ; high 16 bits of (double)2^-32

F32_TWOP31_HI       = 0x4F00    ; high 16 bits of (float)2^31
F32_TWOP32_HI       = 0x4F80    ; high 16 bits of (float)2^32
F32_TWOP_NEG_32_HI  = 0x2F80    ; high 16 bits of (float)2^-32
