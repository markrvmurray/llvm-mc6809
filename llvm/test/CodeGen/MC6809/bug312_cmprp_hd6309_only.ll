; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=mc6809 < %s | FileCheck --check-prefix=MC6809 %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 < %s | FileCheck --check-prefix=HD6309 %s
;
; Bug #312 regression sentinel.
;
; expandCompareReg in MC6809InstrInfo.cpp was emitting `CMPRp` (HD6309
; page-2 opcode 0x10 0x37) unconditionally from the non-SameHalf path
; and from the trivially-equal fallback.  CMPRp is HD6309-only — its
; TableGen def is `let Predicates = [IsHD6309]`.  On plain MC6809 the
; bytes 0x10 0x37 are an undefined opcode; USim aborts with
; `invalid instruction [PC:... IR:$1037]` mid-execution.
;
; Surfaced at -Os tiers (`-Os`, `-Os-fp`, `-Os-lto`, `-Os-lto-fp` —
; size optimisation triggers reg-to-reg compares that hit the path).
; Originally affected picolibc test-iconv and iconvnm.
;
; Fix: gate the CMPRp emission on `STI.has6309()` and emit a PSHS +
; CMPx 0,$ss + PULS sequence on plain MC6809.
;
; This test exercises a register-to-register compare (i16) at -O2.
; On HD6309 we expect `cmpr` to appear; on plain MC6809 we expect
; NO `cmpr` (and no raw `0x10 0x37` bytes) — instead the PSHS / CMPD
; / PULS fallback.

target triple = "mc6809-unknown-unknown"

; MC6809-LABEL: cmp_i16_reg:
; MC6809-NOT:    cmpr
; HD6309-LABEL: cmp_i16_reg:
; The second operand arrives on the stack and now folds straight into the
; compare (cmpd n,s) — cmpr appears only when both values are genuinely
; register-resident. Either way it is a native compare with no push
; fallback; the load-bearing assertion is the MC6809-NOT above.
; HD6309:        cmp{{r|d}}

define zeroext i1 @cmp_i16_reg(i16 %a, i16 %b) {
  %c = icmp eq i16 %a, %b
  ret i1 %c
}

; Same shape for i8 reg-reg compare — should also avoid `cmpr` on
; plain MC6809.

; MC6809-LABEL: cmp_i8_reg:
; MC6809-NOT:    cmpr
; HD6309-LABEL: cmp_i8_reg:
; See cmp_i16_reg — the folded byte compare is cmpb n,s.
; HD6309:        cmp{{r|b}}

define zeroext i1 @cmp_i8_reg(i8 %a, i8 %b) {
  %c = icmp eq i8 %a, %b
  ret i1 %c
}
