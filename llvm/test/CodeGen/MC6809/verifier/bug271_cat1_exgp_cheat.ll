; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 -verify-machineinstrs < %s -o /dev/null
;
; Bug #271 cat-1 sentinel — EXGp cheat-path subset.
;
; HD6309 has no immediate-form opcodes for AND/OR/XOR/SBC on the
; W (= AW) register. When regalloc binds the value to AW, the
; expansion in `expandImm` / `expandIdxImm` brackets the operation
; with `EXG D,W; <op> on AD; EXG W,D` — exchanging AW with AD,
; running the op on AD, then exchanging back. Same for AE↔AA and
; AF↔AB.
;
; Pre-fix: both EXGp emissions read the scratch register (AD / AA /
; AB) and the destination unconditionally. The scratch side has no
; meaningful value before the first EXG (only DestReg is live), and
; DestReg has no meaningful value before the second EXG (the first
; EXG moved its value into the scratch). The verifier flagged these
; reads as "Using an undefined physical register" — 10 such hits at
; -Og hd6309 mame across functions like toascii_l, fileno,
; __gets_chk, putc.
;
; Fix: mark the undef-side read with `RegState::Undef` on both EXGp
; instructions. The semantics are correct (we don't care what
; value is on the undef side — the EXG just shuffles bits), but
; the verifier needs the explicit annotation.
;
; This sentinel is the reduced toascii_l shape: an i16 first
; argument (passed in IX) that gets AND'd with 0x7F and returned.
; Regalloc tends to land the value in AW under -O1, triggering the
; cheat-path bracket.

target triple = "mc6809-unknown-unknown"

define i16 @bug271_cat1_exgp_toascii_like(i16 %c, i16 %unused) nounwind {
entry:
  %r = and i16 %c, 127
  ret i16 %r
}
