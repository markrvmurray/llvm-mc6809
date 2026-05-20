; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 -mcpu=mc6809 < %s | FileCheck %s
;
; Bug #306 regression sentinel.
;
; At -O0, an i1 PHI value zero-extended to i16 and stored to memory
; passes through MERGE_LOHI_i16 with LoReg=AB, HiReg=AA, DstReg=AD.
; The pre-fix expansion collapsed to zero machine instructions in that
; shape (both loadByteInto calls early-return on SrcReg==Target; the
; trailing dematerializeReg(AD, AD) is a no-op).  Post-RA DCE then
; noticed the upstream `Load_i8_Imm 0 → $aa` (which zeros the high
; byte for the i1→i16 zext) had no remaining consumer and eliminated
; it, leaving the next Store_i16_Mem $ad reading garbage in $aa.
;
; Fix: in the LoReg=AB / HiReg=AA / DstReg=AD shape, when the
; immediately upstream def of $aa or $ab is a DCE-vulnerable opcode
; (LDAi8 / LDBi8 / CLRAa / CLRBa / Load_i8_Imm) AND nothing between
; that def and the MERGE reads $aa / $ab / $ad, emit `$ad = TFRp $ad`
; as a liveness anchor.  The explicit Def of $ad is honoured by
; LivePhysRegs::stepForward; a KILL with implicit-def is not.
;
; This IR mirrors the bug body's repro shape: a `match = cond1 || cond2`
; short-circuit OR produces a `phi i1` at the merge block which is
; zext'd to i16 and stored.

target triple = "mc6809-unknown-unknown"

declare i16 @strcmp(ptr, ptr)

define i16 @bug306_short_circuit_or(ptr %a, ptr %b, ptr %match) {
entry:
  %call = call i16 @strcmp(ptr %a, ptr %b)
  %cmp = icmp eq i16 %call, 0
  br i1 %cmp, label %lor.end, label %lor.rhs

lor.rhs:
  %cmp1 = icmp eq ptr %a, null
  br label %lor.end

lor.end:
  %phi = phi i1 [ true, %entry ], [ %cmp1, %lor.rhs ]
  %z = zext i1 %phi to i16
  store i16 %z, ptr %match, align 1
  ret i16 %z
}

; The anchor's `tfr d,d` appears between the high-byte zeroing
; (LDA #0 from the i1→i16 zext) and the i16 store (STD).  Without
; the anchor, the high-byte zeroing would be DCE'd and the STD
; would write garbage in the $aa half of the stored i16.
;
; CHECK-LABEL: bug306_short_circuit_or:
; CHECK:       lda #0
; CHECK-NEXT:  tfr d,d
; CHECK:       std
