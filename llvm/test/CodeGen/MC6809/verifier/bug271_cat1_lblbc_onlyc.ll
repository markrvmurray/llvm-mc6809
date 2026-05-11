; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 -verify-machineinstrs < %s -o /dev/null
;
; Bug #271 cat-1 residual sentinel — LBlbc OnlyC variant.
;
; The canonical Bbc/LBlbc TableGen declares `Uses = [N, Z, V, C]` —
; the UNION of all condition-code flag dependencies (per design, so
; the scheduler can't reorder past a flag-clobbering instruction).
; Bug #206 added _NoC variants (`Uses = [N, Z, V]`) for cc values
; that don't read C (EQ/NE/VC/VS/PL/MI/GE/LT/GT/LE/RA/INVALID),
; needed because TST sets N/Z/V but hardware-preserves C — the
; verifier would otherwise flag $c as undefined.
;
; This sentinel adds the inverse case: a producer chain that
; sets ONLY C (G_UADDO carry-out), an intervening Store that
; clobbers N/Z/V via hardware side-effect (STD sets N/Z, clears
; V), and a downstream branch on CS (cc=5, reads only C).
; Without the _OnlyC variant, the canonical LBlbc's
; `Uses = [N, Z, V, C]` would have the verifier flag $n/$z/$v
; as undefined post-Store.
;
; Pattern observed at -Og in `c8rtomb` (uchar/c8rtomb.c:98),
; `strftime` (time/strftime.c:1161), and `xdr_int` (xdr/xdr.c:731).
;
; Fix: MC6809CC::doesOnlyReadCarry covers HS/CC and LO/CS;
; pickBbcVariant / pickLBlbcVariant return Bbc_OnlyC /
; LBlbc_OnlyC for those cc values; the post-RA expansion of
; ConditionalBranchRelative / ConditionalLongBranchRelative
; emits only the $c implicit-use operand for the OnlyC variants.
; The reduced Uses set keeps the verifier happy.

target triple = "mc6809-unknown-unknown"

; A 16-bit add-with-overflow producing both a carry-out and an i16
; result; the result is stored before the carry-derived branch.
; The CS branch wants to read only $c — which is preserved across
; the intervening STD by hardware.

define void @bug271_cat1_lblbc_onlyc(i16 %a, i16 %b, ptr %dst, i16 %iflo) nounwind {
entry:
  %sum_o = call { i16, i1 } @llvm.uadd.with.overflow.i16(i16 %a, i16 %b)
  %sum = extractvalue { i16, i1 } %sum_o, 0
  %ovf = extractvalue { i16, i1 } %sum_o, 1
  store i16 %sum, ptr %dst, align 1
  br i1 %ovf, label %if.then, label %if.end

if.then:
  store i16 %iflo, ptr %dst, align 1
  br label %if.end

if.end:
  ret void
}

declare { i16, i1 } @llvm.uadd.with.overflow.i16(i16, i16)
