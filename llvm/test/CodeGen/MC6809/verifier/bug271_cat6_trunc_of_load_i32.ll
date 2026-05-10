; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s -o /dev/null
;
; Bug #271 cat-6 sentinel.
;
; The artifact combiner's `replaceRegOrBuildCopy` helper used to fall
; back to `Builder.buildCopy(DstReg, SrcReg)` whenever
; `canReplaceReg` returned false — including the case where the only
; reason it returned false was that the two virtual registers had
; different LLT types. That produced bare type-mismatched COPYs of
; the shape `s16 = COPY s32_vreg` which `-verify-machineinstrs`
; flags as a sub-register copy missing a sub-reg index.
;
; The picolibc trigger was a 32-bit field load truncated to a 16-bit
; local in libc/search/hash_page.c::__get_page. The fix in
; `LegalizationArtifactCombiner.h::replaceRegOrBuildCopy` switches
; to `Builder.buildAnyExtOrTrunc` when both operands are virtual but
; differ in type — mirroring the pattern already used by
; `tryCombineSExt`.
;
; This sentinel exercises the trunc-of-load-i32 -> i16 shape under
; -O2 hd6309 with -verify-machineinstrs gating.

target triple = "mc6809-unknown-unknown"

%struct.HTAB = type { i16, i16, i32 }

define i16 @bug271_cat6_trunc_of_load(ptr %hashp) nounwind {
entry:
  %bsize_p = getelementptr inbounds %struct.HTAB, ptr %hashp, i16 0, i32 2
  %bsize_i32 = load i32, ptr %bsize_p, align 1
  %is_zero = icmp eq i32 %bsize_i32, 0
  %conv = trunc i32 %bsize_i32 to i16
  %sel = select i1 %is_zero, i16 -1, i16 %conv
  ret i16 %sel
}
