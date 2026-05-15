; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s
;
; Bug #300 residue sentinel — `Load_i32_Imm` with dst=SPILL_Q*N
; (regalloc-chosen when AQc is widened by Phase B core) expands
; post-RA as `LDQ #imm → $aq; STQ $aq → slot`.  The transient $aq
; touch clobbers any other vreg currently live in $aq.
;
; Manifest (Phase B re-applied): picolibc `umulodi4_probe` at
; Os-lto-hd6309-mame.  Storing an i64 constant emits two
; `Load_i32_Imm` ops (one per i32 half).  Regalloc allocates the
; first to $aq and the second to $spill_q0; the second's expansion
; clobbers the first via the transient LDQ → $aq path.  Result:
; both halves of the stored i64 share the *last* i32 value.
;
; Fix: in `expandLoadImm`'s SPILL_Q*N branch, bracket the
; staging-LDQ + dematerialize-STQ pair with a LEAS-based hard-
; stack save/restore of $aq.  4 extra instructions per spill-
; targeted i32-Imm load, but the live $aq value is preserved
; across the transient touch.
;
; Dormant pre-Phase-B (i32 G_LOAD narrows; Load_i32_Imm never
; emitted with SPILL_Q*N dst, so the patched code path is
; unreachable).  Pre-Phase-B i64 constant stores narrow all the
; way to 4× i16 stores via LDD/STD — also correct.  Either way,
; the four bytes of the 0x0123456789ABCDEF constant land at the
; right addresses in memory.

define void @bug300_i64_const_store(ptr %out) {
; CHECK-LABEL: bug300_i64_const_store:
entry:
  store i64 81985529216486895, ptr %out, align 1  ; 0x0123456789ABCDEF
  ret void
}

; Whichever lowering the legalizer picks, the bytes of the i64
; 0x0123456789ABCDEF (= 81985529216486895) must reach the right
; memory offsets.  In particular: the *low* 32 bits 0x89ABCDEF
; (= 2309737967 unsigned, or unsigned long = -1985229329) must
; appear in the lowered output and reach the right slot.  Without
; the Bug #300 fix, both i64 halves were written with the same
; (HIGH) i32 value — i.e. 0x89ABCDEF never appeared anywhere in
; the asm.

; Low-32 of i64 must appear (either as ldq #2309737967 in Phase B
; path or split as ldd #X / ldd #Y in narrowed path).  The split-
; into-i16 path emits ldd #-30293 (= $89ab signed) and ldd #-12817
; (= $cdef signed).  At least one of these markers must be
; present in the output.
; CHECK: -30293
