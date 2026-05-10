; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s -o /dev/null
;
; Bug #271 cat-3 sentinel.
;
; The spill framework's `loadStoreRegStackSlot` used to fall through
; to `loadStoreRegisterStaticStackSlot` for any non-INDEX16 / non-
; ACC16 vreg, and the size-32 path emitted `Store_i32_Mem` /
; `Load_i32_Mem` directly with that vreg as operand. TableGen
; declares those instructions' operand class as `AQc` (singleton AQ
; — required by the strtol / snprintf / asctime correctness paths
; from bug #208 / #210), but a vreg constrained to ACC32 (= AQ +
; SPILL_Q*) is NOT a subclass of AQc — verifier flags it as
; "Illegal virtual register for instruction".
;
; The fix mirrors the existing ACC16 pattern in
; `loadStoreRegStackSlot`: create a fresh AQc-class vreg local to
; the spill site, copy in/out around `Store_i32_Mem` /
; `Load_i32_Mem`. The COPY between the original ACC32 vreg
; (potentially bound to SPILL_Q*) and the new AQc-bound vreg is
; resolved post-RA by `copyPhysReg` via the existing LDQ / STQ
; materialize/dematerialize machinery.
;
; The picolibc trigger sites include libc/string/strncat_s.c,
; libc/stdio/bufio.c (__bufio_flush_locked), and libc/search/hash.c
; (__hash_open) — each with i32 values that survive enough
; register pressure across calls to be spilled as full ACC32.
;
; This sentinel keeps an i32 live across a side-effecting call,
; forcing the spill framework to handle ACC32 storage. Even when
; greedy chooses the ACC16-half spill path instead of full ACC32,
; the post-fix code remains verifier-clean — the compile-clean
; status under `-verify-machineinstrs` is the regression guard.

target triple = "mc6809-unknown-unknown"

declare void @work()

define void @bug271_cat3_acc32_spill(ptr %src, ptr %dst) nounwind {
entry:
  %v = load i32, ptr %src, align 1
  call void @work()
  store i32 %v, ptr %dst, align 1
  ret void
}
