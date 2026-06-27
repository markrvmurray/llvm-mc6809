; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=HD6309
; RUN: llc -mtriple=mc6809 -mcpu=mc6809 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=MC6809
;
; Phase 3 of the i64-i32-i16-i8 plan: HD6309 G_LOAD s32 legalizes to
; native LDQ.  Pre-Phase-B (i.e. Bug #297 commit 5+6 revert at
; ee1d0796ba46) the legalizer narrowed s32 G_LOAD to two i16 G_LOADs
; (a pair of LDDs).  Post-Phase-B re-land via the existing
; LegalTypes32 = {p, s8, s16, s32} + customIf gate (Bug #310 narrow):
; s32 G_LOAD is legal and the selector emits LDQ directly.
;
; Plain MC6809 has no LDQ — stays at the two-LDD decomposition.

target triple = "mc6809-unknown-unknown"

define i32 @load_simple(i32* %p) {
  %v = load i32, i32* %p
  ret i32 %v
}

; HD6309-LABEL: load_simple:
; The i32* arg is passed on the stack (i32 return uses an sret pointer in X), so
; *p is a double-deref that folds to a single indirect LDQ -- still native LDQ,
; not the two-LDD decomposition this test guards against.
; HD6309:       ldq     {{\[?}}
;
; MC6809-LABEL: load_simple:
; MC6809-NOT:   ldq

define i32 @load_offset(i32* %p) {
  %a = getelementptr i32, i32* %p, i16 4
  %v = load i32, i32* %a
  ret i32 %v
}

; HD6309-LABEL: load_offset:
; HD6309:       ldq

define i32 @load_pair(i32* %p, i32* %q) {
  %a = load i32, i32* %p
  %b = load i32, i32* %q
  %s = add i32 %a, %b
  ret i32 %s
}

; HD6309-LABEL: load_pair:
; HD6309:       ldq
; HD6309:       ldq
