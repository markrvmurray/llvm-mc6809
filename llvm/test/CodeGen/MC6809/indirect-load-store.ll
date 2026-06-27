; RUN: llc -mtriple=mc6809 -mcpu=mc6809 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=MC6809
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=HD6309
;
; Indirect-indexed addressing: a pointer that itself lives in memory (loaded
; from a register-based address) and is dereferenced exactly once folds into a
; single [n,r] indirect load/store -- the explicit pointer load AND the index
; register it would occupy both vanish. Selected by the am_ls_indexed_indirect
; pattern (MC6809InstrFamilies.td) via selectLSIndexedIndirect.

target triple = "mc6809-unknown-unknown"

; i8 load, pointer at offset 0 -> ld [,r]
define i8 @ld8(i8** %s) {
  %p = load i8*, i8** %s
  %v = load i8, i8* %p
  ret i8 %v
}
; MC6809-LABEL: ld8:
; MC6809:       ldb     [,x]
; HD6309-LABEL: ld8:
; HD6309:       ldb     [,x]

; i8 store, pointer at offset 0 -> st [,r]
define void @st8(i8** %s, i8 %v) {
  %p = load i8*, i8** %s
  store i8 %v, i8* %p
  ret void
}
; MC6809-LABEL: st8:
; MC6809:       stb     [,x]

; i8 load, pointer at a non-zero offset -> ld [n,r]
define i8 @ld8o(i8** %s) {
  %a = getelementptr i8*, i8** %s, i16 2
  %p = load i8*, i8** %a
  %v = load i8, i8* %p
  ret i8 %v
}
; MC6809-LABEL: ld8o:
; MC6809:       ldb     [4,x]

; i16 load -> ldd [,r]
define i16 @ld16(i16** %s) {
  %p = load i16*, i16** %s
  %v = load i16, i16* %p
  ret i16 %v
}
; MC6809-LABEL: ld16:
; MC6809:       ldd     [,x]

; i16 store -> std [,r]
define void @st16(i16** %s, i16 %v) {
  %p = load i16*, i16** %s
  store i16 %v, i16* %p
  ret void
}
; MC6809-LABEL: st16:
; MC6809:       std     [,x]

; i32 load -- HD6309-only native LDQ indirect -> ldq [,r]
define i32 @ld32(i32** %s) {
  %p = load i32*, i32** %s
  %v = load i32, i32* %p
  ret i32 %v
}
; HD6309-LABEL: ld32:
; HD6309:       ldq     [,{{[xy]}}]

; i32 store -- HD6309-only native STQ indirect -> stq [,r]
define void @st32(i32** %s, i32 %v) {
  %p = load i32*, i32** %s
  store i32 %v, i32* %p
  ret void
}
; HD6309-LABEL: st32:
; HD6309:       stq     [,{{[xy]}}]
