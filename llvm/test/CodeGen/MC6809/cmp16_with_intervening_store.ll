; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 < %s | FileCheck %s
;
; Bug #87 regression test. The compare expansion backwards-scan
; (expandCompareImm, MC6809InstrInfo.cpp) used to rewrite
;
;    stx     N,u
;    ldb     N+1,u
;    ... byte-level update to mem[N..N+1]
;    stb     N+1,u
;    [cmpd N,u]    ; removed by the scan
;    cmpx    #K
;
; on the assumption that the stx made $ix equal to the value at
; offset N. But any byte-level store to the slot between the stx
; and the cmp mutates the in-memory value without updating $ix,
; so the optimization silently compared the wrong value.
;
; The fix extends the scan to bail out if any store touches the
; slot's 2-byte footprint before reaching the stx. For a function
; of the shape `x - 97 < 26 ? x - 32 : x` (picolibc's toupper)
; at -O0, the expansion now emits an `ldd N,u` before the `cmpd`
; instead of reusing $ix.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: test_toupper:
; CHECK:      addb    #159
; CHECK:      adca    #255
; The compare must load D from the slot, not reuse X (which still
; holds the pre-subtract value).
; CHECK:      ldd     {{[0-9]+,u}}
; CHECK-NEXT: cmpd    #26
define i16 @test_toupper(i16 %c) {
entry:
  %0 = add i16 %c, -97
  %or.cond = icmp ult i16 %0, 26
  %sub = add nsw i16 %c, -32
  %cond = select i1 %or.cond, i16 %sub, i16 %c
  ret i16 %cond
}
