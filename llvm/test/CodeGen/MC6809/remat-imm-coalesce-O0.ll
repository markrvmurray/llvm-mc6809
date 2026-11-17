; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O0 -verify-machineinstrs < %s | FileCheck %s

; The register coalescer rematerialises a constant through the target's
; reMaterialize hook too, and requires any physical register the copy
; defines to be dead. A constant load defines N, Z and V; where they are
; not live the copy now marks them dead. Before, the coalescer asserted at
; -O0 on this loop of constant phis (reduced from picolibc's getsubopt).

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: getsubopt:
; CHECK:       rts

define i16 @getsubopt() {
entry:
  br label %for.cond15

for.cond15:                                       ; preds = %land.end34, %entry
  br i1 false, label %land.lhs.true, label %land.end34

land.lhs.true:                                    ; preds = %for.cond15
  br i1 false, label %land.lhs.true22, label %land.end34

land.lhs.true22:                                  ; preds = %land.lhs.true
  br i1 false, label %land.rhs30, label %land.end34

land.rhs30:                                       ; preds = %land.lhs.true22
  br label %land.end34

land.end34:                                       ; preds = %land.rhs30, %land.lhs.true22, %land.lhs.true, %for.cond15
  %0 = phi i1 [ false, %land.lhs.true22 ], [ true, %land.rhs30 ], [ false, %land.lhs.true ], [ false, %for.cond15 ]
  br i1 %0, label %for.cond15, label %for.end36

for.end36:                                        ; preds = %land.end34
  ret i16 0
}
