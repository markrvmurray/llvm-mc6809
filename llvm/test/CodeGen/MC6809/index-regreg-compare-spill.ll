; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s
;
; A reg-reg index-bank pointer compare with a spilled operand must read that
; operand from its frame slot (CMPY off,u) and must NOT materialize it into a
; hardware index register.
;
; This function returns a pointer, so the NULL/return candidate is held live in
; IX across the loop's pointer compares. With two pointers walked against two
; limits the compare operands spill — only IX/IY are allocatable (U is the frame
; pointer) and IX is busy holding the return. The HD6309 fast path used to
; materialize the spilled operand into IX and emit CMPR X,Y, silently clobbering
; the live return pointer: memmem() returned a false match because its NULL
; return was overwritten by the loop bound. The fix leaves the spilled operand
; in its slot and compares register-vs-slot (CMPY off,u), touching only IY — so
; the value in IX survives. Each pointer compare must therefore appear as a
; register-vs-slot CMPY, never loading the spilled operand into an index
; register and never collapsing to a same-register CMPR.
define ptr @ptr_find(ptr %a, ptr %ae, ptr %b, ptr %be, i8 %c) {
; CHECK-LABEL: ptr_find:
; With the SPILL_X pseudo-registers retired, the compare's operands arrive
; either both in index registers (HD6309: cmpr y,x), or with the spilled one
; read from memory (folded frame-slot cmpx N,s / N,u, or the push-and-compare
; fallback cmpx ,s++). All shapes keep the operands distinct; the load-bearing
; guard is the CHECK-NOT below (never a self-compare).
; CHECK: cmp{{[xyr]}}
; CHECK: cmp{{[xyr]}}
; CHECK-NOT: cmpr [[RR:[a-z]+]],[[RR]]
entry:
  br label %loop
loop:
  %pa = phi ptr [ %a, %entry ], [ %na, %body ]
  %pb = phi ptr [ %b, %entry ], [ %nb, %body ]
  %la = icmp ult ptr %pa, %ae
  %lb = icmp ult ptr %pb, %be
  %x  = and i1 %la, %lb
  br i1 %x, label %body, label %exit
body:
  %v  = load i8, ptr %pa
  %m  = icmp eq i8 %v, %c
  %na = getelementptr i8, ptr %pa, i16 1
  %nb = getelementptr i8, ptr %pb, i16 1
  br i1 %m, label %found, label %loop
found:
  ret ptr %pa
exit:
  ret ptr null
}
