; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs %s -o - | FileCheck %s

; Greedy used to give up on this function ("ran out of registers during
; register allocation"). Four pointer arguments and a loop put more index
; values in flight than the two index registers can hold, so values are
; spilled; while eliminating dead defs after a spill, the spiller separates
; a live range into connected components, and LRE_DidCloneVirtReg hands the
; clone a fresh RS_Assign stage so it can be allocated anew -- which means
; it may be split again. The spill path then marked every new vreg of that
; spill unspillable, clone included, on the belief that they were all
; reloads. A clone that may be split but may not be spilled dead-ends the
; splitters: a region split parks the interfering part in a complement
; whose whole purpose is to be spilled if it does not allocate, and an
; unspillable complement has nowhere to go but last-chance recoloring.
;
; Reduced from picolibc's getopt_internal under LTO.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

; CHECK-LABEL: getopt_internal:
; CHECK: rts

define fastcc i16 @getopt_internal(ptr %argv, ptr %shortopts, ptr %longopts, ptr %data, i1 %cmp6, i32 %0, i1 %cond1) {
entry:
  br i1 %cmp6, label %while.body.i, label %if.then382

while.body.i:                                     ; preds = %while.body.i, %entry
  %cmp3.not.i = icmp eq i32 %0, 0
  br i1 %cmp3.not.i, label %while.body.i, label %while.body9.i

while.body9.i:                                    ; preds = %while.body.i
  br i1 %cond1, label %sw.bb307, label %if.then382

sw.bb307:                                         ; preds = %while.body9.i
  store ptr null, ptr %longopts, align 1
  br label %sw.epilog399

if.then382:                                       ; preds = %while.body9.i, %entry
  %1 = load i8, ptr %argv, align 1
  %cmp390 = icmp eq i8 %1, 0
  %spec.store.select = zext i1 %cmp390 to i16
  br label %sw.epilog399

sw.epilog399:                                     ; preds = %if.then382, %sw.bb307
  %storemerge = phi i16 [ %spec.store.select, %if.then382 ], [ 0, %sw.bb307 ]
  store i16 %storemerge, ptr %data, align 1
  store i16 0, ptr %shortopts, align 1
  ret i16 0
}
