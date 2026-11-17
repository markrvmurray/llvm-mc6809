; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O0 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,M6809
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O0 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,H6309
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O1 -verify-machineinstrs < %s -o /dev/null
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O1 -verify-machineinstrs < %s -o /dev/null

; A branch on the borrow of a subtract whose difference is never used. The
; branch tests the physical C flag; it used to leave the subtract with no
; users, so the selector deleted it and the branch tested a stale C (`bhs`
; with nothing before it). The branch now names the borrow through
; selection, and the name is dropped after, so no stray live range reaches
; the register allocator.

target triple = "mc6809-unknown-unknown"

declare { i32, i1 } @llvm.usub.with.overflow.i32(i32, i32)
declare { i16, i1 } @llvm.usub.with.overflow.i16(i16, i16)

; CHECK-LABEL: b32:
; M6809:       subb {{[0-9]+}},s
; M6809:       sbcb {{[0-9]+}},s
; M6809:       sbcb {{[0-9]+}},s
; M6809:       sbcb {{[0-9]+}},s
; H6309:       sbcd 6,s
; CHECK-NEXT:  bhs
define i8 @b32(i32 %a, i32 %b) {
  %r = call { i32, i1 } @llvm.usub.with.overflow.i32(i32 %a, i32 %b)
  %c = extractvalue { i32, i1 } %r, 1
  br i1 %c, label %yes, label %no
yes:
  ret i8 1
no:
  ret i8 2
}

; CHECK-LABEL: b16:
; CHECK:       sbcb 2,s
; CHECK-NEXT:  bhs
define i8 @b16(i16 %a, i16 %b) {
  %r = call { i16, i1 } @llvm.usub.with.overflow.i16(i16 %a, i16 %b)
  %c = extractvalue { i16, i1 } %r, 1
  br i1 %c, label %yes, label %no
yes:
  ret i8 1
no:
  ret i8 2
}
