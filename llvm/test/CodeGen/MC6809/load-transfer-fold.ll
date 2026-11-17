; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s

; A value loaded into D only to be handed to X -- here a call's first
; argument -- is loaded into X directly: `ldd #4096 ; tfr d,x` becomes
; `ldx #4096` (LDX sets N, Z and V as LDD does). The fold applies only when
; D, asked by its register units so that A and B count, is dead after the
; transfer. picolibc's valloc.

target triple = "mc6809-unknown-unknown"

declare ptr @memalign(i16, i16)

; CHECK-LABEL: valloc:
; CHECK-NOT:   tfr d,x
; CHECK:       ldx #4096
; CHECK-NEXT:  lbsr memalign
define ptr @valloc(i16 %s) {
  %r = call ptr @memalign(i16 4096, i16 %s)
  ret ptr %r
}
