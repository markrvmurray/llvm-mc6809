; RUN: llc -mtriple=mc6809-unknown-os9 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Hand-authored: the address of an undefined extern-weak symbol must be null.
; A module is linked at 0 and placed at run time, and its data area is
; addressed from U, so the ordinary forms (`sym,pc` and `mc6809_os9_data(sym),u`)
; would hand back the module's own base or U itself -- both non-null -- for a
; symbol the linker resolved to 0.  Nothing real sits at offset 0 of either
; region (the body starts with _start, the data area with the imaginary
; registers), so the address is built around that sentinel: load the link-time
; offset, and add the region's base only when it is not zero.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-os9"

declare extern_weak void @hook()
@wdata = extern_weak global [0 x i16], align 1

; A weak function: its address is the body offset, plus the module base when
; the symbol is defined.
; CHECK-LABEL: takes_hook_addr:
; CHECK:      ld{{[xy]}} #hook
; CHECK:      b{{eq|ne}} .L
; CHECK:      lea{{[xy]}} hook,pc
define ptr @takes_hook_addr() {
  ret ptr @hook
}

; A weak data-area object: the data-area offset, plus U when defined.
; CHECK-LABEL: takes_data_addr:
; CHECK:      ld{{[xy]}} #mc6809_os9_data(wdata)
; CHECK:      b{{eq|ne}} .L
; CHECK:      lea{{[xy]}} mc6809_os9_data(wdata),u
define ptr @takes_data_addr() {
  ret ptr @wdata
}
