; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s

; A function whose epilogue ends by pulling its saved registers returns in the
; same pull: `puls y,pc`, not `puls y ; rts`. An interrupt handler returns
; with RTI and keeps its pull separate.

target triple = "mc6809-unknown-unknown"

declare i16 @g(i16)

; The register kept across the call is saved, and comes back with the
; return address.
; CHECK-LABEL: saves_y:
; CHECK:       pshs {{[uy]}}
; CHECK:       lbsr g
; CHECK:       puls {{[uy]}},pc
; CHECK-NOT:   rts
; CHECK:       .Lfunc_end
define i16 @saves_y(i16 %x) {
  %a = call i16 @g(i16 %x)
  %b = add i16 %a, %x
  ret i16 %b
}

; Nothing saved: a plain return.
; CHECK-LABEL: saves_nothing:
; CHECK-NOT:   puls
; CHECK:       rts
define i16 @saves_nothing(i16 %x) {
  %b = add i16 %x, 1
  ret i16 %b
}

@ticks = global i16 0

; CHECK-LABEL: handler:
; CHECK-NOT:   ,pc
; CHECK:       rti
define void @handler() "interrupt" {
  %v = load volatile i16, ptr @ticks
  %n = call i16 @g(i16 %v)
  store volatile i16 %n, ptr @ticks
  ret void
}
