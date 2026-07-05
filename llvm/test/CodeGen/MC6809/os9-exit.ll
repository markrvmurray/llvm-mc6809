; RUN: llc -global-isel -mtriple=mc6809-unknown-os9 -O2 %s -o - | FileCheck %s

; The OS-9 _exit leaf syscall, as lowered from the inline-asm C in
; compiler-rt/lib/builtins/mc6809-os9/syscalls.c.  status arrives in X; the
; 'd' constraint moves it to D (B = low byte = status byte), then SWI2 + the
; F$Exit ($06) function code as an inline FCB.  F$Exit never returns.

target triple = "mc6809-unknown-os9"

; CHECK-LABEL: _exit:
; CHECK:      tfr x,d
; CHECK:      swi2
; CHECK-NEXT: .byte 6
define dso_local void @_exit(i16 noundef %status) {
entry:
  tail call void asm sideeffect "swi2\0A\09.byte ${0:c}", "i,d,~{memory}"(i16 6, i16 %status)
  unreachable
}

; The NitrOS-9 native name is an alias to the same address.
; CHECK: .globl F$Exit
; CHECK: F$Exit = _exit
@"F$Exit" = dso_local alias void (i16), ptr @_exit
