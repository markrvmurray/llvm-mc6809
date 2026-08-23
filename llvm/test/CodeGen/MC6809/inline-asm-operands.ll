; RUN: llc -mtriple=mc6809 < %s | FileCheck %s

; Operands of inline asm are printed as the assembler names them.  A register
; is `x`, not the record's own `IX`, which the assembler would read as a
; symbol and leave for the linker to fail on; and a memory operand, whose
; address is held in a register, is the indexed form rather than the bare
; register.

define void @reg_operand(i16 %v) {
; CHECK-LABEL: reg_operand:
; CHECK: ; reg x
; CHECK-NOT: IX
  call void asm sideeffect "; reg $0", "x"(i16 %v)
  ret void
}

define void @mem_operand(ptr %p) {
; CHECK-LABEL: mem_operand:
; CHECK: clr 0,{{[xyus]}}
; CHECK-NOT: clr I
  call void asm sideeffect "clr $0", "*m"(ptr elementtype(i8) %p)
  ret void
}
