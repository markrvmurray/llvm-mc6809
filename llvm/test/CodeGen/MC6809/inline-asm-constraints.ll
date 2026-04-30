; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809-unknown-unknown %s -o - | FileCheck %s
;
; Bug #198: MC6809 inline asm register constraints.
; These were previously rejected by the Clang Sema layer before reaching the backend.
; After the fix to validateAsmConstraint/validateOperandSize in MC6809.cpp they compile.
; Test that the backend correctly allocates the specified registers.

; '=d' constraint — D register (16-bit)
define i16 @test_d_output() {
; CHECK-LABEL: test_d_output:
; CHECK: ;APP
; CHECK: ldd #42
; CHECK: ;NO_APP
  %1 = call i16 asm "ldd #42", "=d"()
  ret i16 %1
}

; '+d' constraint — D register, read-write (16-bit)
define i16 @test_d_readwrite(i16 %x) {
; CHECK-LABEL: test_d_readwrite:
; CHECK: ;APP
; CHECK: addd #1
; CHECK: ;NO_APP
  %1 = call i16 asm "addd #1", "=d,0"(i16 %x)
  ret i16 %1
}

; '=A' constraint — A register (8-bit, high byte of D)
define i8 @test_A_output() {
; CHECK-LABEL: test_A_output:
; CHECK: ;APP
; CHECK: lda #1
; CHECK: ;NO_APP
  %1 = call i8 asm "lda #1", "=A"()
  ret i8 %1
}

; '=B' constraint — B register (8-bit, low byte of D)
define i8 @test_B_output() {
; CHECK-LABEL: test_B_output:
; CHECK: ;APP
; CHECK: ldb #2
; CHECK: ;NO_APP
  %1 = call i8 asm "ldb #2", "=B"()
  ret i8 %1
}

; 'x' input constraint — X register (16-bit)
define void @test_x_input(i16 %p) {
; CHECK-LABEL: test_x_input:
; CHECK: ;APP
; CHECK: ldd ,x
; CHECK: ;NO_APP
  call void asm sideeffect "ldd ,x", "x"(i16 %p)
  ret void
}
