; RUN: llc -O2 -relocation-model=pic -mtriple=mc6809 %s -o - | FileCheck %s

; Bug #361 increment 3: fold a PC-relative index-LEA into the load that
; dereferences it. Reading a global under -relocation-model=pic computes its
; address PC-relative (`leax gp,pc`) then loads through it (`ldx ,x`); fold the
; two into a single PC-relative load (`ldx gp,pc`). dst == base here, so the
; fold is unconditionally safe.

@gp = external global i8*

; CHECK-LABEL: pcrel_fold:
; CHECK:       ldx gp,pc
; CHECK-NOT:   leax gp,pc
; CHECK-NOT:   ldx ,x
define i8* @pcrel_fold() {
  %v = load i8*, i8** @gp
  ret i8* %v
}
