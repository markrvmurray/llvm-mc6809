; The CIEs the compiler writes name the PC (DWARF register 5) as the return
; address column, in .debug_frame and in .eh_frame alike.
;
; RUN: llc -mtriple=mc6809 -filetype=obj -o %t.o %s
; RUN: llvm-dwarfdump --debug-frame --eh-frame %t.o | FileCheck %s

; CHECK: CIE
; CHECK: Return address column: 5

declare void @g(i16)

define void @f(i16 %x) uwtable {
  call void @g(i16 %x)
  call void @g(i16 %x)
  ret void
}
