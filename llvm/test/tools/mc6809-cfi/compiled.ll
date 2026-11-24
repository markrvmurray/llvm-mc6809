; The compiler's CFI passes scripts/mc6809-cfi: callee-saved pushes, a frame
; allocated and released, a temporary pushed and taken back in the body, and a
; block laid out after a return.  Under -O0 too, where nothing is tidied.
;
; RUN: llc -mtriple=mc6809 -O2 --force-dwarf-frame-section -filetype=obj -o %t.o %s
; RUN: %python %S/../../../../scripts/mc6809-cfi check --require %t.o | FileCheck %s
; RUN: llc -mtriple=mc6809 -O0 --force-dwarf-frame-section -filetype=obj -o %t.O0.o %s
; RUN: %python %S/../../../../scripts/mc6809-cfi check --require %t.O0.o | FileCheck %s
; CHECK: mc6809-cfi: 1 object(s), 2 FDE(s), {{[0-9]+}} instruction(s) followed, 0 problem(s)

@sink = global i16 0
declare i16 @leaf(i16)
declare void @take(ptr, i16)

define i16 @middle(i16 %a, i16 %b) {
  %s = add i16 %a, %b
  %r = call i16 @leaf(i16 %s)
  store i16 %r, ptr @sink
  %t = add i16 %r, %a
  ret i16 %t
}

define void @early(i16 %a) {
  %buf = alloca [4 x i16]
  call void @take(ptr %buf, i16 %a)
  %c = icmp eq i16 %a, 0
  br i1 %c, label %quick, label %slow
quick:
  ret void
slow:
  call void @take(ptr %buf, i16 %a)
  call void @take(ptr %buf, i16 %a)
  ret void
}
