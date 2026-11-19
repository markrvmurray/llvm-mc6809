; RUN: not llc -mtriple=mc6809-unknown-os9 -O2 %s -o /dev/null 2>&1 | FileCheck %s

; Realigning a frame needs U as the frame pointer, which OS-9 reserves as
; the process's data-area base. A local aligned beyond 1 byte is an error
; there, not a silently misaligned object.

; CHECK: error: {{.*}}a local aligned beyond 1 byte: OS-9 cannot realign the stack

declare void @use(ptr)

define void @aligned16() {
entry:
  %buf = alloca [32 x i8], align 16
  call void @use(ptr %buf)
  ret void
}
