; A call with both register and stack arguments computes the register ones
; after the stack ones are stored, and a library call that returns its
; destination (memset, memcpy, memmove) is not made to keep a copy of it.
;
; memset(p, c, n): c goes to __memset in B, n on the stack.  With c computed
; first, storing n needed a second 16-bit register the 6809 does not have,
; and n went through a direct-page register with D pushed and pulled around
; it; p was kept in U across the call for the return.
;
; memset(p + n, c, n): the pointer sum is the last use of n, so it comes
; before the load of c.  The other way round, n still held D when c was
; loaded, and c went through a direct-page register instead.
;
; RUN: llc -mtriple=mc6809 -mcpu=mc6809 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs < %s | FileCheck %s

; CHECK-LABEL: f:
; CHECK-NOT:   __rs
; CHECK-NOT:   pshs
; CHECK:       ldd 6,s
; CHECK-NEXT:  std ,s
; CHECK-NEXT:  ldd 4,s
; CHECK-NEXT:  lbsr __memset
; CHECK-NEXT:  leas 2,s
; CHECK-NEXT:  rts

; The destination comes back in X and is used from there.
; CHECK-LABEL: g:
; CHECK:       lbsr memcpy
; CHECK-NEXT:  ldd 10,s
; CHECK-NEXT:  leau d,x
; CHECK-NEXT:  clr ,u

; CHECK-LABEL: h:
; CHECK-NOT:   __rs
; CHECK-NOT:   pshs
; CHECK:       ldd 6,s
; CHECK-NEXT:  std ,s
; CHECK-NEXT:  leax d,x
; CHECK-NEXT:  ldd 4,s
; CHECK-NEXT:  lbsr __memset

define ptr @f(ptr returned %p, i16 %c, i16 %n) #0 {
entry:
  %0 = trunc i16 %c to i8
  tail call void @llvm.memset.p0.i16(ptr align 1 %p, i8 %0, i16 %n, i1 false)
  ret ptr %p
}

define ptr @g(ptr returned %d, ptr %s, i16 %n) #0 {
entry:
  tail call void @llvm.memcpy.p0.p0.i16(ptr align 1 %d, ptr align 1 %s, i16 %n, i1 false)
  %arrayidx = getelementptr inbounds nuw i8, ptr %d, i16 %n
  store i8 0, ptr %arrayidx, align 1
  ret ptr %d
}

define void @h(ptr %p, i16 %c, i16 %n) #0 {
entry:
  %add.ptr = getelementptr inbounds i8, ptr %p, i16 %n
  %0 = trunc i16 %c to i8
  tail call void @llvm.memset.p0.i16(ptr align 1 %add.ptr, i8 %0, i16 %n, i1 false)
  ret void
}

declare void @llvm.memset.p0.i16(ptr writeonly captures(none), i8, i16, i1 immarg)
declare void @llvm.memcpy.p0.p0.i16(ptr noalias writeonly captures(none), ptr noalias readonly captures(none), i16, i1 immarg)

attributes #0 = { nounwind optsize }
