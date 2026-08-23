; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mattr=-static-stack -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,BARE
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809-unknown-os9 -relocation-model=pic -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,OS9

; A direct-page (address-space 1) pointer is the 8-bit in-page offset. Cast
; to a generic pointer it becomes the object's full address: the page base
; (__dp_base_addr on bare metal, U on OS-9) plus the zero-extended offset.
; Cast the other way it is the low byte of the address. A p1 value travels
; in an 8-bit accumulator like any byte, including across calls.

@counter = internal addrspace(1) global i16 0, align 1
@bytes = internal addrspace(1) global [4 x i8] zeroinitializer, align 1

define ptr @addr() {
entry:
  %p = addrspacecast ptr addrspace(1) @counter to ptr
  ret ptr %p
}
; CHECK-LABEL: addr:
; BARE:      ldb #mc6809_8(counter)
; BARE-NEXT: clra
; BARE-NEXT: ldx #__dp_base_addr
; BARE-NEXT: leax d,x
; OS9:       ldb #mc6809_os9_data8(counter)
; OS9-NEXT:  clra
; OS9-NEXT:  leax d,u
; CHECK-NEXT: rts

define ptr @slot(i8 %i) {
entry:
  %e = getelementptr inbounds i8, ptr addrspace(1) @bytes, i8 %i
  %p = addrspacecast ptr addrspace(1) %e to ptr
  ret ptr %p
}
; CHECK-LABEL: slot:
; CHECK:      lda #mc6809_{{os9_data8|8}}(bytes)
; CHECK:      adda
; CHECK:      clra
; BARE:       ldx #__dp_base_addr
; BARE-NEXT:  leax d,x
; OS9:        leax d,u
; CHECK:      rts

define ptr addrspace(1) @back(ptr %p) {
entry:
  %q = addrspacecast ptr %p to ptr addrspace(1)
  ret ptr addrspace(1) %q
}
; CHECK-LABEL: back:
; CHECK:      tfr x,d
; CHECK-NEXT: rts

define ptr addrspace(1) @id(ptr addrspace(1) %p) {
entry:
  ret ptr addrspace(1) %p
}
; CHECK-LABEL: id:
; CHECK-NEXT: ; %bb.0:
; CHECK-NEXT: rts

define i8 @via(ptr addrspace(1) %p) {
entry:
  %v = load i8, ptr addrspace(1) %p, align 1
  ret i8 %v
}
; CHECK-LABEL: via:
; CHECK:      clra
; BARE:       ldx #__dp_base_addr
; BARE-NEXT:  ldb d,x
; OS9:        ldb d,u
; CHECK-NEXT: rts

; In a static initialiser the cast is a link-time expression: the full
; address is the symbol at 16 bits, the direct-page pointer its 8-bit offset.
@p_dp = global ptr addrspacecast (ptr addrspace(1) @counter to ptr), align 1
@g = external global i16, align 1
@back_p = global ptr addrspace(1) addrspacecast (ptr @g to ptr addrspace(1)), align 1
; CHECK-LABEL: p_dp:
; CHECK-NEXT: .short counter
; CHECK-LABEL: back_p:
; BARE-NEXT:  .byte mc6809_8(g)
; OS9-NEXT:   .byte mc6809_os9_data8(g)
