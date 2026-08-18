; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mattr=-static-stack -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,ABS
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mattr=-static-stack -relocation-model=pic -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,PIC
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809-unknown-os9 -relocation-model=pic -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,OS9

; A value loaded once and consumed by an arithmetic, bitwise or compare op
; is that op's memory operand: no register of its own, and for a 16-bit
; value no second accumulator (which the 6809 does not have) -- the shape
; that used to stage the loaded word through an imaginary register.

@d = internal global i16 0, align 1
@c = internal global i8 0, align 1
@dp = internal addrspace(1) global i8 0, align 1

; The loaded value is the LEFT operand of a commutative op: exchanged and
; folded, `d += v` is add-from-memory, store.
define void @rmw16(i16 %v) {
entry:
  %0 = load i16, ptr @d, align 1
  %add = add i16 %0, %v
  store i16 %add, ptr @d, align 1
  ret void
}
; CHECK-LABEL: rmw16:
; ABS:        tfr x,d
; ABS-NEXT:   addd d
; ABS-NEXT:   std d
; ABS-NEXT:   rts
; PIC:        tfr x,d
; PIC-NEXT:   addd d,pc
; PIC-NEXT:   std d,pc
; PIC-NEXT:   rts
; OS9:        lea{{[xy]}} mc6809_os9_data(d),u
; OS9:        tfr x,d
; OS9-NEXT:   addd ,{{[xy]}}
; OS9-NEXT:   std ,{{[xy]}}
; OS9:        rts

define void @rmw8(i8 %v) {
entry:
  %0 = load i8, ptr @c, align 1
  %x = xor i8 %0, %v
  store i8 %x, ptr @c, align 1
  ret void
}
; CHECK-LABEL: rmw8:
; ABS:        eorb c
; ABS-NEXT:   stb c
; PIC:        eorb c,pc
; PIC-NEXT:   stb c,pc
; OS9:        eorb ,{{[xy]}}
; OS9-NEXT:   stb ,{{[xy]}}
; CHECK-NEXT: rts

; A direct-page global takes the direct-page form of the op.
define void @rmwdp(i8 %v) {
entry:
  %0 = load i8, ptr addrspace(1) @dp, align 1
  %o = or i8 %0, %v
  store i8 %o, ptr addrspace(1) @dp, align 1
  ret void
}
; CHECK-LABEL: rmwdp:
; ABS:        orb <mc6809_8(dp)
; ABS-NEXT:   stb <mc6809_8(dp)
; PIC:        orb <mc6809_8(dp)
; PIC-NEXT:   stb <mc6809_8(dp)
; OS9:        orb <mc6809_os9_data8(dp)
; OS9-NEXT:   stb <mc6809_os9_data8(dp)
; CHECK-NEXT: rts

; A compare exchanges its operands and its condition: `d < v` is `v > d`,
; a compare against the symbol.
define i8 @lt(i16 %v) {
entry:
  %0 = load i16, ptr @d, align 1
  %cmp = icmp slt i16 %0, %v
  %r = zext i1 %cmp to i8
  ret i8 %r
}
; CHECK-LABEL: lt:
; ABS:        cmp{{[dx]}} d
; PIC:        cmp{{[dx]}} d,pc
; OS9:        cmp{{[dx]}} ,{{[xy]}}
; CHECK-NEXT: {{l?}}b{{gt|lt}}

; A store to the same memory between the load and its consumer forbids the
; fold: the consumer must not read the new value, so the load stays.
define i16 @clobbered(i16 %v) {
entry:
  %0 = load i16, ptr @d, align 1
  store i16 %v, ptr @d, align 1
  %add = add i16 %0, %v
  ret i16 %add
}
; CHECK-LABEL: clobbered:
; ABS:        ldd d
; PIC:        ldd d,pc
; OS9:        ldd ,{{[xy]}}
; ABS-NOT:    addd d
; PIC-NOT:    addd d,pc
; CHECK:      rts

; A value used twice keeps its register.
define i16 @twice(i16 %v) {
entry:
  %0 = load i16, ptr @d, align 1
  %add = add i16 %0, %v
  %mul = mul i16 %add, %0
  ret i16 %mul
}
; CHECK-LABEL: twice:
; ABS:        ldd d
; PIC:        ldd d,pc
; ABS-NOT:    addd d
; PIC-NOT:    addd d,pc
; CHECK:      rts
