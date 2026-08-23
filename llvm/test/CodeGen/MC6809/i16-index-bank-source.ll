; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mattr=-static-stack -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,MC6809
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mattr=-static-stack -mcpu=hd6309 -O2 %s -o - | FileCheck %s --check-prefixes=CHECK,HD6309

; A 16-bit value that arrives in an index register (a pointer argument
; converted to an integer) is a fine second source for accumulator
; arithmetic and compares: nothing copies it into the accumulator bank
; first. The 6809 reads it through the stack (`pshs x` / `op ,s++`) or, for
; a value still on the argument stack, straight from there; the HD6309 has
; the register-register forms.

; The pointer's own register is the second source: 6809 pushes it and pops
; it back through the subtract; HD6309 subr.
define i16 @sub_ptr(ptr %p, i16 %v) {
  %a = ptrtoint ptr %p to i16
  %r = sub i16 %v, %a
  ret i16 %r
}
; CHECK-LABEL: sub_ptr:
; CHECK:        ldd 2,s
; MC6809-NEXT:  pshs x
; MC6809-NEXT:  subd ,s++
; HD6309-NEXT:  subr x,d
; CHECK-NEXT:   tfr d,x
; CHECK-NEXT:   rts

; A second pointer argument sits on the stack: the bitwise op reads it
; from there (no index-register load, no push, no callee-saved Y).
define i16 @and_two_ptrs(ptr %p, ptr %q) {
  %a = ptrtoint ptr %p to i16
  %b = ptrtoint ptr %q to i16
  %r = and i16 %a, %b
  ret i16 %r
}
; CHECK-LABEL: and_two_ptrs:
; CHECK-NOT:    pshs
; CHECK:        tfr x,d
; MC6809-NEXT:  andb 3,s
; MC6809-NEXT:  anda 2,s
; HD6309-NEXT:  andd 2,s
; CHECK-NEXT:   tfr d,x
; CHECK-NEXT:   rts

; Same for a compare: cmpx against the stacked argument.
define i1 @cmp_two_ptrs(ptr %p, ptr %q) {
  %a = ptrtoint ptr %p to i16
  %b = ptrtoint ptr %q to i16
  %r = icmp slt i16 %a, %b
  ret i1 %r
}
; CHECK-LABEL: cmp_two_ptrs:
; CHECK-NOT:    pshs
; CHECK:        cmpx 2,s
; CHECK-NEXT:   {{l?}}blt

; A pointer loaded from memory and used as an integer right away: the load
; folds into the consumer through its bank copy.
define i16 @sub_loaded_ptr(ptr %pp, i16 %v) {
  %p = load ptr, ptr %pp, align 1
  %a = ptrtoint ptr %p to i16
  %r = sub i16 %v, %a
  ret i16 %r
}
; CHECK-LABEL: sub_loaded_ptr:
; CHECK:        ldd 2,s
; CHECK-NEXT:   subd ,x
; CHECK-NEXT:   tfr d,x
; CHECK-NEXT:   rts

; A compare of an accumulator value against a register operand that must
; survive: the pushed copy is consumed by the compare's post-increment, no
; pull to restore what was never written.
define i16 @cmp_keep_both(i16 %v, ptr %q) {
  %b = ptrtoint ptr %q to i16
  %r = icmp slt i16 %v, %b
  %s = select i1 %r, i16 %v, i16 %b
  ret i16 %s
}
; CHECK-LABEL: cmp_keep_both:
; MC6809:       pshs x
; MC6809-NEXT:  cmpd ,s++
; MC6809-NEXT:  {{l?}}blt
; HD6309:       cmpr x,d
; HD6309-NEXT:  {{l?}}blt
