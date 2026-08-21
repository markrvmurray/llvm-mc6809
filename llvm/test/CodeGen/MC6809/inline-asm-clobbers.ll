; Inline-asm register clobbers have to reach the machine instruction, or the
; allocator keeps a live value in a register the asm destroys.  That is not
; hypothetical: a write() loop kept its counter in D, the `clrb` an asm needed
; took half of it, and the loop printed for ever on a real machine.
;
; The names are the ones clang offers for this target.  They are matched
; against the *assembler* spelling -- `d`, not the TableGen record name `AD` --
; which is why MC6809RegisterInfo overrides getRegAsmName; the inherited one
; returns the record name and every clobber here was silently dropped.

; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -stop-after=irtranslator %s -o - \
; RUN:   | FileCheck %s

define void @clobbers() {
entry:
  call void asm sideeffect "nop", "~{d},~{a},~{b},~{x},~{y},~{u},~{w},~{q},~{e},~{f}"()
  ret void
}

; CHECK: INLINEASM
; CHECK-SAME: early-clobber $ad
; CHECK-SAME: early-clobber $aa
; CHECK-SAME: early-clobber $ab
; CHECK-SAME: early-clobber $ix
; CHECK-SAME: early-clobber $iy
; CHECK-SAME: early-clobber $su
; CHECK-SAME: early-clobber $aw
; CHECK-SAME: early-clobber $aq
; `e` and `f` are the 6309 accumulator halves, not the CC bits named E and F.
; The generic lookup is case-insensitive and chose the flags.
; CHECK-SAME: early-clobber $ae
; CHECK-SAME: early-clobber $af

; And the effect that matters: a value live across such an asm must not stay
; in the clobbered register.
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s --check-prefix=ALLOC

define i16 @live_across(i16 %n) {
entry:
  call void asm sideeffect "nop", "~{d},~{cc}"()
  ret i16 %n
}

; ALLOC-LABEL: live_across:
; ALLOC-NOT: addd
