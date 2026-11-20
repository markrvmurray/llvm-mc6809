; Inline-asm memory operands ("=m") whose addresses the allocator has to
; place: with A, B and X clobbered by each asm, the addresses can only go in
; Y.  The allocator asks the target which class holds an inline-asm memory
; address -- an index register, since the operand prints as `0,r` -- and
; that used to hit an unreachable on both CPUs.
;
; RUN: llc -mtriple=mc6809-unknown-os9 -mcpu=hd6309 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mtriple=mc6809-unknown-os9 -mcpu=mc6809 -verify-machineinstrs < %s | FileCheck %s

; CHECK-LABEL: main:
; CHECK:       leay mc6809_os9_data(w1),u
; CHECK:       std 0,y
; CHECK:       leay mc6809_os9_data(w2),u
; CHECK:       std 0,y
; CHECK:       leay mc6809_os9_data(w3),u
; CHECK:       std 0,y
; CHECK:       lbsr printf

@w1 = internal global i16 0, align 1
@w2 = internal global i16 0, align 1
@w3 = internal global i16 0, align 1
@.str = private unnamed_addr constant [16 x i8] c"%04x %04x %04x\0A\00", align 1
define dso_local noundef i16 @main() #0 {
entry:
  tail call void asm sideeffect "ldd #$$1234\0A\09swi2\0A\09.byte 12\0A\09std $0", "=*m,~{a},~{b},~{x},~{cc},~{memory}"(ptr nonnull elementtype(i16) @w1)
  tail call void asm sideeffect "ldd #$$1234\0A\09lda #99\0A\09swi2\0A\09.byte 143\0A\09std $0", "=*m,~{a},~{b},~{x},~{cc},~{memory}"(ptr nonnull elementtype(i16) @w2)
  tail call void asm sideeffect "ldd #$$5678\0A\09ldx #1\0A\09swi2\0A\09.byte 10\0A\09std $0", "=*m,~{a},~{b},~{x},~{cc},~{memory}"(ptr nonnull elementtype(i16) @w3)
  %0 = load i16, ptr @w1, align 1
  %1 = load i16, ptr @w2, align 1
  %2 = load i16, ptr @w3, align 1
  %call = tail call i16 (ptr, ...) @printf(ptr noundef nonnull dereferenceable(1) @.str, i16 noundef %0, i16 noundef %1, i16 noundef %2)
  ret i16 0
}
declare noundef i16 @printf(ptr noundef readonly captures(none), ...)
attributes #0 = { minsize nounwind optsize }
