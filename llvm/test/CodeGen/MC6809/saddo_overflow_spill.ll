; RUN: llc -O0 -global-isel -mtriple=mc6809 %s -o - | FileCheck %s
; RUN: llc -O0 -global-isel -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s

; A signed add-overflow branch must NOT be lowered to a direct branch-on-V
; (`lbvc`/`lbvs`). Every 6809 store clears CC.V, and the register
; allocator inserts a spill store between the V-producing `adca`/`sbca` and the
; branch (always at -O0, where the sum is spilled the instant it is defined).
; A live-V branch then reads a V the spill has already cleared and silently
; misses the overflow. The overflow bit must be captured into a byte at the
; producer (`tfr cc,<r>`) before any store.
;
; Repro shape from the bug: saddo of x and a runtime-negated y, spilling under
; -O0 byte pressure. Verified on-usim: the pre-fix code missed overflow for the
; INT16_MIN edge (e.g. saddo16(-32768, 1) returned 32767 instead of the
; overflow sentinel).

declare { i16, i1 } @llvm.sadd.with.overflow.i16(i16, i16)

define i16 @saddo16(i16 %x, i16 %y) {
; CHECK-LABEL: saddo16:
; The overflow flag is captured off CC into a byte, not branched on live.
; CHECK:      tfr cc,
; CHECK-NOT:  lbvc
; CHECK-NOT:  lbvs
; CHECK-NOT:  bvc
; CHECK-NOT:  bvs
entry:
  %ny = sub i16 0, %y
  %o = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %x, i16 %ny)
  %bit = extractvalue { i16, i1 } %o, 1
  %sum = extractvalue { i16, i1 } %o, 0
  br i1 %bit, label %ovf, label %ok
ovf:
  ret i16 -999
ok:
  ret i16 %sum
}
