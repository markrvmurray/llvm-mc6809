; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -O2 \
; RUN:     -mc6809-static-stack-dp-avail=200 %s -o - \
; RUN:   | FileCheck %s --check-prefix=DP
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -O2 \
; RUN:     -mc6809-static-stack-dp-avail=0 %s -o - | FileCheck %s --check-prefix=EXT

; A static-stack frame access that lands inside a BUNDLE must still have its
; TI_STATIC_STACK target index rewritten to the frame global.
;
; The HD6309 indexed byte/word arithmetic expansion has no encoding for an
; operation whose destination is AW, so it brackets the op between a pair of
; EXG AW<->AD and bundles the three together. When the bracketed op is a frame
; access, its target index sits in the bundle interior. MC6809StaticStackAlloc
; iterated the block with MachineBasicBlock's default iterator, which visits
; only bundle heads, so that one operand was never rewritten and reached
; MCInstLower as a raw target index -- killing the compiler with
; "Operand type not implemented."
;
; AW exists only on HD6309, which is why base 6809 never saw this.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

define dso_local i16 @issignaling(double noundef %x) #0 {
entry:
  %0 = bitcast double %x to i64
  %shl = shl i64 %0, 1
  %xor = xor i64 %shl, 4503599627370496
  %cmp = icmp ugt i64 %xor, -4503599627370496
  %conv = zext i1 %cmp to i16
  ret i16 %conv
}

; The bundled EOR must name the frame symbol. Page-0 placement additionally
; swaps it to the 1-byte direct-page form, exactly like the unbundled accesses
; around it.
; DP: eord {{.*}}issignaling_sstk
; DP: .section{{.*}}.dp.bss.static_stack

; With no page-0 budget the same access stays extended, but still resolves to
; the symbol rather than a target index.
; EXT: eord {{.*}}issignaling_sstk

attributes #0 = { minsize nofree norecurse nosync nounwind optsize willreturn memory(none) "target-cpu"="hd6309" }
