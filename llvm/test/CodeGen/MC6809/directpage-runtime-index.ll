; NOTE: Hand-authored — do not autogen. The CHECK lines pin the specific
; addressing the runtime-indexed direct-page lowering produces.
; RUN: llc -mtriple=mc6809 -relocation-model=static -O0 -filetype=asm -o - %s | FileCheck %s
; RUN: llc -mtriple=mc6809 -relocation-model=pic    -O0 -filetype=asm -o - %s | FileCheck %s

; A RUNTIME index into an addrspace(1) (direct-page) object cannot use direct
; mode, which needs a constant 8-bit operand. At -O0 the index is not folded to
; a constant, so a G_PTR_ADD on a p1 pointer reaches the backend. It is lowered
; to an 8-bit in-page add (the object never leaves its 256-byte page), then the
; load forms the object's full 16-bit address — the linker-provided direct-page
; base __dp_base_addr plus the in-page offset — and uses ordinary indexed
; addressing.
;
; The in-page offset of the object (mc6809_8(zero_array)) is fixed at link time
; and so already position-independent: it must be materialised as an immediate
; in BOTH the static and PIC models, never PC-relatively (which would combine
; the direct-page-offset relocation with PCR and produce garbage).

@zero_array = internal addrspace(1) global [4 x i8] zeroinitializer, align 1

define i8 @probe(i16 %i) {
entry:
  %arrayidx = getelementptr inbounds [4 x i8], ptr addrspace(1) @zero_array, i16 0, i16 %i
  %v = load i8, ptr addrspace(1) %arrayidx, align 1
  ret i8 %v
}

; CHECK-LABEL: probe:
; The object's in-page offset, as a position-independent immediate.
; CHECK: ldx #mc6809_8(zero_array)
; The direct-page base, then the full address via indexed addressing.
; CHECK: ldx #__dp_base_addr
; CHECK: ldb d,x

; Must never use the (HD6309-only) ABX, nor a PC-relative direct-page offset.
; CHECK-NOT: abx
; CHECK-NOT: leax mc6809_8({{.*}}),pc
