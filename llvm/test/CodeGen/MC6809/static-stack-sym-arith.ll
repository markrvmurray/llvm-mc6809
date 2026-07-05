; RUN: llc -mtriple=mc6809 -mattr=+static-stack -mc6809-static-stack-dp-avail=0 -O2 -verify-machineinstrs %s -o - | FileCheck %s --check-prefix=SS
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -mc6809-static-stack-dp-avail=0 -O2 -verify-machineinstrs %s -o - | FileCheck %s --check-prefix=SS63
; RUN: llc -mtriple=mc6809 -mattr=+static-stack -O2 -relocation-model=pic -verify-machineinstrs %s -o - | FileCheck %s --check-prefix=PIC
; RUN: llc -mtriple=mc6809 -mattr=-static-stack -O2 -verify-machineinstrs %s -o - | FileCheck %s --check-prefix=DYN
; RUN: llc -mtriple=mc6809 -mattr=+static-stack -mc6809-static-stack-dp-avail=0 -O0 -verify-machineinstrs %s -o - | FileCheck %s --check-prefix=SS0

; Static-stack (_Sym) siblings of the arithmetic / bitwise / compare _Mem
; pseudos: when a non-reentrant function's local moves to the static frame,
; an operation reading it folds to the extended (or PC-relative under PIC)
; form against the per-function static-stack symbol instead of reloading
; through a register. The DYN run (feature off) is the contrast — the same
; IR keeps its dynamic frame and references no static-stack symbol.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; i16 add against a static-frame local: ADDD extended on both CPUs.
; At -O0 the add arrives as the legalizer's byte carry-chain instead —
; ADDB low (symbol+1) first, then ADCA high (symbol+0) — which used to
; crash outright: the StaticStackAlloc resolver pass was gated on the opt
; level while the frame marking is not, so the TI_STATIC_STACK operands
; reached MCInstLower unresolved (bug #397).
define dso_local i16 @ss_add16(i16 noundef %x, i16 noundef %n) local_unnamed_addr #0 {
; SS-LABEL: ss_add16:
; SS:         addd .Lss_add16_sstk
; SS63-LABEL: ss_add16:
; SS63:         addd .Lss_add16_sstk
; PIC-LABEL: ss_add16:
; PIC:         addd .Lss_add16_sstk,pc
; DYN-LABEL: ss_add16:
; DYN-NOT:     _sstk
; SS0-LABEL: ss_add16:
; SS0:         addb .Lss_add16_sstk+1
; SS0:         adca .Lss_add16_sstk
entry:
  %slot = alloca i16, align 1
  store volatile i16 %n, ptr %slot, align 1
  %ld = load volatile i16, ptr %slot, align 1
  %sum = add i16 %x, %ld
  ret i16 %sum
}

; Byte add against a static-frame local: ADDB extended.
define dso_local i8 @ss_add8(i8 noundef %x, i16 noundef %n) local_unnamed_addr #0 {
; SS-LABEL: ss_add8:
; SS:         addb .Lss_add8_sstk
; DYN-LABEL: ss_add8:
; DYN-NOT:     _sstk
entry:
  %slot = alloca i8, align 1
  %nt = trunc i16 %n to i8
  store volatile i8 %nt, ptr %slot, align 1
  %ld = load volatile i8, ptr %slot, align 1
  %sum = add i8 %x, %ld
  ret i8 %sum
}

; i16 AND against a static-frame local. Base 6809 has no ANDD: the _Sym
; expansion splits into two extended byte ops — low byte at symbol+1
; (big-endian), high byte at symbol+0. HD6309 uses ANDD extended directly.
define dso_local i16 @ss_and16(i16 noundef %x, i16 noundef %n) local_unnamed_addr #0 {
; SS-LABEL: ss_and16:
; SS:         andb .Lss_and16_sstk+1
; SS:         anda .Lss_and16_sstk
; SS63-LABEL: ss_and16:
; SS63:         andd .Lss_and16_sstk
; DYN-LABEL: ss_and16:
; DYN-NOT:     _sstk
entry:
  %slot = alloca i16, align 1
  store volatile i16 %n, ptr %slot, align 1
  %ld = load volatile i16, ptr %slot, align 1
  %r = and i16 %x, %ld
  ret i16 %r
}

; i16 compare against a static-frame local: the LHS arrives in X, so the
; index-domain compare folds to CMPX extended against the symbol.
define dso_local i16 @ss_cmp16(i16 noundef %x, i16 noundef %n) local_unnamed_addr #0 {
; SS-LABEL: ss_cmp16:
; SS:         cmpx .Lss_cmp16_sstk
; DYN-LABEL: ss_cmp16:
; DYN-NOT:     _sstk
entry:
  %slot = alloca i16, align 1
  store volatile i16 %n, ptr %slot, align 1
  %ld = load volatile i16, ptr %slot, align 1
  %c = icmp eq i16 %x, %ld
  br i1 %c, label %t, label %f

t:
  ret i16 1

f:
  ret i16 %x
}

attributes #0 = { nounwind "nonreentrant" }
