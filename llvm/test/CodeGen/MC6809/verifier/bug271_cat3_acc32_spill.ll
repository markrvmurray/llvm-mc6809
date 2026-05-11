; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #271 cat-3 (closed 2026-05-11):
; The spill framework's `loadStoreRegStackSlot` emits Load_i32_Mem /
; Store_i32_Mem with the original spilled vreg as operand. The vreg's
; class is ACC32 (= AQ + SPILL_Q0..3), but TableGen declares the
; instructions' operand class as AQc (singleton AQ) per Bug #208
; round 4 / Bug #210 round 5 — those constraints are load-bearing
; for user codegen's sub-reg aliasing reasoning, so widening
; Load_i32_Mem itself was rejected. Earlier `506beb30d6b0` attempt
; (reverted at `9892ecb62451`) added a fresh AQc-class temp + COPY
; in the spill framework — closed 20 of 26 hits at the time but
; pushed already-tight functions over the regalloc-pressure cliff
; via the live-range extension the COPY introduced.
;
; Fix: add spill-only variants `SpillLoad_i32_Mem` /
; `SpillStore_i32_Mem` with operand class ACC32 and explicit Defs
; covering the AQ sub-reg clobber set ([AA, AB, AE, AF, AD, AW]),
; mirroring the worst-case clobber AQc would have inferred via
; class hierarchy. The spill framework emits these; their post-RA
; expander is shared with Load_i32_Mem / Store_i32_Mem so codegen
; is identical. User-codegen path remains AQc-constrained — no
; live-range extension, no regalloc regression.
;
; Closes 53 of 59 Og-hd6309-mame verifier hits in
; libc/{stdio/bufio,search/{hash,hash_page},xdr/xdr_rec,time/{mktime,tzset},
; string/{strncat_s,strncpy_s}}.

; CHECK-LABEL: spill_i32_across_call:
; CHECK-NOT: Bad machine code

declare void @noinline_use(i32)

define i32 @spill_i32_across_call(i32 %a, i32 %b, i32 %c) {
entry:
  ; Three i32 values force regalloc to spill at least one to the
  ; SPILL_Q* register class. The call across the spilled values
  ; ensures the spill framework's loadStoreRegStackSlot fires for
  ; ACC32-class vregs.
  %sum = add i32 %a, %b
  call void @noinline_use(i32 %sum)
  %sum2 = add i32 %sum, %c
  call void @noinline_use(i32 %sum2)
  ret i32 %sum2
}
