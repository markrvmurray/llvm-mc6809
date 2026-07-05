; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -O2 \
; RUN:     -mc6809-static-stack-dp-avail=200 %s -o - \
; RUN:   | FileCheck %s --check-prefix=DP --implicit-check-not='mc6809_8({{.*}}),pc'
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -O2 \
; RUN:     -mc6809-static-stack-dp-avail=0 %s -o - | FileCheck %s --check-prefix=EXT
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -O2 \
; RUN:     -mc6809-static-stack-dp-avail=200 -filetype=obj %s -o %t.o
; RUN: llvm-readobj --section-headers %t.o | FileCheck %s --check-prefix=NOBITS

; Direct-page placement of the static-stack frame. When the
; whole static stack fits in the page-0 window (`-mc6809-static-stack-dp-avail`),
; the frame global lands in `.dp.bss` and every frame access uses the 1-byte
; direct-page opcode (`ldd <sym`) instead of 3-byte extended — erasing the
; base-MC6809 `.text` cost of static-stack. i32 is HD6309-only, so -mcpu=hd6309.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; A non-reentrant leaf whose i32 accumulate spills to the static frame.
;
; With DP placement the frame accesses take the direct-page form (`<mc6809_8(...)`)
; but the address-OF the frame (LEA, for the indexed load base) must stay a full
; 16-bit reference — a truncated `leay <mc6809_8(...)` would be a miscompile.
define dso_local i32 @ss_leaf(i32 noundef %seed, i16 noundef %n) local_unnamed_addr #0 {
; DP-LABEL: ss_leaf:
; DP:         .Lss_leaf_sstk
; The i32 spill store/reload and the addw/adcd accumulate all use the 1-byte
; direct-page form against the page-0 frame.
; DP:         stq{{.*}}<mc6809_8(.Lss_leaf_sstk
; DP:         addw{{.*}}<mc6809_8(.Lss_leaf_sstk
; DP:         adcd{{.*}}<mc6809_8(.Lss_leaf_sstk
; The address-of the frame (a PC-relative leay for the index-load base) keeps a
; full 16-bit reference — it must NOT be truncated to the 8-bit mc6809_8() form,
; which would collapse `leay sym,pc` to the 8-bit PC-relative encoding and
; compute a wrong pointer. Enforced globally by the DP run's
; --implicit-check-not='mc6809_8({{.*}}),pc'.
;
; EXT-LABEL: ss_leaf:
; Feature on but DP budget 0: extended addressing, no direct-page `<` form.
; EXT:         stq{{.*}}.Lss_leaf_sstk
; EXT-NOT:     <mc6809_8
entry:
  %buf = alloca [4 x i32], align 1
  store i32 %seed, ptr %buf, align 1
  %add.1 = add nsw i32 %seed, 3
  %gep.1 = getelementptr inbounds nuw i8, ptr %buf, i16 4
  store i32 %add.1, ptr %gep.1, align 1
  %add.2 = add nsw i32 %seed, 6
  %gep.2 = getelementptr inbounds nuw i8, ptr %buf, i16 8
  store i32 %add.2, ptr %gep.2, align 1
  %add.3 = add nsw i32 %seed, 9
  %gep.3 = getelementptr inbounds nuw i8, ptr %buf, i16 12
  store i32 %add.3, ptr %gep.3, align 1
  %cmp = icmp sgt i16 %n, 0
  br i1 %cmp, label %body, label %done

done:                                             ; preds = %body, %entry
  %acc.lcssa = phi i32 [ 0, %entry ], [ %add8, %body ]
  ret i32 %acc.lcssa

body:                                             ; preds = %entry, %body
  %i = phi i16 [ %inc, %body ], [ 0, %entry ]
  %acc = phi i32 [ %add8, %body ], [ 0, %entry ]
  %and = and i16 %i, 3
  %idx = getelementptr inbounds nuw [4 x i8], ptr %buf, i16 %and
  %ld = load i32, ptr %idx, align 1
  %add8 = add nsw i32 %ld, %acc
  %inc = add nuw nsw i16 %i, 1
  %ec = icmp eq i16 %inc, %n
  br i1 %ec, label %done, label %body
}

; The frame global is emitted in the direct-page `.dp.bss` region as @nobits:
; zeroed at startup, no image bytes, and matching the linker's zero-init page-0
; region. A stray @progbits here would force the whole .dp.bss output loaded.
; NOBITS:      Name: .dp.bss.static_stack
; NOBITS-NEXT: Type: SHT_NOBITS

attributes #0 = { nounwind "nonreentrant" "target-cpu"="hd6309" }
