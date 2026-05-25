; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301 (2026-05-16): signed/unsigned i32
; ICMP against a constant RHS with a G_BRCOND consumer must lower
; via the fused CompareBranch_i32_Imm pseudo (post-RA-expanding to
; SUBW #lo + SBCD #hi + LB<cc>) NOT via the __cmpsi2 libcall.
;
; This is THE critical foundation step that fixes Bug A from the
; previously-reverted attempt that lacked the fused pseudo — the
; separated Compare_i32_Imm + ConditionalImm path leaves CCond dead
; and the conditional branch reads undef CC flags.

; CHECK-LABEL: i32_slt_brcond:
define i16 @i32_slt_brcond(i32 %x) {
entry:
  ; SUBW #42 + SBCD #0 + branch.  The branch folder may invert the
  ; predicate to fall through to iftrue, hence we accept either b{lt|ge}.
  ; CHECK: subw #42
  ; CHECK: sbcd #0
  ; CHECK: {{l?}}b{{(lt|ge)}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp slt i32 %x, 42
  br i1 %cmp, label %iftrue, label %iffalse
iftrue:
  ret i16 1
iffalse:
  ret i16 0
}

; CHECK-LABEL: i32_ult_brcond:
define i16 @i32_ult_brcond(i32 %x) {
entry:
  ; Unsigned predicate → BLO / BHS branch.
  ; CHECK: subw #1000
  ; CHECK: sbcd #0
  ; CHECK: {{l?}}b{{(lo|hs)}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp ult i32 %x, 1000
  br i1 %cmp, label %iftrue, label %iffalse
iftrue:
  ret i16 1
iffalse:
  ret i16 0
}

; CHECK-LABEL: i32_sgt_big_brcond:
define i16 @i32_sgt_big_brcond(i32 %x) {
entry:
  ; Bug #346: signed GT is lowered as GE against K+1 (0x12345678+1 =
  ; 0x12345679) so the branch reads only N/V (correct after SBCD) and not
  ; the stale high-half Z. Low half becomes 0x5679 = 22137; high unchanged.
  ; CHECK: subw #22137
  ; CHECK: sbcd #4660
  ; CHECK: {{l?}}b{{(ge|lt)}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sgt i32 %x, 305419896
  br i1 %cmp, label %iftrue, label %iffalse
iftrue:
  ret i16 1
iffalse:
  ret i16 0
}

; CHECK-LABEL: i32_slt_reg_falls_through:
define i16 @i32_slt_reg_falls_through(i32 %x, i32 %y) {
entry:
  ; Register RHS (not constant) — step 3.1 only handles constant RHS,
  ; so this falls through to the __cmpsi2 libcall path.  Steps 5/6
  ; will land Compare_i32_Reg / Compare_i32_Mem.
  ; CHECK: lbsr __cmpsi2
  %cmp = icmp slt i32 %x, %y
  br i1 %cmp, label %iftrue, label %iffalse
iftrue:
  ret i16 1
iffalse:
  ret i16 0
}
