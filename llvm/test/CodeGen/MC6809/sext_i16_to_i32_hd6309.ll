; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 < %s | FileCheck --check-prefix=HD6309 %s
; RUN: llc -mtriple=mc6809 -mcpu=mc6809 -O1 < %s | FileCheck --check-prefix=MC6809 %s
;
; Bug #161 Phase 4 sentinel: HD6309 SEXW for i16 → i32 sign-extend.
;
; On HD6309 the legalizer now marks G_SEXT (s32, s16) as legal (via a
; SEX32Implicit pseudo) so that instruction selection can emit a single
; SEXW instead of the 4-instruction branch-diamond that plain 6809 uses.
;
; Key check: HD6309 emits 'sexw'; MC6809 emits the sign-bit diamond
; (cmpd #0 / branch / lda #0 or #1 / …).

define i32 @sext_arg(i16 %x) {
; HD6309-LABEL: sext_arg:
; HD6309: sexw
; HD6309-NOT: lblt
;
; MC6809-LABEL: sext_arg:
; MC6809-NOT: sexw
; MC6809:     cmp{{[dxy]}} #0
; MC6809:     {{l?}}blt
  %ext = sext i16 %x to i32
  ret i32 %ext
}

define i32 @sext_load(ptr %p) {
; HD6309-LABEL: sext_load:
; HD6309: sexw
; HD6309-NOT: lblt
;
; MC6809-LABEL: sext_load:
; MC6809-NOT: sexw
; MC6809:     cmp{{[dxy]}} #0
; MC6809:     {{l?}}blt
  %v = load i16, ptr %p
  %ext = sext i16 %v to i32
  ret i32 %ext
}
