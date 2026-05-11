; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #274 (Og-hd6309-mame hits in libc/stdio/bufio.c, libc/search/hash.c,
; libc/search/hash_page.c, libc/time/{mktime,tzset}.c, libc/xdr/xdr_rec.c):
; SEX32Implicit's TableGen InOperandList was AWc:$src, but isel placed
; the source vreg in ADc class — typically when MERGE_LOHI_i16 produces
; into AD. constrainSelectedInstRegOperands cannot retighten ADc → AWc
; (disjoint classes), so the resulting MIR carries an `acc16:ADc`
; operand where the pseudo expected `acc16:AWc`. -verify-machineinstrs
; flagged "Expected a AWc register, but got a ADc register".
;
; Fix mirrors ZEX32Implicit's Bug #161/#208 pattern: broaden
; InOperandList to ACC16:$src and have the post-RA expansion route
; non-AW sources through copyPhysReg(AW, src) before SEXWx. The
; intermediate AW write is safe — SEXW writes AD next anyway.

; CHECK-LABEL: sext_i16_to_i32_via_merge:
; CHECK-NOT: Bad machine code

define i32 @sext_i16_to_i32_via_merge(i8 %lo, i8 %hi) {
entry:
  ; Force a MERGE_LOHI_i16 → ADc class, then sign-extend to i32.
  %lo16 = zext i8 %lo to i16
  %hi16 = zext i8 %hi to i16
  %hishl = shl i16 %hi16, 8
  %merged = or i16 %hishl, %lo16
  %ext = sext i16 %merged to i32
  ret i32 %ext
}
