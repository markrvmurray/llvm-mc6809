; RUN: llc -mtriple=mc6809 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -stop-after=instruction-select \
; RUN:   -o - %s 2>&1 | FileCheck %s

; A carry or overflow flag consumed as a value is captured by the producer
; itself: the flag's s1 vreg becomes an implicit def of the SetCarry /
; SetOverflow pseudo with a byte register class, and the pseudo's post-RA
; expansion deposits the 0/1 flag byte into it right after the instruction
; that set the flag. No separate materialisation pseudo is built for it --
; a separate instruction let the allocator's spill of the producer's own
; result land between the two, and STB clears V -- and the old
; MaterializeCarryToByte_i8 / MaterializeOverflowToByte_i8 forms with a
; phantom BIT1 input are gone entirely.

; CHECK-LABEL: name: carry_anyext_to_byte
; The carry-out of the high limb, zero-extended to i8: the limb's pseudo
; carries the byte as an implicit def and the extension is a plain copy.
; CHECK-NOT:  MaterializeCC
; CHECK:      AddSetCarryUse_i8_{{.*}}implicit-def %[[B:[0-9]+]]
; CHECK-NEXT: COPY %[[B]]
; CHECK-NOT:  MaterializeCC
; CHECK-NOT:  MaterializeCarryToByte_i8
; CHECK-NOT:  MaterializeOverflowToByte_i8
define i8 @carry_anyext_to_byte(i16 %a, i16 %b) {
entry:
  %sum = call { i16, i1 } @llvm.uadd.with.overflow.i16(i16 %a, i16 %b)
  %carry = extractvalue { i16, i1 } %sum, 1
  %byte = zext i1 %carry to i8
  ret i8 %byte
}

; CHECK-LABEL: name: overflow_anyext_to_byte
; Signed-add overflow-out, the same way.
; CHECK-NOT:  MaterializeCC
; CHECK:      AddSetOverflowUse_i8_{{.*}}implicit-def %[[B:[0-9]+]]
; CHECK-NEXT: COPY %[[B]]
; CHECK-NOT:  MaterializeCC
; CHECK-NOT:  MaterializeOverflowToByte_i8
; CHECK-NOT:  MaterializeCarryToByte_i8
define i8 @overflow_anyext_to_byte(i16 %a, i16 %b) {
entry:
  %sum = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %a, i16 %b)
  %ov = extractvalue { i16, i1 } %sum, 1
  %byte = zext i1 %ov to i8
  ret i8 %byte
}

declare { i16, i1 } @llvm.uadd.with.overflow.i16(i16, i16)
declare { i16, i1 } @llvm.sadd.with.overflow.i16(i16, i16)
