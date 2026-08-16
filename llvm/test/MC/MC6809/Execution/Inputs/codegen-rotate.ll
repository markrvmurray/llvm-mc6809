; LLVM IR functions for the 8-bit rotate execution test.
;
; The 6809 has no 8-bit rotate; ROL/ROR are 9-bit rotates through the
; carry. The backend builds a rotate from a shift that exists only to put
; the wrap-around bit into C, followed by ROL/ROR. That producer's byte
; result is dead, and it used to be deleted — leaving a bare rotate that
; brought in whatever the carry held. SimplifyCFG forms exactly this
; rotate for switch-range reduction, which is how it reached picolibc's
; vfprintf and made every conversion with a precision or length modifier
; fall through to the default arm.

declare i8 @llvm.fshl.i8(i8, i8, i8)
declare i8 @llvm.fshr.i8(i8, i8, i8)

define dso_local i8 @ror1(i8 noundef %x) local_unnamed_addr {
entry:
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 7)
  ret i8 %r
}

define dso_local i8 @rol1(i8 noundef %x) local_unnamed_addr {
entry:
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 1)
  ret i8 %r
}

define dso_local i8 @rol3(i8 noundef %x) local_unnamed_addr {
entry:
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 3)
  ret i8 %r
}

define dso_local i8 @ror3(i8 noundef %x) local_unnamed_addr {
entry:
  %r = call i8 @llvm.fshr.i8(i8 %x, i8 %x, i8 3)
  ret i8 %r
}

; The switch-range-reduction shape from vfprintf's size-modifier dispatch:
; subtract the smallest case value, rotate right by one, dispatch. Returns
; a small code per recognised character, 0 for anything else.
define dso_local i8 @classify(i8 noundef %c) local_unnamed_addr {
entry:
  %sub = add i8 %c, -42
  %rot = call i8 @llvm.fshl.i8(i8 %sub, i8 %sub, i8 7)
  switch i8 %rot, label %default [
    i8 0, label %star
    i8 2, label %dot
    i8 33, label %ell
    i8 31, label %aitch
  ]
star:
  ret i8 1
dot:
  ret i8 2
ell:
  ret i8 3
aitch:
  ret i8 4
default:
  ret i8 0
}

; 16-bit rotates. These carry a 16-bit amount, which the legalizer has to
; narrow itself; and the low byte's carry-in is bit 7 of the *other* byte,
; so it goes through the general byte-to-carry path rather than the
; single-byte fast path.
declare i16 @llvm.fshl.i16(i16, i16, i16)
declare i16 @llvm.fshr.i16(i16, i16, i16)

define dso_local i16 @rol16(i16 noundef %x) local_unnamed_addr {
entry:
  %r = call i16 @llvm.fshl.i16(i16 %x, i16 %x, i16 1)
  ret i16 %r
}

define dso_local i16 @ror16(i16 noundef %x) local_unnamed_addr {
entry:
  %r = call i16 @llvm.fshr.i16(i16 %x, i16 %x, i16 1)
  ret i16 %r
}
