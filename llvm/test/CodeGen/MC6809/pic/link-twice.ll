; Bug #197 v1: end-to-end PIC sanity check.
;
; The canonical proof that the backend really produces position-
; independent code: the same ELF object linked at two different
; -Ttext addresses must have byte-for-byte identical .text bytes
; (the load address only shifts where they land, never their
; content). Anything else means the codegen has snuck an absolute
; address into .text somewhere.
;
; Static (non-PIC) codegen FAILS this check because LDX #x bakes the
; final address of `x` into the LDX immediate operand. PIC succeeds
; because all references are PC-relative, resolved by the linker
; from `target - PC_at_offset`.

; RUN: llc -mtriple=mc6809 -O2 --relocation-model=pic -filetype=obj < %s -o %t.o

; RUN: ld.lld -m mc6809elf --image-base=0 -Ttext=0x4000 %t.o -o %t.4000.elf
; RUN: ld.lld -m mc6809elf --image-base=0 -Ttext=0x6000 %t.o -o %t.6000.elf

; Extract just the .text contents (-x .text -> raw hex of the section
; bytes; the section header offsets vary by load address but the
; contents must not).
; RUN: llvm-objcopy --dump-section=.text=%t.4000.text %t.4000.elf /dev/null
; RUN: llvm-objcopy --dump-section=.text=%t.6000.text %t.6000.elf /dev/null
; RUN: cmp %t.4000.text %t.6000.text

; (No FileCheck needed — `cmp` exiting 0 IS the check. lit fails the
; test if cmp returns non-zero.)

target triple = "mc6809-unknown-unknown"

@x = global i8 0
@y = global i8 1

define i8 @load_x() {
  %v = load i8, ptr @x
  ret i8 %v
}

define i8 @load_y() {
  %v = load i8, ptr @y
  ret i8 %v
}

define void @store_x(i8 %v) {
  store i8 %v, ptr @x
  ret void
}

define ptr @addr_x() {
  ret ptr @x
}

; A function pointer take + indirect call — exercises the "&func"
; lowering path which under PIC must also become LEAX func,PCR.
define i8 @call_via_ptr() {
  %fnptr = alloca ptr
  store ptr @load_x, ptr %fnptr
  %f = load ptr, ptr %fnptr
  %r = call i8 %f()
  ret i8 %r
}
