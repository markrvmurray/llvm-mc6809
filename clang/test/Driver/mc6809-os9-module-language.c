// An OS-9 program module says what language its code is in, and the kernel
// matches a module by type *and* language.  OS-9 defines a language for 6309
// object code -- $17, Obj6309 -- and the obvious thing to do with -mcpu=hd6309
// is to emit it.  That produces a module nothing will run: NitrOS-9 asks for
// $11, and its own 6309 build marks every module $11 too, so a $17 module is
// refused by a 6309 system exactly as it is by a 6809 one -- error 234,
// non-existent module, which reads like a missing file.
//
// So the driver never overrides the language.  lld defaults to $11 and still
// takes --os9-type from anybody who wants the other one deliberately.

// RUN: %clang --target=mc6809-unknown-os9 -mcpu=hd6309 -### %s 2>&1 \
// RUN:   | FileCheck %s
// RUN: %clang --target=mc6809-unknown-os9 -mcpu=6309 -### %s 2>&1 \
// RUN:   | FileCheck %s
// RUN: %clang --target=mc6809-unknown-os9 -### %s 2>&1 | FileCheck %s

// CHECK-NOT: "--os9-type={{.*}}"

int main(void) { return 0; }
