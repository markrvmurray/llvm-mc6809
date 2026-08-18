// REQUIRES: mc6809-registered-target
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -O2 -S -o - %s | FileCheck %s
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -O2 -emit-obj -o %t.o %s
// RUN: llvm-objdump -d --triple=mc6809-unknown-os9 %t.o | FileCheck %s --check-prefix=OBJ

// The `os9 <code>` construct from C inline asm: the function code is an
// immediate constraint printed bare (%0) or via %c0, both of which the
// assembler accepts, and it lands as SWI2 + the in-line code byte.

#define F_Exit 0x06
#define I_Write 0x8A

// CHECK-LABEL: _exit:
// CHECK: os9 6
// OBJ: 10 3f 06 {{.*}}os9 $6
__attribute__((noreturn)) void _exit(int status) {
  asm volatile("os9 %0" : : "i"(F_Exit), "d"(status) : "memory");
  __builtin_unreachable();
}

// CHECK-LABEL: write_ok:
// CHECK: os9 138
// OBJ: 10 3f 8a {{.*}}os9 $8a
_Bool write_ok(void) {
  _Bool err;
  asm volatile("os9 %c1" : "=c"(err) : "i"(I_Write) : "memory");
  return !err;
}
