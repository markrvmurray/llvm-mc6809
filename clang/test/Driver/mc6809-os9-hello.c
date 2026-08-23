// Bug #163 Phase 2 — END-TO-END test: hello.c through the full pipeline.
//
// Drives the complete OS-9 build:
//
//   1. `clang -target mc6809-unknown-os9` compiles this hello.c and
//      links it with libclang_rt.os9.a (crt0 + syscalls) into a module
//      body using the mc6809-os9.lds linker script (auto-passed by the
//      driver).
//   2. The os9-link post-link wrapper prepends the OS-9 header,
//      computes parity + CRC.
//   3. os9-module-check validates the final module.
//
// The C source below uses the syscall stubs directly (no picolibc
// — that lands with Phase 4).  The expected runtime behaviour on a
// NitrOS-9 emulator is: write "Hello, world!\n" to path 1 (stdout)
// and F$Exit cleanly.
//
// Driver auto-invocation of os9-link is intentionally NOT wired here
// (it would need __mem_size extraction from a parallel ELF link); for
// Phase 2 the build is two commands.  Phase 3 / native lld OS-9
// output subsumes os9-link.
//
// REQUIRES: mc6809-registered-target
//
// TRUE ONE-SHOT.  Phase 3 wired the lld OS-9 writer + driver auto-
// passing, so a bare clang invocation now produces a finished .os9
// module: the driver derives --os9-name from the output stem (here
// "hello"), the linker script's __mem_size symbol supplies M$Mem,
// the .lds's OUTPUT_FORMAT(os9-program-module) triggers the wrapper.
// No --os9-* flags, no Python post-link, no separate .S inputs.
//
// RUN: %clang -target mc6809-unknown-os9 %s -o %t.os9
// RUN: %S/../../../tools/os9-module-check %t.os9 \
// RUN:   | FileCheck %s --check-prefix=VALID
//
// VALID: OK ({{[0-9]+}} bytes)
// VALID: type/lang: $11 (Prgrm|Objct)
// VALID: exec offset: $000D (13)
// VALID: name: "{{.+}}"
// VALID: CRC: valid (full CRC = $800FE3)
//
// Body content sanity: the string "Hello, world!\n" must appear in
// the module (.rodata flows through the link unchanged) and the
// I$Write + F$Exit syscall sequences must be present.
//
// RUN: xxd -p -c 65536 %t.os9 > %t.os9.hex
// RUN: FileCheck %s --check-prefix=GREETING < %t.os9.hex
// GREETING: 48656c6c6f2c20776f726c64210a
// RUN: FileCheck %s --check-prefix=WRITE_CALL < %t.os9.hex
// WRITE_CALL: 103f8a
// RUN: FileCheck %s --check-prefix=EXIT_CALL < %t.os9.hex
// EXIT_CALL: 103f06

#include <os9.h>

int main(void) {
    /* Using the native NitrOS-9 syscall name via <os9.h>.  The same
       call could be spelled `_write(...)` -- both are the same function
       (the alias in syscalls.c). */
    I$Write(1, "Hello, world!\n", 14);
    return 0;
}
