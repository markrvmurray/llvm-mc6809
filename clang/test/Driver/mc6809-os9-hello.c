// Bug #163 Phase 2 — END-TO-END test: hello.c through the full pipeline.
//
// Drives the complete OS-9 build:
//
//   1. `clang -target mc6809-unknown-os9` compiles this hello.c +
//      crt0.S + syscalls.S into a flat module body using the
//      mc6809-os9.lds linker script (auto-passed by the driver).
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
// Build pipeline (manual chain):
// RUN: %clang -target mc6809-unknown-os9 \
// RUN:   -Wl,-L,%S/../../../compiler-rt/lib/builtins/mc6809-os9 \
// RUN:   %s \
// RUN:   %S/../../../compiler-rt/lib/builtins/mc6809-os9/crt0.S \
// RUN:   %S/../../../compiler-rt/lib/builtins/mc6809-os9/syscalls.S \
// RUN:   -o %t.body
//
// RUN: %S/../../../tools/os9-link %t.body \
// RUN:   --name hello --exec 13 --mem 1024 -o %t.os9
//
// RUN: %S/../../../tools/os9-module-check %t.os9 \
// RUN:   | FileCheck %s --check-prefix=VALID
//
// VALID: OK ({{[0-9]+}} bytes)
// VALID: type/lang: $11 (Prgrm|Objct)
// VALID: exec offset: $000D (13)
// VALID: name: "hello"
// VALID: CRC: valid (full CRC = $800FE3)
//
// Body content sanity: the string "Hello, world!\n" must appear as
// 14 ASCII bytes in the module body (.rodata is in the body, accessed
// PCR by the _write call).  Note the string starts at some PCR offset
// — we don't pin its exact location, just its bytes.
//
// RUN: xxd -p -c 65536 %t.body > %t.body.hex
// RUN: FileCheck %s --check-prefix=GREETING < %t.body.hex
// GREETING: 48656c6c6f2c20776f726c64210a
//
// And the I$Write syscall sequence (10 3f 8a) and F$Exit
// (10 3f 06) must both appear in the body (the CRT calls main() →
// _write → _exit chain).
// RUN: FileCheck %s --check-prefix=WRITE_CALL < %t.body.hex
// WRITE_CALL: 103f8a
// RUN: FileCheck %s --check-prefix=EXIT_CALL < %t.body.hex
// EXIT_CALL: 103f06

extern int _write(int fd, const char *buf, int n);

int main(void) {
    _write(1, "Hello, world!\n", 14);
    return 0;
}
