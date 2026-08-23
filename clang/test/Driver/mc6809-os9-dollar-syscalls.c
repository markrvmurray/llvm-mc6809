// NitrOS-9 dollar-in-identifier syscall names.
//
// Asserts that the mc6809-unknown-os9 driver auto-enables
// -fdollars-in-identifiers (so a #include <os9.h> consumer can use
// F$Exit / I$Read / I$Write verbatim without per-file flags), that the
// runtime defines those names at the same addresses as the POSIX-style
// ones, and that the direct shims in os9.h are inline `os9` instructions
// rather than calls.
//
// REQUIRES: mc6809-registered-target

#include <os9.h>

void use_dollar(void) {
    I$Write(1, "hi", 2);
    F$Exit(0);
}

void use_posix(void) {
    _write(1, "hi", 2);
    _exit(0);
}

int use_direct(void) {
    char buf[4];
    int n = 2, path;
    int err = _os_write(1, "hi", &n);
    err += _os_writeln(1, "hi", &n);
    err += _os_read(0, buf, &n);
    err += _os_readln(0, buf, &n);
    err += _os_open("/dd/x", OS9_READ, &path);
    err += _os_close(path);
    return err;
}

// 1. Driver passes -fdollars-in-identifiers to cc1 for the OS-9 triple.
// RUN: %clang -### -target mc6809-unknown-os9 -c %s \
// RUN:   -I %S/../../../compiler-rt/lib/builtins/mc6809-os9/include 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CC1
// CC1: "-fdollars-in-identifiers"

// 2. The compile: both name flavours are external references; the direct
//    shims are not -- they are the system-call instructions themselves.
// RUN: %clang -target mc6809-unknown-os9 -Os \
// RUN:   -I %S/../../../compiler-rt/lib/builtins/mc6809-os9/include \
// RUN:   -c %s -o %t.o
// RUN: llvm-readelf -s %t.o | FileCheck %s --check-prefix=REFS
// REFS-DAG: UND _exit
// REFS-DAG: UND _write
// REFS-DAG: UND F$Exit
// REFS-DAG: UND I$Write
// REFS-DAG: UND errno
// REFS-NOT: _os_
// RUN: llvm-objdump -d --triple=mc6809-unknown-os9 %t.o \
// RUN:   | FileCheck %s --check-prefix=SHIMS
// SHIMS-LABEL: <use_direct>:
// SHIMS-DAG: os9 $8a
// SHIMS-DAG: os9 $8c
// SHIMS-DAG: os9 $89
// SHIMS-DAG: os9 $8b
// SHIMS-DAG: os9 $84
// SHIMS-DAG: os9 $8f

// 3. The runtime object defines both name flavours at the same addresses
//    (the alias attribute in syscalls.c) and errno.
// RUN: %clang -target mc6809-unknown-os9 -Os \
// RUN:   -I %S/../../../compiler-rt/lib/builtins/mc6809-os9/include \
// RUN:   -c %S/../../../compiler-rt/lib/builtins/mc6809-os9/syscalls.c \
// RUN:   -o %t.syscalls.o
// RUN: llvm-readelf -s %t.syscalls.o | FileCheck %s --check-prefix=DEFS
// DEFS-DAG: FUNC{{.+}}_exit
// DEFS-DAG: FUNC{{.+}}F$Exit
// DEFS-DAG: FUNC{{.+}}_write
// DEFS-DAG: FUNC{{.+}}I$Write
// DEFS-DAG: FUNC{{.+}}_read
// DEFS-DAG: FUNC{{.+}}I$Read
// DEFS-DAG: OBJECT{{.+}}errno

// 4. Linking pulls in syscalls.o cleanly -- both call sites resolve.
//    Override the script's OS-9 default with --oformat=binary; this
//    test is checking symbol resolution, not the module wrapper.
//    _exit gives the floating-point module back when there is one, which
//    reaches crt1's module pointer and the F$UnLink stub.  Neither is in
//    this link, deliberately -- it is the shims' own symbols being checked.
//    The module pointer is a real word in .data because the writer will not
//    take a data-area relocation against anything else; nothing here runs.
// RUN: echo '.data'                     > %t.stub.s
// RUN: echo '.globl __os9_fp_module'   >> %t.stub.s
// RUN: echo '__os9_fp_module: .word 0' >> %t.stub.s
// RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj %t.stub.s -o %t.stub.o
// RUN: ld.lld -T %S/../../../compiler-rt/lib/builtins/mc6809-os9/mc6809-os9.lds \
// RUN:   --oformat=binary --defsym=_start=use_posix --defsym=__os9_unlink=0 \
// RUN:   %t.o %t.syscalls.o %t.stub.o -o %t.body
// RUN: test -s %t.body
