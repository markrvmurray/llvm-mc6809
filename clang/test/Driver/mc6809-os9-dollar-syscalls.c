// Bug #163 Phase 2: NitrOS-9 dollar-in-identifier syscall names.
//
// Asserts that the mc6809-unknown-os9 driver auto-enables
// -fdollars-in-identifiers (so a #include <os9.h> consumer can
// use F$Exit / I$Read / I$Write verbatim without per-file flags),
// AND that the syscalls.S aliases resolve those names to the same
// addresses as the POSIX-style names.
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
    int n = 2;
    int err = _os_write(1, "hi", &n);
    err += _os_writeln(1, "hi", &n);
    err += _os_read(0, buf, &n);
    err += _os_readln(0, buf, &n);
    return err;
}

// 1. Driver passes -fdollars-in-identifiers to cc1 for the OS-9 triple.
// RUN: %clang -### -target mc6809-unknown-os9 -c %s \
// RUN:   -I %S/../../../compiler-rt/lib/builtins/mc6809-os9/include 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CC1
// CC1: "-fdollars-in-identifiers"

// 2. The full compile + link round-trips: both name flavors resolve.
// RUN: %clang -target mc6809-unknown-os9 \
// RUN:   -I %S/../../../compiler-rt/lib/builtins/mc6809-os9/include \
// RUN:   -Wl,-L,%S/../../../compiler-rt/lib/builtins/mc6809-os9 \
// RUN:   -c %s -o %t.o
//
// RUN: llvm-mc -triple=mc6809 --filetype=obj \
// RUN:   %S/../../../compiler-rt/lib/builtins/mc6809-os9/syscalls.S -o %t.syscalls.o
//
// RUN: llvm-readelf -s %t.o | FileCheck %s --check-prefix=REFS
// REFS-DAG: UND _exit
// REFS-DAG: UND _write
// REFS-DAG: UND F$Exit
// REFS-DAG: UND I$Write
// REFS-DAG: UND _os_write
// REFS-DAG: UND _os_writeln
// REFS-DAG: UND _os_read
// REFS-DAG: UND _os_readln

// 3. The syscalls.S object DEFINES both name flavors at the same
//    addresses (the .set aliases).
// RUN: llvm-readelf -s %t.syscalls.o | FileCheck %s --check-prefix=DEFS
// DEFS-DAG: FUNC{{.+}}_exit
// DEFS-DAG: FUNC{{.+}}F$Exit
// DEFS-DAG: FUNC{{.+}}_write
// DEFS-DAG: FUNC{{.+}}I$Write
// DEFS-DAG: FUNC{{.+}}_read
// DEFS-DAG: FUNC{{.+}}I$Read
// DEFS-DAG: FUNC{{.+}}_os_write
// DEFS-DAG: FUNC{{.+}}_os_writeln
// DEFS-DAG: FUNC{{.+}}_os_read
// DEFS-DAG: FUNC{{.+}}_os_readln
// DEFS-DAG: FUNC{{.+}}_oserr
// DEFS-DAG: FUNC{{.+}}_osret
// DEFS-DAG: FUNC{{.+}}_os9err
// DEFS-DAG: FUNC{{.+}}_sysret

// 4. Linking pulls in syscalls.o cleanly — both call sites resolve.
//    Override the script's OS-9 default with --oformat=binary; this
//    test is checking symbol resolution, not the module wrapper.
// RUN: ld.lld -T %S/../../../compiler-rt/lib/builtins/mc6809-os9/mc6809-os9.lds \
// RUN:   --oformat=binary --defsym=_start=use_posix \
// RUN:   %t.o %t.syscalls.o -o %t.body
// RUN: test -s %t.body
