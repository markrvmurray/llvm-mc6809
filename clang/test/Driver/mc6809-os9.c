// Bug #163 Phase 2: clang driver awareness of the OS-9 triple.
//
// Asserts that `clang -target mc6809-unknown-os9 ...` produces:
//
//   1. A cc1 invocation with the canonical OS-9 triple.
//   2. A linker invocation that:
//        - uses mc6809-os9.lds (NOT link.ld);
//        - does NOT pull in the bare-metal startup libs (crt0/crt/c/
//          mc6809rt) but the OS-9 runtime (libclang_rt.os9: crt0 +
//          syscalls) and the OS-9 build of the compiler builtins, both
//          from the per-triple resource directory.
//
// A parallel plain-bare-metal run must continue to pull in the
// bare-metal defaults — guards against the OS-9 branch accidentally
// short-circuiting the bare-metal path.

void _start(void) {}

// Compile job: cc1 sees the canonical OS-9 triple.
// RUN: %clang -### -target mc6809-unknown-os9 -c -nostdinc %s -o /dev/null 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CC1
// CC1: "-triple" "mc6809-unknown-os9"

// Link job — OS-9 path.
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s -o %t.body 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LINK_OS9
// LINK_OS9:     ld.lld
// LINK_OS9:     "-Tmc6809-os9.lds"
// LINK_OS9-SAME: "-L{{.*}}mc6809-unknown-os9"
// LINK_OS9-SAME: "-lclang_rt.os9"
// LINK_OS9-SAME: "-lclang_rt.builtins"
// LINK_OS9-NOT: "-Tlink.ld"
// LINK_OS9-NOT: "-l:crt0.o"
// LINK_OS9-NOT: "-lcrt0"
// LINK_OS9-NOT: "-lmc6809rt"

// -nodefaultlibs keeps the startup files but drops the builtins;
// -nostdlib drops both.
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc -nodefaultlibs %s -o %t.body 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NODEFAULT
// NODEFAULT:     "-lclang_rt.os9"
// NODEFAULT-NOT: "-lclang_rt.builtins"
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc -nostdlib %s -o %t.body 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOSTDLIB
// NOSTDLIB-NOT: "-lclang_rt.os9"
// NOSTDLIB-NOT: "-lclang_rt.builtins"

// Link job — bare-metal path (must remain unchanged).
// RUN: %clang -### -target mc6809 -nostdinc %s -o %t.elf 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LINK_BARE
// LINK_BARE:     ld.lld
// LINK_BARE:     "-Tlink.ld"
// LINK_BARE-NOT: "-Tmc6809-os9.lds"
