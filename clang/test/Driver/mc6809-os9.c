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

// The module name comes from the output file, keeping only what both kernels
// accept in a name: Level 1 ends a name at a dash, and allows 29 characters.
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s -o %t.dir/my-prog.os9 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NAME_DASH
// NAME_DASH: "--os9-name=myprog"
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s -o %t.dir/Ab_9.x-y.os9 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NAME_DOT
// NAME_DOT: "--os9-name=Ab_9.xy"
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s -o %t.dir/.my-prog.os9 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NAME_LEADING_DOT
// NAME_LEADING_DOT: "--os9-name=myprog"
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s \
// RUN:   -o %t.dir/abcdefghijklmnopqrstuvwxyz0123456789 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NAME_LONG
// NAME_LONG: "--os9-name=abcdefghijklmnopqrstuvwxyz012"
// Nothing usable is left, so no name is passed and the linker asks for one.
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s -o %t.dir/--- 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NAME_NONE
// NAME_NONE-NOT: "--os9-name
// A name asked for is used as it stands.
// RUN: %clang -### -target mc6809-unknown-os9 -nostdinc %s -o %t.dir/x -Wl,--os9-name=my-prog 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NAME_GIVEN
// NAME_GIVEN: "--os9-name=my-prog"
// NAME_GIVEN-NOT: "--os9-name=x"

// Link job — bare-metal path (must remain unchanged).
// RUN: %clang -### -target mc6809 -nostdinc %s -o %t.elf 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LINK_BARE
// LINK_BARE:     ld.lld
// LINK_BARE:     "-Tlink.ld"
// LINK_BARE-NOT: "-Tmc6809-os9.lds"

// -mcpu=hd6309 searches the HD6309 build of the OS-9 builtins first.
// RUN: %clang -### -target mc6809-unknown-os9 -mcpu=hd6309 -nostdinc %s -o %t.body 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LINK_OS9_6309
// LINK_OS9_6309:      ld.lld
// LINK_OS9_6309-SAME: "-L{{.*}}mc6809-unknown-os9{{/|\\\\}}hd6309"
// LINK_OS9_6309-SAME: "-lclang_rt.builtins"
