// Bare metal finds its C library the way the OS-9 triple does: in the
// sysroot beside the compiler.  Without one it keeps the older runtime's
// link line, so a build that supplies its own paths is unaffected.
//
// RUN: rm -rf %t && mkdir -p %t/sysroot/lib %t/sysroot/include %t/empty
// RUN: touch %t/sysroot/lib/libc.a

// With a sysroot: picolibc's start-up object, its library and the layer that
// gives it a machine, the compiler's builtins, and its linker script.
// RUN: %clang --target=mc6809-unknown-unknown --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=SYSROOT
// SYSROOT: "-internal-isystem" "{{.*}}sysroot{{/|\\\\}}include"
// SYSROOT: "-l:crt0.o"
// SYSROOT: "--start-group" "-lc" "-lsemihost" "-lclang_rt.builtins" "--end-group"
// SYSROOT: "-Tpicolibc.ld"
// SYSROOT-NOT: "-Tlink.ld"

// Without one, exactly what it emitted before: the older runtime's names.
// RUN: %clang --target=mc6809-unknown-unknown --sysroot=%t/empty -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOSYSROOT
// NOSYSROOT: "-l:crt0.o" "-lcrt0" "-lcrt" "-lc" "-lmc6809rt"
// NOSYSROOT: "-Tlink.ld"

// -nostdlib means what it says, sysroot or not: no libraries, no start-up,
// no script, and no search path for any of them.
// RUN: %clang --target=mc6809-unknown-unknown --sysroot=%t/sysroot -nostdlib -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOSTDLIB
// NOSTDLIB-NOT: "-lc"
// NOSTDLIB-NOT: "-lsemihost"
// NOSTDLIB-NOT: "-T
// NOSTDLIB-NOT: sysroot{{/|\\\\}}lib"

int main(void) { return 0; }
