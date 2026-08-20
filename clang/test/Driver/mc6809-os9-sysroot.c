// The C library that goes with a clang is found beside it -- <clang>/../lib/
// clang-runtimes/<triple> -- unless --sysroot says otherwise.  A link then
// needs nothing named on the command line.
//
// The derived location cannot be faked with a symlink: clang resolves its
// own path to the real binary, so a link farm still finds the installation
// it points into.  What is checked here is what the driver does with a
// sysroot once it has one, and that it does nothing without.
//
// RUN: rm -rf %t && mkdir -p %t/sysroot/lib %t/sysroot/include %t/empty
// RUN: touch %t/sysroot/lib/libc.a

// The library and its system layer are linked as a group -- each needs the
// other -- and the library's headers are on the include path.
// RUN: %clang --target=mc6809-unknown-os9 --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=SYSROOT
// SYSROOT: "-internal-isystem" "{{.*}}sysroot{{/|\\\\}}include"
// SYSROOT: "--start-group" "-lc" "-los9" "--end-group"

// A sysroot with no library in it changes nothing, so a tree that has never
// had a picolibc installed beside it links exactly as it did before.
// RUN: %clang --target=mc6809-unknown-os9 --sysroot=%t/empty -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOLIBC
// NOLIBC-NOT: "-lc"

// -nostdlib means what it says: no library, and no startup either.
// RUN: %clang --target=mc6809-unknown-os9 --sysroot=%t/sysroot -nostdlib -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOSTDLIB
// NOSTDLIB-NOT: "-lc"
// NOSTDLIB-NOT: "-lclang_rt.os9"

int main(void) { return 0; }
