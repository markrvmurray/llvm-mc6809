// A sysroot may hold more than one library, with multilib.yaml saying which
// flags choose which.  clang puts <sysroot>/<Dir> on the library path ahead
// of the default, so a variant is found first and the default stays behind
// it as the fallback.
//
// RUN: rm -rf %t && mkdir -p %t/sysroot/lib %t/sysroot/hd6309/lib
// RUN: touch %t/sysroot/lib/libc.a %t/sysroot/hd6309/lib/libc.a
// RUN: echo 'MultilibVersion: 1.0'                             > %t/sysroot/multilib.yaml
// RUN: echo 'Variants:'                                       >> %t/sysroot/multilib.yaml
// RUN: echo '- Dir: lib'                                      >> %t/sysroot/multilib.yaml
// RUN: echo '  Flags: [--target=mc6809-unknown-unknown, -mcpu=mc6809]' >> %t/sysroot/multilib.yaml
// RUN: echo '- Dir: hd6309/lib'                               >> %t/sysroot/multilib.yaml
// RUN: echo '  Flags: [--target=mc6809-unknown-unknown, -mcpu=hd6309]' >> %t/sysroot/multilib.yaml
// RUN: echo 'Mappings: []'                                    >> %t/sysroot/multilib.yaml

// Plain 6809 gets the default library, and no complaint about matching
// nothing -- which is what happens if the default has no rule of its own.
// RUN: %clang --target=mc6809-unknown-unknown --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=PLAIN
// PLAIN-NOT: warning: no multilib
// PLAIN-NOT: -L{{.*}}hd6309
// PLAIN: "-L{{.*}}sysroot{{/|\\\\}}lib"

// -mcpu=hd6309 gets the variant first, then the default.
// RUN: %clang --target=mc6809-unknown-unknown -mcpu=hd6309 --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=HD6309
// HD6309-NOT: warning: no multilib
// HD6309: "-L{{.*}}sysroot{{/|\\\\}}hd6309{{/|\\\\}}lib" "-L{{.*}}sysroot{{/|\\\\}}lib"

// `-mcpu=6309` is the same processor spelled differently, and must choose the
// same library.
// RUN: %clang --target=mc6809-unknown-unknown -mcpu=6309 --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=HD6309

int main(void) { return 0; }
