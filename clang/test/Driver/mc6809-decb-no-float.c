// DECB has no floating-point C library.  Its multilib.yaml (written by
// scripts/roll-toolchain.sh) says so with an Error rule, so `-mlibc=float`
// stops the build with that message instead of quietly linking the integer
// library, whose printf prints nothing for `%f`.
//
// RUN: rm -rf %t && mkdir -p %t/sysroot/lib %t/sysroot/hd6309
// RUN: echo 'MultilibVersion: 1.0'                             > %t/sysroot/multilib.yaml
// RUN: echo 'Variants:'                                       >> %t/sysroot/multilib.yaml
// RUN: echo '- Dir: lib'                                      >> %t/sysroot/multilib.yaml
// RUN: echo '  Flags: [-mcpu=mc6809, -mlibc=integer]'         >> %t/sysroot/multilib.yaml
// RUN: echo '- Dir: hd6309'                                   >> %t/sysroot/multilib.yaml
// RUN: echo '  Flags: [-mcpu=hd6309, -mlibc=integer]'         >> %t/sysroot/multilib.yaml
// RUN: echo '- Error: there is no floating-point C library for this target; leave out -mlibc=float' >> %t/sysroot/multilib.yaml
// RUN: echo '  Flags: [-mlibc=float]'                         >> %t/sysroot/multilib.yaml
// RUN: echo 'Mappings: []'                                    >> %t/sysroot/multilib.yaml

// RUN: not %clang --target=mc6809-unknown-decb -mlibc=float --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=FLOAT
// FLOAT: error: there is no floating-point C library for this target; leave out -mlibc=float

// RUN: not %clang --target=mc6809-unknown-decb -mcpu=hd6309 -mlibc=float --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=FLOAT

// Without it, the integer library is chosen with no complaint.
// RUN: %clang --target=mc6809-unknown-decb --sysroot=%t/sysroot -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=PLAIN
// PLAIN-NOT: error:
// PLAIN-NOT: warning: no multilib
// PLAIN: "-L{{.*}}sysroot{{/|\\\\}}lib"

int main(void) { return 0; }
