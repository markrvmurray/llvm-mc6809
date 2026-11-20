// A DECB program uses the bare-metal builtins, the machine being the same;
// -mcpu=hd6309 searches their HD6309 build first.

int main(void) { return 0; }

// RUN: %clang -### --target=mc6809-unknown-decb -mcpu=hd6309 %s -o %t.bin 2>&1 \
// RUN:   | FileCheck %s --check-prefix=HD6309
// HD6309:      ld.lld
// HD6309-SAME: "-L{{[^"]*}}mc6809-unknown-unknown{{/|\\\\}}hd6309" "-L{{[^"]*}}mc6809-unknown-unknown"

// RUN: %clang -### --target=mc6809-unknown-decb %s -o %t.bin 2>&1 \
// RUN:   | FileCheck %s --check-prefix=PLAIN
// PLAIN:     ld.lld
// PLAIN-NOT: "-L{{[^"]*}}{{/|\\\\}}hd6309"
