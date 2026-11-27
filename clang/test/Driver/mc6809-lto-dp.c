// Direct-page allocation at link time is not finished, and miscompiles stack
// frames, so the driver refuses its flags rather than pass them on.
//
// RUN: not %clang -### --target=mc6809-unknown-unknown -flto -mlto-dp=16 %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LTODP
// RUN: not %clang -### --target=mc6809-unknown-os9 -flto -mlto-dp=16 %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=LTODP
// RUN: not %clang -### --target=mc6809-unknown-unknown -flto -mlto-dp=16 -mreserve-dp=4 %s 2>&1 \
// RUN:   | FileCheck %s --check-prefixes=LTODP,RESERVE
// RUN: %clang -### --target=mc6809-unknown-unknown -flto %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NONE

// LTODP: error: '-mlto-dp=16' is not supported yet: direct-page allocation can miscompile stack frames
// RESERVE: error: '-mreserve-dp=4' is not supported yet: direct-page allocation can miscompile stack frames
// LTODP-NOT: -dp-avail

// NONE-NOT: error:
// NONE-NOT: -dp-avail

int main(void) { return 0; }
