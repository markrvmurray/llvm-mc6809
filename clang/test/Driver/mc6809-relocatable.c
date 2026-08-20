// A partial link is not a final link.  There is no entry point to keep
// anything alive, so garbage collection would collect everything, and a
// section sort means nothing without a layout -- left in, `-r` produced an
// object holding a file name and nothing else, which is what every start-up
// object picolibc installed used to be.
//
// RUN: %clang --target=mc6809-unknown-unknown -r -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=RELOC
// RELOC-NOT: "--gc-sections"
// RELOC-NOT: "--sort-section=alignment"
// RELOC-NOT: "-e0"

// An ordinary link still gets all three.
// RUN: %clang --target=mc6809-unknown-unknown -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=FINAL
// FINAL: "--gc-sections" "--sort-section=alignment"

int f(int x) { return x; }
