// Which start-up object gets linked.  picolibc's plain crt0 does not call
// exit(), because a program on hardware does not return from main; under a
// simulator that means it prints its output and then runs for ever.
//
// RUN: %clang --target=mc6809-unknown-unknown -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=DEFAULT
// DEFAULT: "-l:crt0.o"

// RUN: %clang --target=mc6809-unknown-unknown -mcrt0=semihost -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=SEMIHOST
// SEMIHOST: "-l:crt0-semihost.o"
// SEMIHOST-NOT: "-l:crt0.o"

// RUN: %clang --target=mc6809-unknown-unknown -mcrt0=minimal -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=MINIMAL
// MINIMAL: "-l:crt0-minimal.o"

// Spelling the default out is the same as not asking.
// RUN: %clang --target=mc6809-unknown-unknown -mcrt0=default -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=DEFAULT

// -nostartfiles still means no start-up at all.
// RUN: %clang --target=mc6809-unknown-unknown -mcrt0=semihost -nostartfiles -### %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOSTART
// NOSTART-NOT: "-l:crt0

int main(void) { return 0; }
