// The 6809 addresses its frame from S, so the driver asks for no frame
// pointer at any optimisation level; -fno-omit-frame-pointer still gets one.
// RUN: %clang -### --target=mc6809 -O0 -c %s 2>&1 | FileCheck %s --check-prefix=OMIT
// RUN: %clang -### --target=mc6809 -O2 -c %s 2>&1 | FileCheck %s --check-prefix=OMIT
// RUN: %clang -### --target=mc6809-unknown-os9 -O0 -c %s 2>&1 | FileCheck %s --check-prefix=OMIT
// RUN: %clang -### --target=mc6809 -O2 -fno-omit-frame-pointer -c %s 2>&1 | FileCheck %s --check-prefix=KEEP

// OMIT: "-mframe-pointer=none"
// KEEP: "-mframe-pointer=all"
