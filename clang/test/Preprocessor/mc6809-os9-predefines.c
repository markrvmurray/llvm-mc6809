// RUN: %clang_cc1 -E -dM -triple mc6809-unknown-os9 -o - %s | FileCheck --check-prefix=OS9 %s
// RUN: %clang_cc1 -E -dM -triple mc6809-unknown-unknown -o - %s | FileCheck --check-prefix=BARE %s

// OS9: #define __MC6809__ 1
// OS9: #define __OS9__ 1
// OS9: #define __os9__ 1
// BARE-NOT: __OS9__
// BARE-NOT: __os9__
