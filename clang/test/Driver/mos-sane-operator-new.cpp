// Under LTO, MOS defaults to -fno-assume-sane-operator-new: the SDK's C++
// runtime is LTO'd into the program, so operator new's internals are not
// inaccessible memory. Without LTO the assumption is as safe as on hosted
// targets, so the upstream default stands. An explicit
// -fassume-sane-operator-new restores the upstream default even under LTO.
// RUN: %clang -target mos -flto \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-NO-SANE %s
// RUN: %clang -target mos -flto=thin \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-NO-SANE %s
// RUN: %clang -target mos -flto -fno-assume-sane-operator-new \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-NO-SANE %s
// RUN: %clang -target mos \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-SANE %s
// RUN: %clang -target mos -flto -fno-lto \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-SANE %s
// RUN: %clang -target mos -flto -fassume-sane-operator-new \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-SANE %s

// CHECK-NO-SANE: "-fno-assume-sane-operator-new"
// CHECK-SANE-NOT: "-fno-assume-sane-operator-new"
