// The OS-9 target makes '\n' a carriage return unless told otherwise; the
// other MC6809 targets keep it a line feed unless asked.
//
// RUN: %clang -### --target=mc6809-unknown-os9 -c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=ON
// RUN: %clang -### --target=mc6809-unknown-os9 -fno-os9-newline -c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=OFF
// RUN: %clang -### --target=mc6809-unknown-os9 -fno-os9-newline -fos9-newline -c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=ON
// RUN: %clang -### --target=mc6809-unknown-unknown -c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=OFF
// RUN: %clang -### --target=mc6809-unknown-decb -c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=OFF
// RUN: %clang -### --target=mc6809-unknown-unknown -fos9-newline -c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=ON

// ON:  "-cc1" {{.*}}"-fos9-newline"
// OFF: "-cc1"
// OFF-NOT: "-fos9-newline"
