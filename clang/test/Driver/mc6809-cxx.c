// C++ on this target is the language over the C library: there is no standard
// library, and libc++ would not fit a 64K address space if there were.  What
// the driver has to do is find the headers that give the C library its C++
// spelling, link the small runtime that carries operator new and the guard
// functions, and turn off the two features nothing here implements.

// RUN: rm -rf %t && mkdir -p %t/sysroot/lib %t/sysroot/include
// RUN: touch %t/sysroot/lib/libc.a

// The C++ header directory sits with the compiler's own resource files, and
// is only added for a C++ compile.
// RUN: %clangxx --target=mc6809-unknown-unknown --sysroot=%t/sysroot -### -x c++ %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CXXINC
// CXXINC: "-internal-isystem" "{{.*}}include{{/|\\\\}}c++"

// RUN: %clang --target=mc6809-unknown-unknown --sysroot=%t/sysroot -### -x c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CINC
// CINC-NOT: "-internal-isystem" "{{.*}}include{{/|\\\\}}c++"

// A C++ link gets the support library, inside the group with the C library:
// it calls malloc, and nothing but the program calls it.
// RUN: %clangxx --target=mc6809-unknown-unknown --sysroot=%t/sysroot -### -x c++ %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CXXLIB
// CXXLIB: "--start-group" "-lclang_rt.cxx" "-lc"

// A C link does not.
// RUN: %clang --target=mc6809-unknown-unknown --sysroot=%t/sysroot -### -x c %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=CLIB
// CLIB-NOT: -lclang_rt.cxx

// Exceptions and RTTI are off unless asked for: there is no unwinder and no
// type_info hierarchy, so a try or a dynamic_cast should be refused where it
// is written rather than at link time by a mangled name.
// RUN: %clangxx --target=mc6809-unknown-unknown -### -x c++ %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=NOEH
// NOEH-NOT: "-fcxx-exceptions"
// NOEH: "-fno-rtti"

// Asking still works, for somebody who has brought their own runtime.
// RUN: %clangxx --target=mc6809-unknown-unknown -fexceptions -frtti -### -x c++ %s 2>&1 \
// RUN:   | FileCheck %s --check-prefix=YESEH
// YESEH: "-fcxx-exceptions"
// YESEH-NOT: "-fno-rtti"

int main(void) { return 0; }
