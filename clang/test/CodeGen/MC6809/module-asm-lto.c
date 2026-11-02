// RUN: %clang --target=mc6809-unknown-unknown -O2 -flto -c %s -o %t.bc
// RUN: llvm-nm %t.bc | FileCheck %s

// Writing LTO bitcode parses file-scope asm to find its symbols, which needs a
// null target streamer. Without one, this used to abort the compiler. The
// .directpage directive reaches the parser's cast to the target's own streamer.

// CHECK-DAG: w foo
// CHECK-DAG: T main

__asm__(".weak foo\n");
__asm__(".directpage foo\n");

int main(void) { return 0; }
