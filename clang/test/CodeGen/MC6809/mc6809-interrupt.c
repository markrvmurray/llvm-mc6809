// RUN: %clang_cc1 -triple mc6809-unknown-unknown -emit-llvm -o - %s | FileCheck %s

// An interrupt handler reaches the backend as a function with the "interrupt"
// attribute, which makes it return with RTI and preserve the direct-page
// registers. An ordinary function carries no such mark.

volatile unsigned char ticks;

// CHECK: define{{.*}} void @handler() [[HANDLER:#[0-9]+]]
__attribute__((interrupt)) void handler(void) { ticks++; }

// CHECK: define{{.*}} void @ordinary() [[ORDINARY:#[0-9]+]]
void ordinary(void) { ticks++; }

// CHECK: attributes [[HANDLER]] = {{{.*}}"interrupt"
// CHECK-NOT: attributes [[ORDINARY]] = {{{.*}}"interrupt"
