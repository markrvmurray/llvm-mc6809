// RUN: %clang_cc1 -triple mc6809-unknown-unknown -emit-llvm -o - %s | FileCheck %s

// An interrupt handler reaches the backend as a function with the "interrupt"
// attribute, which makes it return with RTI and preserve what the interrupted
// code may be using. An ordinary function carries no such mark.

volatile unsigned char ticks;

// CHECK: define{{.*}} void @handler() [[HANDLER:#[0-9]+]]
__attribute__((interrupt)) void handler(void) { ticks++; }

// A FIRQ handler says which it is: the processor stacks less for it.
// CHECK: define{{.*}} void @fast_handler() [[FAST:#[0-9]+]]
__attribute__((interrupt("firq"))) void fast_handler(void) { ticks++; }

// CHECK: define{{.*}} void @ordinary() [[ORDINARY:#[0-9]+]]
void ordinary(void) { ticks++; }

// CHECK: attributes [[HANDLER]] = {{{.*}}"interrupt"{{ }}
// CHECK: attributes [[FAST]] = {{{.*}}"interrupt"="firq"
// CHECK-NOT: attributes [[ORDINARY]] = {{{.*}}"interrupt"
