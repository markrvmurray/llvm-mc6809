// REQUIRES: mc6809-registered-target
// RUN: %clang_cc1 -triple mc6809 -O1 -emit-llvm -o - %s | FileCheck %s

// Taking a direct-page object's address as an ordinary pointer is an
// implicit conversion that lowers to an addrspacecast; a call through it
// passes the full address.

static __attribute__((directpage)) int counter;
void use(int *p);

// CHECK-LABEL: define {{.*}}void @pass_it(
// CHECK: call void @use(ptr {{.*}}addrspacecast (ptr addrspace(1) @counter to ptr))
void pass_it(void) { use(&counter); }

// CHECK-LABEL: define {{.*}}i8 @byte_of(
// CHECK: getelementptr inbounds {{.*}}i8, ptr addrspacecast (ptr addrspace(1) @counter to ptr), i16 %{{.*}}
// CHECK: load i8, ptr %
unsigned char byte_of(unsigned char i) { return ((unsigned char *)&counter)[i]; }
