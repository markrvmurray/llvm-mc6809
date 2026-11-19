// RUN: %clang_cc1 -triple mc6809 -emit-llvm -o - %s | FileCheck %s

// Every type is 1-aligned on the 6809, and clang must not prefer more for
// a long long or a double: that would pad globals and make a function
// with such a local realign its frame.

// CHECK: @d = {{.*}}global double 1.000000e+00, align 1
// CHECK: @ll = {{.*}}global i64 2, align 1
double d = 1.0;
long long ll = 2;

void use(void *);

// CHECK-LABEL: define {{.*}}void @locals(
// CHECK: alloca double, align 1
// CHECK: alloca i64, align 1
void locals(void) {
  double x = 1.0;
  long long y = 2;
  use(&x);
  use(&y);
}
