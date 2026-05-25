// RUN: %clang_cc1 %s -triple mc6809 -emit-llvm -o - | FileCheck %s
//
// Bug #343: complex float/double multiply and divide must lower to the
// compiler-rt helpers __mulsc3 / __muldc3 / __divsc3 / __divdc3, which
// mc6809 provides via libclang_rt.builtins. The complex result is returned
// through an sret pointer; the helper definitions are compiled by the same
// clang for the same target, so call site and definition share that ABI.

// CHECK-LABEL: @mul_f(
// CHECK: call void @__mulsc3(ptr
_Complex float mul_f(_Complex float a, _Complex float b) { return a * b; }

// CHECK-LABEL: @mul_d(
// CHECK: call void @__muldc3(ptr
_Complex double mul_d(_Complex double a, _Complex double b) { return a * b; }

// CHECK-LABEL: @div_f(
// CHECK: call void @__divsc3(ptr
_Complex float div_f(_Complex float a, _Complex float b) { return a / b; }

// CHECK-LABEL: @div_d(
// CHECK: call void @__divdc3(ptr
_Complex double div_d(_Complex double a, _Complex double b) { return a / b; }
