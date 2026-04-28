// RUN: %clang_cc1 %s -triple mc6809 -emit-llvm -o - | FileCheck %s

// Bug #178: variables carrying __attribute__((directpage)) must
// produce LLVM addrspace(1) globals; the MC6809 backend then selects
// DP-mode opcodes for them.

// CHECK: @g_static = internal addrspace(1) global i16 0
static int g_static __attribute__((directpage));

// CHECK: @g_macro = internal addrspace(1) global i16 0
static __directpage int g_macro;

// CHECK: @g_dp = internal addrspace(1) global i16 0
static int g_dp __dp;

// CHECK: @g_byte = internal addrspace(1) global i8 0
static char g_byte __directpage;

// Without the attribute → default address space.
// CHECK: @g_normal = internal global i16 0
static int g_normal;

int read_dp(void) {
  return g_static + g_macro + g_dp + g_byte + g_normal;
}
