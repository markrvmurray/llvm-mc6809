/*
 * harness-sub32.c — C test driver for codegen-sub32.ll. Tests
 * 32-bit subtraction (general two-arg form).
 *
 * sub_i32 returns i32, and gcc6809's long-return CC differs from
 * LLVM-MC6809's, so the harness calls a void-returning out-pointer
 * wrapper (sub_i32_w) defined in codegen-sub32.ll. The wrapper is
 * compiled by LLVM, so the inner i32 call uses LLVM's native
 * calling convention.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern void sub_i32_w(unsigned long *out, unsigned long a, unsigned long b);

void test_main(void) {
    unsigned long r;

    /* sub_i32(5, 3) = 2 */
    sub_i32_w(&r, 5UL, 3UL);
    h_putlong(r); h_putnl();
    /* CHECK: 00000002 */

    /* sub_i32(0x00010000, 1) = 0x0000FFFF (borrow from high word) */
    sub_i32_w(&r, 0x00010000UL, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 0000FFFF */

    /* sub_i32(0, 1) = 0xFFFFFFFF (unsigned underflow) */
    sub_i32_w(&r, 0UL, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFF */

    /* sub_i32(0x80000000, 1) = 0x7FFFFFFF (signed overflow) */
    sub_i32_w(&r, 0x80000000UL, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 7FFFFFFF */

    /* sub_i32(0xACF13568, 0x9ABCDEF0) = 0x12345678 */
    sub_i32_w(&r, 0xACF13568UL, 0x9ABCDEF0UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 12345678 */

    /* sub_i32(100, 100) = 0 */
    sub_i32_w(&r, 100UL, 100UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00000000 */

    /* sub_i32(0x12345678, 0xACF13568) = 0x65432110 */
    sub_i32_w(&r, 0x12345678UL, 0xACF13568UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 65432110 */
}
