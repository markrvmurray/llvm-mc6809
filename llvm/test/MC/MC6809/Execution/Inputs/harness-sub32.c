/*
 * harness-sub32.c — C test driver for codegen-sub32.ll. Tests
 * 32-bit subtraction.
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call sub_i32 directly with no
 * bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long sub_i32(unsigned long a, unsigned long b);

void test_main(void) {
    /* sub_i32(5, 3) = 2 */
    h_putlong(sub_i32(5UL, 3UL));
    h_putnl();
    /* CHECK: 00000002 */

    /* sub_i32(0x00010000, 1) = 0x0000FFFF (borrow from high word) */
    h_putlong(sub_i32(0x00010000UL, 1UL));
    h_putnl();
    /* CHECK-NEXT: 0000FFFF */

    /* sub_i32(0, 1) = 0xFFFFFFFF (unsigned underflow) */
    h_putlong(sub_i32(0UL, 1UL));
    h_putnl();
    /* CHECK-NEXT: FFFFFFFF */

    /* sub_i32(0x80000000, 1) = 0x7FFFFFFF (signed overflow) */
    h_putlong(sub_i32(0x80000000UL, 1UL));
    h_putnl();
    /* CHECK-NEXT: 7FFFFFFF */

    /* sub_i32(0xACF13568, 0x9ABCDEF0) = 0x12345678 */
    h_putlong(sub_i32(0xACF13568UL, 0x9ABCDEF0UL));
    h_putnl();
    /* CHECK-NEXT: 12345678 */

    /* sub_i32(100, 100) = 0 */
    h_putlong(sub_i32(100UL, 100UL));
    h_putnl();
    /* CHECK-NEXT: 00000000 */

    /* sub_i32(0x12345678, 0xACF13568) = 0x65432110 */
    h_putlong(sub_i32(0x12345678UL, 0xACF13568UL));
    h_putnl();
    /* CHECK-NEXT: 65432110 */
}
