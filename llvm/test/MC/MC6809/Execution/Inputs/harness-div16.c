/*
 * harness-div16.c — C test driver for codegen-div16.ll. Tests
 * 16-bit signed/unsigned division and remainder. Lowered to libcalls
 * (__udivhi3 / __divhi3 / __umodhi3 / __modhi3 in divhi.inc) by LLVM,
 * so this exercises the codegen + libcall + runtime path together.
 *
 * No bridge wrappers needed: signatures are plain (i16, i16) → i16.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int test_udiv16(unsigned int a, unsigned int b);
extern          int test_sdiv16(         int a,          int b);
extern unsigned int test_urem16(unsigned int a, unsigned int b);
extern          int test_srem16(         int a,          int b);

void test_main(void) {
    /* ===== Unsigned division ===== */
    h_putx(test_udiv16(100, 10));      h_putnl();   /* CHECK: 000A */
    h_putx(test_udiv16(0xFFFFu, 256)); h_putnl();   /* CHECK-NEXT: 00FF */
    h_putx(test_udiv16(7, 2));         h_putnl();   /* CHECK-NEXT: 0003 */
    h_putx(test_udiv16(0, 5));         h_putnl();   /* CHECK-NEXT: 0000 */

    /* ===== Unsigned remainder ===== */
    h_putx(test_urem16(100, 10));      h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(test_urem16(7, 2));         h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(test_urem16(255, 16));      h_putnl();   /* CHECK-NEXT: 000F */
    h_putx(test_urem16(1000, 7));      h_putnl();   /* CHECK-NEXT: 0006 */

    /* ===== Signed division ===== */
    h_putx((unsigned)test_sdiv16( 100,  10)); h_putnl();   /* CHECK-NEXT: 000A */
    h_putx((unsigned)test_sdiv16(-100,  10)); h_putnl();   /* CHECK-NEXT: FFF6 */
    h_putx((unsigned)test_sdiv16( 100, -10)); h_putnl();   /* CHECK-NEXT: FFF6 */
    h_putx((unsigned)test_sdiv16(-100, -10)); h_putnl();   /* CHECK-NEXT: 000A */

    /* ===== Signed remainder ===== */
    h_putx((unsigned)test_srem16( 7,  2)); h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx((unsigned)test_srem16(-7,  2)); h_putnl();   /* CHECK-NEXT: FFFF */
    h_putx((unsigned)test_srem16( 7, -2)); h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx((unsigned)test_srem16(-7, -2)); h_putnl();   /* CHECK-NEXT: FFFF */
}
