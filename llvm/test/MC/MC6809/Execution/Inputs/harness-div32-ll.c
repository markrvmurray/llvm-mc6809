/*
 * harness-div32-ll.c — C test driver for codegen-div32.ll. Tests
 * 32-bit division and remainder (signed and unsigned) compiled
 * from LLVM IR (which lowers internally to __udivsi3 / __divsi3 /
 * __umodsi3 / __modsi3 libcalls).
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call test_udiv32 / test_sdiv32 /
 * test_urem32 / test_srem32 directly with no bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long test_udiv32(unsigned long a, unsigned long b);
extern long          test_sdiv32(long a,          long b);
extern unsigned long test_urem32(unsigned long a, unsigned long b);
extern long          test_srem32(long a,          long b);

void test_main(void) {
    /* udiv32: 100 / 7 = 14 (0x0000000E) */
    h_putlong(test_udiv32(100UL, 7UL));
    h_putnl();
    /* CHECK: 0000000E */

    /* sdiv32: -100 / 7 = -14 (0xFFFFFFF2) */
    h_putlong((unsigned long)test_sdiv32(-100L, 7L));
    h_putnl();
    /* CHECK-NEXT: FFFFFFF2 */

    /* urem32: 100 % 7 = 2 */
    h_putlong(test_urem32(100UL, 7UL));
    h_putnl();
    /* CHECK-NEXT: 00000002 */

    /* srem32: -100 % 7 = -2 (0xFFFFFFFE) */
    h_putlong((unsigned long)test_srem32(-100L, 7L));
    h_putnl();
    /* CHECK-NEXT: FFFFFFFE */
}
