/*
 * harness-mul32-ll.c — C test driver for codegen-mul32.ll. Tests
 * 32-bit multiply compiled from LLVM IR (which lowers to the
 * __mulsi3 libcall).
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call test_mul32 directly with
 * no bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long test_mul32      (unsigned long a, unsigned long b);
extern unsigned long test_mul32_const(unsigned long a);

void test_main(void) {
    /* test_mul32(7, 13) = 91 = 0x0000005B */
    h_putlong(test_mul32(7UL, 13UL));
    h_putnl();
    /* CHECK: 0000005B */

    /* test_mul32(0x10000, 0x10000) = 0x00000000 (truncated to 32 bits) */
    h_putlong(test_mul32(0x10000UL, 0x10000UL));
    h_putnl();
    /* CHECK-NEXT: 00000000 */

    /* test_mul32(0x12345, 0x10) = 0x00123450 */
    h_putlong(test_mul32(0x12345UL, 0x10UL));
    h_putnl();
    /* CHECK-NEXT: 00123450 */

    /* test_mul32(0xFFFFFFFF, 2) = 0xFFFFFFFE (unsigned wraparound) */
    h_putlong(test_mul32(0xFFFFFFFFUL, 2UL));
    h_putnl();
    /* CHECK-NEXT: FFFFFFFE */

    /* test_mul32(0xACF135, 0x100) = 0xACF13500 */
    h_putlong(test_mul32(0xACF135UL, 0x100UL));
    h_putnl();
    /* CHECK-NEXT: ACF13500 */

    /* test_mul32_const(7) = 7 * 30000 = 210000 = 0x00033450 */
    h_putlong(test_mul32_const(7UL));
    h_putnl();
    /* CHECK-NEXT: 00033450 */
}
