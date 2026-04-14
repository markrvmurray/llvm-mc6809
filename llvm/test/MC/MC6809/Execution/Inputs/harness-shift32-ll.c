/*
 * harness-shift32-ll.c — C test driver for codegen-shift32.ll.
 * Tests 32-bit shifts (shl, lshr, ashr) compiled from LLVM IR
 * (which lowers internally to __ashlsi3 / __lshrsi3 / __ashrsi3
 * libcalls).
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call test_shl32 / test_lshr32 /
 * test_ashr32 directly with no bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long test_shl32 (unsigned long a, unsigned long b);
extern unsigned long test_lshr32(unsigned long a, unsigned long b);
extern   signed long test_ashr32(  signed long a,   signed long b);

void test_main(void) {
    /* shl32: 1 << 16 = 0x00010000 */
    h_putlong(test_shl32(1UL, 16UL));
    h_putnl();
    /* CHECK: 00010000 */

    /* shl32: 0xFF << 4 = 0x00000FF0 */
    h_putlong(test_shl32(0xFFUL, 4UL));
    h_putnl();
    /* CHECK-NEXT: 00000FF0 */

    /* lshr32: 0x00010000 >> 8 = 0x00000100 */
    h_putlong(test_lshr32(0x00010000UL, 8UL));
    h_putnl();
    /* CHECK-NEXT: 00000100 */

    /* ashr32: 0x80000000 >> 16 = 0xFFFF8000 (sign extends) */
    h_putlong((unsigned long)test_ashr32((signed long)0x80000000L, 16L));
    h_putnl();
    /* CHECK-NEXT: FFFF8000 */
}
