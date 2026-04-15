/*
 * harness-varargs.c — C test driver for codegen-varargs.ll. Tests
 * variadic function calls (va_start / va_arg) end-to-end across
 * the gcc6809 ↔ LLVM-MC6809 boundary.
 *
 * gcc6809 and LLVM-MC6809 share the same variadic CC after ABI #5:
 * ALL args (including the named ones) go on the stack as 16-bit
 * slots in declaration order, with caller cleanup. The return
 * value uses the regular RetCC (i16 → IX). The harness calls
 * sum_va2 / sum_va4 / get_nth_va directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int sum_va2   (unsigned int n, ...);
extern unsigned int sum_va4   (unsigned int n, ...);
extern unsigned int get_nth_va(unsigned int n, ...);

void test_main(void) {
    /* sum_va2(2, 100, 200) = 300 = 0x012C */
    h_putx(sum_va2(2, (unsigned int)100, (unsigned int)200));
    h_putnl();
    /* CHECK: 012C */

    /* sum_va2(2, 0x1234, 0x5678) = 0x68AC */
    h_putx(sum_va2(2, (unsigned int)0x1234, (unsigned int)0x5678));
    h_putnl();
    /* CHECK-NEXT: 68AC */

    /* sum_va4(4, 10, 20, 30, 40) = 100 = 0x0064 */
    h_putx(sum_va4(4,
                   (unsigned int)10, (unsigned int)20,
                   (unsigned int)30, (unsigned int)40));
    h_putnl();
    /* CHECK-NEXT: 0064 */

    /* sum_va4(4, 0x1000, 0x0200, 0x0030, 0x0004) = 0x1234 */
    h_putx(sum_va4(4,
                   (unsigned int)0x1000, (unsigned int)0x0200,
                   (unsigned int)0x0030, (unsigned int)0x0004));
    h_putnl();
    /* CHECK-NEXT: 1234 */

    /* get_nth_va(1, 0xAA, 0xBB, 0xCC, 0xDD) = 0xAA (first vararg) */
    h_putx(get_nth_va(1,
                      (unsigned int)0xAA, (unsigned int)0xBB,
                      (unsigned int)0xCC, (unsigned int)0xDD));
    h_putnl();
    /* CHECK-NEXT: 00AA */

    /* get_nth_va(4, 0xAA, 0xBB, 0xCC, 0xDD) = 0xDD (fourth vararg) */
    h_putx(get_nth_va(4,
                      (unsigned int)0xAA, (unsigned int)0xBB,
                      (unsigned int)0xCC, (unsigned int)0xDD));
    h_putnl();
    /* CHECK-NEXT: 00DD */
}
