/*
 * harness-shift32-ll.c — C test driver for codegen-shift32.ll.
 * Tests 32-bit shifts (shl, lshr, ashr) compiled from LLVM IR.
 *
 * The functions return i32, and gcc6809's long-return CC differs
 * from LLVM-MC6809's, so the harness calls void-returning out-pointer
 * wrappers (test_shl32_w / test_lshr32_w / test_ashr32_w) defined in
 * codegen-shift32.ll. The wrappers are compiled by LLVM, so the
 * inner i32 calls (and their nested __ashlsi3/__lshrsi3/__ashrsi3
 * libcalls) use LLVM's native calling convention.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern void test_shl32_w (unsigned long *out, unsigned long a, unsigned long b);
extern void test_lshr32_w(unsigned long *out, unsigned long a, unsigned long b);
extern void test_ashr32_w(unsigned long *out, unsigned long a, unsigned long b);

void test_main(void) {
    unsigned long r;

    /* shl32: 1 << 16 = 0x00010000 */
    test_shl32_w(&r, 1UL, 16UL);
    h_putlong(r); h_putnl();
    /* CHECK: 00010000 */

    /* shl32: 0xFF << 4 = 0x00000FF0 */
    test_shl32_w(&r, 0xFFUL, 4UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00000FF0 */

    /* lshr32: 0x00010000 >> 8 = 0x00000100 */
    test_lshr32_w(&r, 0x00010000UL, 8UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00000100 */

    /* ashr32: 0x80000000 >> 16 = 0xFFFF8000 (sign extends) */
    test_ashr32_w(&r, 0x80000000UL, 16UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: FFFF8000 */
}
