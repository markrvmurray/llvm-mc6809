/*
 * harness-add32.c — C test driver for codegen-add32.ll. Tests
 * 32-bit addition (general two-arg form and add-by-constant form).
 *
 * Both functions return i32, and gcc6809's long-return CC differs
 * from LLVM-MC6809's, so the harness calls void-returning out-pointer
 * wrappers (add_i32_w, add_i32_const_w) defined in codegen-add32.ll.
 * The wrappers are compiled by LLVM, so the inner i32 calls use
 * LLVM's native calling convention.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern void add_i32_w(unsigned long *out, unsigned long a, unsigned long b);
extern void add_i32_const_w(unsigned long *out, unsigned long a);

void test_main(void) {
    unsigned long r;

    /* add_i32(1, 2) = 3 */
    add_i32_w(&r, 1UL, 2UL);
    h_putlong(r); h_putnl();
    /* CHECK: 00000003 */

    /* add_i32(0x0000FFFF, 1) = 0x00010000 (carry into high word) */
    add_i32_w(&r, 0x0000FFFFUL, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00010000 */

    /* add_i32(0x7FFFFFFF, 1) = 0x80000000 (signed overflow) */
    add_i32_w(&r, 0x7FFFFFFFUL, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 80000000 */

    /* add_i32(0xFFFFFFFF, 1) = 0x00000000 (unsigned overflow) */
    add_i32_w(&r, 0xFFFFFFFFUL, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00000000 */

    /* add_i32(0x12345678, 0x9ABCDEF0) = 0xACF13568 */
    add_i32_w(&r, 0x12345678UL, 0x9ABCDEF0UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: ACF13568 */

    /* add_i32_const(1) = 1 + 30000 = 30001 = 0x00007531 */
    add_i32_const_w(&r, 1UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00007531 */

    /* add_i32_const(0xFFFF0000) = 0xFFFF7530 */
    add_i32_const_w(&r, 0xFFFF0000UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: FFFF7530 */

    /* Commutativity: add_i32(0x9ABCDEF0, 0x12345678) = 0xACF13568 */
    add_i32_w(&r, 0x9ABCDEF0UL, 0x12345678UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: ACF13568 */
}
