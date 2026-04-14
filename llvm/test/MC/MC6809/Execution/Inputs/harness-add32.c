/*
 * harness-add32.c — C test driver for codegen-add32.ll. Tests
 * 32-bit addition (general two-arg form and add-by-constant form).
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call add_i32 / add_i32_const
 * directly with no bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long add_i32      (unsigned long a, unsigned long b);
extern unsigned long add_i32_const(unsigned long a);

void test_main(void) {
    /* add_i32(1, 2) = 3 */
    h_putlong(add_i32(1UL, 2UL));
    h_putnl();
    /* CHECK: 00000003 */

    /* add_i32(0x0000FFFF, 1) = 0x00010000 (carry into high word) */
    h_putlong(add_i32(0x0000FFFFUL, 1UL));
    h_putnl();
    /* CHECK-NEXT: 00010000 */

    /* add_i32(0x7FFFFFFF, 1) = 0x80000000 (signed overflow) */
    h_putlong(add_i32(0x7FFFFFFFUL, 1UL));
    h_putnl();
    /* CHECK-NEXT: 80000000 */

    /* add_i32(0xFFFFFFFF, 1) = 0x00000000 (unsigned overflow) */
    h_putlong(add_i32(0xFFFFFFFFUL, 1UL));
    h_putnl();
    /* CHECK-NEXT: 00000000 */

    /* add_i32(0x12345678, 0x9ABCDEF0) = 0xACF13568 */
    h_putlong(add_i32(0x12345678UL, 0x9ABCDEF0UL));
    h_putnl();
    /* CHECK-NEXT: ACF13568 */

    /* add_i32_const(1) = 1 + 30000 = 30001 = 0x00007531 */
    h_putlong(add_i32_const(1UL));
    h_putnl();
    /* CHECK-NEXT: 00007531 */

    /* add_i32_const(0xFFFF0000) = 0xFFFF7530 */
    h_putlong(add_i32_const(0xFFFF0000UL));
    h_putnl();
    /* CHECK-NEXT: FFFF7530 */

    /* Commutativity: add_i32(0x9ABCDEF0, 0x12345678) = 0xACF13568 */
    h_putlong(add_i32(0x9ABCDEF0UL, 0x12345678UL));
    h_putnl();
    /* CHECK-NEXT: ACF13568 */
}
