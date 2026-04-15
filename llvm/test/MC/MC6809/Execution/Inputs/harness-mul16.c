/*
 * harness-mul16.c — C test driver for codegen-mul16.ll. Tests
 * 16-bit multiply (which lowers internally to the __mulhi3
 * libcall).
 *
 * gcc6809 and LLVM-MC6809 share the same i16 calling convention,
 * so the harness calls test_mul16 directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int test_mul16(unsigned int a, unsigned int b);

void test_main(void) {
    /* 6 * 7 = 42 = 0x002A */
    h_putx(test_mul16(6, 7));
    h_putnl();
    /* CHECK: 002A */

    /* 100 * 200 = 20000 = 0x4E20 */
    h_putx(test_mul16(100, 200));
    h_putnl();
    /* CHECK-NEXT: 4E20 */

    /* 255 * 255 = 65025 = 0xFE01 */
    h_putx(test_mul16(255, 255));
    h_putnl();
    /* CHECK-NEXT: FE01 */

    /* 0 * 12345 = 0 */
    h_putx(test_mul16(0, 12345));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* 1 * 0xABCD = 0xABCD (identity) */
    h_putx(test_mul16(1, 0xABCD));
    h_putnl();
    /* CHECK-NEXT: ABCD */

    /* 256 * 256 = 65536 truncated = 0x0000 */
    h_putx(test_mul16(256, 256));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* 0x80 * 2 = 0x0100 (carry into high byte) */
    h_putx(test_mul16(0x80, 2));
    h_putnl();
    /* CHECK-NEXT: 0100 */

    /* 0x101 * 0x101 = 0x10201 truncated = 0x0201 */
    h_putx(test_mul16(0x101, 0x101));
    h_putnl();
    /* CHECK-NEXT: 0201 */

    /* Commutativity: 13 * 500 = 6500 = 0x1964 */
    h_putx(test_mul16(13, 500));
    h_putnl();
    /* CHECK-NEXT: 1964 */

    /* 500 * 13 = 6500 = 0x1964 (same result) */
    h_putx(test_mul16(500, 13));
    h_putnl();
    /* CHECK-NEXT: 1964 */

    /* 0xFFFF * 0xFFFF = 0xFFFE0001 truncated = 0x0001 */
    h_putx(test_mul16(0xFFFF, 0xFFFF));
    h_putnl();
    /* CHECK-NEXT: 0001 */
}
