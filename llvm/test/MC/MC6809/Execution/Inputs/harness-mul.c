/*
 * harness-mul.c — C test driver for codegen-mul.ll. Tests the
 * 8-bit multiply functions.
 *
 * gcc6809 and LLVM-MC6809 share the same i8 calling convention,
 * so the harness calls these functions directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern signed char mul_i8       (signed char a, signed char b);
extern signed char mul_i8_const (signed char a);
extern signed char mul_i8_chain (signed char a, signed char b, signed char c);

void test_main(void) {
    /* mul_i8(6, 7) = 42 = 0x2A */
    h_puthex((unsigned char)mul_i8(6, 7));
    h_putnl();
    /* CHECK: 2A */

    /* mul_i8(0, 5) = 0 */
    h_puthex((unsigned char)mul_i8(0, 5));
    h_putnl();
    /* CHECK-NEXT: 00 */

    /* mul_i8(15, 17) = 255 = 0xFF */
    h_puthex((unsigned char)mul_i8(15, 17));
    h_putnl();
    /* CHECK-NEXT: FF */

    /* mul_i8_const(6) = 6*7 = 42 = 0x2A */
    h_puthex((unsigned char)mul_i8_const(6));
    h_putnl();
    /* CHECK-NEXT: 2A */

    /* mul_i8_chain(2, 3, 7) = 2*3*7 = 42 = 0x2A */
    h_puthex((unsigned char)mul_i8_chain(2, 3, 7));
    h_putnl();
    /* CHECK-NEXT: 2A */
}
