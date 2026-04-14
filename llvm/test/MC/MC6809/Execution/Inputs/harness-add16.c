/*
 * harness-add16.c — C test driver for codegen-add16.ll. Tests
 * the 16-bit add functions.
 *
 * gcc6809 and LLVM-MC6809 share the same i16 calling convention
 * (first i16 arg in IX, return in IX), so the harness calls these
 * functions directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int add_i16       (unsigned int a, unsigned int b);
extern unsigned int add_i16_const (unsigned int a);
extern unsigned int add_i16_chain (unsigned int a, unsigned int b, unsigned int c);

void test_main(void) {
    /* add_i16(100, 200) = 300 = 0x012C */
    h_putx(add_i16(100, 200));
    h_putnl();
    /* CHECK: 012C */

    /* add_i16(0xFFFF, 1) = 0 (overflow) */
    h_putx(add_i16(0xFFFF, 1));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* add_i16(0x8000, 0x8000) = 0 (overflow) */
    h_putx(add_i16(0x8000, 0x8000));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* add_i16_const(234) = 234 + 1000 = 1234 = 0x04D2 */
    h_putx(add_i16_const(234));
    h_putnl();
    /* CHECK-NEXT: 04D2 */

    /* add_i16_chain(100, 200, 300) = 600 = 0x0258 */
    h_putx(add_i16_chain(100, 200, 300));
    h_putnl();
    /* CHECK-NEXT: 0258 */

    /* Commutativity: add_i16(200, 100) = 300 = 0x012C */
    h_putx(add_i16(200, 100));
    h_putnl();
    /* CHECK-NEXT: 012C */
}
