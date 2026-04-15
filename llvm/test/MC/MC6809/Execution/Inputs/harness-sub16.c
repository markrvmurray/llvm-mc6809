/*
 * harness-sub16.c — C test driver for codegen-sub16.ll. Tests
 * the 16-bit subtract functions.
 *
 * gcc6809 and LLVM-MC6809 share the same i16 calling convention,
 * so the harness calls these functions directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int sub_i16       (unsigned int a, unsigned int b);
extern unsigned int sub_i16_const (unsigned int a);

void test_main(void) {
    /* sub_i16(500, 200) = 300 = 0x012C */
    h_putx(sub_i16(500, 200));
    h_putnl();
    /* CHECK: 012C */

    /* sub_i16(0, 1) = 0xFFFF (-1 unsigned) */
    h_putx(sub_i16(0, 1));
    h_putnl();
    /* CHECK-NEXT: FFFF */

    /* sub_i16(1000, 1000) = 0 */
    h_putx(sub_i16(1000, 1000));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* sub_i16_const(1234) = 1234 - 100 = 1134 = 0x046E */
    h_putx(sub_i16_const(1234));
    h_putnl();
    /* CHECK-NEXT: 046E */

    /* sub_i16(0x8000, 1) = 0x7FFF */
    h_putx(sub_i16(0x8000, 1));
    h_putnl();
    /* CHECK-NEXT: 7FFF */
}
