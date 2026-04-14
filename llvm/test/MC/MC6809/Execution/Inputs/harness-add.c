/*
 * harness-add.c — C test driver for codegen-add.ll. Tests the
 * 8-bit add functions (add_s_i8 and add_s_i8_consts).
 *
 * gcc6809 and LLVM-MC6809 share the same i8 calling convention
 * (first i8 in B, remaining i8 args in 1-byte stack slots,
 * i8 return in B), so the harness calls these functions
 * directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern signed char add_s_i8       (signed char a, signed char b,
                                   signed char c, signed char d,
                                   signed char e);
extern signed char add_s_i8_consts(signed char a, signed char b,
                                   signed char c, signed char d);

void test_main(void) {
    /* add_s_i8(1, 2, 3, 4, 5) = 15 = 0x0F */
    h_puthex((unsigned char)add_s_i8(1, 2, 3, 4, 5));
    h_putnl();
    /* CHECK: 0F */

    /* add_s_i8(0x10, 0x10, 0x10, 0x10, 0x02) = 0x42 */
    h_puthex((unsigned char)add_s_i8(0x10, 0x10, 0x10, 0x10, 0x02));
    h_putnl();
    /* CHECK-NEXT: 42 */

    /* add_s_i8_consts(0x10, 0x10, 0x10, 0x0D) = 0x10+0x10+0x10+0x0D+5 = 0x42 */
    h_puthex((unsigned char)add_s_i8_consts(0x10, 0x10, 0x10, 0x0D));
    h_putnl();
    /* CHECK-NEXT: 42 */
}
