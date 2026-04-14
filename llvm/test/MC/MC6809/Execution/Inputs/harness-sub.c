/*
 * harness-sub.c — C test driver for codegen-sub.ll. Tests the
 * 8-bit subtract functions (sub_simple and sub_s_i8_consts).
 *
 * gcc6809 and LLVM-MC6809 share the same i8 calling convention,
 * so the harness calls these functions directly.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern signed char sub_simple      (signed char a, signed char b);
extern signed char sub_s_i8_consts (signed char a, signed char b,
                                    signed char c, signed char d,
                                    signed char e);

void test_main(void) {
    /* sub_simple(0x50, 0x08) = 0x48 */
    h_puthex((unsigned char)sub_simple(0x50, 0x08));
    h_putnl();
    /* CHECK: 48 */

    /* sub_simple(0x42, 0x00) = 0x42 */
    h_puthex((unsigned char)sub_simple(0x42, 0x00));
    h_putnl();
    /* CHECK-NEXT: 42 */

    /* sub_s_i8_consts(0,0,0,0,0) = 5 - 0 - 0 - 0 - 0 - 0 = 5 */
    h_puthex((unsigned char)sub_s_i8_consts(0, 0, 0, 0, 0));
    h_putnl();
    /* CHECK-NEXT: 05 */

    /* sub_s_i8_consts(1,1,1,1,1) = 5 - 1 - 1 - 1 - 1 - 1 = 0 */
    h_puthex((unsigned char)sub_s_i8_consts(1, 1, 1, 1, 1));
    h_putnl();
    /* CHECK-NEXT: 00 */
}
