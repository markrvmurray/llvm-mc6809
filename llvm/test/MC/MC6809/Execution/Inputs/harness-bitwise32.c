/*
 * harness-bitwise32.c — C test driver for codegen-bitwise32.ll.
 * Tests 32-bit bitwise AND / OR / XOR.
 *
 * The functions return i32, and gcc6809's long-return CC differs
 * from LLVM-MC6809's, so the harness calls void-returning out-pointer
 * wrappers (test_and32_w / test_or32_w / test_xor32_w) defined in
 * codegen-bitwise32.ll. The wrappers are compiled by LLVM, so the
 * inner i32 calls use LLVM's native calling convention.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern void test_and32_w(unsigned long *out, unsigned long a, unsigned long b);
extern void test_or32_w (unsigned long *out, unsigned long a, unsigned long b);
extern void test_xor32_w(unsigned long *out, unsigned long a, unsigned long b);

void test_main(void) {
    unsigned long r;

    /* and32(0x12345678, 0xFF00FF00) = 0x12005600 */
    test_and32_w(&r, 0x12345678UL, 0xFF00FF00UL);
    h_putlong(r); h_putnl();
    /* CHECK: 12005600 */

    /* or32(0x12005600, 0x00340078) = 0x12345678 */
    test_or32_w(&r, 0x12005600UL, 0x00340078UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 12345678 */

    /* xor32(0xFFFFFFFF, 0xAAAAAAAA) = 0x55555555 */
    test_xor32_w(&r, 0xFFFFFFFFUL, 0xAAAAAAAAUL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 55555555 */

    /* xor32(0x12345678, 0x12345678) = 0x00000000 (self-XOR) */
    test_xor32_w(&r, 0x12345678UL, 0x12345678UL);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00000000 */
}
