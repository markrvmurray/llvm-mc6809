/*
 * harness-bitwise32.c — C test driver for codegen-bitwise32.ll.
 * Tests 32-bit bitwise AND / OR / XOR.
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call test_and32 / test_or32 /
 * test_xor32 directly with no bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long test_and32(unsigned long a, unsigned long b);
extern unsigned long test_or32 (unsigned long a, unsigned long b);
extern unsigned long test_xor32(unsigned long a, unsigned long b);

void test_main(void) {
    /* and32(0x12345678, 0xFF00FF00) = 0x12005600 */
    h_putlong(test_and32(0x12345678UL, 0xFF00FF00UL));
    h_putnl();
    /* CHECK: 12005600 */

    /* or32(0x12005600, 0x00340078) = 0x12345678 */
    h_putlong(test_or32(0x12005600UL, 0x00340078UL));
    h_putnl();
    /* CHECK-NEXT: 12345678 */

    /* xor32(0xFFFFFFFF, 0xAAAAAAAA) = 0x55555555 */
    h_putlong(test_xor32(0xFFFFFFFFUL, 0xAAAAAAAAUL));
    h_putnl();
    /* CHECK-NEXT: 55555555 */

    /* xor32(0x12345678, 0x12345678) = 0x00000000 (self-XOR) */
    h_putlong(test_xor32(0x12345678UL, 0x12345678UL));
    h_putnl();
    /* CHECK-NEXT: 00000000 */
}
