/*
 * harness-bitwise.c — C test driver for codegen-bitwise.ll. Tests
 * i8 and i16 bitwise AND/OR/XOR functions. All function signatures
 * use plain i8/i16 args and returns, so no bridge wrappers needed.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned char test_and8(unsigned char a, unsigned char b);
extern unsigned char test_or8 (unsigned char a, unsigned char b);
extern unsigned char test_xor8(unsigned char a, unsigned char b);
extern unsigned int  test_and16(unsigned int a, unsigned int b);
extern unsigned int  test_or16 (unsigned int a, unsigned int b);
extern unsigned int  test_xor16(unsigned int a, unsigned int b);

void test_main(void) {
    /* ===== AND i8 ===== */
    h_puthex(test_and8(0xFF, 0x0F)); h_putnl();   /* CHECK: 0F */
    h_puthex(test_and8(0xA5, 0x5A)); h_putnl();   /* CHECK-NEXT: 00 */
    h_puthex(test_and8(0xF0, 0xFF)); h_putnl();   /* CHECK-NEXT: F0 */

    /* ===== OR i8 ===== */
    h_puthex(test_or8(0xA0, 0x05)); h_putnl();    /* CHECK-NEXT: A5 */
    h_puthex(test_or8(0x00, 0x00)); h_putnl();    /* CHECK-NEXT: 00 */
    h_puthex(test_or8(0x0F, 0xF0)); h_putnl();    /* CHECK-NEXT: FF */

    /* ===== XOR i8 ===== */
    h_puthex(test_xor8(0xFF, 0xAA)); h_putnl();   /* CHECK-NEXT: 55 */
    h_puthex(test_xor8(0xAA, 0xAA)); h_putnl();   /* CHECK-NEXT: 00 */
    h_puthex(test_xor8(0x00, 0xFF)); h_putnl();   /* CHECK-NEXT: FF */

    /* ===== AND i16 ===== */
    h_putx(test_and16(0xFF00u, 0x00FFu)); h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(test_and16(0xABCDu, 0xFF00u)); h_putnl();   /* CHECK-NEXT: AB00 */

    /* ===== OR i16 ===== */
    h_putx(test_or16(0xFF00u, 0x00FFu));  h_putnl();   /* CHECK-NEXT: FFFF */
    h_putx(test_or16(0x1234u, 0x0000u));  h_putnl();   /* CHECK-NEXT: 1234 */

    /* ===== XOR i16 ===== */
    h_putx(test_xor16(0xFFFFu, 0xAAAAu)); h_putnl();   /* CHECK-NEXT: 5555 */
    h_putx(test_xor16(0x1234u, 0x1234u)); h_putnl();   /* CHECK-NEXT: 0000 */
}
