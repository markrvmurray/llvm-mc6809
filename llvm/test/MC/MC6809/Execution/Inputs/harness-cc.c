/*
 * harness-cc.c — gcc6809 driver for the 8-bit calling-convention
 * validation test (calling-convention.s).
 *
 * gcc6809 (the reference compiler) and LLVM-MC6809 share ABI
 * version 1: arg dispatch is by *type* with two independent
 * register slots — the first i8 arg fills B, the first i16 arg
 * fills X, anything past those slots goes onto the stack (i8 in
 * 1-byte slots, i16 in 2-byte slots). Return i8 in B, return i16
 * in X.
 *
 * Every test function below takes only i8 args, so for an n-arg
 * call the first byte goes in B and the remaining n-1 bytes are
 * pushed via `stb ,-s` so they sit at S+2, S+3, ... after the
 * return address. With prototypes in scope here, gcc6809 does NOT
 * promote the unsigned char arguments — they go through the i8
 * path verbatim, exactly as the llc-compiled callees expect.
 *
 * If LLVM-MC6809 ever diverges from gcc6809 on this layout (the
 * exact bug class fixed in #69), the link still succeeds but the
 * runtime output mismatches and FileCheck fails. That is the
 * failure mode this test exists to catch.
 */

#include "harness-common.h"

extern unsigned char test_identity8(unsigned char a);
extern unsigned char test_add8     (unsigned char a, unsigned char b);
extern unsigned char test_add3     (unsigned char a, unsigned char b,
                                    unsigned char c);
extern unsigned char test_nested   (unsigned char x);
extern unsigned char test_factorial(unsigned char n);

void test_main(void) {
    /* identity8(0x42) = 0x42 */
    h_puthex(test_identity8(0x42));
    h_putnl();
    /* CHECK: 42 */

    /* add8(0x12, 0x30) = 0x42 — exercises 2nd i8 arg from 2,s */
    h_puthex(test_add8(0x12, 0x30));
    h_putnl();
    /* CHECK-NEXT: 42 */

    /* add3(0x12, 0x10, 0x20) = 0x42 — 2nd from 2,s, 3rd from 3,s */
    h_puthex(test_add3(0x12, 0x10, 0x20));
    h_putnl();
    /* CHECK-NEXT: 42 */

    /* nested(0x41) = add8(0x41, 1) = 0x42 — nested dispatch */
    h_puthex(test_nested(0x41));
    h_putnl();
    /* CHECK-NEXT: 42 */

    /* factorial(5) = 120 = 0x78 — recursive, exercises i8 mul */
    h_puthex(test_factorial(5));
    h_putnl();
    /* CHECK-NEXT: 78 */
}
