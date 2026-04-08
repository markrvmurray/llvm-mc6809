/*
 * harness-cc16.c — gcc6809 driver for the 16-bit calling-convention
 * validation test (calling-convention-16.s).
 *
 * gcc6809 and LLVM-MC6809 both target ABI version 1: arg dispatch is
 * by *type* with two independent register slots — the first i8 arg
 * fills B, the first i16 arg fills X, anything past those slots goes
 * on the stack (i8 in 1-byte slots, i16 in 2-byte slots). Return i16
 * in X.
 *
 * Each test function below takes only i16 args, so the first arg
 * fills X and the second is pushed via `stx ,--s` (landing at S+2
 * after jsr saves the return address). The two halves of this test
 * are compiled by different compilers but share that ABI by design.
 */

#include "harness-common.h"

extern unsigned int test_identity16(unsigned int a);
extern unsigned int test_add16     (unsigned int a, unsigned int b);
extern unsigned int test_sub16     (unsigned int a, unsigned int b);

void test_main(void) {
    /* identity16(0xBEEF) = 0xBEEF — exercises X-register pass-through */
    h_putx(test_identity16(0xBEEF));
    h_putnl();
    /* CHECK: BEEF */

    /* add16(0x1234, 0x4321) = 0x5555 — first arg in X, second on stack */
    h_putx(test_add16(0x1234, 0x4321));
    h_putnl();
    /* CHECK-NEXT: 5555 */

    /* sub16(0x8000, 0x0001) = 0x7FFF */
    h_putx(test_sub16(0x8000, 0x0001));
    h_putnl();
    /* CHECK-NEXT: 7FFF */
}
