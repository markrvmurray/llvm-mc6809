/*
 * harness-i32args.c — ABI #3 validation: i32 args in functions
 * that don't return long.
 *
 * Without ABI #3's CCIfSplit rules, the first split-i16 piece of
 * an i32 arg would land in IX whenever IX is free, mismatching
 * gcc6809 (which always passes long args on the stack regardless
 * of register availability). All other ABI #3 tests are masked
 * by sret-IX displacement; these are the only tests that
 * directly exercise the rule.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int  low16     (unsigned long x);
extern unsigned int  high16    (unsigned long x);
extern unsigned int  diff_low16(unsigned long a, unsigned long b);
extern void          store_long(unsigned long *dst, unsigned long val);

void test_main(void) {
    /* low16(0xCAFEBABE) = 0xBABE */
    h_putx(low16(0xCAFEBABEUL));
    h_putnl();
    /* CHECK: BABE */

    /* high16(0xCAFEBABE) = 0xCAFE */
    h_putx(high16(0xCAFEBABEUL));
    h_putnl();
    /* CHECK-NEXT: CAFE */

    /* diff_low16(0x12345678, 0x12340000) = 0x5678 */
    h_putx(diff_low16(0x12345678UL, 0x12340000UL));
    h_putnl();
    /* CHECK-NEXT: 5678 */

    /* store_long: pointer first → IX, long second → stack */
    unsigned long buf = 0;
    store_long(&buf, 0xDEADBEEFUL);
    h_putlong(buf);
    h_putnl();
    /* CHECK-NEXT: DEADBEEF */
}
