/*
 * harness-div64.c — driver for codegen-div64.ll. Exercises 64-bit
 * sdiv/udiv/srem/urem across small dividends, divisor > dividend
 * (zero quotient, dividend remainder), divisor crossing 32-bit
 * boundary, sign-bit-set values (signed vs unsigned divergence).
 *
 * No row uses divisor=0 — division by zero is UB in C and the
 * harness can't usefully test it.
 */

#include "harness-common.h"

extern void sdiv64_w(unsigned long *r, unsigned long *a, unsigned long *b);
extern void udiv64_w(unsigned long *r, unsigned long *a, unsigned long *b);
extern void srem64_w(unsigned long *r, unsigned long *a, unsigned long *b);
extern void urem64_w(unsigned long *r, unsigned long *a, unsigned long *b);

static void h_putlonglong(unsigned long *v) {
    h_putlong(v[0]);
    h_putlong(v[1]);
}

void test_main(void) {
    unsigned long r[2], a[2], b[2];

    /* 100 / 7 = 14 rem 2 (signed and unsigned same — both positive small) */
    a[0] = 0; a[1] = 100; b[0] = 0; b[1] = 7;
    sdiv64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK: 000000000000000E */
    udiv64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 000000000000000E */
    srem64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000002 */
    urem64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000002 */

    /* divisor > dividend: 5 / 100 = 0 rem 5 */
    a[0] = 0; a[1] = 5; b[0] = 0; b[1] = 100;
    sdiv64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000000 */
    srem64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000005 */

    /* Crossing 32-bit boundary: 0x100000000 / 2 = 0x80000000 */
    a[0] = 1; a[1] = 0; b[0] = 0; b[1] = 2;
    udiv64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000080000000 */

    /* -100 / 7 (signed): -14 rem -2.   Two's complement -14 = 0xFFFFFFFFFFFFFFF2,
     *                                   Two's complement  -2 = 0xFFFFFFFFFFFFFFFE */
    a[0] = 0xFFFFFFFFUL; a[1] = 0xFFFFFF9CUL;  /* -100 */
    b[0] = 0; b[1] = 7;
    sdiv64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFFFFFFFFF2 */
    srem64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFFFFFFFFFE */

    /* As unsigned: 0xFFFFFFFFFFFFFF9C / 7 = 0x2492492492492484 with
     * remainder 0 (divides evenly: 7 * 0x2492492492492484 = the
     * input). Verified against Python's int division. */
    udiv64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 2492492492492484 */
    urem64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000000 */
}
