/*
 * harness-cmp64.c — driver for codegen-cmp64.ll. Exercises the six
 * i64 ICMP forms (eq, ne, slt, sgt, ult, ugt). The remaining four
 * (sle, sge, ule, uge) share legalizer paths with their LT/GT
 * counterparts, so cycling all six is enough to detect a sign or
 * carry regression in the __cmpdi2 / __ucmpdi2 paths.
 *
 * Result of each comparison is stored as a single byte (0 or 1) by
 * the LLVM-side wrapper; the harness prints it as one hex digit.
 *
 * Signed vs unsigned divergence is exercised by row 4 of each test
 * (a = 0xFFFFFFFFFFFFFFFF). As signed, that's -1, so slt/-1 vs 1
 * is true (1) and sgt/-1 vs 1 is false (0). As unsigned, that's
 * 2^64-1, so ult/MAX vs 1 is false (0) and ugt/MAX vs 1 is true (1).
 * Any sign-confusion in the legalizer flips those four bits.
 */

#include "harness-common.h"

extern void eq64_w (unsigned char *r, unsigned long *a, unsigned long *b);
extern void ne64_w (unsigned char *r, unsigned long *a, unsigned long *b);
extern void slt64_w(unsigned char *r, unsigned long *a, unsigned long *b);
extern void sgt64_w(unsigned char *r, unsigned long *a, unsigned long *b);
extern void ult64_w(unsigned char *r, unsigned long *a, unsigned long *b);
extern void ugt64_w(unsigned char *r, unsigned long *a, unsigned long *b);

static void run(unsigned long *a, unsigned long *b) {
    unsigned char r;
    eq64_w(&r, a, b);  h_putnibble(r);
    ne64_w(&r, a, b);  h_putnibble(r);
    slt64_w(&r, a, b); h_putnibble(r);
    sgt64_w(&r, a, b); h_putnibble(r);
    ult64_w(&r, a, b); h_putnibble(r);
    ugt64_w(&r, a, b); h_putnibble(r);
    h_putnl();
}

void test_main(void) {
    unsigned long a[2], b[2];

    /* Row 1: a == b. eq=1 ne=0 slt=0 sgt=0 ult=0 ugt=0  → 100000 */
    a[0] = 0; a[1] = 42; b[0] = 0; b[1] = 42; run(a, b);
    /* CHECK: 100000 */

    /* Row 2: a < b (small positives). eq=0 ne=1 slt=1 sgt=0 ult=1 ugt=0  → 011010 */
    a[0] = 0; a[1] = 1; b[0] = 0; b[1] = 2; run(a, b);
    /* CHECK-NEXT: 011010 */

    /* Row 3: a > b (small positives). eq=0 ne=1 slt=0 sgt=1 ult=0 ugt=1  → 010101 */
    a[0] = 0; a[1] = 2; b[0] = 0; b[1] = 1; run(a, b);
    /* CHECK-NEXT: 010101 */

    /* Row 4: a = -1 (= 0xFFFFFFFFFFFFFFFF), b = 1. Signed: -1 < 1.
     * Unsigned: 2^64-1 > 1. eq=0 ne=1 slt=1 sgt=0 ult=0 ugt=1  → 011001 */
    a[0] = 0xFFFFFFFFUL; a[1] = 0xFFFFFFFFUL; b[0] = 0; b[1] = 1; run(a, b);
    /* CHECK-NEXT: 011001 */

    /* Row 5: a differs only in the high byte (i.e. low halves equal).
     * Catches a legalizer that compares only the low long.
     * a = 0x0100000000000000, b = 0. eq=0 ne=1 slt=0 sgt=1 ult=0 ugt=1  → 010101 */
    a[0] = 0x01000000UL; a[1] = 0; b[0] = 0; b[1] = 0; run(a, b);
    /* CHECK-NEXT: 010101 */

    /* Row 6: INT64_MIN vs INT64_MAX. signed: MIN < MAX. unsigned: MIN
     * (= 0x8000…) > MAX (= 0x7FFF…). eq=0 ne=1 slt=1 sgt=0 ult=0 ugt=1  → 011001 */
    a[0] = 0x80000000UL; a[1] = 0;
    b[0] = 0x7FFFFFFFUL; b[1] = 0xFFFFFFFFUL;
    run(a, b);
    /* CHECK-NEXT: 011001 */
}
