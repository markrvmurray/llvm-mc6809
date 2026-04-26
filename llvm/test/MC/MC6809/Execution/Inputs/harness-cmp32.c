/*
 * harness-cmp32.c — driver for codegen-cmp32.ll. Exercises the six
 * i32 ICMP forms (eq, ne, slt, sgt, ult, ugt). Mirrors
 * harness-cmp64.c one width down.
 *
 * Result of each comparison is stored as a single byte (0 or 1) by
 * the LLVM-side wrapper; the harness prints it as one hex digit.
 *
 * Signed vs unsigned divergence is exercised by row 4 of each test
 * (a = 0xFFFFFFFF). As signed, that's -1, so slt/-1 vs 1 is true
 * (1) and sgt/-1 vs 1 is false (0). As unsigned, that's 2^32-1, so
 * ult/MAX vs 1 is false (0) and ugt/MAX vs 1 is true (1). Any
 * sign-confusion in the legalizer or in __cmpsi2/__ucmpsi2 flips
 * those four bits.
 */

#include "harness-common.h"

extern void eq32_w (unsigned char *r, unsigned long *a, unsigned long *b);
extern void ne32_w (unsigned char *r, unsigned long *a, unsigned long *b);
extern void slt32_w(unsigned char *r, unsigned long *a, unsigned long *b);
extern void sgt32_w(unsigned char *r, unsigned long *a, unsigned long *b);
extern void ult32_w(unsigned char *r, unsigned long *a, unsigned long *b);
extern void ugt32_w(unsigned char *r, unsigned long *a, unsigned long *b);

static void run(unsigned long *a, unsigned long *b) {
    unsigned char r;
    eq32_w (&r, a, b); h_putnibble(r);
    ne32_w (&r, a, b); h_putnibble(r);
    slt32_w(&r, a, b); h_putnibble(r);
    sgt32_w(&r, a, b); h_putnibble(r);
    ult32_w(&r, a, b); h_putnibble(r);
    ugt32_w(&r, a, b); h_putnibble(r);
    h_putnl();
}

void test_main(void) {
    unsigned long a, b;

    /* Row 1: a == b. eq=1 ne=0 slt=0 sgt=0 ult=0 ugt=0  → 100000 */
    a = 42; b = 42; run(&a, &b);
    /* CHECK: 100000 */

    /* Row 2: a < b (small positives). eq=0 ne=1 slt=1 sgt=0 ult=1 ugt=0  → 011010 */
    a = 1; b = 2; run(&a, &b);
    /* CHECK-NEXT: 011010 */

    /* Row 3: a > b (small positives). eq=0 ne=1 slt=0 sgt=1 ult=0 ugt=1  → 010101 */
    a = 2; b = 1; run(&a, &b);
    /* CHECK-NEXT: 010101 */

    /* Row 4: a = -1 (= 0xFFFFFFFF), b = 1. Signed: -1 < 1.
     * Unsigned: 2^32-1 > 1. eq=0 ne=1 slt=1 sgt=0 ult=0 ugt=1  → 011001 */
    a = 0xFFFFFFFFUL; b = 1; run(&a, &b);
    /* CHECK-NEXT: 011001 */

    /* Row 5: high-byte-only difference. a = 0x01000000, b = 0.
     * Catches a legalizer that compares only the low 16 bits.
     * eq=0 ne=1 slt=0 sgt=1 ult=0 ugt=1  → 010101 */
    a = 0x01000000UL; b = 0; run(&a, &b);
    /* CHECK-NEXT: 010101 */

    /* Row 6: INT32_MIN (0x80000000) vs INT32_MAX (0x7FFFFFFF).
     * Signed: MIN < MAX. Unsigned: MIN > MAX (sign bit makes it bigger).
     * eq=0 ne=1 slt=1 sgt=0 ult=0 ugt=1  → 011001 */
    a = 0x80000000UL; b = 0x7FFFFFFFUL; run(&a, &b);
    /* CHECK-NEXT: 011001 */
}
