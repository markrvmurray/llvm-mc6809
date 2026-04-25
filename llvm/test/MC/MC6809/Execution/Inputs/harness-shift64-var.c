/*
 * harness-shift64-var.c — driver for codegen-shift64-var.ll. Each
 * row runs all three shift directions (shl, lshr, ashr) on the
 * same operand at one count from the off-by-one risk set
 * {0, 1, 7, 8, 31, 32, 63}. Off-by-one or byte/bit confusion
 * surfaces as a single-row mismatch.
 */

#include "harness-common.h"

extern void shl64_w (unsigned long *r, unsigned long *a, unsigned int n);
extern void lshr64_w(unsigned long *r, unsigned long *a, unsigned int n);
extern void ashr64_w(unsigned long *r, unsigned long *a, unsigned int n);

static void h_putlonglong(unsigned long *v) {
    h_putlong(v[0]);
    h_putlong(v[1]);
}

static void run(unsigned long *a, unsigned int n) {
    unsigned long r[2];
    shl64_w (r, a, n); h_putlonglong(r); h_putnl();
    lshr64_w(r, a, n); h_putlonglong(r); h_putnl();
    ashr64_w(r, a, n); h_putlonglong(r); h_putnl();
}

void test_main(void) {
    unsigned long a[2];

    /* Operand: 0xFEDCBA9876543210 (each nibble distinct, signed-negative
     * in MSB so ashr vs lshr divergence is visible). */
    a[0] = 0xFEDCBA98UL; a[1] = 0x76543210UL;

    run(a, 0);
    /* CHECK: FEDCBA9876543210 */
    /* CHECK-NEXT: FEDCBA9876543210 */
    /* CHECK-NEXT: FEDCBA9876543210 */

    run(a, 1);
    /* shl1   = FDB9753 0ECA8642 0  → FDB97530ECA86420
     * lshr1  = 7F6E5D4C3B2A1908
     * ashr1  = FF6E5D4C3B2A1908                                  */
    /* CHECK-NEXT: FDB97530ECA86420 */
    /* CHECK-NEXT: 7F6E5D4C3B2A1908 */
    /* CHECK-NEXT: FF6E5D4C3B2A1908 */

    run(a, 7);
    /* shl7  = 6E5D4C3B2A190800  (top 7 bits lost off MSB)
     * lshr7 = 01FDB97530ECA864
     * ashr7 = FFFDB97530ECA864                                   */
    /* CHECK-NEXT: 6E5D4C3B2A190800 */
    /* CHECK-NEXT: 01FDB97530ECA864 */
    /* CHECK-NEXT: FFFDB97530ECA864 */

    run(a, 8);
    /* shl8  = DCBA987654321000
     * lshr8 = 00FEDCBA98765432
     * ashr8 = FFFEDCBA98765432                                   */
    /* CHECK-NEXT: DCBA987654321000 */
    /* CHECK-NEXT: 00FEDCBA98765432 */
    /* CHECK-NEXT: FFFEDCBA98765432 */

    run(a, 31);
    /* shl31  = 3B2A190800000000  (low 31 bits = 0)
     * lshr31 = 00000001FDB97530
     * ashr31 = FFFFFFFFFDB97530                                  */
    /* CHECK-NEXT: 3B2A190800000000 */
    /* CHECK-NEXT: 00000001FDB97530 */
    /* CHECK-NEXT: FFFFFFFFFDB97530 */

    run(a, 32);
    /* shl32  = 7654321000000000
     * lshr32 = 00000000FEDCBA98
     * ashr32 = FFFFFFFFFEDCBA98                                  */
    /* CHECK-NEXT: 7654321000000000 */
    /* CHECK-NEXT: 00000000FEDCBA98 */
    /* CHECK-NEXT: FFFFFFFFFEDCBA98 */

    run(a, 63);
    /* shl63  = 0000000000000000  (only bit 0 of operand survives, but
     *                              bit 0 is 0 here)
     * lshr63 = 0000000000000001
     * ashr63 = FFFFFFFFFFFFFFFF                                  */
    /* CHECK-NEXT: 0000000000000000 */
    /* CHECK-NEXT: 0000000000000001 */
    /* CHECK-NEXT: FFFFFFFFFFFFFFFF */
}
