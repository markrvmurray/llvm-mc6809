/*
 * harness-mul64.c — driver for codegen-mul64.ll. Tests __muldi3
 * across small × small, full-width × small, full-width × full-width,
 * INT64_MIN × −1 (overflow into INT64_MIN unchanged), and
 * 0xFFFFFFFFFFFFFFFF × 1 (identity, sanity).
 */

#include "harness-common.h"

extern void mul64_w(unsigned long *result, unsigned long *a, unsigned long *b);

static void h_putlonglong(unsigned long *v) {
    h_putlong(v[0]);
    h_putlong(v[1]);
}

void test_main(void) {
    unsigned long r[2], a[2], b[2];

    /* 2 × 3 = 6 */
    a[0] = 0; a[1] = 2; b[0] = 0; b[1] = 3;
    mul64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK: 0000000000000006 */

    /* Full-width × small: 0x0123456789ABCDEF × 2 = 0x02468ACF13579BDE */
    a[0] = 0x01234567UL; a[1] = 0x89ABCDEFUL; b[0] = 0; b[1] = 2;
    mul64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 02468ACF13579BDE */

    /* 0xFFFFFFFF × 0xFFFFFFFF = 0xFFFFFFFE00000001
     * (low halves only — exercises 32×32→64 partial product alignment) */
    a[0] = 0; a[1] = 0xFFFFFFFFUL; b[0] = 0; b[1] = 0xFFFFFFFFUL;
    mul64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFE00000001 */

    /* INT64_MIN × -1 — well-defined under unsigned arithmetic but
     * overflows signed; result wraps back to INT64_MIN itself. */
    a[0] = 0x80000000UL; a[1] = 0;
    b[0] = 0xFFFFFFFFUL; b[1] = 0xFFFFFFFFUL;
    mul64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 8000000000000000 */

    /* 0xFFFFFFFFFFFFFFFF × 1 = 0xFFFFFFFFFFFFFFFF (identity) */
    a[0] = 0xFFFFFFFFUL; a[1] = 0xFFFFFFFFUL; b[0] = 0; b[1] = 1;
    mul64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFFFFFFFFFF */

    /* 0 × anything = 0 */
    a[0] = 0; a[1] = 0;
    b[0] = 0xDEADBEEFUL; b[1] = 0xCAFEBABEUL;
    mul64_w(r, a, b); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000000 */
}
