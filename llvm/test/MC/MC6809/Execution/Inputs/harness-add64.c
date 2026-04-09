/*
 * harness-add64.c — C test driver for codegen-add64.ll. Tests
 * 64-bit addition and subtraction via pointer-based wrappers.
 *
 * gcc6809's long long is 4 bytes (non-conformant), so we can't
 * use long long to hold 8-byte values. Instead, we represent i64
 * as arrays of 2 unsigned longs (big-endian: [0]=hi, [1]=lo) and
 * call the LLVM wrapper functions that take pointers.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions.
 */

#include "harness-common.h"

extern void add64_w(unsigned long *result, unsigned long *a, unsigned long *b);
extern void sub64_w(unsigned long *result, unsigned long *a, unsigned long *b);

/* Print an i64 value as 16 hex digits (hi long, then lo long). */
static void h_putlonglong(unsigned long *v) {
    h_putlong(v[0]);
    h_putlong(v[1]);
}

void test_main(void) {
    unsigned long r[2], a[2], b[2];

    /* 1 + 2 = 3 */
    a[0] = 0; a[1] = 1;
    b[0] = 0; b[1] = 2;
    add64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK: 0000000000000003 */

    /* Carry from lo long into hi long: 0x00000000FFFFFFFF + 1 = 0x0000000100000000 */
    a[0] = 0; a[1] = 0xFFFFFFFFUL;
    b[0] = 0; b[1] = 1;
    add64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000100000000 */

    /* Large values: 0x123456789ABCDEF0 + 0x0000000000000010 = 0x123456789ABCDF00 */
    a[0] = 0x12345678UL; a[1] = 0x9ABCDEF0UL;
    b[0] = 0; b[1] = 0x10;
    add64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 123456789ABCDF00 */

    /* Full overflow: 0xFFFFFFFFFFFFFFFF + 1 = 0 */
    a[0] = 0xFFFFFFFFUL; a[1] = 0xFFFFFFFFUL;
    b[0] = 0; b[1] = 1;
    add64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000000 */

    /* Subtraction: 5 - 3 = 2 */
    a[0] = 0; a[1] = 5;
    b[0] = 0; b[1] = 3;
    sub64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000000002 */

    /* Borrow across long boundary: 0x0000000100000000 - 1 = 0x00000000FFFFFFFF */
    a[0] = 1; a[1] = 0;
    b[0] = 0; b[1] = 1;
    sub64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 00000000FFFFFFFF */

    /* Commutativity: same as large add but args swapped */
    a[0] = 0; a[1] = 0x10;
    b[0] = 0x12345678UL; b[1] = 0x9ABCDEF0UL;
    add64_w(r, a, b);
    h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 123456789ABCDF00 */
}
