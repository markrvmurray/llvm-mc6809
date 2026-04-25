/*
 * harness-sext64.c — driver for codegen-sext64.ll. Probes sext and
 * zext from i8/i16/i32 to i64 with both negative (sign-bit-set) and
 * positive operands.
 */

#include "harness-common.h"

extern void sext_i8_w (unsigned long *r, signed char x);
extern void sext_i16_w(unsigned long *r, signed int x);   /* int = i16 here */
extern void sext_i32_w(unsigned long *r, unsigned long *xp);
extern void zext_i8_w (unsigned long *r, unsigned char x);
extern void zext_i16_w(unsigned long *r, unsigned int x);
extern void zext_i32_w(unsigned long *r, unsigned long *xp);

static void h_putlonglong(unsigned long *v) {
    h_putlong(v[0]);
    h_putlong(v[1]);
}

void test_main(void) {
    unsigned long r[2], xp[1];

    /* sext i8: 0x7F → 0x000000000000007F (positive) */
    sext_i8_w(r, 0x7F); h_putlonglong(r); h_putnl();
    /* CHECK: 000000000000007F */

    /* sext i8: 0x80 (= -128) → 0xFFFFFFFFFFFFFF80 */
    sext_i8_w(r, (signed char)0x80); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFFFFFFFF80 */

    /* sext i16: 0x7FFF → 0x0000000000007FFF */
    sext_i16_w(r, 0x7FFF); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 0000000000007FFF */

    /* sext i16: 0x8000 (= -32768) → 0xFFFFFFFFFFFF8000 */
    sext_i16_w(r, (signed int)(short)0x8000); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFFFFFF8000 */

    /* sext i32: 0x7FFFFFFF → 0x000000007FFFFFFF */
    xp[0] = 0x7FFFFFFFUL; sext_i32_w(r, xp); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 000000007FFFFFFF */

    /* sext i32: 0x80000000 (= INT32_MIN) → 0xFFFFFFFF80000000 */
    xp[0] = 0x80000000UL; sext_i32_w(r, xp); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: FFFFFFFF80000000 */

    /* zext i8: 0xFF → 0x00000000000000FF (no sign extension) */
    zext_i8_w(r, 0xFF); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 00000000000000FF */

    /* zext i16: 0xFFFF → 0x000000000000FFFF */
    zext_i16_w(r, 0xFFFFu); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 000000000000FFFF */

    /* zext i32: 0xFFFFFFFF → 0x00000000FFFFFFFF */
    xp[0] = 0xFFFFFFFFUL; zext_i32_w(r, xp); h_putlonglong(r); h_putnl();
    /* CHECK-NEXT: 00000000FFFFFFFF */
}
