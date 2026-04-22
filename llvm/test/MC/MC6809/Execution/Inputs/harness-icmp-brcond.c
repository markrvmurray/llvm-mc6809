/*
 * harness-icmp-brcond.c — driver for codegen-icmp-brcond.ll. Bug #114:
 * the i16 unsigned `<` comparison consumed by a `select i1, i16, i16`
 * always returned the false-side value because the materialised bool
 * register got clobbered by the select's TrueValue load before tsta
 * could read it. Fix bypasses the materialise-then-test path entirely.
 */

#include "harness-common.h"

extern unsigned int ult_i16(unsigned int a, unsigned int b);
extern unsigned int ugt_i16(unsigned int a, unsigned int b);
extern unsigned int ule_i16(unsigned int a, unsigned int b);
extern unsigned int uge_i16(unsigned int a, unsigned int b);
extern unsigned int eq_i16 (unsigned int a, unsigned int b);
extern unsigned int ne_i16 (unsigned int a, unsigned int b);

void test_main(void) {
    /* The bug specifically affects "high bytes differ" comparisons —
       picolibc test-memchr-simple's failing path was 0x011D < 0xC709
       (haystack RAM pointer < flash string pointer). Exercise the
       same shape plus a few small-vs-big and equal cases. */

    /* ult: a < b */
    h_putx(ult_i16(0x011D, 0xC709)); h_putnl();   /* CHECK: 0001 */
    h_putx(ult_i16(0xC709, 0x011D)); h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(ult_i16(5, 200));         h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(ult_i16(200, 5));         h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(ult_i16(42, 42));         h_putnl();   /* CHECK-NEXT: 0000 */

    /* ugt: a > b */
    h_putx(ugt_i16(0x011D, 0xC709)); h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(ugt_i16(0xC709, 0x011D)); h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(ugt_i16(42, 42));         h_putnl();   /* CHECK-NEXT: 0000 */

    /* ule: a <= b */
    h_putx(ule_i16(5, 200));         h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(ule_i16(200, 5));         h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(ule_i16(42, 42));         h_putnl();   /* CHECK-NEXT: 0001 */

    /* uge: a >= b */
    h_putx(uge_i16(5, 200));         h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(uge_i16(200, 5));         h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(uge_i16(42, 42));         h_putnl();   /* CHECK-NEXT: 0001 */

    /* eq */
    h_putx(eq_i16(42, 42));          h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(eq_i16(5, 200));          h_putnl();   /* CHECK-NEXT: 0000 */

    /* ne */
    h_putx(ne_i16(42, 42));          h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(ne_i16(5, 200));          h_putnl();   /* CHECK-NEXT: 0001 */
}
