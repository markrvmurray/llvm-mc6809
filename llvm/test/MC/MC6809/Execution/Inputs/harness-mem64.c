/*
 * harness-mem64.c — driver for codegen-mem64.ll. Each round-trips
 * a single 8-byte test pattern through one of four memory shapes
 * (stack, global, struct field, variable-index array). If load and
 * store agree on layout the same pattern comes back unchanged.
 *
 * Pattern is 0xDEADBEEFCAFEBABE chosen so a single-bit error in
 * any of the 8 bytes prints differently. A second pass with all
 * zeros catches stuck-at-1 bugs.
 */

#include "harness-common.h"

extern void stack64_w (unsigned long *r, unsigned long *i);
extern void global64_w(unsigned long *r, unsigned long *i);
extern void struct64_w(unsigned long *r, unsigned long *i);
extern void array64_w (unsigned long *r, unsigned long *i, unsigned int idx);

static void h_putlonglong(unsigned long *v) {
    h_putlong(v[0]);
    h_putlong(v[1]);
}

static void cycle(unsigned long *in) {
    unsigned long r[2];
    stack64_w (r, in); h_putlonglong(r); h_putnl();
    global64_w(r, in); h_putlonglong(r); h_putnl();
    struct64_w(r, in); h_putlonglong(r); h_putnl();
    array64_w (r, in, 2); h_putlonglong(r); h_putnl();
}

void test_main(void) {
    unsigned long in[2];

    /* Pattern 1: distinct-byte witness. */
    in[0] = 0xDEADBEEFUL; in[1] = 0xCAFEBABEUL;
    cycle(in);
    /* CHECK: DEADBEEFCAFEBABE */
    /* CHECK-NEXT: DEADBEEFCAFEBABE */
    /* CHECK-NEXT: DEADBEEFCAFEBABE */
    /* CHECK-NEXT: DEADBEEFCAFEBABE */

    /* Pattern 2: all-zeros (stuck-at-1 detector). */
    in[0] = 0; in[1] = 0;
    cycle(in);
    /* CHECK-NEXT: 0000000000000000 */
    /* CHECK-NEXT: 0000000000000000 */
    /* CHECK-NEXT: 0000000000000000 */
    /* CHECK-NEXT: 0000000000000000 */

    /* Pattern 3: all-ones (stuck-at-0 detector). */
    in[0] = 0xFFFFFFFFUL; in[1] = 0xFFFFFFFFUL;
    cycle(in);
    /* CHECK-NEXT: FFFFFFFFFFFFFFFF */
    /* CHECK-NEXT: FFFFFFFFFFFFFFFF */
    /* CHECK-NEXT: FFFFFFFFFFFFFFFF */
    /* CHECK-NEXT: FFFFFFFFFFFFFFFF */
}
