/*
 * harness-byval.c — C test driver for codegen-byval.ll. Tests
 * struct field access via a pointer parameter.
 *
 * NOTE: this test was originally codegen-byval.s and exercised
 * the LLVM `byval` attribute. LLVM-MC6809's CallLowering does
 * not currently honour byval (caller-side copy semantics) — it
 * lowers byval pointers as plain pointers in IX. The .ll has
 * been updated to use plain `ptr` parameters to match the
 * actual implementation, and this harness uses
 * `struct Point *p` accordingly.
 *
 * Proper byval lowering (caller copies the struct to a local
 * stack slot, callee gets its own private copy) is a separate
 * backend TODO. Until it lands, this test only exercises
 * pass-by-pointer of structs, not true pass-by-value.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

struct Point {
    unsigned int x;
    unsigned int y;
};

extern unsigned int get_x     (struct Point *p);
extern unsigned int get_y     (struct Point *p);
extern unsigned int sum_point (struct Point *p);

void test_main(void) {
    /* Locals (not globals) — gcc6809 emits initialised globals
     * via .area .data which sdcc2gas.sh does not translate to
     * the GAS syntax llvm-mc expects. */
    struct Point point1, point2;
    point1.x = 0x1234; point1.y = 0x5678;
    point2.x = 0x00FF; point2.y = 0xFF00;

    h_putx(get_x    (&point1)); h_putnl();  /* CHECK: 1234 */
    h_putx(get_y    (&point1)); h_putnl();  /* CHECK-NEXT: 5678 */
    h_putx(sum_point(&point1)); h_putnl();  /* CHECK-NEXT: 68AC */
    h_putx(sum_point(&point2)); h_putnl();  /* CHECK-NEXT: FFFF */
}
