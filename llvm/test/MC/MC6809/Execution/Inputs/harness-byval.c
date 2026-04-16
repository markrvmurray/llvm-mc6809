/*
 * harness-byval.c — C test driver for codegen-byval.ll. Tests
 * struct pass-by-value (LLVM `byval` attribute) end-to-end
 * across the gcc6809 ↔ LLVM-MC6809 boundary.
 *
 * Both compilers push the struct's bytes directly onto the
 * outgoing arg stack as a contiguous block. The callee receives
 * a pointer to that in-frame block (no register involved).
 *
 * The C source uses natural struct-by-value form; gcc6809
 * compiles it the same way LLVM-MC6809 expects.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

struct Point {
    unsigned int x;
    unsigned int y;
};

extern unsigned int get_x     (struct Point p);
extern unsigned int get_y     (struct Point p);
extern unsigned int sum_point (struct Point p);

void test_main(void) {
    /* Locals (not globals) — gcc6809 emits initialised globals via
     * .area .data which sdcc2gas.sh does not translate to GAS. */
    struct Point point1, point2;
    point1.x = 0x1234; point1.y = 0x5678;
    point2.x = 0x00FF; point2.y = 0xFF00;

    h_putx(get_x    (point1)); h_putnl();  /* CHECK: 1234 */
    h_putx(get_y    (point1)); h_putnl();  /* CHECK-NEXT: 5678 */
    h_putx(sum_point(point1)); h_putnl();  /* CHECK-NEXT: 68AC */
    h_putx(sum_point(point2)); h_putnl();  /* CHECK-NEXT: FFFF */
}
