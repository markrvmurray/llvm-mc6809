/*
 * harness-loop.c — C test driver for codegen-loop.ll. Tests loop
 * control flow (single- and multi-value PHI loops with back-edges
 * and exit blocks).
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int countdown(int n);
extern unsigned int sum_to_n (int n);

void test_main(void) {
    /* countdown: returns original n after looping */
    h_putx(countdown(5));    h_putnl();  /* CHECK: 0005 */
    h_putx(countdown(1));    h_putnl();  /* CHECK-NEXT: 0001 */
    h_putx(countdown(0));    h_putnl();  /* CHECK-NEXT: 0000 */
    h_putx(countdown(256));  h_putnl();  /* CHECK-NEXT: 0100 */
    h_putx(countdown(-1));   h_putnl();  /* CHECK-NEXT: FFFF */

    /* sum_to_n: multi-value loop (sum + counter PHIs) */
    h_putx(sum_to_n(3));     h_putnl();  /* CHECK-NEXT: 0006 */
    h_putx(sum_to_n(10));    h_putnl();  /* CHECK-NEXT: 0037 */
    h_putx(sum_to_n(0));     h_putnl();  /* CHECK-NEXT: 0000 */
    h_putx(sum_to_n(1));     h_putnl();  /* CHECK-NEXT: 0001 */
    h_putx(sum_to_n(100));   h_putnl();  /* CHECK-NEXT: 13BA */
}
