/*
 * harness-switch.c — C test driver for codegen-switch.ll. Tests
 * a 5-case switch statement (which lowers to a jump table).
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int switch5(unsigned int x);

void test_main(void) {
    h_putx(switch5(0));      h_putnl();  /* CHECK: 0064 */
    h_putx(switch5(1));      h_putnl();  /* CHECK-NEXT: 00C8 */
    h_putx(switch5(4));      h_putnl();  /* CHECK-NEXT: 01F4 */
    h_putx(switch5(5));      h_putnl();  /* CHECK-NEXT: 03E7 */
    h_putx(switch5(0xFFFF)); h_putnl();  /* CHECK-NEXT: 03E7 */
}
