/*
 * harness-shift32-var.c — C test driver for codegen-shift32-var.ll.
 * Tests i32 variable shift compiled from LLVM IR (which lowers to
 * the __ashlsi3 libcall).
 *
 * Both gcc6809 and LLVM-MC6809 (after ABI #4) return long via
 * sret-in-IX, so the harness can call shl32 directly with no
 * bridge wrapper.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned long shl32(unsigned long a, unsigned char n);

void test_main(void) {
    /* shl32(1, 16) = 0x00010000 */
    h_putlong(shl32(1UL, 16));
    h_putnl();
    /* CHECK: 00010000 */

    /* shl32(0x0000DEAD, 8) = 0x00DEAD00 */
    h_putlong(shl32(0x0000DEADUL, 8));
    h_putnl();
    /* CHECK-NEXT: 00DEAD00 */
}
