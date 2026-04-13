/*
 * harness-shift32-var.c — C test driver for codegen-shift32-var.ll.
 * Tests i32 variable shift compiled from LLVM IR (which lowers to
 * the __ashlsi3 libcall).
 *
 * shl32 returns i32, and gcc6809's long-return CC differs from
 * LLVM-MC6809's, so the harness calls a void-returning out-pointer
 * wrapper (shl32_w) defined in codegen-shift32-var.ll. The wrapper
 * is compiled by LLVM, so the inner i32 call (and the __ashlsi3
 * libcall it expands to) uses LLVM's native calling convention.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern void shl32_w(unsigned long *out, unsigned long a, unsigned char n);

void test_main(void) {
    unsigned long r;

    /* shl32(1, 16) = 0x00010000 */
    shl32_w(&r, 1UL, 16);
    h_putlong(r); h_putnl();
    /* CHECK: 00010000 */

    /* shl32(0x0000DEAD, 8) = 0x00DEAD00 */
    shl32_w(&r, 0x0000DEADUL, 8);
    h_putlong(r); h_putnl();
    /* CHECK-NEXT: 00DEAD00 */
}
