/*
 * harness-interop.c — gcc6809 driver for the LLVM-MC6809 ↔ gcc6809
 * interop test (interop.s).
 *
 * gcc6809 (the reference compiler) and LLVM-MC6809 share ABI
 * version 1: arg dispatch is by *type* with two independent
 * register slots — the first i8 fills B, the first i16 fills X,
 * anything past those slots goes on the stack (i8 in 1-byte slots,
 * i16 in 2-byte slots). Return i8 in B, return i16 in X.
 *
 * Each call below was probed against gcc6809 before this file was
 * written:
 *
 *   llvm_add8(a, b)      — a in B, b pushed via `stb ,-s` (lands
 *                          at 2,s after jsr saves the return addr)
 *   llvm_negate8(a)      — a in B (single i8 arg)
 *   llvm_identity16(a)   — a in X (first i16 arg)
 *
 * If LLVM-MC6809 ever drifts away from gcc6809 on any of these
 * dispatch decisions, the link still succeeds but the runtime
 * output mismatches FileCheck — that is the failure mode this
 * test catches.
 */

#include "harness-common.h"

extern unsigned char llvm_add8     (unsigned char a, unsigned char b);
extern signed char   llvm_negate8  (signed char a);
extern unsigned int  llvm_identity16(unsigned int a);

void test_main(void) {
    /* llvm_add8(0x30, 0x12) = 0x42 — i8 in B + i8 at 2,s */
    h_puthex(llvm_add8(0x30, 0x12));
    h_putnl();
    /* CHECK: 42 */

    /* llvm_negate8(0xBE) = -0xBE = 0x42 (mod 256) — single i8 in B */
    h_puthex((unsigned char)llvm_negate8((signed char)0xBE));
    h_putnl();
    /* CHECK-NEXT: 42 */

    /* llvm_identity16(0xBEEF) = 0xBEEF — first i16 in X */
    h_putx(llvm_identity16(0xBEEF));
    h_putnl();
    /* CHECK-NEXT: BEEF */

    /* llvm_identity16(0x1234) = 0x1234 — second call, same path */
    h_putx(llvm_identity16(0x1234));
    h_putnl();
    /* CHECK-NEXT: 1234 */
}
