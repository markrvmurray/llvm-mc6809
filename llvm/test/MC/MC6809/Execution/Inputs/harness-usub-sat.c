/*
 * harness-usub-sat.c — driver for codegen-usub-sat.ll. Tests
 * llvm.usub.sat.{i8,i16} (unsigned saturating subtract).
 *
 * Per bug #115: the MC6809 backend mis-lowered the carry-out
 * from G_USUBO, so usub_sat returned 0 for nearly every input.
 * The bug was masked at -O0/-O1/-O2/-O3 alike because the same
 * legalizer path is used.
 *
 * Compiled with gcc6809; usat_i{8,16} compiled by LLVM-MC6809.
 */

#include "harness-common.h"

extern unsigned char usat_i8 (unsigned char  a, unsigned char  b);
extern unsigned int  usat_i16(unsigned int   a, unsigned int   b);

void test_main(void) {
    /* 8-bit, no underflow: 100 - 0 = 100 = 0x64 */
    h_puthex(usat_i8(100, 0));
    h_putnl();
    /* CHECK: 64 */

    /* 8-bit, no underflow: 100 - 50 = 50 = 0x32 */
    h_puthex(usat_i8(100, 50));
    h_putnl();
    /* CHECK-NEXT: 32 */

    /* 8-bit, exact zero boundary: 100 - 100 = 0 */
    h_puthex(usat_i8(100, 100));
    h_putnl();
    /* CHECK-NEXT: 00 */

    /* 8-bit, underflow saturates to zero: 100 - 200 = 0 */
    h_puthex(usat_i8(100, 200));
    h_putnl();
    /* CHECK-NEXT: 00 */

    /* 8-bit, max-input no underflow: 0xFF - 0x01 = 0xFE */
    h_puthex(usat_i8(0xFF, 0x01));
    h_putnl();
    /* CHECK-NEXT: FE */

    /* 16-bit, no underflow: 1024 - 0 = 0x0400 */
    h_putx(usat_i16(1024, 0));
    h_putnl();
    /* CHECK-NEXT: 0400 */

    /* 16-bit, no underflow: 1024 - 1 = 0x03FF
       (this was the canonical failing case behind FORTIFY chk_fail) */
    h_putx(usat_i16(1024, 1));
    h_putnl();
    /* CHECK-NEXT: 03FF */

    /* 16-bit, no underflow: 1024 - 1023 = 1 */
    h_putx(usat_i16(1024, 1023));
    h_putnl();
    /* CHECK-NEXT: 0001 */

    /* 16-bit, exact zero boundary: 1024 - 1024 = 0 */
    h_putx(usat_i16(1024, 1024));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* 16-bit, underflow saturates to zero: 1024 - 2000 = 0 */
    h_putx(usat_i16(1024, 2000));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* 16-bit, far underflow: 0 - 0xFFFF = 0 */
    h_putx(usat_i16(0, 0xFFFF));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* 16-bit, max - 1 = max - 1 = 0xFFFE (no underflow) */
    h_putx(usat_i16(0xFFFF, 1));
    h_putnl();
    /* CHECK-NEXT: FFFE */
}
