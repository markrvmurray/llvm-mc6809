/*
 * harness-shift.c — C test driver for codegen-shift.ll. Tests i8 and
 * i16 shifts (shl/lshr/ashr), both with constant amounts (selected
 * by the isel) and with variable amounts (lowered to libcalls into
 * shiftqi3.inc / shifthi3.inc). All function signatures use plain
 * i8/i16 args and returns, so no bridge wrappers are needed.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

/* Constant-amount i8 shifts */
extern unsigned char test_shl8_1(unsigned char a);
extern unsigned char test_shl8_4(unsigned char a);
extern unsigned char test_lshr8_1(unsigned char a);
extern unsigned char test_lshr8_4(unsigned char a);
extern signed char   test_ashr8_1(signed char a);
extern signed char   test_ashr8_4(signed char a);

/* Constant-amount i16 shifts */
extern unsigned int test_shl16_1(unsigned int a);
extern unsigned int test_lshr16_1(unsigned int a);
extern unsigned int test_lshr16_4(unsigned int a);
extern          int test_ashr16_1(         int a);
extern          int test_ashr16_4(         int a);

/* Variable-amount shifts (lowered to libcalls in shiftqi3.inc /
   shifthi3.inc — exercises both the LLVM-emitted libcall path and
   the runtime library implementation). */
extern unsigned char test_shl8_var(unsigned char a, unsigned char b);
extern unsigned char test_lshr8_var(unsigned char a, unsigned char b);
extern signed char   test_ashr8_var(signed char a, signed char b);
extern unsigned int  test_shl16_var(unsigned int a, unsigned int b);
extern unsigned int  test_lshr16_var(unsigned int a, unsigned int b);
extern          int  test_ashr16_var(         int a,          int b);

void test_main(void) {
    /* ===== Constant-amount i8 shifts ===== */
    h_puthex(test_shl8_1(0x42));   h_putnl();   /* CHECK: 84 */
    h_puthex(test_shl8_4(0x0F));   h_putnl();   /* CHECK-NEXT: F0 */
    h_puthex(test_lshr8_1(0x84));  h_putnl();   /* CHECK-NEXT: 42 */
    h_puthex(test_lshr8_4(0xF0));  h_putnl();   /* CHECK-NEXT: 0F */
    h_puthex((unsigned char)test_ashr8_1((signed char)0x80));  h_putnl();   /* CHECK-NEXT: C0 */
    h_puthex((unsigned char)test_ashr8_1(0x40));               h_putnl();   /* CHECK-NEXT: 20 */
    h_puthex((unsigned char)test_ashr8_4((signed char)0x80));  h_putnl();   /* CHECK-NEXT: F8 */
    h_puthex((unsigned char)test_ashr8_4(0x7F));               h_putnl();   /* CHECK-NEXT: 07 */

    /* ===== Constant-amount i16 shifts ===== */
    h_putx(test_shl16_1(0x1234));   h_putnl();   /* CHECK-NEXT: 2468 */
    h_putx(test_shl16_1(0x8001));   h_putnl();   /* CHECK-NEXT: 0002 */
    h_putx(test_lshr16_1(0x2468));  h_putnl();   /* CHECK-NEXT: 1234 */
    h_putx(test_lshr16_1(0x8000));  h_putnl();   /* CHECK-NEXT: 4000 */
    h_putx((unsigned)test_ashr16_1((int)0x8000));  h_putnl();   /* CHECK-NEXT: C000 */
    h_putx(test_lshr16_4(0xABCD));  h_putnl();   /* CHECK-NEXT: 0ABC */
    h_putx((unsigned)test_ashr16_4((int)0x8000));  h_putnl();   /* CHECK-NEXT: F800 */

    /* ===== Variable-amount i8 shifts (libcall path) ===== */
    h_puthex(test_shl8_var(1, 7));               h_putnl();   /* CHECK-NEXT: 80 */
    h_puthex(test_lshr8_var(0x80, 7));           h_putnl();   /* CHECK-NEXT: 01 */
    h_puthex((unsigned char)test_ashr8_var((signed char)0x80, 4));  h_putnl();   /* CHECK-NEXT: F8 */

    /* ===== Variable-amount i16 shifts (libcall path) ===== */
    h_putx(test_shl16_var(0x1234, 4));           h_putnl();   /* CHECK-NEXT: 2340 */
    h_putx(test_lshr16_var(0xABCD, 8));          h_putnl();   /* CHECK-NEXT: 00AB */
    h_putx((unsigned)test_ashr16_var((int)0x8000, 8));  h_putnl();   /* CHECK-NEXT: FF80 */
}
