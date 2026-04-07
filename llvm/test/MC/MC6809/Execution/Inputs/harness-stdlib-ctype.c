/*
 * C test harness for stdlib (abs, atoi, div) and ctype functions.
 * Compiled with gcc6809, linked with LLVM-compiled functions under test.
 * Validates calling convention interop between GCC and LLVM MC6809.
 */

#include "harness-common.h"

/* Functions under test (compiled by LLVM from codegen-stdlib-ctype.ll) */
extern int test_abs(int i);
extern int test_atoi(const char *s);
extern int test_atoi_neg(const char *s);
extern void test_div(int *result, int numer, int denom);
extern int test_or16(int a, int b);
extern int test_and16(int a, int b);
extern int test_isdigit(int c);
extern int test_isalpha(int c);
extern int test_isupper(int c);
extern int test_islower(int c);

void test_main(void) {
    int divbuf[2];

    /* abs */
    h_putx(test_abs(5));       h_putnl();  /* 0005 */
    h_putx(test_abs(-3));      h_putnl();  /* 0003 */
    h_putx(test_abs(0));       h_putnl();  /* 0000 */

    /* atoi (positive, bug #52 fix) */
    h_putx(test_atoi("42"));   h_putnl();  /* 002A */
    h_putx(test_atoi("0"));    h_putnl();  /* 0000 */
    h_putx(test_atoi("999"));  h_putnl();  /* 03E7 */

    /* atoi_neg (positive and negative -- bug #49 fix) */
    h_putx(test_atoi_neg("42"));   h_putnl();  /* 002A */
    h_putx(test_atoi_neg("-3"));   h_putnl();  /* FFFD */
    h_putx(test_atoi_neg("0"));    h_putnl();  /* 0000 */
    h_putx(test_atoi_neg("-1"));   h_putnl();  /* FFFF */
    h_putx(test_atoi_neg("-999")); h_putnl();  /* FC19 */

    /* div: 17 / 5 = 3 rem 2 */
    test_div(divbuf, 17, 5);
    h_putx(divbuf[0]); h_putx(divbuf[1]); h_putnl();  /* 00030002 */

    /* 16-bit bitwise */
    h_putx(test_or16(0x00FF, (int)0xFF00));  h_putnl();  /* FFFF */
    h_putx(test_and16(0x0F0F, (int)0xFF00)); h_putnl();  /* 0F00 */

    /* ctype */
    h_putx(test_isdigit(0x35)); h_putnl();  /* 0001 -- '5' */
    h_putx(test_isdigit(0x41)); h_putnl();  /* 0000 -- 'A' */
    h_putx(test_isdigit(0x20)); h_putnl();  /* 0000 -- ' ' */
    h_putx(test_isalpha(0x61)); h_putnl();  /* 0001 -- 'a' */
    h_putx(test_isalpha(0x33)); h_putnl();  /* 0000 -- '3' */
    h_putx(test_isupper(0x41)); h_putnl();  /* 0001 -- 'A' */
    h_putx(test_isupper(0x61)); h_putnl();  /* 0000 -- 'a' */
    h_putx(test_isupper(0x21)); h_putnl();  /* 0000 -- '!' */
    h_putx(test_islower(0x7A)); h_putnl();  /* 0001 -- 'z' */
    h_putx(test_islower(0x5A)); h_putnl();  /* 0000 -- 'Z' */
}
