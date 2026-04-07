/*
 * harness-picolibc-all.c — C test driver for the full picolibc
 * execution test (string.h, ctype.h, stdlib.h, stdio.h).
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test (codegen-picolibc.ll). Validates calling-convention
 * interop end-to-end.
 *
 * For functions whose LLVM CC differs from gcc6809's CC (long
 * return, varargs), the IR provides bridge wrappers (test_*_w,
 * test_printf_*) with non-variadic, non-long-returning signatures
 * that both compilers handle the same way. We call those wrappers
 * here. See the long comment in /tmp/picolibc-all.c (the source of
 * codegen-picolibc.ll) for details on the CC differences.
 */

#include "harness-common.h"

/* ===== Functions under test (compiled by LLVM from codegen-picolibc.ll) ===== */

/* string.h */
extern void *test_memcpy(void *dst, const void *src, unsigned int n);
extern void *test_memset(void *dst, int c, unsigned int n);
extern void *test_memmove(void *dst, const void *src, unsigned int n);
extern int test_memcmp(const void *s1, const void *s2, unsigned int n);
extern void *test_memchr(const void *s, int c, unsigned int n);
extern char *test_strcpy(char *dst, const char *src);
extern char *test_strncpy(char *dst, const char *src, unsigned int n);
extern char *test_strcat(char *dst, const char *src);
extern char *test_strncat(char *dst, const char *src, unsigned int n);
extern unsigned int test_strlen(const char *s);
extern char *test_strchr(const char *s, int c);
extern char *test_strrchr(const char *s, int c);
extern char *test_strstr(const char *haystack, const char *needle);
extern unsigned int test_strspn(const char *s, const char *accept);
extern unsigned int test_strcspn(const char *s, const char *reject);
extern char *test_strpbrk(const char *s, const char *accept);
extern int test_strncmp(const char *s1, const char *s2, unsigned int n);
extern char *test_strtok(char *str, const char *delim);

/* ctype.h */
extern int test_isspace(int c);
extern int test_isprint(int c);
extern int test_isxdigit(int c);
extern int test_tolower(int c);
extern int test_toupper(int c);
extern int test_ispunct(int c);
extern int test_iscntrl(int c);
extern int test_isgraph(int c);

/* stdlib.h */
extern int test_rand(void);
extern void test_srand(unsigned seed);

/* Long-returning bridges (LLVM split-result CC → gcc6809 out-pointer CC) */
extern void test_labs_w(long *out, long x);
extern void test_atol_w(long *out, const char *s);
extern void test_strtol_w(long *out, const char *nptr, int base);
extern void test_strtoul_w(unsigned long *out, const char *nptr, int base);

/* stdio.h */
extern int test_putchar(int c);

/* Variadic-bridge wrappers (gcc6809 normal CC → LLVM varargs CC) */
extern void test_printf_4(const char *fmt, int a, const char *b, int c, int d);
extern void test_printf_d(const char *fmt, int v);
extern void test_printf_uxX(const char *fmt, unsigned u, unsigned x, unsigned X);

/* String literals — keep them as globals so the linker places them
   in .rodata next to the other test data. (Local "..." literals work
   too but get duplicated by gcc6809; sharing them via globals is a
   small code-size win.) */
static const char str_hello[] = "Hello";
static const char str_help[]  = "Help!";
static const char str_hi[]    = "Hi";
static const char str_he[]    = "He";
static const char str_yex[]   = "y!";
static const char str_hox[]   = "ho!";
static const char str_ll[]    = "ll";
static const char str_xy[]    = "xy";
static const char str_empty[] = "";
static const char str_elH[]   = "elH";
static const char str_lo[]    = "lo";
static const char str_abc[]   = "a,b,c";
static const char str_comma[] = ",";

static const char str_42[]    = "42";
static const char str_neg3[]  = "-3";
static const char str_zero[]  = "0";
static const char str_ff[]    = "FF";
static const char str_0x10[]  = "0x10";
static const char str_777[]   = "777";
static const char str_12345[] = "12345";
static const char str_ffff[]  = "FFFF";

static const char str_ok[]          = "ok";
static const char fmt_printf_4[]    = "Hi=%d %s %x %c %%\n";
static const char fmt_printf_d[]    = "%d\n";
static const char fmt_printf_uxX[]  = "%u %x %X\n";

static char buf[16];

void test_main(void) {
    /* === string.h: memcpy ===
       memcpy(buf, "Hello", 5)  →  "Hello" */
    test_memcpy(buf, str_hello, 5);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_puthex(buf[3]); h_puthex(buf[4]);
    h_putnl();
    /* CHECK: 48656C6C6F */

    /* === memset: 4 bytes of 0xAA === */
    test_memset(buf, 0xAA, 4);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]); h_puthex(buf[3]);
    h_putnl();
    /* CHECK-NEXT: AAAAAAAA */

    /* === memmove forward (dst < src or non-overlap) === */
    test_memmove(buf, str_hello, 5);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_puthex(buf[3]); h_puthex(buf[4]);
    h_putnl();
    /* CHECK-NEXT: 48656C6C6F */

    /* === memmove backward (overlapping, dst > src) ===
       buf currently = "Hello"; shift right by 1: buf+0..4 → buf+1..5 */
    test_memmove(buf + 1, buf, 5);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_puthex(buf[3]); h_puthex(buf[4]); h_puthex(buf[5]);
    h_putnl();
    /* CHECK-NEXT: 4848656C6C6F */

    /* === strcpy: copy "Hi" === */
    test_strcpy(buf, str_hi);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_putnl();
    /* CHECK-NEXT: 486900 */

    /* === ctype: isspace === */
    h_putx(test_isspace(0x20)); h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(test_isspace(0x41)); h_putnl();   /* CHECK-NEXT: 0000 */
    h_putx(test_isspace(0x09)); h_putnl();   /* CHECK-NEXT: 0001 */

    /* === isprint === */
    h_putx(test_isprint(0x41)); h_putnl();   /* CHECK-NEXT: 0001 */
    h_putx(test_isprint(0x01)); h_putnl();   /* CHECK-NEXT: 0000 */

    /* === toupper === */
    h_putx(test_toupper(0x61)); h_putnl();   /* CHECK-NEXT: 0041 */
    h_putx(test_toupper(0x5A)); h_putnl();   /* CHECK-NEXT: 005A */

    /* === memcmp: equal === */
    h_putx(test_memcmp(str_hello, str_hello, 5)); h_putnl();
    /* CHECK-NEXT: 0000 */

    /* === memcmp: "Hello" vs "Help!" diff at 'l'(0x6C) - 'p'(0x70) = -4 === */
    h_putx(test_memcmp(str_hello, str_help, 5)); h_putnl();
    /* CHECK-NEXT: FFFC */

    /* === isxdigit === */
    h_putx(test_isxdigit(0x39)); h_putnl();  /* CHECK-NEXT: 0001  '9' */
    h_putx(test_isxdigit(0x67)); h_putnl();  /* CHECK-NEXT: 0000  'g' */

    /* === tolower === */
    h_putx(test_tolower(0x41)); h_putnl();   /* CHECK-NEXT: 0061  'A'→'a' */
    h_putx(test_tolower(0x7A)); h_putnl();   /* CHECK-NEXT: 007A  'z' */

    /* === strchr: find 'l' in "Hello" → offset 2 ===
       The function returns a pointer; we report the byte offset
       (matches the original asm test's `subd #str_hello`). */
    {
        char *p = test_strchr(str_hello, 0x6C);
        h_putx((unsigned)(p - str_hello));
        h_putnl();
    }
    /* CHECK-NEXT: 0002 */

    /* === strchr: find 'z' in "Hello" → NULL === */
    h_putx((unsigned)test_strchr(str_hello, 0x7A));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* === strncmp: equal first 3 === */
    h_putx(test_strncmp(str_hello, str_help, 3)); h_putnl();
    /* CHECK-NEXT: 0000 */

    /* === strncmp: 5 chars, diff at 'l' vs 'p' === */
    h_putx(test_strncmp(str_hello, str_help, 5)); h_putnl();
    /* CHECK-NEXT: FFFC */

    /* === strlen("Hello") = 5 === */
    h_putx(test_strlen(str_hello));
    h_putnl();
    /* CHECK-NEXT: 0005 */

    /* === strrchr: find LAST 'l' in "Hello" → offset 3 ===
       (KNOWN BUG #59 historically, fixed.) */
    {
        char *p = test_strrchr(str_hello, 0x6C);
        h_putx((unsigned)(p - str_hello));
        h_putnl();
    }
    /* CHECK-NEXT: 0003 */

    /* === strrchr: 'z' → NULL === */
    h_putx((unsigned)test_strrchr(str_hello, 0x7A));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* === strstr("Hello", "ll") → offset 2 ===
       (KNOWN BUG #58 historically — left in test even when fragile.) */
    {
        char *p = test_strstr(str_hello, str_ll);
        h_putx((unsigned)(p - str_hello));
        h_putnl();
    }
    /* CHECK-NEXT: 0002 */

    /* === strstr("Hello", "xy") → NULL === */
    h_putx((unsigned)test_strstr(str_hello, str_xy));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* === strstr("Hello", "") → "Hello" (offset 0) === */
    {
        char *p = test_strstr(str_hello, str_empty);
        h_putx((unsigned)(p - str_hello));
        h_putnl();
    }
    /* CHECK-NEXT: 0000 */

    /* === memchr("Hello", 'l', 5) → offset 2 === */
    {
        void *p = test_memchr(str_hello, 0x6C, 5);
        h_putx((unsigned)((char *)p - str_hello));
        h_putnl();
    }
    /* CHECK-NEXT: 0002 */

    /* === memchr("Hello", 'z', 5) → NULL === */
    h_putx((unsigned)test_memchr(str_hello, 0x7A, 5));
    h_putnl();
    /* CHECK-NEXT: 0000 */

    /* === strspn("Hello", "elH") = 4 === */
    h_putx(test_strspn(str_hello, str_elH));
    h_putnl();
    /* CHECK-NEXT: 0004 */

    /* === strcspn("Hello", "lo") = 2 === */
    h_putx(test_strcspn(str_hello, str_lo));
    h_putnl();
    /* CHECK-NEXT: 0002 */

    /* === strpbrk("Hello", "lo") → "llo" (offset 2) === */
    {
        char *p = test_strpbrk(str_hello, str_lo);
        h_putx((unsigned)(p - str_hello));
        h_putnl();
    }
    /* CHECK-NEXT: 0002 */

    /* === strncpy(buf, "Hi", 4) → "Hi\0\0" === */
    test_strncpy(buf, str_hi, 4);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]); h_puthex(buf[3]);
    h_putnl();
    /* CHECK-NEXT: 48690000 */

    /* === strcpy(buf, "He"); strcat(buf, "y!") → "Hey!\0" === */
    test_strcpy(buf, str_he);
    test_strcat(buf, str_yex);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_puthex(buf[3]); h_puthex(buf[4]);
    h_putnl();
    /* CHECK-NEXT: 4865792100 */

    /* === strcpy(buf, "Hi"); strncat(buf, "ho!", 2) → "Hiho\0" === */
    test_strcpy(buf, str_hi);
    test_strncat(buf, str_hox, 2);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_puthex(buf[3]); h_puthex(buf[4]);
    h_putnl();
    /* CHECK-NEXT: 4869686F00 */

    /* === strtok: tokenize "a,b,c" on "," ===
       Copy source into buf first since strtok modifies its input. */
    test_memcpy(buf, str_abc, 6);
    {
        /* "a" at offset 0 */
        char *t = test_strtok(buf, str_comma);
        h_putx((unsigned)(t - buf)); h_putnl();
        /* CHECK-NEXT: 0000 */

        /* "b" at offset 2 */
        t = test_strtok((char *)0, str_comma);
        h_putx((unsigned)(t - buf)); h_putnl();
        /* CHECK-NEXT: 0002 */

        /* "c" at offset 4 */
        t = test_strtok((char *)0, str_comma);
        h_putx((unsigned)(t - buf)); h_putnl();
        /* CHECK-NEXT: 0004 */

        /* NULL */
        t = test_strtok((char *)0, str_comma);
        h_putx((unsigned)t); h_putnl();
        /* CHECK-NEXT: 0000 */
    }

    /* === ispunct === */
    h_putx(test_ispunct(0x21)); h_putnl();   /* CHECK-NEXT: 0001  '!' */
    h_putx(test_ispunct(0x41)); h_putnl();   /* CHECK-NEXT: 0000  'A' */
    h_putx(test_ispunct(0x20)); h_putnl();   /* CHECK-NEXT: 0000  ' ' */

    /* === iscntrl === */
    h_putx(test_iscntrl(0x0A)); h_putnl();   /* CHECK-NEXT: 0001  '\n' */
    h_putx(test_iscntrl(0x7F)); h_putnl();   /* CHECK-NEXT: 0001  DEL */
    h_putx(test_iscntrl(0x41)); h_putnl();   /* CHECK-NEXT: 0000  'A' */

    /* === isgraph === */
    h_putx(test_isgraph(0x41)); h_putnl();   /* CHECK-NEXT: 0001  'A' */
    h_putx(test_isgraph(0x20)); h_putnl();   /* CHECK-NEXT: 0000  ' ' */
    h_putx(test_isgraph(0x7F)); h_putnl();   /* CHECK-NEXT: 0000  DEL */

    /* === labs (long return → out-pointer wrapper) === */
    {
        long r;
        test_labs_w(&r, 5);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000005 */

        test_labs_w(&r, 0);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000000 */

        test_labs_w(&r, -3);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000003 */
    }

    /* === srand(42); rand(); rand(); rand() ===
       LCG: state = state*75 + 17, return state & 0x7FFF */
    test_srand(42);
    h_putx(test_rand()); h_putnl();   /* CHECK-NEXT: 0C5F */
    h_putx(test_rand()); h_putnl();   /* CHECK-NEXT: 1FE6 */
    h_putx(test_rand()); h_putnl();   /* CHECK-NEXT: 5873 */

    test_srand(1);
    h_putx(test_rand()); h_putnl();   /* CHECK-NEXT: 005C */

    /* === atol === */
    {
        long r;
        test_atol_w(&r, str_42);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 0000002A */

        test_atol_w(&r, str_neg3);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: FFFFFFFD */

        test_atol_w(&r, str_zero);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000000 */
    }

    /* === strtol === */
    {
        long r;
        test_strtol_w(&r, str_42, 10);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 0000002A */

        test_strtol_w(&r, str_neg3, 10);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: FFFFFFFD */

        test_strtol_w(&r, str_ff, 16);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 000000FF */

        test_strtol_w(&r, str_0x10, 0);  /* base=0 → auto-detect 0x prefix */
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000010 */

        test_strtol_w(&r, str_777, 8);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 000001FF */
    }

    /* === strtoul === */
    {
        unsigned long r;
        test_strtoul_w(&r, str_12345, 10);
        h_putlong(r); h_putnl();
        /* CHECK-NEXT: 00003039 */

        test_strtoul_w(&r, str_ffff, 16);
        h_putlong(r); h_putnl();
        /* CHECK-NEXT: 0000FFFF */
    }

    /* === putchar('!') — direct ACIA write via volatile pointer === */
    test_putchar(0x21);
    h_putnl();
    /* CHECK-NEXT: ! */

    /* === printf via varargs bridges ===
       The wrappers in codegen-picolibc.ll forward fixed-arg calls
       (which gcc6809 handles correctly) into LLVM-side variadic
       calls (which test_printf expects). */
    test_printf_4(fmt_printf_4, 42, str_ok, 0xBEEF, 'Z');
    /* CHECK-NEXT: Hi=42 ok beef Z % */

    test_printf_d(fmt_printf_d, -1);
    /* CHECK-NEXT: -1 */

    test_printf_uxX(fmt_printf_uxX, 65535u, 255u, 255u);
    /* CHECK-NEXT: 65535 ff FF */
}
