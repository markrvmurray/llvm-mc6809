/*
 * harness-if.c — C test driver for codegen-if.ll (icmp + select on
 * i8/i16/i32). Compiled with gcc6809, linked with the LLVM-compiled
 * functions under test. Validates the conditional / select-lowering
 * paths end-to-end at every opt level.
 */

#define ACIA_STATUS (*(volatile unsigned char *)0xBF00)
#define ACIA_DATA   (*(volatile unsigned char *)0xBF01)

static void h_putc(unsigned char c) {
    while (!(ACIA_STATUS & 0x02))
        ;
    ACIA_DATA = c;
}

static void h_putnibble(unsigned char n) {
    h_putc(n < 10 ? '0' + n : 'A' + n - 10);
}

static void h_puthex(unsigned char b) {
    h_putnibble(b >> 4);
    h_putnibble(b & 0x0F);
}

static void h_putx(unsigned int x) {
    h_puthex((unsigned char)(x >> 8));
    h_puthex((unsigned char)(x & 0xFF));
}

static void h_putlong(unsigned long l) {
    h_putx((unsigned int)(l >> 16));
    h_putx((unsigned int)(l & 0xFFFFu));
}

static void h_putnl(void) {
    h_putc('\n');
}

/* ===== Functions under test (from codegen-if.ll) =====
 *
 * The 32-bit return functions are called via out-pointer wrappers
 * because the long return CCs differ between the two compilers.
 * The i32-arg-with-i16-return functions use pointer-to-long wrappers
 * because gcc6809 and LLVM-MC6809 disagree on i32 stack-arg layout.
 * Everything else (including the i8 functions) is called directly.
 */

/* Direct calls (gcc6809 and LLVM-MC6809 agree) */
extern signed char     test_max_s8(signed char a, signed char b);
extern signed int      test_max_s16(signed int a, signed int b);
extern unsigned char   test_max_u8(unsigned char a, unsigned char b);
extern unsigned int    test_max_u16(unsigned int a, unsigned int b);
extern signed char     test_min_s8(signed char a, signed char b);
extern unsigned char   test_eq8(signed char a, signed char b);
extern unsigned char   test_ne8(signed char a, signed char b);
extern unsigned int    test_eq16(unsigned int a, unsigned int b);
extern unsigned int    test_ne16(unsigned int a, unsigned int b);

/* Long-return bridges (out-pointer pattern) */
extern void test_max_s32_w(long *out, long a, long b);
extern void test_min_u32_w(unsigned long *out, unsigned long a, unsigned long b);

/* i32-arg bridges with i16 return (pointer-to-long inputs because
   gcc6809 and LLVM-MC6809 disagree on i32 stack-arg layout) */
extern int test_eq32_w(const long *a, const long *b);
extern int test_ne32_w(const long *a, const long *b);

void test_main(void) {
    /* ===== Signed max i8 (sgt) ===== */
    h_puthex((unsigned char)test_max_s8(5, 3));   h_putnl();   /* CHECK: 05 */
    h_puthex((unsigned char)test_max_s8(3, 5));   h_putnl();   /* CHECK-NEXT: 05 */
    h_puthex((unsigned char)test_max_s8(-1, 1));  h_putnl();   /* CHECK-NEXT: 01 */
    h_puthex((unsigned char)test_max_s8(7, 7));   h_putnl();   /* CHECK-NEXT: 07 */

    /* ===== Signed max i16 (sgt) ===== */
    h_putx((unsigned)test_max_s16(1000, 500));    h_putnl();   /* CHECK-NEXT: 03E8 */
    h_putx((unsigned)test_max_s16(-1, 1));        h_putnl();   /* CHECK-NEXT: 0001 */

    /* ===== Unsigned max i8 (ugt) ===== */
    h_puthex(test_max_u8(0xFF, 0x01));            h_putnl();   /* CHECK-NEXT: FF */

    /* ===== Unsigned max i16 (ugt) ===== */
    h_putx(test_max_u16(0xFFFFu, 0x0001u));       h_putnl();   /* CHECK-NEXT: FFFF */

    /* ===== Signed min i8 (slt) ===== */
    h_puthex((unsigned char)test_min_s8(5, 3));   h_putnl();   /* CHECK-NEXT: 03 */
    h_puthex((unsigned char)test_min_s8(-1, 1));  h_putnl();   /* CHECK-NEXT: FF */

    /* ===== Equality / not-equal i8 ===== */
    h_puthex(test_eq8(42, 42));   h_putnl();                    /* CHECK-NEXT: 01 */
    h_puthex(test_eq8(42, 43));   h_putnl();                    /* CHECK-NEXT: 00 */
    h_puthex(test_ne8(42, 43));   h_putnl();                    /* CHECK-NEXT: 01 */
    h_puthex(test_ne8(42, 42));   h_putnl();                    /* CHECK-NEXT: 00 */

    /* ===== Equality / not-equal i16 ===== */
    h_putx(test_eq16(0x1234, 0x1234));  h_putnl();              /* CHECK-NEXT: 0001 */
    h_putx(test_eq16(0x1234, 0x5678));  h_putnl();              /* CHECK-NEXT: 0000 */
    h_putx(test_ne16(0x1234, 0x5678));  h_putnl();              /* CHECK-NEXT: 0001 */
    h_putx(test_ne16(0xAAAAu, 0xAAAAu)); h_putnl();             /* CHECK-NEXT: 0000 */

    /* ===== i32 signed max (sgt + select) ===== */
    /* Bug #30 fix: CMPX used for hi-word to avoid D clobber */
    {
        long r;

        test_max_s32_w(&r, 3L, 1L);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000003 */

        test_max_s32_w(&r, 1L, 3L);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000003 */

        test_max_s32_w(&r, -1L, 1L);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 00000001 */

        /* Same hi, different lo: tests the lo-word compare path */
        test_max_s32_w(&r, 0x12345678L, 0x12345679L);
        h_putlong((unsigned long)r); h_putnl();
        /* CHECK-NEXT: 12345679 */
    }

    /* ===== i32 unsigned min (ult + select) ===== */
    {
        unsigned long r;
        test_min_u32_w(&r, 5UL, 3UL);
        h_putlong(r); h_putnl();
        /* CHECK-NEXT: 00000003 */
    }

    /* ===== i32 equality / not-equal — via pointer-to-long wrappers ===== */
    {
        long a, b;

        a = 0x12345678L; b = 0x12345678L;
        h_putx(test_eq32_w(&a, &b)); h_putnl();
        /* CHECK-NEXT: 0001 */

        a = 0x12345678L; b = 0x12345679L;  /* lo differs */
        h_putx(test_eq32_w(&a, &b)); h_putnl();
        /* CHECK-NEXT: 0000 */

        a = 0x12345678L; b = 0x12355678L;  /* hi differs */
        h_putx(test_eq32_w(&a, &b)); h_putnl();
        /* CHECK-NEXT: 0000 */

        a = 0L; b = 0L;
        h_putx(test_eq32_w(&a, &b)); h_putnl();
        /* CHECK-NEXT: 0001 */

        a = 0x12345678L; b = 0x12345678L;
        h_putx(test_ne32_w(&a, &b)); h_putnl();              /* equal */
        /* CHECK-NEXT: 0000 */

        a = 0x12345678L; b = 0xFFFFFFFFL;
        h_putx(test_ne32_w(&a, &b)); h_putnl();              /* totally different */
        /* CHECK-NEXT: 0001 */
    }
}
