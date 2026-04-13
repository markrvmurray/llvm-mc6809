/*
 * C test harness for bugs #46 (G_ABS), #47 (atoi/null pointer),
 * #48 (LEASi operand fix / 16-bit bitwise via push/pull).
 * Compiled with gcc6809, linked with LLVM-compiled functions under test.
 */

#define ACIA_STATUS (*(volatile unsigned char *)0xBF00)
#define ACIA_DATA   (*(volatile unsigned char *)0xBF01)

static void h_putc(unsigned char c) {
    while (!(ACIA_STATUS & 0x02)) ;
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

static void h_putnl(void) { h_putc('\n'); }

extern int test_abs(int i);
extern int test_atoi_neg(const char *s);
extern int test_or16(int a, int b);
extern int test_and16(int a, int b);
extern int test_xor16(int a, int b);

void test_main(void) {
    /* abs */
    h_putx(test_abs(5));   h_putnl();  /* 0005 */
    h_putx(test_abs(-3));  h_putnl();  /* 0003 */
    h_putx(test_abs(0));   h_putnl();  /* 0000 */
    h_putx(test_abs(-1));  h_putnl();  /* 0001 */

    /* atoi */
    h_putx(test_atoi_neg("42"));  h_putnl();  /* 002A */
    h_putx(test_atoi_neg("0"));   h_putnl();  /* 0000 */
    h_putx(test_atoi_neg("123")); h_putnl();  /* 007B */

    /* 16-bit bitwise */
    h_putx(test_or16(0x00FF, (int)0xFF00));   h_putnl();  /* FFFF */
    h_putx(test_and16(0x0F0F, (int)0xFF00));  h_putnl();  /* 0F00 */
    h_putx(test_xor16((int)0xAAAA, 0x5555));  h_putnl();  /* FFFF */
}
