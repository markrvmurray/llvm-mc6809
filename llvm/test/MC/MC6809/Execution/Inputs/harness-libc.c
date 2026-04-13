/*
 * C test harness for libc smoke test (strlen, strcmp, atoi).
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

extern int my_strlen(const char *s);
extern int my_strcmp(const char *a, const char *b);
extern int my_atoi(const char *s);

void test_main(void) {
    /* strlen */
    h_putx(my_strlen("Hello")); h_putnl();  /* 0005 */
    h_putx(my_strlen(""));      h_putnl();  /* 0000 */

    /* strcmp */
    h_putx(my_strcmp("Hello", "Hello")); h_putnl();  /* 0000 */
    h_putx(my_strcmp("Hello", "Hell"));  h_putnl();  /* 006F */
    h_putx(my_strcmp("Hell", "Help"));   h_putnl();  /* FFFC */

    /* atoi */
    h_putx(my_atoi("123"));  h_putnl();  /* 007B */
    h_putx(my_atoi("9999")); h_putnl();  /* 270F */
    h_putx(my_atoi(""));     h_putnl();  /* 0000 */
}
