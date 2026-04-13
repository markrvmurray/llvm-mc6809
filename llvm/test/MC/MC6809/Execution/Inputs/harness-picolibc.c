/*
 * C test harness for picolibc-style functions: memcpy, memset, strcpy,
 * isspace, isprint, isxdigit, tolower, toupper.
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

/* Functions under test */
extern void *test_memcpy(void *dst, const void *src, unsigned int n);
extern void *test_memset(void *dst, int c, unsigned int n);
extern char *test_strcpy(char *dst, const char *src);
extern int test_isspace(int c);
extern int test_isprint(int c);
extern int test_isxdigit(int c);
extern int test_tolower(int c);
extern int test_toupper(int c);

static int h_strcmp(const char *a, const char *b) {
    while (*a && *a == *b) { a++; b++; }
    return *(unsigned char *)a - *(unsigned char *)b;
}

void test_main(void) {
    char buf[16];

    /* memcpy */
    test_memset(buf, 0, 16);
    test_memcpy(buf, "Hello", 5);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]);
    h_puthex(buf[3]); h_puthex(buf[4]); h_putnl();
    /* 48656C6C6F = "Hello" */

    /* memset */
    test_memset(buf, 0xAA, 4);
    h_puthex(buf[0]); h_puthex(buf[1]); h_puthex(buf[2]); h_puthex(buf[3]); h_putnl();
    /* AAAAAAAA */

    /* strcpy */
    test_strcpy(buf, "Hi");
    h_putx(h_strcmp(buf, "Hi") == 0 ? 1 : 0); h_putnl();
    /* 0001 */

    /* isspace */
    h_putx(test_isspace(' '));   h_putnl();  /* 0001 */
    h_putx(test_isspace('A'));   h_putnl();  /* 0000 */
    h_putx(test_isspace('\t'));  h_putnl();  /* 0001 */

    /* isprint */
    h_putx(test_isprint('A'));   h_putnl();  /* 0001 */
    h_putx(test_isprint(0x01));  h_putnl();  /* 0000 */

    /* isxdigit */
    h_putx(test_isxdigit('9')); h_putnl();  /* 0001 */
    h_putx(test_isxdigit('f')); h_putnl();  /* 0001 */
    h_putx(test_isxdigit('g')); h_putnl();  /* 0000 */

    /* tolower */
    h_putx(test_tolower('A'));   h_putnl();  /* 0061 = 'a' */
    h_putx(test_tolower('z'));   h_putnl();  /* 007A = 'z' (unchanged) */

    /* toupper */
    h_putx(test_toupper('a'));   h_putnl();  /* 0041 = 'A' */
    h_putx(test_toupper('Z'));   h_putnl();  /* 005A = 'Z' (unchanged) */
}
