/*
 * C test harness for libc smoke test (strlen, strcmp, atoi).
 * Compiled with gcc6809, linked with LLVM-compiled functions under test.
 */

#include "harness-common.h"

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
