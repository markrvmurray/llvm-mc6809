/* Harness for codegen-O0-reloads.ll. */
#include "harness-common.h"

extern int value_after_compare(char *p);
extern char add_through_local_pointer(char *o);

static char buf[2] = {9, 5};

void test_main(void) {
    h_putx((unsigned)value_after_compare((char *)0x1234)); h_putnl();      /* the argument */
    h_putx((unsigned)value_after_compare((char *)100));    h_putnl();      /* 1 */
    h_puthex((unsigned char)add_through_local_pointer(buf));               /* o[1] */
    h_puthex((unsigned char)buf[0]); h_putnl();                            /* o[0] untouched */
}
