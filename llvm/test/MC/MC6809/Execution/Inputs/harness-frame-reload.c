/* Harness for codegen-frame-reload.ll. */
#include "harness-common.h"

extern int n;
extern int reload_after_call(int b);
extern unsigned char reload_after_call8(unsigned char b);
extern int and_after_call(int b);
extern int post_inc(int a, int b);
extern int post_dec_sub(int a, int b);

void test_main(void) {
    n = 0; h_putx((unsigned)reload_after_call(1));   h_putnl(); /* 10 + 1 */
    n = 0; h_puthex(reload_after_call8(1));          h_putnl(); /* 10 ^ 1 */
    n = 0; h_putx((unsigned)and_after_call(0x0F));   h_putnl(); /* 10 & 15 */
    h_putx((unsigned)post_inc(1, 16));               h_putnl(); /* 16 + 1 */
    h_putx((unsigned)post_dec_sub(1, 16));           h_putnl(); /* 16 - 1 */
}
