/*
 * harness-bool-stack.c - C test driver for codegen-bool-stack.ll, also calling
 * through a _Bool prototype compiled here.
 */

#include "harness-common.h"

extern unsigned char call3(unsigned char, unsigned char, unsigned char);
extern unsigned char weigh3(_Bool, _Bool, _Bool);

void test_main(void) {
    h_puthex(call3(0, 0, 0)); h_putnl();
    h_puthex(call3(1, 0, 0)); h_putnl();
    h_puthex(call3(0, 1, 0)); h_putnl();
    h_puthex(call3(0, 0, 1)); h_putnl();
    h_puthex(call3(1, 1, 1)); h_putnl();
    h_puthex(call3(200, 0, 7)); h_putnl();
    h_puthex(weigh3(0, 1, 1)); h_putnl();
    h_puthex(weigh3(1, 0, 1)); h_putnl();
}
