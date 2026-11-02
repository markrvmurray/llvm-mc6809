/*
 * harness-clz16.c - C test driver for codegen-clz16.ll. Prints leading-zero
 * counts for inputs either side of each byte boundary.
 */

#include "harness-common.h"

extern unsigned int clz16(unsigned int);
extern unsigned int clz16_nz(unsigned int);
extern unsigned char clz8(unsigned char);

void test_main(void) {
    h_puthex(clz16(0x0000)); h_putnl();
    h_puthex(clz16(0x0001)); h_putnl();
    h_puthex(clz16(0x0080)); h_putnl();
    h_puthex(clz16(0x00FF)); h_putnl();
    h_puthex(clz16(0x0100)); h_putnl();
    h_puthex(clz16(0x01FF)); h_putnl();
    h_puthex(clz16(0x4000)); h_putnl();
    h_puthex(clz16(0x8000)); h_putnl();
    h_puthex(clz16(0xFFFF)); h_putnl();
    h_puthex(clz16_nz(0x0040)); h_putnl();
    h_puthex(clz16_nz(0x0200)); h_putnl();
    h_puthex(clz8(0x00)); h_putnl();
    h_puthex(clz8(0x01)); h_putnl();
    h_puthex(clz8(0x10)); h_putnl();
    h_puthex(clz8(0x80)); h_putnl();
    /* Through the C builtin, whose argument is a 16-bit int. */
    h_puthex(__builtin_clz(0x0300)); h_putnl();
}
