/*
 * harness-lshr32-high.c - C test driver for codegen-lshr32-high.ll.
 */

#include "harness-common.h"

extern unsigned long by16(unsigned long);
extern unsigned long by24(unsigned long);
extern unsigned char top_byte(unsigned long);

void test_main(void) {
    h_putlong(by16(0x89ABCDEFUL)); h_putnl();
    h_putlong(by16(0x0000FFFFUL)); h_putnl();
    h_putlong(by24(0x89ABCDEFUL)); h_putnl();
    h_putlong(by24(0x00FFFFFFUL)); h_putnl();
    h_putlong(by24(0xFF000000UL)); h_putnl();
    h_puthex(top_byte(0x89ABCDEFUL)); h_putnl();
    h_puthex(top_byte(0x7F000000UL)); h_putnl();
}
