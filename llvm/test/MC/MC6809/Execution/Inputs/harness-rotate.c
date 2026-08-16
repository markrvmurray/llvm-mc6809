/*
 * harness-rotate.c — driver for codegen-rotate.ll. Tests 8-bit rotates.
 *
 * The 6809 has no 8-bit rotate; ROL/ROR rotate through the carry, so the
 * backend has to put the wrap-around bit into C first. The instruction
 * that does that has a dead byte result and used to be deleted, leaving a
 * bare ROL/ROR whose result depended on whatever the carry held. Each
 * case below is chosen so the wrong-carry answer differs from the right
 * one, and the values are set up so the carry flag left by the preceding
 * call is unlikely to happen to be correct.
 */

#include "harness-common.h"

extern unsigned char ror1(unsigned char x);
extern unsigned char rol1(unsigned char x);
extern unsigned char rol3(unsigned char x);
extern unsigned char ror3(unsigned char x);
extern unsigned char classify(unsigned char c);
extern unsigned int  rol16(unsigned int x);
extern unsigned int  ror16(unsigned int x);

void test_main(void) {
    /* ror1: bit 0 must arrive at bit 7 */
    h_puthex(ror1(0x01));      /* 0x80 */
    h_putnl();
    /* CHECK: 80 */
    h_puthex(ror1(0x02));      /* 0x01 — bit 0 clear, top must be clear */
    h_putnl();
    /* CHECK-NEXT: 01 */
    h_puthex(ror1(0xFF));      /* 0xFF */
    h_putnl();
    /* CHECK-NEXT: FF */
    h_puthex(ror1(0x2E));      /* '.' - 42 = 4 -> 2 is the vfprintf case; here 0x2E -> 0x17 */
    h_putnl();
    /* CHECK-NEXT: 17 */

    /* rol1: bit 7 must arrive at bit 0 */
    h_puthex(rol1(0x80));      /* 0x01 */
    h_putnl();
    /* CHECK-NEXT: 01 */
    h_puthex(rol1(0x40));      /* 0x80 — bit 7 clear, bit 0 must be clear */
    h_putnl();
    /* CHECK-NEXT: 80 */
    h_puthex(rol1(0xC3));      /* 0x87 */
    h_putnl();
    /* CHECK-NEXT: 87 */

    /* rol3 / ror3: multi-step chains, each step a real rotate */
    h_puthex(rol3(0x81));      /* 0x0C */
    h_putnl();
    /* CHECK-NEXT: 0C */
    h_puthex(ror3(0x81));      /* 0x30 */
    h_putnl();
    /* CHECK-NEXT: 30 */
    h_puthex(rol3(0xE0));      /* 0x07 */
    h_putnl();
    /* CHECK-NEXT: 07 */

    /* classify: the vfprintf size-modifier dispatch */
    h_puthex(classify('*'));   /* 1 */
    h_putnl();
    /* CHECK-NEXT: 01 */
    h_puthex(classify('.'));   /* 2 — this is the one that fell to default */
    h_putnl();
    /* CHECK-NEXT: 02 */
    h_puthex(classify('l'));   /* 3 */
    h_putnl();
    /* CHECK-NEXT: 03 */
    h_puthex(classify('h'));   /* 4 */
    h_putnl();
    /* CHECK-NEXT: 04 */
    h_puthex(classify('d'));   /* 0 — an odd offset must miss every case */
    h_putnl();
    /* CHECK-NEXT: 00 */
    h_puthex(classify('+'));   /* 0 — 43-42 = 1, odd */
    h_putnl();
    /* CHECK-NEXT: 00 */

    /* 16-bit: bit 15 must arrive at bit 0, and bit 0 at bit 15 */
    h_putx(rol16(0x8001));     /* 0x0003 */
    h_putnl();
    /* CHECK-NEXT: 0003 */
    h_putx(rol16(0x4000));     /* 0x8000 — bit 15 clear, bit 0 must be clear */
    h_putnl();
    /* CHECK-NEXT: 8000 */
    h_putx(ror16(0x8001));     /* 0xC000 */
    h_putnl();
    /* CHECK-NEXT: C000 */
    h_putx(ror16(0x0002));     /* 0x0001 — bit 0 clear, bit 15 must be clear */
    h_putnl();
    /* CHECK-NEXT: 0001 */
}
