/*
 * harness-mulh.c - C test driver for codegen-mulh.ll. Prints the high half of
 * unsigned and signed multiplies at each width; the expected values in
 * codegen-mulh.s are computed independently of the compiler.
 */

#include "harness-common.h"

extern unsigned char umulh8(unsigned char, unsigned char);
extern signed char smulh8(signed char, signed char);
extern unsigned int umulh16(unsigned int, unsigned int);
extern int smulh16(int, int);
extern unsigned long umulh32(unsigned long, unsigned long);
extern long smulh32(long, long);

void test_main(void) {
    h_puthex(umulh8((unsigned char)0x0, (unsigned char)0xFF)); h_putnl();
    h_puthex(umulh8((unsigned char)0x1, (unsigned char)0x1)); h_putnl();
    h_puthex(umulh8((unsigned char)0x12, (unsigned char)0x34)); h_putnl();
    h_puthex(umulh8((unsigned char)0x7F, (unsigned char)0x80)); h_putnl();
    h_puthex(umulh8((unsigned char)0x80, (unsigned char)0x80)); h_putnl();
    h_puthex(umulh8((unsigned char)0xFF, (unsigned char)0x1)); h_putnl();
    h_puthex(umulh8((unsigned char)0xFF, (unsigned char)0xFF)); h_putnl();
    h_puthex(umulh8((unsigned char)0xFE, (unsigned char)0x7F)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0x0, (signed char)0xFF)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0x1, (signed char)0x1)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0x12, (signed char)0x34)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0x7F, (signed char)0x80)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0x80, (signed char)0x80)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0xFF, (signed char)0x1)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0xFF, (signed char)0xFF)); h_putnl();
    h_puthex((unsigned char)smulh8((signed char)0xFE, (signed char)0x7F)); h_putnl();
    h_putx(umulh16((unsigned int)0x0u, (unsigned int)0xFFFFu)); h_putnl();
    h_putx(umulh16((unsigned int)0x1u, (unsigned int)0x1u)); h_putnl();
    h_putx(umulh16((unsigned int)0x1234u, (unsigned int)0x5678u)); h_putnl();
    h_putx(umulh16((unsigned int)0x7FFFu, (unsigned int)0x8000u)); h_putnl();
    h_putx(umulh16((unsigned int)0x8000u, (unsigned int)0x8000u)); h_putnl();
    h_putx(umulh16((unsigned int)0xFFFFu, (unsigned int)0x1u)); h_putnl();
    h_putx(umulh16((unsigned int)0xFFFFu, (unsigned int)0xFFFFu)); h_putnl();
    h_putx(umulh16((unsigned int)0x100u, (unsigned int)0x100u)); h_putnl();
    h_putx(umulh16((unsigned int)0xFF00u, (unsigned int)0xFFu)); h_putnl();
    h_putx((unsigned int)smulh16((int)0x0u, (int)0xFFFFu)); h_putnl();
    h_putx((unsigned int)smulh16((int)0x1u, (int)0x1u)); h_putnl();
    h_putx((unsigned int)smulh16((int)0x1234u, (int)0x5678u)); h_putnl();
    h_putx((unsigned int)smulh16((int)0x7FFFu, (int)0x8000u)); h_putnl();
    h_putx((unsigned int)smulh16((int)0x8000u, (int)0x8000u)); h_putnl();
    h_putx((unsigned int)smulh16((int)0xFFFFu, (int)0x1u)); h_putnl();
    h_putx((unsigned int)smulh16((int)0xFFFFu, (int)0xFFFFu)); h_putnl();
    h_putx((unsigned int)smulh16((int)0x100u, (int)0x100u)); h_putnl();
    h_putx((unsigned int)smulh16((int)0xFF00u, (int)0xFFu)); h_putnl();
    h_putlong(umulh32((unsigned long)0x0ul, (unsigned long)0xFFFFFFFFul)); h_putnl();
    h_putlong(umulh32((unsigned long)0x1ul, (unsigned long)0x1ul)); h_putnl();
    h_putlong(umulh32((unsigned long)0x12345678ul, (unsigned long)0x9ABCDEF0ul)); h_putnl();
    h_putlong(umulh32((unsigned long)0x7FFFFFFFul, (unsigned long)0x80000000ul)); h_putnl();
    h_putlong(umulh32((unsigned long)0x80000000ul, (unsigned long)0x80000000ul)); h_putnl();
    h_putlong(umulh32((unsigned long)0xFFFFFFFFul, (unsigned long)0x1ul)); h_putnl();
    h_putlong(umulh32((unsigned long)0xFFFFFFFFul, (unsigned long)0xFFFFFFFFul)); h_putnl();
    h_putlong(umulh32((unsigned long)0x10000ul, (unsigned long)0x10000ul)); h_putnl();
    h_putlong(umulh32((unsigned long)0xFFFF0000ul, (unsigned long)0xFFFFul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0x0ul, (long)0xFFFFFFFFul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0x1ul, (long)0x1ul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0x12345678ul, (long)0x9ABCDEF0ul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0x7FFFFFFFul, (long)0x80000000ul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0x80000000ul, (long)0x80000000ul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0xFFFFFFFFul, (long)0x1ul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0xFFFFFFFFul, (long)0xFFFFFFFFul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0x10000ul, (long)0x10000ul)); h_putnl();
    h_putlong((unsigned long)smulh32((long)0xFFFF0000ul, (long)0xFFFFul)); h_putnl();
}
