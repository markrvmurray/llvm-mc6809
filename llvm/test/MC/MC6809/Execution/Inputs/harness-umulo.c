/*
 * harness-umulo.c -- C test driver for codegen-umulo.s.
 * Unsigned multiply with overflow (__builtin_mul_overflow) at 8, 16 and 32
 * bits: every pair of bytes, and edge-biased pseudo-random pairs at 16 and
 * 32 bits, against a multiply twice as wide. Prints the number of operand
 * pairs checked, then the number of wrong products or overflow flags.
 */

#include "harness-common.h"

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;
typedef unsigned long long u64;

__attribute__((noinline)) int o8(u8 a, u8 b, u8 *r) { return __builtin_mul_overflow(a, b, r); }
__attribute__((noinline)) int o16(u16 a, u16 b, u16 *r) { return __builtin_mul_overflow(a, b, r); }
__attribute__((noinline)) int o32(u32 a, u32 b, u32 *r) { return __builtin_mul_overflow(a, b, r); }

static u32 seed = 12345;
static u32 rnd(void) {
    seed = seed * 1103515245UL + 12345UL;
    return seed;
}

/* Small values, powers of two, one either side of them, and plain random. */
static u32 pick(unsigned bits) {
    u32 r = rnd(), v = rnd();
    unsigned k = (unsigned)(r >> 8) % bits;
    switch ((unsigned)r & 7) {
    case 0: return v & 0xFF;
    case 1: return (u32)1 << k;
    case 2: return ((u32)1 << k) - 1;
    case 3: return ((u32)1 << k) + (v & 3);
    default: return v >> (32 - bits) >> (v & 7);
    }
}

void test_main(void) {
    u16 n = 0, bad = 0;
    unsigned a, b, i;
    for (a = 0; a < 256; a++)
        for (b = 0; b < 256; b++) {
            u8 r;
            int o = o8(a, b, &r);
            u16 p = (u16)(a * b);
            n++;
            if (r != (u8)p || o != (p > 0xFF))
                bad++;
        }
    for (i = 0; i < 6000; i++) {
        u16 x = (u16)pick(16), y = (u16)pick(16), r;
        int o = o16(x, y, &r);
        u32 p = (u32)x * y;
        n++;
        if (r != (u16)p || o != (p > 0xFFFFUL))
            bad++;
    }
    for (i = 0; i < 1500; i++) {
        u32 x = pick(32), y = pick(32), r;
        int o = o32(x, y, &r);
        u64 p = (u64)x * y;
        n++;
        if (r != (u32)p || o != (p > 0xFFFFFFFFULL))
            bad++;
    }
    h_putx(n);  /* 65536 + 6000 + 1500 = 73036, modulo 2^16 = $1D4C */
    h_putnl();
    h_putx(bad);
    h_putnl();
}
