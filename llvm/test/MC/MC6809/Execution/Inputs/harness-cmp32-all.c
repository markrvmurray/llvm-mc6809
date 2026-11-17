/* Harness for codegen-cmp32-all.ll. */
#include "harness-common.h"

extern unsigned char cmp_all(long a, long b);
extern unsigned char cmp_ug(unsigned long a, unsigned long b);

static const long vals[] = {0L, 1L, -1L, 0x7FFFFFFFL, (long)0x80000000UL,
                            0x00010000L, 0x0000FFFFL, -0x10000L, 0x12345678L};

void test_main(void) {
    for (unsigned i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        for (unsigned j = 0; j < sizeof(vals) / sizeof(vals[0]); j++) {
            h_puthex(cmp_all(vals[i], vals[j]));
            h_puthex(cmp_ug((unsigned long)vals[i], (unsigned long)vals[j]));
        }
        h_putnl();
    }
}
