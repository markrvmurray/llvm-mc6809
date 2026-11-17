/* Harness for codegen-shift32-bytes.ll. */
#include "harness-common.h"

typedef unsigned long (*ufn)(unsigned long);
typedef long (*sfn)(long);
extern unsigned long shl8(unsigned long), shl16(unsigned long), shl24(unsigned long);
extern unsigned long lshr8(unsigned long), lshr16(unsigned long), lshr24(unsigned long);
extern long ashr8(long), ashr16(long), ashr24(long);
extern long sext8in32(long), sext16in32(long);

static const unsigned long vals[] = {0x12345678UL, 0x87654321UL, 0xFFFFFFFFUL,
                                     0x80000000UL, 0x0000807FUL};

void test_main(void) {
    static const ufn u[] = {shl8, shl16, shl24, lshr8, lshr16, lshr24};
    static const sfn s[] = {ashr8, ashr16, ashr24, sext8in32, sext16in32};
    for (unsigned i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        for (unsigned f = 0; f < 6; f++) {
            h_putlong(u[f](vals[i]));
            h_putc(' ');
        }
        for (unsigned f = 0; f < 5; f++) {
            h_putlong((unsigned long)s[f]((long)vals[i]));
            h_putc(' ');
        }
        h_putnl();
    }
}
