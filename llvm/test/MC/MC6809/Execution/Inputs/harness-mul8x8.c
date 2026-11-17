/* Harness for codegen-mul8x8.ll. */
#include "harness-common.h"

extern unsigned umul8(unsigned char a, unsigned char b);
extern unsigned umul8_200(unsigned char a);
extern unsigned umul8_load(const unsigned char *p, const unsigned char *q);

static const unsigned char vals[] = {0, 1, 2, 3, 15, 16, 127, 128, 129, 200, 254, 255};

void test_main(void) {
    for (unsigned i = 0; i < sizeof(vals); i++) {
        for (unsigned j = 0; j < sizeof(vals); j++)
            h_putx(umul8(vals[i], vals[j]));
        h_putnl();
    }
    for (unsigned i = 0; i < sizeof(vals); i++)
        h_putx(umul8_200(vals[i]));
    h_putnl();
    for (unsigned i = 0; i < sizeof(vals); i++)
        h_putx(umul8_load(&vals[i], &vals[sizeof(vals) - 1 - i]));
    h_putnl();
}
