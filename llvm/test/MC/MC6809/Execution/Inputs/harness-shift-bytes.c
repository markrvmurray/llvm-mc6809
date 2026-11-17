/* Harness for codegen-shift-bytes.ll. */
#include "harness-common.h"

extern unsigned shl8(unsigned), shl11(unsigned), lshr8(unsigned), lshr13(unsigned);
extern int ashr8(int), ashr12(int), ashr15(int), sdiv4(int), sdiv256(int);
extern unsigned bext(unsigned char);

static const int vals[] = {0, 1, 3, -1, -3, 255, -256, 0x1234, -0x1234, 32767, -32768, -4, -5};

void test_main(void) {
    for (unsigned i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        int v = vals[i];
        h_putx(shl8(v)); h_putx(shl11(v)); h_putx(lshr8(v)); h_putx(lshr13(v));
        h_putx(ashr8(v)); h_putx(ashr12(v)); h_putx(ashr15(v));
        h_putx(sdiv4(v)); h_putx(sdiv256(v)); h_putx(bext((unsigned char)v));
        h_putnl();
    }
}
