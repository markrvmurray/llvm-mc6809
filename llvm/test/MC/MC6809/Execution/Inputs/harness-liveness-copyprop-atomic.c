/*
 * harness-liveness-copyprop-atomic.c — drives the Bug #370 soundness sentinel.
 *
 * Calls the two frame-/copy-prop-heavy functions in
 * liveness-copyprop-atomic.ll with fixed inputs and prints their results as
 * 4-hex-digit words. The expected values are hand-computed and must be
 * identical at every optimisation level: a regression that makes the index
 * reload elimination (Bug #363/#366) or atomic-register copy-propagation
 * (Bug #357) UNSOUND would change them.
 */
#include "harness-common.h"

extern unsigned short frame_sum(unsigned short a, unsigned short b,
                                unsigned short c);
extern unsigned short ptr_roundtrip(unsigned short *base, int n);

static unsigned short data[5] = {0x0101, 0x0202, 0x0303, 0x0404, 0x0505};

void test_main(void) {
    /* arr = {0x11,0x22,0x44,0x33,0x66,0x55}; running acc = s ^ (s<<1) -> 0x1709 */
    h_putx(frame_sum(0x0011, 0x0022, 0x0044));
    h_putnl();
    /* sum of data words = 0x0F0F */
    h_putx(ptr_roundtrip(data, 5));
    h_putnl();
}
