/* Double to single conversion, __truncdfsf2.
 *
 * The MC6839 ROM converts with FMOV, and gets every case right except a
 * result below FLT_MIN.  Its FMOVE denormalizes such a result correctly
 * (UNFLNT), then runs the invalid-result check meant for results that
 * did not underflow (CKINVD).  That check knows a denormal only by the
 * exponent it has before UNFLNT's final adjustment, so it calls the
 * denormal "unnormalized", raises invalid operation 16 and returns a
 * NaN.  Arithmetic is not affected: VALID runs the check only when
 * there was no underflow.  The ROM is shipped verbatim, so the
 * conversion of a tiny value is done here instead.
 *
 * Every float below FLT_MIN is a multiple of 2^-149.  Adding a constant
 * c whose last place is 2^-149 (c = 2^-97 has 52 fraction bits below
 * it) rounds d to that grid in the current rounding mode, and
 * subtracting c again is exact.  Giving c the sign of d makes the
 * rounding act on |d|, so toward zero, upward and downward all come out
 * as they should.  The rounded value times 2^149 is then the float's
 * fraction field exactly; a value that rounds up to FLT_MIN gives 2^23,
 * which is FLT_MIN's encoding.
 */

#include <stdint.h>

extern float __mc6839_truncdfsf2(double);
extern unsigned char __mc6839_fpcb_f32[6];

#define FP_STAT_UNF 0x04
#define DOUBLE_EXP_FLT_MIN 0x381 /* biased exponent of 2^-126 */

float __truncdfsf2(double d);

float __truncdfsf2(double d) {
    unsigned char hi[2];
    unsigned exp;
    int neg;
    double r, mag;
    uint32_t bits;
    float f;

    __builtin_memcpy(hi, &d, sizeof hi); /* the sign and exponent, big-endian */
    neg = hi[0] & 0x80;
    exp = ((unsigned)(hi[0] & 0x7f) << 4) | (hi[1] >> 4);
    if (exp >= DOUBLE_EXP_FLT_MIN || d == 0)
        return __mc6839_truncdfsf2(d);

    r = neg ? (d - 0x1p-97) + 0x1p-97 : (d + 0x1p-97) - 0x1p-97;
    mag = neg ? -r : r;
    bits = (uint32_t)(int32_t)(mag * 0x1p149); /* at most 2^23 */
    if (neg)
        bits |= UINT32_C(0x80000000);
    /* The add raised inexact if the value was rounded; a tiny inexact
     * result is an underflow as well. */
    if (r != d)
        __mc6839_fpcb_f32[2] |= FP_STAT_UNF;
    __builtin_memcpy(&f, &bits, sizeof f);
    return f;
}
