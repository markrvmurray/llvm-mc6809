/* Conversion of a double to unsigned 32-bit integer, through the signed
 * conversion (MC6839 FFIXD, __fixdfsi).
 *
 * A value below 2^(N-1) converts as a signed one would.  One in
 * [2^(N-1), 2^N) has 2^(N-1) taken off first -- exactly, as the two are
 * within a factor of two of each other -- and put back as an integer.  As
 * in the generic C versions, a value of -1 or less gives 0 and one of 2^N
 * or more gives the largest unsigned value; C leaves both undefined.  A
 * negative value above -1 truncates to 0, which C does define.
 *
 * fixunssfsi.c, fixunsdfdi.c and fixunssfdi.c are the same for the other
 * widths, one function a file so that a program pulls in only the one it
 * uses.
 */

#include <stdint.h>

uint32_t __fixunsdfsi(double a);

uint32_t __fixunsdfsi(double a) {
    if (!(a > -1.0))
        return 0;
    if (a < 0x1p31)
        return (uint32_t)(int32_t)a;
    if (a >= 0x1p32)
        return UINT32_MAX;
    return (uint32_t)(int32_t)(a - 0x1p31) + UINT32_C(0x80000000);
}
