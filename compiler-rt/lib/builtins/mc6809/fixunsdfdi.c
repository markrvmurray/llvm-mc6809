/* Conversion of a double to unsigned 64-bit integer, through the signed
 * conversion (__fixdfdi in floatdi.S); see fixunsdfsi.c.
 */

#include <stdint.h>

uint64_t __fixunsdfdi(double a);

uint64_t __fixunsdfdi(double a) {
    if (!(a > -1.0))
        return 0;
    if (a < 0x1p63)
        return (uint64_t)(int64_t)a;
    if (a >= 0x1p64)
        return UINT64_MAX;
    return (uint64_t)(int64_t)(a - 0x1p63) + UINT64_C(0x8000000000000000);
}
