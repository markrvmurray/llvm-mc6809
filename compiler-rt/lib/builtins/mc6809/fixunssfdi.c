/* Conversion of a float to unsigned 64-bit integer, through the signed
 * conversion (__fixsfdi in floatdi.S); see fixunsdfsi.c.
 */

#include <stdint.h>

uint64_t __fixunssfdi(float a);

uint64_t __fixunssfdi(float a) {
    if (!(a > -1.0f))
        return 0;
    if (a < 0x1p63f)
        return (uint64_t)(int64_t)a;
    if (a >= 0x1p64f)
        return UINT64_MAX;
    return (uint64_t)(int64_t)(a - 0x1p63f) + UINT64_C(0x8000000000000000);
}
