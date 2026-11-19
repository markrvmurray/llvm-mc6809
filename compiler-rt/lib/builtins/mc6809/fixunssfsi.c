/* Conversion of a float to unsigned 32-bit integer, through the signed
 * conversion (MC6839 FFIXD, __fixsfsi); see fixunsdfsi.c.
 */

#include <stdint.h>

uint32_t __fixunssfsi(float a);

uint32_t __fixunssfsi(float a) {
    if (!(a > -1.0f))
        return 0;
    if (a < 0x1p31f)
        return (uint32_t)(int32_t)a;
    if (a >= 0x1p32f)
        return UINT32_MAX;
    return (uint32_t)(int32_t)(a - 0x1p31f) + UINT32_C(0x80000000);
}
