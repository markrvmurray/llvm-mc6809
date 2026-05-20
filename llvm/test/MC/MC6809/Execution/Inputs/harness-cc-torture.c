/*
 * harness-cc-torture.c — driver for the i32/i64 calling-convention
 * torture test (calling-convention-torture.s).
 *
 * Each function under test in cc_funcs_torture.ll picks ONE field
 * out of its argument list and returns it.  The harness calls each
 * with sentinel values and prints the returned field as hex.  The
 * .s file matches each printed hex string with CHECK lines.
 *
 * Both sides of the call (harness + functions) are built by clang
 * targeting mc6809, so the ABI is consistent — bugs surface as
 * wrong-value, not as link errors.
 *
 * Sentinel values (must match cc_funcs_torture.ll's commentary):
 *   i8  : 0x5A
 *   i16 : 0xA55A
 *   i32 : 0xCAFEF00D
 *   i64 : 0xFFEEDDCCBBAA9988
 */

#include "harness-common.h"

/* Print an unsigned long long (64 bits) as 16 uppercase hex digits. */
static void h_putll(unsigned long long x) {
    h_putlong((unsigned long)(x >> 32));
    h_putlong((unsigned long)(x & 0xFFFFFFFFUL));
}

extern unsigned long  i32_identity      (unsigned long a);
extern unsigned long  i32_swap          (unsigned long a, unsigned long b);
extern unsigned long  i32_third_of_three(unsigned long a, unsigned long b, unsigned long c);
extern unsigned long  i32_mix           (unsigned char h, unsigned long x, unsigned int t);
extern unsigned int   i32_low16         (unsigned long x);
extern unsigned int   i32_high16        (unsigned long x);
extern unsigned char  i32_byte0         (unsigned long x);
extern unsigned char  i32_byte3         (unsigned long x);

extern unsigned long long i64_identity (unsigned long long a);
extern unsigned long      i64_low32    (unsigned long long x);
extern unsigned long      i64_high32   (unsigned long long x);
extern unsigned char      i64_byte0    (unsigned long long x);
extern unsigned char      i64_byte7    (unsigned long long x);
extern unsigned long long i64_swap     (unsigned long long a, unsigned long long b);
extern unsigned long long i64_sandwich (unsigned char h,
                                        unsigned long long x,
                                        unsigned char t);

void test_main(void) {
    /* ---- i32 ---- */

    /* identity(0xCAFEF00D) = 0xCAFEF00D */
    h_putlong(i32_identity(0xCAFEF00DUL));
    h_putnl();
    /* CHECK: CAFEF00D */

    /* swap returns the SECOND arg.  First is decoy 0xDEADBEEF, second
       is the sentinel — proves arg #2 reaches the callee unswapped. */
    h_putlong(i32_swap(0xDEADBEEFUL, 0xCAFEF00DUL));
    h_putnl();
    /* CHECK-NEXT: CAFEF00D */

    /* third_of_three returns arg #3 — forces stack-pass of the third
       arg (no register slot left for it after the first two). */
    h_putlong(i32_third_of_three(0xDEAD0001UL, 0xDEAD0002UL, 0xCAFEF00DUL));
    h_putnl();
    /* CHECK-NEXT: CAFEF00D */

    /* i32_mix(0x5A, 0xCAFEF00D, 0xA55A) returns the i32 sentinel.
       The i8/i32/i16 interleave forces proper width handling. */
    h_putlong(i32_mix(0x5A, 0xCAFEF00DUL, 0xA55AU));
    h_putnl();
    /* CHECK-NEXT: CAFEF00D */

    /* low16 / high16 of 0xCAFEF00D = 0xF00D / 0xCAFE */
    h_putx(i32_low16 (0xCAFEF00DUL));
    h_putnl();
    /* CHECK-NEXT: F00D */
    h_putx(i32_high16(0xCAFEF00DUL));
    h_putnl();
    /* CHECK-NEXT: CAFE */

    /* byte0 / byte3 of 0xCAFEF00D = 0x0D / 0xCA */
    h_puthex(i32_byte0(0xCAFEF00DUL));
    h_putnl();
    /* CHECK-NEXT: 0D */
    h_puthex(i32_byte3(0xCAFEF00DUL));
    h_putnl();
    /* CHECK-NEXT: CA */

    /* ---- i64 ---- */

    /* identity(0xFFEEDDCCBBAA9988) = 0xFFEEDDCCBBAA9988 */
    h_putll(i64_identity(0xFFEEDDCCBBAA9988ULL));
    h_putnl();
    /* CHECK-NEXT: FFEEDDCCBBAA9988 */

    /* low32 / high32 = 0xBBAA9988 / 0xFFEEDDCC */
    h_putlong(i64_low32 (0xFFEEDDCCBBAA9988ULL));
    h_putnl();
    /* CHECK-NEXT: BBAA9988 */
    h_putlong(i64_high32(0xFFEEDDCCBBAA9988ULL));
    h_putnl();
    /* CHECK-NEXT: FFEEDDCC */

    /* byte0 / byte7 = 0x88 / 0xFF */
    h_puthex(i64_byte0(0xFFEEDDCCBBAA9988ULL));
    h_putnl();
    /* CHECK-NEXT: 88 */
    h_puthex(i64_byte7(0xFFEEDDCCBBAA9988ULL));
    h_putnl();
    /* CHECK-NEXT: FF */

    /* swap returns arg #2.  Decoy first, sentinel second. */
    h_putll(i64_swap(0xDEADC0DEDEADC0DEULL, 0xFFEEDDCCBBAA9988ULL));
    h_putnl();
    /* CHECK-NEXT: FFEEDDCCBBAA9988 */

    /* sandwich: i8 0x5A, i64 sentinel, i8 0xA5 — returns the i64.
       Verifies the i64 packs correctly when sandwiched between i8s. */
    h_putll(i64_sandwich(0x5A, 0xFFEEDDCCBBAA9988ULL, 0xA5));
    h_putnl();
    /* CHECK-NEXT: FFEEDDCCBBAA9988 */
}
