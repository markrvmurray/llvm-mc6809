// i64/u64 → f32/f64 conversion — MC6809 wrappers.
//
// The original mc6809/fp_conv.S wrappers for __floatdidf, __floatundidf,
// __floatdisf, __floatundisf were marked "TODO: full i64 range. For
// now uses low 32 bits" and called MC6839's FFLTD as if the input
// were i32. That truncated the high 32 bits, returning a 32-bit-
// magnitude result with the sign decided by bit 31 of the LOW word
// (since FFLTD treats input as signed i32). Surface: picolibc
// test-rand at any FP level — arc4random_double does
// `(double)bits / (double)UINT64_MAX` where bits is u64. With the
// truncation, both numerator and denominator collapse to nonsense,
// failing the mean/stddev acceptance band. Bug #264.
//
// This implementation expresses each conversion as
//   (double)hi * 2^32 + (double)lo
// using the existing __float{un,}sidf / __float{un,}sisf helpers
// (which are correct over the full u32/i32 range). For signed
// conversions the high half stays signed; the low half is always
// unsigned (since after extracting the upper bits as signed, the
// remainder is positive).
//
// Constants:
//   4294967296.0  = 2^32 as f64
//   4294967296.0f = 2^32 as f32 (exact — fits in mantissa)
//
// No idiom-recognition risk: the body never matches llvm.bswap /
// llvm.ctpop / llvm.fpext-rounded-via-bittricks — just two helper
// calls plus a single FMUL + FADD per output type.

#include <stdint.h>

extern double __floatsidf(int32_t);
extern double __floatunsidf(uint32_t);
extern float  __floatsisf(int32_t);
extern float  __floatunsisf(uint32_t);

// Only the path actually exercised by Bug #264 (picolibc test-rand
// arc4random_double) is fixed here. The signed counterpart
// (__floatdidf) and the f32 variants (__floatundisf, __floatdisf)
// still carry the "TODO: full i64 range" tech debt in
// fp_conv.S — restoring them as C wrappers tripped the Os-lto-
// hd6309-mame-fp binary over its .bss-vs-.stack budget. Tracked
// for future cleanup; not blocking Bug #264.

double __floatundidf(uint64_t u) {
    double hi = __floatunsidf((uint32_t)(u >> 32));
    double lo = __floatunsidf((uint32_t)u);
    return hi * 4294967296.0 + lo;
}
