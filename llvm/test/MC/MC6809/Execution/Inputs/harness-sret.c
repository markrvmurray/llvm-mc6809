/*
 * harness-sret.c — C test driver for codegen-sret.ll. Tests
 * struct return via the sret-in-IX convention (the same
 * mechanism used for i32 return after ABI #4).
 *
 * Both compilers use sret-via-IX for any return value larger
 * than 2 bytes. The 4-byte struct Pair triggers it. The C
 * harness can call make_pair via the natural struct-return
 * form; gcc6809 emits the same IX-passing code that
 * LLVM-MC6809 expects.
 *
 * The .ll already uses the explicit `sret(%struct.Pair)`
 * attribute on the first parameter, so the make_pair function
 * declaration here matches via the natural C struct-return
 * form (gcc6809 promotes it to sret form internally).
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

struct Pair {
    unsigned int x;
    unsigned int y;
};

extern struct Pair make_pair(unsigned int x, unsigned int y);

void test_main(void) {
    struct Pair p;

    p = make_pair(0x1234, 0x5678);
    h_putx(p.x); h_putnl();  /* CHECK: 1234 */
    h_putx(p.y); h_putnl();  /* CHECK-NEXT: 5678 */

    p = make_pair(0xBEEF, 0xCAFE);
    h_putx(p.x); h_putnl();  /* CHECK-NEXT: BEEF */
    h_putx(p.y); h_putnl();  /* CHECK-NEXT: CAFE */
}
