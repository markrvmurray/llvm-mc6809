/*
 * harness-globals.c — C test driver for codegen-globals.ll. Tests
 * global variable access (load / store / increment) for i8, i16
 * and i32 globals.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int  get_counter  (void);
extern void          set_counter  (unsigned int v);
extern void          inc_counter  (void);
extern unsigned char get_value    (void);

extern unsigned long get_counter32(void);
extern void          set_counter32(unsigned long v);
extern void          inc_counter32(void);

void test_main(void) {
    /* get_counter() initially = 0 */
    h_putx(get_counter()); h_putnl();
    /* CHECK: 0000 */

    /* set_counter(0x1234) → get_counter() = 0x1234 */
    set_counter(0x1234);
    h_putx(get_counter()); h_putnl();
    /* CHECK-NEXT: 1234 */

    /* inc_counter() → 0x1235 */
    inc_counter();
    h_putx(get_counter()); h_putnl();
    /* CHECK-NEXT: 1235 */

    /* inc_counter() twice → 0x1237 */
    inc_counter();
    inc_counter();
    h_putx(get_counter()); h_putnl();
    /* CHECK-NEXT: 1237 */

    /* get_value() = 0 (uninitialized .bss global) */
    h_puthex(get_value()); h_putnl();
    /* CHECK-NEXT: 00 */

    /* ===== i32 globals — exercises ABI #3 (i32 arg) and ABI #4 (i32 sret return) ===== */
    /* counter32 starts at 0; inc 3 times → 3 */
    inc_counter32();
    inc_counter32();
    inc_counter32();
    h_putlong(get_counter32()); h_putnl();
    /* CHECK-NEXT: 00000003 */

    /* set_counter32(0xCAFEBABE) → 0xCAFEBABE */
    set_counter32(0xCAFEBABEUL);
    h_putlong(get_counter32()); h_putnl();
    /* CHECK-NEXT: CAFEBABE */
}
