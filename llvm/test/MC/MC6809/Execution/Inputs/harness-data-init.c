/*
 * harness-data-init.c — C test driver for codegen-data-init.ll.
 * Tests initialized .data globals (i16 counter starting at 42,
 * i8 flag starting at 1) with the implicit ROM→RAM copy at startup.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int  get_counter(void);
extern unsigned char get_flag   (void);
extern void          inc_counter(void);

void test_main(void) {
    /* counter initially = 42 = 0x002A (from .data init) */
    h_putx(get_counter()); h_putnl();
    /* CHECK: 002A */

    /* flag = 1 (from .data init) */
    h_puthex(get_flag()); h_putnl();
    /* CHECK-NEXT: 01 */

    /* inc_counter() then get_counter() = 43 = 0x002B */
    inc_counter();
    h_putx(get_counter()); h_putnl();
    /* CHECK-NEXT: 002B */

    /* inc_counter() twice more → 45 = 0x002D */
    inc_counter();
    inc_counter();
    h_putx(get_counter()); h_putnl();
    /* CHECK-NEXT: 002D */
}
