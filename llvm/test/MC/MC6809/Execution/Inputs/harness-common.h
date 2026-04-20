/*
 * harness-common.h — shared output helpers for the C test harnesses.
 *
 * Each harness-*.c file in this directory drives a set of LLVM-MC6809
 * functions and prints results to the ACIA in a fixed hex format that
 * the corresponding codegen-*.s test file matches via FileCheck. The
 * I/O helpers are identical across all harnesses, so they live here.
 *
 * Usage: every harness-*.c starts with
 *
 *     #include "harness-common.h"
 *
 * and then declares its own externs and test_main(). Compiled by
 * gcc6809 (m6809-unknown-gcc) which uses the same `__gcccall`
 * convention LLVM-MC6809 emits, so the LLVM-compiled functions under
 * test link directly against the harness with no ABI shim.
 *
 * Memory map (matches Inputs/runtime.inc):
 *   0xC000  ACIA status register (TDRE bit = 0x02)
 *   0xC001  ACIA data register
 *   0xC002  halt/exit device
 */

#ifndef HARNESS_COMMON_H
#define HARNESS_COMMON_H

#define ACIA_STATUS (*(volatile unsigned char *)0xC000)
#define ACIA_DATA   (*(volatile unsigned char *)0xC001)

/* Spin until the ACIA is ready, then write one byte. */
static void h_putc(unsigned char c) {
    while (!(ACIA_STATUS & 0x02))
        ;
    ACIA_DATA = c;
}

/* Print a single hex digit (0-15) as ASCII '0'-'9' or 'A'-'F'. */
static void h_putnibble(unsigned char n) {
    h_putc(n < 10 ? '0' + n : 'A' + n - 10);
}

/* Print one byte as 2 uppercase hex digits. */
static void h_puthex(unsigned char b) {
    h_putnibble(b >> 4);
    h_putnibble(b & 0x0F);
}

/* Print a 16-bit word as 4 uppercase hex digits (big-endian). */
static void h_putx(unsigned int x) {
    h_puthex((unsigned char)(x >> 8));
    h_puthex((unsigned char)(x & 0xFF));
}

/* Print a 32-bit long as 8 uppercase hex digits (big-endian). */
static void h_putlong(unsigned long l) {
    h_putx((unsigned int)(l >> 16));
    h_putx((unsigned int)(l & 0xFFFFu));
}

/* Print a newline (LF). */
static void h_putnl(void) {
    h_putc('\n');
}

#endif /* HARNESS_COMMON_H */
