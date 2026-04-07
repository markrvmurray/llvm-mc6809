/*
 * harness-ptr.c — C test driver for codegen-ptr.ll. Tests pointer
 * dereference, i16/i8 array indexing, struct-like field access.
 * No bridge wrappers needed: all signatures are pointer + i16
 * combinations that gcc6809 and LLVM-MC6809 agree on.
 *
 * Compiled with gcc6809, linked with the LLVM-compiled functions
 * under test.
 */

#include "harness-common.h"

extern unsigned int  array_get(const unsigned int *a, unsigned int i);
extern void          array_set(unsigned int *a, unsigned int i, unsigned int val);
extern unsigned int  deref(const unsigned int *p);
extern void          deref_write(unsigned int *p, unsigned int val);
extern unsigned char byte_get(const unsigned char *a, unsigned int i);
extern unsigned int  struct_field(const void *p);

/* Test data — placed in .rom by gcc6809 (the data is const). */
static const unsigned int  arr_i16[5] = { 0x1111, 0x2222, 0x3333, 0x4444, 0x5555 };
static const unsigned char arr_i8[5]  = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };

/* Struct-like data: 3×i16, accessed as a 6-byte block. */
static const unsigned int struct_data[3] = { 0x0001, 0x0002, 0xBEEF };

/* Writable buffer */
static unsigned int buf[5];

void test_main(void) {
    /* ===== Pointer dereference ===== */
    h_putx(deref(&arr_i16[0])); h_putnl();   /* CHECK: 1111 */

    /* ===== Array indexing (i16) ===== */
    h_putx(array_get(arr_i16, 0)); h_putnl();   /* CHECK-NEXT: 1111 */
    h_putx(array_get(arr_i16, 2)); h_putnl();   /* CHECK-NEXT: 3333 */
    h_putx(array_get(arr_i16, 4)); h_putnl();   /* CHECK-NEXT: 5555 */

    /* ===== Byte array indexing (i8) ===== */
    h_puthex(byte_get(arr_i8, 0)); h_putnl();   /* CHECK-NEXT: AA */
    h_puthex(byte_get(arr_i8, 3)); h_putnl();   /* CHECK-NEXT: DD */

    /* ===== Pointer write + read back ===== */
    deref_write(&buf[0], 0xCAFEu);
    h_putx(deref(&buf[0])); h_putnl();   /* CHECK-NEXT: CAFE */

    /* ===== Array store + read back ===== */
    array_set(buf, 1, 0xDEADu);
    h_putx(array_get(buf, 1)); h_putnl();   /* CHECK-NEXT: DEAD */

    /* ===== Struct field access ===== */
    h_putx(struct_field(struct_data)); h_putnl();   /* CHECK-NEXT: BEEF */
}
