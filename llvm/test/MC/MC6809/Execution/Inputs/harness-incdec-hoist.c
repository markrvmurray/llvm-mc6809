/* Harness for codegen-incdec-hoist.ll. */
#include "harness-common.h"

extern char cell;
extern void bump_after_call(char *p, char c);
extern char read_then_bump(char *p, char c);

void test_main(void) {
    cell = 0; bump_after_call(&cell, 1); h_puthex((unsigned char)cell); h_putnl();  /* 100 + 1 */
    cell = 0; bump_after_call(&cell, 0); h_puthex((unsigned char)cell); h_putnl();  /* 100 - 1 */
    cell = 5; h_puthex((unsigned char)read_then_bump(&cell, 1));                    /* 5 */
    h_puthex((unsigned char)cell); h_putnl();                                       /* then 6 */
}
