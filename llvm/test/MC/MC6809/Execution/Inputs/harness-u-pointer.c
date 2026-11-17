/* Harness for codegen-u-pointer.ll. */
#include "harness-common.h"

extern int walk(int n, int *out);

void sink(int *lo, int *hi) { (void)lo; (void)hi; }

void test_main(void) {
    int out[6];
    for (int n = 1; n <= 6; n++) {
        h_puthex((unsigned char)walk(n, out));
        for (int i = 0; i < n; i++)
            h_puthex((unsigned char)out[i]);
        h_putnl();
    }
}
