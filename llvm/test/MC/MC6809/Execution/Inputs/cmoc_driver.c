// CMOC driver: calls LLVM-compiled functions via __gcccall convention.
// Uses external puthex16/putnl for output (provided by test harness).
#include <cmoc.h>

int __gcccall llvm_sum(int a, int b);
int __gcccall llvm_mul(int a, int b);

// Provided by test harness
void __gcccall puthex16(unsigned int w);
void __gcccall putnl(void);
void __gcccall halt(void);

int main() {
    puthex16(llvm_sum(3, 4));
    putnl();

    puthex16(llvm_sum(0x1234, 0x5678));
    putnl();

    puthex16(llvm_mul(6, 7));
    putnl();

    halt();
    return 0;
}
