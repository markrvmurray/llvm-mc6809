// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 write OK

#include <os9.h>

int main(void) {
    int n = 19;
    return _os_write(1, "LLVM OS-9 write OK\r", &n);
}
