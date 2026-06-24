// Demonstrates the current LLVM/OS-9 writable-global bug.
//
// Run this module twice from the same booted OS-9 disk:
//
//   globaldatabug >>>-globaldatabug.1.out
//   globaldatabug >>>-globaldatabug.2.out
//
// Correct OS-9 behavior:
//   each process gets fresh data storage, so both runs should start with
//   initialized_value == 0x1234 and uninitialized_value == 0x0000.
//
// Current LLVM behavior:
//   writable globals are addressed PC-relative, so stores modify the
//   reentrant module image instead of the process data area. The second run
//   can observe values written by the first run, or corrupt module-adjacent
//   bytes for .bss.

#include <os9.h>

int initialized_value = 0x1234;
int uninitialized_value;

static void write_all(const char *s, int n) {
    int count = n;
    _os_write(1, s, &count);
}

static void write_cstr(const char *s) {
    int n = 0;
    while (s[n])
        ++n;
    write_all(s, n);
}

static void write_hex16(unsigned v) {
    char out[4];
    static const char digits[] = "0123456789ABCDEF";
    out[0] = digits[(v >> 12) & 15];
    out[1] = digits[(v >> 8) & 15];
    out[2] = digits[(v >> 4) & 15];
    out[3] = digits[v & 15];
    write_all(out, 4);
}

int main(void) {
    write_cstr("start init=");
    write_hex16((unsigned)initialized_value);
    write_cstr(" bss=");
    write_hex16((unsigned)uninitialized_value);
    write_cstr("\r");

    initialized_value = 0x5678;
    uninitialized_value = 0x9ABC;

    write_cstr("after init=");
    write_hex16((unsigned)initialized_value);
    write_cstr(" bss=");
    write_hex16((unsigned)uninitialized_value);
    write_cstr("\r");

    return 0;
}
