// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 no bss OK
// OS9: LLVM OS-9 no bss OK

#include <os9.h>

int first_value = 0x1111;
int second_value = 0x2222;
int *first_ptr = &first_value;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (first_value != 0x1111 || second_value != 0x2222)
    return WRITE_LINE("bad no bss data\r");
  if (*first_ptr != 0x1111)
    return WRITE_LINE("bad no bss pointer\r");

  *first_ptr = 0x3333;
  if (first_value != 0x3333)
    return WRITE_LINE("bad no bss store\r");

  return WRITE_LINE("LLVM OS-9 no bss OK\r");
}
