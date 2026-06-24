// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 function pointer OK
// OS9: LLVM OS-9 function pointer OK

#include <os9.h>

static int add_five(int value) {
  return value + 5;
}

int (*callback)(int) = add_five;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (callback(7) != 12)
    return WRITE_LINE("bad function pointer call\r");

  return WRITE_LINE("LLVM OS-9 function pointer OK\r");
}
