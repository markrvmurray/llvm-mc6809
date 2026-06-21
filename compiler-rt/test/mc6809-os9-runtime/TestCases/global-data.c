// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 global data OK

#include <os9.h>

int initialized_value = 0x1234;
int uninitialized_value;

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (initialized_value != 0x1234)
    return write_line("bad initialized global\r", 24);
  if (uninitialized_value != 0)
    return write_line("bad zero global\r", 16);

  initialized_value = 0x5678;
  uninitialized_value = 0x1234;

  if (initialized_value != 0x5678)
    return write_line("bad initialized store\r", 23);
  if (uninitialized_value != 0x1234)
    return write_line("bad zero store\r", 16);

  return write_line("LLVM OS-9 global data OK\r", 25);
}
