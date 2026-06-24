// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 exit status OK
// OS9: LLVM OS-9 exit status OK

#include <os9.h>

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (WRITE_LINE("LLVM OS-9 exit status OK\r") != 0)
    return 1;
  return 0;
}
