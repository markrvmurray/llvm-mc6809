// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 no data OK
// OS9: LLVM OS-9 no data OK

#include <os9.h>

int only_bss;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (only_bss != 0)
    return WRITE_LINE("bad no data bss\r");
  only_bss = 0x1234;
  if (only_bss != 0x1234)
    return WRITE_LINE("bad no data store\r");

  return WRITE_LINE("LLVM OS-9 no data OK\r");
}
