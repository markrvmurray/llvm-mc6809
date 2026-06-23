// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 scalar data OK
// OS9: LLVM OS-9 scalar data OK

#include <os9.h>

int first = 0x1234;
int second = 0x5678;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (first != 0x1234)
    return WRITE_LINE("bad scalar first\r");
  if (second != 0x5678)
    return WRITE_LINE("bad scalar second\r");

  first = 0x2222;
  second = 0x3333;
  if (first != 0x2222 || second != 0x3333)
    return WRITE_LINE("bad scalar store\r");

  return WRITE_LINE("LLVM OS-9 scalar data OK\r");
}
