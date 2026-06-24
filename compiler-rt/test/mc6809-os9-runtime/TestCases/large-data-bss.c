// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 large data bss OK
// OS9: LLVM OS-9 large data bss OK

#include <os9.h>

int data_start = 0x1111;
int data_filler[148] = {[147] = 0x2222};
int data_end = 0x3333;

int bss_start;
int bss_filler[148];
int bss_end;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (data_start != 0x1111)
    return WRITE_LINE("bad large data start\r");
  if (data_end != 0x3333)
    return WRITE_LINE("bad large data end\r");
  if (bss_start != 0 || bss_end != 0)
    return WRITE_LINE("bad large bss zero\r");

  data_start = 0x4444;
  data_end = 0x5555;
  bss_start = 0x6666;
  bss_end = 0x7777;

  if (data_start != 0x4444 || data_end != 0x5555)
    return WRITE_LINE("bad large data store\r");
  if (bss_start != 0x6666 || bss_end != 0x7777)
    return WRITE_LINE("bad large bss store\r");

  return WRITE_LINE("LLVM OS-9 large data bss OK\r");
}
