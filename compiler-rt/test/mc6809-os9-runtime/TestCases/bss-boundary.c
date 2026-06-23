// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 bss boundary OK
// OS9: LLVM OS-9 bss boundary OK

#include <os9.h>

int before_bss;
int target_bss;
int after_bss;
int *target_ptr = &target_bss;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (before_bss != 0 || target_bss != 0 || after_bss != 0)
    return WRITE_LINE("bad bss initial state\r");

  *target_ptr = 0x7777;
  if (before_bss != 0)
    return WRITE_LINE("bad bss before guard\r");
  if (target_bss != 0x7777)
    return WRITE_LINE("bad bss target store\r");
  if (after_bss != 0)
    return WRITE_LINE("bad bss after guard\r");

  return WRITE_LINE("LLVM OS-9 bss boundary OK\r");
}
