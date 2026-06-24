// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 pointer array OK
// OS9: LLVM OS-9 pointer array OK

#include <os9.h>

int first = 0x1111;
int second = 0x2222;
int zeroed;
int *ptrs[] = {&first, &second, &zeroed};

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (*ptrs[0] != 0x1111)
    return WRITE_LINE("bad pointer array first\r");
  if (*ptrs[1] != 0x2222)
    return WRITE_LINE("bad pointer array second\r");
  if (*ptrs[2] != 0)
    return WRITE_LINE("bad pointer array zero\r");

  *ptrs[0] = 0x3333;
  *ptrs[1] = 0x4444;
  *ptrs[2] = 0x5555;

  if (first != 0x3333)
    return WRITE_LINE("bad pointer array first store\r");
  if (second != 0x4444)
    return WRITE_LINE("bad pointer array second store\r");
  if (zeroed != 0x5555)
    return WRITE_LINE("bad pointer array zero store\r");

  return WRITE_LINE("LLVM OS-9 pointer array OK\r");
}
