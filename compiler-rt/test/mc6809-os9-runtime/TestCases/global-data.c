// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 global data OK

#include <os9.h>

int initialized_value = 0x1234;
int uninitialized_value;
const int ro_value = 0x726f;

int *initialized_ptr = &initialized_value;
int *uninitialized_ptr = &uninitialized_value;
const int *ro_ptr = &ro_value;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (initialized_value != 0x1234)
    return WRITE_LINE("bad initialized global\r");
  if (uninitialized_value != 0)
    return WRITE_LINE("bad zero global\r");
  if (*initialized_ptr != 0x1234)
    return WRITE_LINE("bad initialized pointer\r");
  if (*uninitialized_ptr != 0)
    return WRITE_LINE("bad zero pointer\r");
  if (*ro_ptr != 0x726f)
    return WRITE_LINE("bad ro pointer\r");

  initialized_value = 0x5678;
  uninitialized_value = 0x1234;
  *initialized_ptr = 0x2222;
  *uninitialized_ptr = 0x3333;

  if (initialized_value != 0x2222)
    return WRITE_LINE("bad initialized store\r");
  if (uninitialized_value != 0x3333)
    return WRITE_LINE("bad zero store\r");

  return WRITE_LINE("LLVM OS-9 global data OK\r");
}
