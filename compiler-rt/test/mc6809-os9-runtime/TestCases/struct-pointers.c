// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 struct pointers OK
// OS9: LLVM OS-9 struct pointers OK

#include <os9.h>

struct Record {
  int tag;
  int *value;
  const char *name;
};

int global_value = 0x1357;
const char record_name[] = "LLVM OS-9 struct pointers OK\r";
struct Record record = {7, &global_value, record_name};

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  int n = sizeof(record_name) - 1;

  if (record.tag != 7)
    return WRITE_LINE("bad struct tag\r");
  if (*record.value != 0x1357)
    return WRITE_LINE("bad struct data pointer\r");

  *record.value = 0x2468;
  if (global_value != 0x2468)
    return WRITE_LINE("bad struct store\r");

  return _os_write(1, record.name, &n);
}
