// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 string pointer OK
// OS9: LLVM OS-9 string pointer OK

#include <os9.h>

const char message[] = "LLVM OS-9 string pointer OK\r";
const char *message_ptr = message;

int main(void) {
  int n = sizeof(message) - 1;
  return _os_write(1, message_ptr, &n);
}
