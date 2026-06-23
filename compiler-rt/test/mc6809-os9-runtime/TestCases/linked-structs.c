// REQUIRES: mc6809-os9-mame-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 linked structs OK
// OS9: LLVM OS-9 linked structs OK

#include <os9.h>

struct Node {
  int value;
  struct Node *next;
};

struct Node tail = {2, 0};
struct Node head = {1, &tail};
struct Node *head_ptr = &head;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (head_ptr->value != 1)
    return WRITE_LINE("bad linked head\r");
  if (head_ptr->next->value != 2)
    return WRITE_LINE("bad linked tail\r");
  if (head_ptr->next->next != 0)
    return WRITE_LINE("bad linked null\r");

  head_ptr->next->value = 3;
  if (tail.value != 3)
    return WRITE_LINE("bad linked store\r");

  return WRITE_LINE("LLVM OS-9 linked structs OK\r");
}
