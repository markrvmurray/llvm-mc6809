// REQUIRES: mc6809-os9-runtime
// RUN: %run_os9_case %s OS9
// ARGS: one two
// The start-up code turns the shell's parameter string into argc/argv
// (argv[0] is the module's own name), and claims a heap above the stack
// from the kernel (F$Mem): __os9_heap_end - __os9_heap_cur is at least the
// 16 KB default request minus what the module already had.
// OS9: argc=3 argv0=argvheap argv1=one argv2=two heap=ok
// OS9: argc=3 argv0=argvheap argv1=one argv2=two heap=ok
#include <os9.h>

static void put(const char *s) {
  int n = 0;
  while (s[n])
    n++;
  _os_write(1, s, &n);
}

int main(int argc, char **argv) {
  put("argc=");
  char d[2] = {(char)('0' + argc), 0};
  put(d);
  put(" argv0=");
  put(argv[0]);
  for (int i = 1; i < argc; i++) {
    put(" argv");
    d[0] = (char)('0' + i);
    put(d);
    put("=");
    put(argv[i]);
  }
  put(" heap=");
  unsigned room = (unsigned)(__os9_heap_end - __os9_heap_cur);
  put(room >= 8192 ? "ok" : "small");
  put("\r");
  return 0;
}
