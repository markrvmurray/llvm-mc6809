// REQUIRES: mc6809-os9-runtime
// RUN: %run_os9_case %s OS9
// CFLAGS: -Wl,--defsym=__heap_size=0x3000
//
// The start-up code takes more data area from the kernel (F$Mem) as the
// heap in place of the static pool -- but only when that is more than the
// pool. With a 12 KB pool the module already asks for most of the 16 KB
// default at fork, so the growth on offer is smaller than the pool, and
// taking it would leave the program less heap than it declared. (At Level
// 1 the same happens whenever the program lands with a page or two free
// above its data area.)
//
// OS9: heap=pool
// OS9: heap=pool
#include <os9.h>

extern unsigned __os9_pool_len;

static void put(const char *s) {
  int n = 0;
  while (s[n])
    n++;
  _os_write(1, s, &n);
}

int main(void) {
  unsigned room = (unsigned)(__os9_heap_end - __os9_heap_cur);
  put(room >= __os9_pool_len ? "heap=pool" : "heap=small");
  put("\r");
  return 0;
}
