// REQUIRES: mc6809-os9-runtime
// RUN: %run_os9_case %s OS9
// CFLAGS: -fno-builtin
//
// The heap grows.  F$Mem adds pages at the far end of the data area, which
// is where the heap is, so it extends in place -- and it has to: sbrk() has
// already taken the address it will hand back before it asks for more, so
// the start of the heap may not move.
//
// OS9: first=0 gained=yes start=same again=0 grew=yes
#include <os9.h>

extern char *__os9_heap_cur;
extern char *__os9_heap_end;
extern int __os9_grow(unsigned);

static void put(const char *s) {
  int n = 0;
  while (s[n] != 0)
    n++;
  _os_write(1, s, &n);
}

static void putn(long v) {
  char b[8];
  int i = 0;
  if (v < 0) { put("-"); v = -v; }
  do { b[i++] = (char)('0' + (int)(v % 10)); v /= 10; } while (v);
  while (i--) { int one = 1; _os_write(1, &b[i], &one); }
}

int main(void) {
  char *start0 = __os9_heap_cur;
  char *end0 = __os9_heap_end;

  int rc = __os9_grow(4096);
  put("first="); putn(rc);
  put(" gained=");
  put(__os9_heap_end - end0 >= 4096 ? "yes" : "NO");
  put(" start=");
  put(__os9_heap_cur == start0 ? "same" : "MOVED");

  // Twice, because a heap that can only grow once is not a growing heap.
  char *end1 = __os9_heap_end;
  rc = __os9_grow(2048);
  put(" again="); putn(rc);
  put(" grew=");
  put(__os9_heap_end - end1 >= 2048 ? "yes" : "NO");
  put("\n");
  return 0;
}
