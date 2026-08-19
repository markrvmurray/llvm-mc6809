// REQUIRES: mc6809-os9-runtime
// RUN: %run_os9_case %s OS9
// A path-name system call leaves X pointing past the name it parsed, so a
// second call on the same name must load X again: if the compiler is
// allowed to think the pointer survived, the second call gets whatever the
// first one stopped on.  The name has to arrive in a register for the test
// to mean anything -- the address of a literal is cheap enough to
// rematerialise, which hides the bug.  Opening "." for reading fails
// because it is a directory; the retry with the directory bit is what a
// caller wanting any path on the device does.
// OS9: first=214 second=0
#include <os9.h>

static void put(const char *s) {
  int n = 0;
  while (s[n])
    n++;
  _os_write(1, s, &n);
}

static void put_int(int v) {
  char buf[8];
  int i = sizeof buf - 1;

  buf[i] = '\0';
  if (v == 0)
    buf[--i] = '0';
  while (v > 0) {
    buf[--i] = (char)('0' + v % 10);
    v /= 10;
  }
  put(buf + i);
}

static const char dot[] = ".";
/* Volatile so the name reaches the calls as a value in a register, the way
 * it does when the caller is in another translation unit. */
static const char *volatile any_path = dot;

static int first;

__attribute__((noinline)) static int open_any(const char *name, int *fdp) {
  first = _os_open(name, OS9_READ, fdp);
  if (first == 0)
    return 0;
  return _os_open(name, OS9_DIR | OS9_READ, fdp);
}

int main(void) {
  int second, fd = -1;

  second = open_any(any_path, &fd);
  if (second == 0)
    _os_close(fd);
  put("first=");
  put_int(first);
  put(" second=");
  put_int(second);
  put("\r");
  return 0;
}
