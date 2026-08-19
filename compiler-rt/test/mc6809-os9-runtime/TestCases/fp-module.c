// REQUIRES: mc6809-os9-runtime
// RUN: %run_os9_case %s OS9
// Floating point on OS-9 is the FPO9 module -- the MC6839 ROM, which
// Motorola shipped as an OS-9 module -- rather than eight kilobytes
// carried inside every program that divides two numbers.  Start-up links
// it (loading it from the execution directory when nothing else has it
// yet) and _exit gives it back, so the second of the two runs below has
// to find it again from scratch.
// OS9: base=ok sum=575 prod=787 div=1555 widen=225 fromint=4200 cmp=1
// OS9: base=ok sum=575 prod=787 div=1555 widen=225 fromint=4200 cmp=1
#include <os9.h>

extern void *__os9_fp_module;

static void put(const char *s) {
  int n = 0;
  while (s[n])
    n++;
  _os_write(1, s, &n);
}

static void put_long(long v) {
  char buf[12];
  int i = sizeof buf - 1;
  int neg = v < 0;

  buf[i] = '\0';
  if (neg)
    v = -v;
  if (v == 0)
    buf[--i] = '0';
  while (v > 0) {
    buf[--i] = (char)('0' + v % 10);
    v /= 10;
  }
  if (neg)
    buf[--i] = '-';
  put(buf + i);
}

static void show(const char *name, long v) {
  put(name);
  put_long(v);
}

int main(void) {
  volatile double a = 3.5, b = 2.25;
  volatile float f = 1.5f;
  volatile int i = 42;

  put(__os9_fp_module ? "base=ok" : "base=none");
  show(" sum=", (long)((a + b) * 100));
  show(" prod=", (long)(a * b * 100));
  show(" div=", (long)(a / b * 1000));
  show(" widen=", (long)((double)f * f * 100)); /* f32 -> f64 */
  show(" fromint=", (long)((double)i * 100));   /* i32 -> f64 */
  show(" cmp=", a > b ? 1 : 0);
  put("\r");
  return 0;
}
