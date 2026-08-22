// REQUIRES: mc6809-os9-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 builtins OK
// OS9: LLVM OS-9 builtins OK

// Division, modulo, multiplication and shifts of 16- and 32-bit values are
// libcalls into the OS-9 build of the compiler builtins.

#include <os9.h>

static unsigned volatile a = 0x1234, b = 0x0056;
static int volatile sa = -1000, sb = 7;
static unsigned long volatile la = 0x12345678UL, lb = 0x00001000UL;
static long volatile sla = -123456L;
static unsigned char volatile sh = 5;

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  if (a / b != 54 || a % b != 16)
    return WRITE_LINE("bad u16 div\r");
  if (sa / sb != -142 || sa % sb != -6)
    return WRITE_LINE("bad s16 div\r");
  if (la / lb != 0x12345UL || la % lb != 0x678UL)
    return WRITE_LINE("bad u32 div\r");
  if (sla / 100L != -1234L || sla % 100L != -56L)
    return WRITE_LINE("bad s32 div\r");
  if (la * 3UL != 0x369D0368UL)
    return WRITE_LINE("bad u32 mul\r");
  if ((la >> sh) != 0x0091A2B3UL || (la << sh) != 0x468ACF00UL)
    return WRITE_LINE("bad u32 shift\r");
  if ((sla >> sh) != -3858L)
    return WRITE_LINE("bad s32 shift\r");
  return WRITE_LINE("LLVM OS-9 builtins OK\r");
}
