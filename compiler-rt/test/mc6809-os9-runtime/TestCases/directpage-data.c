// REQUIRES: mc6809-os9-runtime
//
// RUN: %run_os9_case %s OS9
//
// OS9: LLVM OS-9 direct page OK
// OS9: LLVM OS-9 direct page OK

// Direct-page objects live in the first page of the process data area
// (DP = U >> 8): initialised ones are copied there, zeroed ones cleared,
// a pointer held in the page is rebased, byte-wide direct addressing reads
// and writes them, and their address as an ordinary pointer works.

#include <os9.h>

static __attribute__((directpage)) unsigned char dp_init = 0x5A;
static __attribute__((directpage)) int dp_zero;
static __attribute__((directpage)) const char *dp_ptr = "LLVM OS-9 direct page OK\r";
static int table[3] = {1, 2, 3};
static int *volatile p_table = table;
static unsigned char *volatile p_dp = &dp_init;   /* full address of a dp object */

#define WRITE_LINE(s) write_line((s), sizeof(s) - 1)

static int write_line(const char *s, int n) {
  return _os_write(1, s, &n);
}

int main(void) {
  int n = 25;
  if (dp_init != 0x5A)
    return WRITE_LINE("bad dp init\r");
  if (dp_zero != 0)
    return WRITE_LINE("bad dp zero\r");
  dp_zero = p_table[1] + p_table[2];
  if (dp_zero != 5)
    return WRITE_LINE("bad dp store\r");
  dp_init += 1;
  if (dp_init != 0x5B)
    return WRITE_LINE("bad dp update\r");
  *p_dp += 1;
  if (dp_init != 0x5C)
    return WRITE_LINE("bad dp pointer\r");
  return _os_write(1, dp_ptr, &n);
}
