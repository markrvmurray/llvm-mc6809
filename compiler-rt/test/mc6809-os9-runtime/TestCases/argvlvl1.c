// REQUIRES: mc6809-os9-runtime, nitros9-l1
// RUN: %run_os9_case %s OS9
// LEVEL: 1
// ARGS: one two
//
// argv[0] is not passed to the program: the shell strips the command name
// from the parameter string, so the start-up code recovers it from the
// module's own header -- PC-relative back to the module base, then the name
// offset the header holds, then the fcs-style name whose last character
// carries the high bit.
//
// Level 2 cannot test that.  There a task owns its address space, the data
// area base U is 0, and the module sits at a fixed place; an offset and an
// address are the same number, so getting the two confused still works.
// Level 1 shares one 64K between everything: U is nonzero, the module is
// wherever it fitted, and the module body and the data area are genuinely
// different places -- which is what the start-up code has to cross to put a
// name from the module into an argv[] in the data area.
//
// OS9: argc=3 argv0=argvlvl1 argv1=one argv2=two
// OS9: base=nonzero data=nonzero apart=yes
// OS9: argc=3 argv0=argvlvl1 argv1=one argv2=two
// OS9: base=nonzero data=nonzero apart=yes
#include <os9.h>

extern const unsigned char *__os9_module;
extern char *__os9_data_base;

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
    char k[2] = {(char)('0' + i), 0};
    put(k);
    put("=");
    put(argv[i]);
  }
  put("\n");

  // Both are real addresses here, and the name came out of one of them into
  // the other.  At Level 2 the data base is 0 and this says nothing.
  put("base=");
  put(__os9_module ? "nonzero" : "ZERO");
  put(" data=");
  put(__os9_data_base ? "nonzero" : "ZERO");
  put(" apart=");
  put((const unsigned char *)__os9_data_base != __os9_module ? "yes" : "no");
  put("\n");
  return 0;
}
