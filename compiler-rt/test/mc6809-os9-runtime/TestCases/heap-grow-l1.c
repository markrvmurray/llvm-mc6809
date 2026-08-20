// REQUIRES: mc6809-os9-runtime, nitros9-l1
// RUN: %run_os9_case %s OS9
// LEVEL: 1
// CFLAGS: -fno-builtin
//
// The same growth at Level 1, where the data area is not a task's own
// address space but room in the one everything shares -- so what is past
// the far end is whatever the system has not given to something else.
//
// OS9: first=0 gained=yes start=same again=0 grew=yes
#include "heap-grow.c"
