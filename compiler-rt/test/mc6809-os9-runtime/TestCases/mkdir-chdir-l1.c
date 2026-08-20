// REQUIRES: mc6809-os9-runtime, nitros9-l1
// RUN: %run_os9_case %s OS9
// CFLAGS: -fno-builtin
// LEVEL: 1
//
// The same directory calls at Level 1, where a program shares one address
// space with the system and its data area does not start at zero.
//
// OS9: makdir=0 chdir=0 wrote=5 read=hello chdir-back=0 from-parent=0 rmfile=0
#include "mkdir-chdir.c"
