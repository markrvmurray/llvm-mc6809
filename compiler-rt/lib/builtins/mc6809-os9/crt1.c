//===-- crt1.c - OS-9 / NitrOS-9 program start-up, the C half ------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// crt0.S has copied the initialised data, rebased its pointers and zeroed
// the bss; it stores what the kernel handed it (the data-area base, the
// parameter string and its length, the top of the area, the module base)
// and the linker script's numbers in the variables below, then calls
// __os9_crt1(), which:
//
//   1. claims the heap: asks the kernel (F$Mem) for __os9_mem_request bytes
//      of data area in total when the process has less, and hands sbrk()
//      the pages that appear above the old top -- above the stack and the
//      parameter area, so growing the area again later extends the heap
//      contiguously with nothing to move (__os9_grow).  When the area is
//      already big enough, or the kernel refuses, the heap is the static
//      pool the linker script carved out below the stack (__heap_start,
//      __heap_size);
//
//   2. builds argc/argv: argv[0] is the module's own name (the shell strips
//      the command name from the parameter string); argv[1..] are the words
//      of the parameter string, which ends in a carriage return and is
//      split on blanks in place (it lives in the writable data area);
//
//   3. runs the constructors, calls main(argc, argv), runs the destructors
//      (which is where a C library flushes its streams and runs atexit()
//      handlers) and exits with main's return value (F$Exit).
//
// Compiled C reaches a linker symbol's address U-relatively (the data
// model), so the script's plain numbers (offsets, sizes) come in through
// crt0.S, which can load them as immediates.
//
//===----------------------------------------------------------------------===//

#include <os9.h>

// From crt0.S: the kernel's entry registers and the linker script's numbers.
char *__os9_data_base;          // U
char *__os9_param;              // X: the parameter string
unsigned __os9_param_len;       // D: its length, including the trailing CR
char *__os9_top;                // Y: one past the data area (the stack lives
                                //    just below the parameter string)
const unsigned char *__os9_module; // the module body (its header at 0)
unsigned __os9_mem_want;        // __os9_mem_request: total size to ask for
unsigned __os9_pool_off;        // __heap_start: U-relative static pool
unsigned __os9_pool_len;        // __heap_size

// The heap sbrk() hands out.
char *__os9_heap_cur;
char *__os9_heap_end;
char __os9_progname[30];        // a module name is at most 29 characters

#define OS9_MAX_ARGS 32

extern int main(int, char **);

// The constructor and destructor tables, in the data area (the linker script
// gathers them into .data, whose pointers the CRT has already rebased).  The
// start-up code walks them itself rather than calling a C library's
// __libc_init_array: a weak reference to that would not pull it out of the
// library's archive, and a program built without a C library has none at all.
// A destructor here is also how the library flushes its streams and how
// atexit() handlers run (its .fini_array_onexit entry), so a main() that
// returns gets the same treatment as one that calls exit().
extern void (*__preinit_array_start[])(void);
extern void (*__preinit_array_end[])(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);
extern void (*__fini_array_start[])(void);
extern void (*__fini_array_end[])(void);

static void run_constructors(void) {
  for (void (**fn)(void) = __preinit_array_start; fn != __preinit_array_end;
       fn++)
    (*fn)();
  for (void (**fn)(void) = __init_array_start; fn != __init_array_end; fn++)
    (*fn)();
}

// Destructors run in reverse, as the C standard wants for atexit().
static void run_destructors(void) {
  for (void (**fn)(void) = __fini_array_end; fn != __fini_array_start;)
    (*--fn)();
}

static void claim_heap(void) {
  __os9_heap_cur = __os9_data_base + __os9_pool_off;
  __os9_heap_end = __os9_heap_cur + __os9_pool_len;
  unsigned have = (unsigned)(__os9_top - __os9_data_base);
  if (__os9_mem_want <= have)
    return;
  void *top;
  if (_os_mem(__os9_mem_want, 0, &top) != 0 || (char *)top <= __os9_top)
    return;
  // The new pages sit above everything the kernel laid out: they are the
  // heap, and another F$Mem would extend them in place.
  __os9_heap_cur = __os9_top;
  __os9_heap_end = (char *)top;
  __os9_top = (char *)top;
}

// Grow the heap by at least `bytes`; 0 on success.  Not yet: the heap is
// whatever claim_heap() found.  (A live version asks F$Mem for more and
// moves __os9_heap_end up -- possible when the heap is the region above
// the old top, where growth is contiguous.)
int __os9_grow(unsigned bytes) {
  (void)bytes;
  return -1;
}

static void module_name(void) {
  // Universal header: name offset at 4 (big-endian); the name is fcs-style,
  // last character with its high bit set.
  const unsigned char *m = __os9_module;
  unsigned off = ((unsigned)m[4] << 8) | m[5];
  const unsigned char *p = m + off;
  unsigned i = 0;
  while (i < sizeof __os9_progname - 1) {
    unsigned char c = *p++;
    __os9_progname[i++] = (char)(c & 0x7F);
    if (c & 0x80)
      break;
  }
  __os9_progname[i] = 0;
}

static int is_blank(char c) { return c == ' ' || c == '\t'; }

void __os9_crt1(void) {
  claim_heap();
  module_name();

  // Tokenise the parameter string in place: words separated by blanks,
  // terminated by the shell's CR (or the length).
  char *argv[OS9_MAX_ARGS + 1];
  char *q = __os9_param;
  char *end = q + __os9_param_len;
  int argc = 0;
  argv[argc++] = __os9_progname;
  while (q < end && *q != '\r' && argc < OS9_MAX_ARGS) {
    while (q < end && is_blank(*q))
      q++;
    if (q >= end || *q == '\r')
      break;
    argv[argc++] = q;
    while (q < end && *q != '\r' && !is_blank(*q))
      q++;
    if (q < end)
      *q++ = 0; // the blank or the CR becomes the word's terminator
  }
  argv[argc] = 0;

  run_constructors();
  int status = main(argc, argv);
  run_destructors();
  _exit(status);
}
