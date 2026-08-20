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
//   1. links the floating-point module when the program has any floating
//      point in it: the MC6839 is not part of an OS-9 program the way it
//      is part of a bare-metal one, it is the FPO9 module, shared by
//      everything using it.  _exit gives it back (F$UnLink);
//
//   2. claims the heap: asks the kernel (F$Mem) for __os9_mem_request bytes
//      of data area in total when the process has less, and hands sbrk()
//      the pages that appear past the old top -- past the stack and the
//      parameter area, so growing again later extends the heap in place
//      with nothing to move (__os9_grow).  When the whole request cannot
//      be had, it asks for less rather than giving up: a system with no
//      DAT has one 64K space for everything, and a program left with only
//      the static pool cannot even open a file.  The stack is whatever the
//      module header reserved (__stack_size), which the kernel must
//      satisfy at fork;
//
//   3. builds argc/argv: argv[0] is the module's own name (the shell strips
//      the command name from the parameter string); argv[1..] are the words
//      of the parameter string, which ends in a carriage return and is
//      split on blanks in place (it lives in the writable data area);
//
//   4. runs the constructors, calls main(argc, argv), runs the destructors
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
unsigned __os9_stack_size;      // __stack_size: kept clear of the heap

// The floating-point module, while this program holds it: _exit unlinks it.
void *__os9_fp_module;

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
  char *pool = __os9_data_base + __os9_pool_off;
  unsigned have = (unsigned)(__os9_top - __os9_data_base);
  unsigned want = __os9_mem_want;
  void *top;

  // The kernel lays the parameter area, and the stack below it, at the far
  // end of what it granted at fork -- the module header's ask, so the stack
  // has __stack_size and nothing has to guess.  Growing the area adds space
  // past all of that, and those pages are the heap: nothing moves, and a
  // later grow (__os9_grow) extends it in place.
  __os9_heap_cur = pool;
  __os9_heap_end = pool + __os9_pool_len;

  // Ask for less rather than nothing.  A system without a DAT has one 64K
  // space for everything, so a large program often cannot have the whole
  // request -- and taking the static pool instead leaves it with a heap
  // too small to open a file.  Halve until the kernel agrees or there is
  // nothing worth asking for.
  while (want > have + 256) {
    if (_os_mem(want, 0, &top) == 0 && (char *)top > __os9_top) {
      __os9_heap_cur = __os9_top;
      __os9_heap_end = (char *)top;
      __os9_top = (char *)top;
      return;
    }
    want = have + (want - have) / 2;
  }
}

// Grow the heap by at least `bytes`; 0 on success.  F$Mem adds pages at the
// far end of the data area, which is where the heap is, so the heap grows
// in place and nothing has to move.
//
// It must not move, either: sbrk() takes the address it is going to hand
// back before it asks for more, so this may extend the end and nothing
// else.  When the kernel refused at fork and the static pool is the heap,
// the pool is not at the far end -- there is nothing to extend, and saying
// so is better than relocating a heap someone already holds a pointer into.
int __os9_grow(unsigned bytes) {
  if (__os9_heap_end != __os9_top)
    return -1;

  // F$Mem is told the size the area should be, not the increment, and gives
  // back whole 256-byte pages -- so an ask for less than a page still moves
  // the top by one.  The size is 16 bits like every address here, so an ask
  // that wraps is an ask that cannot be met.
  unsigned want = (unsigned)(__os9_top - __os9_data_base) + bytes;
  void *top;
  if (want < bytes || _os_mem(want, 0, &top) != 0)
    return -1;
  if ((char *)top <= __os9_heap_end)      // nothing gained; leave as it was
    return -1;

  __os9_heap_end = (char *)top;
  __os9_top = (char *)top;
  return 0;
}

// Set by the MC6839 layer of the compiler's runtime, which is in the
// program only if it does floating point at all; a weak reference keeps a
// program that does none from carrying any of this.
extern void __mc6839_set_rom_base(void *__base) __attribute__((weak));

// The ROM is the FPO9 module: already in memory if something else got
// there first, otherwise loaded from the execution directory, which is
// where the program itself came from.  A program that needs floating
// point and cannot find it cannot run, so say so and stop with the
// kernel's own reason -- which is as often "no room for it" (the module,
// the program and its data area share one 64K address space) as it is
// "no such module" -- rather than call through an empty pointer.
static void claim_fp(void) {
  struct __os9_module m;
  int err;

  if (!__mc6839_set_rom_base)
    return;
  m.name = "FPO9";
  m.type = 0; // any type: the ROM is a user-defined type ($B1)
  err = __os9_link(&m);
  if (err != 0) {
    m.name = "fpo9";
    err = __os9_load(&m);
  }
  if (err != 0) {
    static const char missing[] = "fpo9: no floating point module\r";
    int n = sizeof missing - 1;
    _os_write(2, missing, &n);
    _exit(err);
  }
  __os9_fp_module = m.header;
  __mc6839_set_rom_base(m.header);
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
  // The floating-point module first: it has to fit in the address space
  // this process shares between its program, its data area and anything
  // it links, and the heap is the elastic one of the three.
  claim_fp();
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
