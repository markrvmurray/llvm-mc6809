//===-- crt0.c - DECB program entry ---------------------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Entry for a CoCo Disk Extended Color BASIC machine-language program
// (`--target=mc6809-unknown-decb`): what LOADM puts in memory and EXEC jumps
// to.
//
// What BASIC has already done, and this therefore does not:
//
//   - copied the whole file into RAM at the load address, so initialised
//     data is already where it belongs and nothing has to be copied;
//   - set the stack pointer, so there is no stack to establish.
//
// What is left: the direct page register, the two zero-initialised areas
// (which are not in the file), the constructors, and main() -- after which
// this returns, which is what EXEC expects and how a program ends here.
//
// The symbols come from mc6809-decb.lds.  In C rather than assembly so that
// calling main() is the compiler's business rather than a guess about which
// register an argument arrives in.
//
//===----------------------------------------------------------------------===//

extern char __dp_base_addr[];
extern char __dp_bss_start[], __dp_bss_end[];
extern char __bss_start[], __bss_end[];

// Weak: a program that registered no constructors does not pull in the
// library's walker, and the reference resolves to nothing.
extern void __libc_init_array(void) __attribute__((weak));

extern int main(int, char **);

static void zero(char *from, char *to) {
  while (from != to)
    *from++ = 0;
}

__attribute__((section(".text.startup"), used)) void _start(void) {
  // The compiler reaches direct-page objects through DP, and nothing else
  // sets it.  Loading the address and keeping its high byte needs no more
  // than an ordinary 16-bit reference to the address the script put it at.
  __asm__ __volatile__("ldx #__dp_base_addr\n\ttfr x,d\n\ttfr a,dp"
                       :
                       :
                       : "d", "x");

  zero(__dp_bss_start, __dp_bss_end);
  zero(__bss_start, __bss_end);

  if (__libc_init_array)
    __libc_init_array();

  // EXEC passes nothing, and inventing an argv[0] would be inventing a name
  // the program was never given.
  (void)main(0, (char **)0);
}
