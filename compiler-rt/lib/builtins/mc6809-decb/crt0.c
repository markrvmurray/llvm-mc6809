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

// Where BASIC's stack was when it entered us; declared in decb-private.h and
// used by _exit().  In .data rather than .bss on purpose: it is written
// before the zeroing below, and a variable in .bss would be wiped a few
// instructions after being set -- which is exactly what happened the first
// time, and printed SP=0000.
__attribute__((section(".data"))) void *__decb_entry_sp;

// And BASIC's direct page, for the same reason: this program runs with DP
// pointing at its own page ($3F), and Color BASIC reaches its working storage
// through DP.  Handing it back a foreign direct page leaves the interpreter
// reading our memory as its own.
__attribute__((section(".data"))) unsigned char __decb_entry_dp;

static void zero(char *from, char *to) {
  while (from != to)
    *from++ = 0;
}

// The entry proper, in assembly so that nothing is pushed before S is read:
// an ordinary function's prologue runs first, and saving S after it records
// our own frame rather than the stack BASIC handed over -- which is what the
// first attempt did, and it reported a return address pointing into BASIC's
// token table.
//
// It is also where a debugger's backtrace should end: BASIC's frames above
// it have no unwind information, so its CFI leaves the return address
// undefined, and it calls __decb_main rather than branching there so that
// the program's frames chain up to it.  The extra return address costs
// nothing: __decb_main goes home through the S saved here.  The CFI is
// written out here, frame and all, because a C function only has a CFI
// frame to add to when it is compiled with debug or unwind information.
//
// Everything here, and the way home below, reaches its variables and
// __decb_main PC-relative, as compiled code does, so the start-up code adds
// no address of its own that ties the program to where it was linked.
__asm__(".pushsection .text.startup,\"ax\",@progbits\n"
        "\t.cfi_sections .debug_frame\n"
        "\t.globl _start\n"
        "\t.type _start,@function\n"
        "_start:\n"
        "\t.cfi_startproc\n"
        "\t.cfi_def_cfa s, 0\n"
        "\t.cfi_undefined pc\n"
        "\tsts __decb_entry_sp,pc\n"
        "\ttfr dp,a\n"
        "\tsta __decb_entry_dp,pc\n"
        "\tlbsr __decb_main\n"
        "\t.cfi_endproc\n"
        "\t.size _start, .-_start\n"
        "\t.popsection\n");

__attribute__((section(".text.startup"), used)) void __decb_main(void) {

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

  // Home.  EXEC enters with a JSR -- the stack BASIC handed over holds a
  // return address into the ROM -- so RTS is the way back, and restoring S
  // first makes it work whatever the program did to the stack in between.
  // Falling off the end of this function would do the same thing only while
  // everything in between was balanced.
  __asm__ __volatile__("lda __decb_entry_dp,pc\n\t"
                       "tfr a,dp\n\t"
                       "lds __decb_entry_sp,pc\n\t"
                       "rts");
  __builtin_unreachable();
}
