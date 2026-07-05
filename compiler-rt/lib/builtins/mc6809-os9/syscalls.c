//===-- syscalls.c - OS-9 / NitrOS-9 leaf syscalls (C) -------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// POSIX-style leaf syscalls expressed as C with inline asm.  The compiler
// marshals the OS-9 entry registers via the register constraints and (where
// relevant) captures the CC.C error flag via the 'c' constraint, so the
// hand-written prologue/epilogue bookkeeping of the assembly version is gone.
//
// Only _exit lives here today.  _read / _write also want C, but the compiler
// stages their i16 byte count through an imaginary-register direct-page slot
// (__rs0), and the os9 runtime does not yet define the __rc*/__rs* direct-page
// region every non-trivial C function may use.  They stay in syscalls.S until
// that region is designed (tracked separately).  _exit needs no scratch, so it
// converts cleanly.
//
// OS-9 syscall ABI (per llvm/docs/MC6809-OS9.md):
//
//   Entry registers vary per syscall.
//   Exit: CC.C clear = success, CC.C set = error with the OS-9 code in B.
//
//===----------------------------------------------------------------------===//

#define F_Exit 0x06

//===----------------------------------------------------------------------===//
// void _exit(int status) __attribute__((noreturn))
//
//   OS-9:  F$Exit -> B = status byte
//
// F$Exit never returns; 'd' places status in D (B = low byte).  The function
// code is emitted as the inline data byte after SWI2 via the 'i' immediate.
//===----------------------------------------------------------------------===//

__attribute__((noreturn)) void _exit(int status) {
  asm volatile("swi2\n\t.byte %c0" : : "i"(F_Exit), "d"(status) : "memory");
  __builtin_unreachable();
}

// NitrOS-9-native alias (same address, native name).  Requires
// -fdollars-in-identifiers (auto-enabled by the mc6809-unknown-os9 driver).
__attribute__((noreturn, alias("_exit"))) void F$Exit(int status);
