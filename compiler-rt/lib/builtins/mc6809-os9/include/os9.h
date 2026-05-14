/*===-- os9.h - C declarations for the OS-9 / NitrOS-9 syscall API -*- C -*-===
 *
 * Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
 * See https://llvm.org/LICENSE.txt for license information.
 * SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
 *
 * Bug #163 Phase 2: minimal OS-9 syscall surface.
 *
 * Two name flavors per syscall are exported by the syscalls.S object;
 * the C consumer can use whichever reads more naturally:
 *
 *   POSIX style       NitrOS-9 native
 *   ─────────────     ─────────────────
 *   _exit             F$Exit
 *   _write            I$Write
 *   _read             I$Read
 *
 * The NitrOS-9 names use `$` in identifiers, which is allowed by
 * the MC6809 assembler and by clang when -fdollars-in-identifiers
 * is in effect.  The mc6809-unknown-os9 clang driver auto-enables
 * that flag — match gcc6809's longstanding posture, so ports of
 * existing NitrOS-9 source compile without per-file flags.  If
 * you build for a different triple, pass -fdollars-in-identifiers
 * yourself, or use the POSIX-style names exclusively.
 *
 * errno wiring is NOT yet plumbed (Phase 2 PoC scope).  Error returns
 * surface as -1; the actual error code from the SWI2 trap exit is
 * currently discarded.  The picolibc-on-OS9 Phase 4 commit will wire
 * errno from the SWI2 B-register on CC.C=1.
 *
 *===----------------------------------------------------------------------===*/

#ifndef __MC6809_OS9_H__
#define __MC6809_OS9_H__

#ifdef __cplusplus
extern "C" {
#endif

/*── POSIX-style names ────────────────────────────────────────────────*/

void _exit(int __status) __attribute__((__noreturn__));

int  _write(int __fd, const char *__buf, int __n);
int  _read(int __fd, void *__buf, int __n);


/*── NitrOS-9 native names ────────────────────────────────────────────*
 *
 * The dollar-named decls below require -fdollars-in-identifiers.  The
 * mc6809-unknown-os9 driver auto-enables it; consumers that explicitly
 * pass -fno-dollars-in-identifiers should #define OS9_NO_DOLLAR_NAMES
 * before including this header.
 */

#ifndef OS9_NO_DOLLAR_NAMES

void F$Exit(int __status) __attribute__((__noreturn__));

int  I$Write(int __fd, const char *__buf, int __n);
int  I$Read(int __fd, void *__buf, int __n);

#endif  /* !OS9_NO_DOLLAR_NAMES */

#ifdef __cplusplus
}
#endif

#endif  /* __MC6809_OS9_H__ */
