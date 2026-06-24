/*===-- os9.h - C declarations for the OS-9 / NitrOS-9 syscall API -*- C -*-===
 *
 * Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
 * See https://llvm.org/LICENSE.txt for license information.
 * SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
 *
 * Minimal OS-9 syscall surface.
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
 * The POSIX-style wrappers return -1 on failure.  The direct _os_* wrappers
 * return the OS-9 error code itself.
 *
 *===----------------------------------------------------------------------===*/

#ifndef __MC6809_OS9_H__
#define __MC6809_OS9_H__

#ifdef __cplusplus
extern "C" {
#endif

/* POSIX-style names. */

void _exit(int __status) __attribute__((__noreturn__));

int  _write(int __fd, const char *__buf, int __n);
int  _read(int __fd, void *__buf, int __n);

/* Direct OS-9 wrappers.
 *
 * The count pointer receives the actual byte count on success.  Return
 * value is 0 on success, or the OS-9 error code on failure.
 */

int  _os_write(int __fd, const void *__buf, int *__countp);
int  _os_writeln(int __fd, const void *__buf, int *__countp);
int  _os_read(int __fd, void *__buf, int *__countp);
int  _os_readln(int __fd, void *__buf, int *__countp);

/* NitrOS-9 native names.
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
