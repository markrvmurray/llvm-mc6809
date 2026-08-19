//===-- syscalls.c - OS-9 / NitrOS-9 POSIX-style system calls -------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// The out-of-line half of the OS-9 system-call layer: errno, the POSIX-style
// _exit/_read/_write, and their NitrOS-9 names. The calls themselves are the
// inline shims in os9.h; nothing here touches a register by hand.
//
//===----------------------------------------------------------------------===//

#include <os9.h>

// The C library's when one is linked (picolibc's is a plain int too); this
// definition serves programs without one.
__attribute__((weak)) int errno;

// F$Exit takes the status in B and does not return.
__attribute__((noreturn)) void _exit(int status) {
  OS9_SYSCALL(OS9_F_EXIT, (), ("d"(status)));
  __builtin_unreachable();
}

// Bytes written, or -1 with errno set.
int _write(int fd, const void *buf, int n) {
  return _os_write(fd, buf, &n) ? -1 : n;
}

// Bytes read (0 at end of file), or -1 with errno set.
int _read(int fd, void *buf, int n) {
  return _os_read(fd, buf, &n) ? -1 : n;
}

// The same functions under their OS-9 names.
__attribute__((noreturn, alias("_exit"))) void F$Exit(int status);
__attribute__((alias("_write"))) int I$Write(int fd, const void *buf, int n);
__attribute__((alias("_read"))) int I$Read(int fd, void *buf, int n);
