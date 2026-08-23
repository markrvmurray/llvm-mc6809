/*===-- os9.h - OS-9 / NitrOS-9 system calls from C ---------------*- C -*-===
 *
 * Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
 * See https://llvm.org/LICENSE.txt for license information.
 * SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
 *
 * An OS-9 system call is one instruction, `os9 <code>` (SWI2 followed by the
 * function-code byte), with its arguments and results in registers and the
 * carry flag reporting failure (error code in B).  The shims below are that
 * one instruction with the register traffic left to the compiler: each is a
 * static inline function whose body is a single OS9_SYSCALL, so a call to
 * _os_write() compiles to a few register moves and the `os9` itself.
 *
 * Three flavours of name are available:
 *
 *   direct OS-9        _os_read _os_readln _os_write _os_writeln _os_open
 *                      _os_close            0 on success, else the OS-9
 *                                           error code (also left in errno)
 *   POSIX style        _exit _read _write   -1 on failure, errno set
 *   NitrOS-9 native    F$Exit I$Read I$Write  the POSIX functions under
 *                                           their OS-9 names; `$` in an
 *                                           identifier needs
 *                                           -fdollars-in-identifiers, which
 *                                           the mc6809-unknown-os9 driver
 *                                           enables (define
 *                                           OS9_NO_DOLLAR_NAMES to hide them)
 *
 * Path numbers are file descriptors: 0/1/2 = stdin/stdout/stderr, no
 * translation.
 *
 *===----------------------------------------------------------------------===*/

#ifndef __MC6809_OS9_H__
#define __MC6809_OS9_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Function codes (os9.d names without the `$`). */
#define OS9_F_EXIT   0x06
#define OS9_F_MEM    0x07
#define OS9_F_TIME   0x15
#define OS9_I_OPEN   0x84
#define OS9_I_SEEK   0x88
#define OS9_I_READ   0x89
#define OS9_I_WRITE  0x8A
#define OS9_I_READLN 0x8B
#define OS9_I_WRITLN 0x8C
#define OS9_I_CLOSE  0x8F

/* Error codes this header interprets itself. */
#define OS9_E_EOF    0xD3

/* I$Open modes. */
#define OS9_READ     0x01
#define OS9_WRITE    0x02
#define OS9_UPDATE   (OS9_READ | OS9_WRITE)

/* The OS-9 error code of the last failed call.  Lives in the process data
 * area like every other writable object. */
extern int errno;

/*
 * OS9_SYSCALL(code, (outputs), (inputs))
 *
 * The system call itself: `os9 code` with the given asm operand lists, each
 * wrapped in one pair of parentheses.  Register constraints: A, B, d (D),
 * x, y; "c" is the carry flag (1 = the call failed, B holds the error code).
 * U and DP are the process data-area base and must not be named.  Memory is
 * assumed clobbered, so buffers the kernel reads or writes need no further
 * annotation.
 *
 *   unsigned char err, code;
 *   OS9_SYSCALL(OS9_I_CLOSE, ("=c"(err), "=B"(code)), ("A"(path)));
 */
#define __OS9_UNPAREN(...) __VA_ARGS__
#define OS9_SYSCALL(code, outputs, inputs)                                    \
  __asm__ __volatile__("os9 %c[__os9_code]"                                   \
                       : __OS9_UNPAREN outputs                                \
                       : [__os9_code] "i"(code), __OS9_UNPAREN inputs         \
                       : "memory")

/* Record a failure and hand back its code. */
static inline int __os9_fail(unsigned char __code) {
  errno = __code;
  return __code;
}

/* I$Read / I$ReadLn: A = path, X = buffer, Y = bytes wanted -> Y = bytes read.
 * End of file is a successful zero-byte read.  *countp is the request on
 * entry and the actual count on success. */
#define __OS9_READ_SHIM(name, code)                                           \
  static inline int name(int __path, void *__buf, int *__countp) {           \
    unsigned char __err, __ecode;                                             \
    int __n = *__countp;                                                      \
    OS9_SYSCALL(code, ("=c"(__err), "=B"(__ecode), "+y"(__n)),                \
                ("A"((unsigned char)__path), "x"(__buf)));                    \
    if (__err) {                                                              \
      if (__ecode != OS9_E_EOF)                                               \
        return __os9_fail(__ecode);                                           \
      __n = 0;                                                                \
    }                                                                         \
    *__countp = __n;                                                          \
    return 0;                                                                 \
  }

/* I$Write / I$WritLn: A = path, X = buffer, Y = bytes -> Y = bytes written.
 * A zero-length write is a no-op. */
#define __OS9_WRITE_SHIM(name, code)                                          \
  static inline int name(int __path, const void *__buf, int *__countp) {     \
    unsigned char __err, __ecode;                                             \
    int __n = *__countp;                                                      \
    if (__n == 0)                                                             \
      return 0;                                                               \
    OS9_SYSCALL(code, ("=c"(__err), "=B"(__ecode), "+y"(__n)),                \
                ("A"((unsigned char)__path), "x"(__buf)));                    \
    if (__err)                                                                \
      return __os9_fail(__ecode);                                             \
    *__countp = __n;                                                          \
    return 0;                                                                 \
  }

__OS9_READ_SHIM(_os_read, OS9_I_READ)
__OS9_READ_SHIM(_os_readln, OS9_I_READLN)
__OS9_WRITE_SHIM(_os_write, OS9_I_WRITE)
__OS9_WRITE_SHIM(_os_writeln, OS9_I_WRITLN)

/* I$Open: A = mode, X = path name -> A = path number. */
static inline int _os_open(const char *__name, int __mode, int *__pathp) {
  unsigned char __err, __ecode, __path;
  OS9_SYSCALL(OS9_I_OPEN, ("=c"(__err), "=B"(__ecode), "=A"(__path)),
              ("A"((unsigned char)__mode), "x"(__name)));
  if (__err)
    return __os9_fail(__ecode);
  *__pathp = __path;
  return 0;
}

/* I$Close: A = path number. */
static inline int _os_close(int __path) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_I_CLOSE, ("=c"(__err), "=B"(__ecode)),
              ("A"((unsigned char)__path)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* POSIX-style entry points, in libclang_rt.os9.a. */
void _exit(int __status) __attribute__((__noreturn__));
int  _write(int __fd, const void *__buf, int __n);
int  _read(int __fd, void *__buf, int __n);

#ifndef OS9_NO_DOLLAR_NAMES
void F$Exit(int __status) __attribute__((__noreturn__));
int  I$Write(int __fd, const void *__buf, int __n);
int  I$Read(int __fd, void *__buf, int __n);
#endif /* !OS9_NO_DOLLAR_NAMES */

#ifdef __cplusplus
}
#endif

#endif /* __MC6809_OS9_H__ */
