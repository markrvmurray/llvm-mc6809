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
 * The shims (_os_read, _os_write, _os_open, _os_create, __os9_seek ...)
 * return 0 on success, else the OS-9 error code (also left in errno).
 * libclang_rt.os9.a adds _exit and the byte-count _read/_write (with their
 * F$Exit/I$Read/I$Write names); the POSIX names a C library expects (read,
 * write, open, lseek, sbrk ...) are picolibc's OS-9 layer (libos/os9),
 * built on these shims.
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
#define OS9_F_LINK   0x00
#define OS9_F_LOAD   0x01
#define OS9_F_UNLINK 0x02
#define OS9_F_EXIT   0x06
#define OS9_F_MEM    0x07
#define OS9_F_SLEEP  0x0A
#define OS9_F_ID     0x0C
#define OS9_F_TIME   0x15
#define OS9_F_TPS    0x25 /* Level 2 */
#define OS9_F_XTIME  0x56 /* Level 2 */
#define OS9_I_DUP    0x82
#define OS9_I_CREATE 0x83
#define OS9_I_OPEN   0x84
#define OS9_I_MAKDIR 0x85
#define OS9_I_CHGDIR 0x86
#define OS9_I_DELETE 0x87
#define OS9_I_SEEK   0x88
#define OS9_I_READ   0x89
#define OS9_I_WRITE  0x8A
#define OS9_I_READLN 0x8B
#define OS9_I_WRITLN 0x8C
#define OS9_I_GETSTT 0x8D
#define OS9_I_SETSTT 0x8E
#define OS9_I_CLOSE  0x8F

/* I$GetStt / I$SetStt function codes (SS.* in os9.d). */
#define OS9_SS_OPT   0x00 /* get/set the 32-byte path options */
#define OS9_SS_READY 0x01 /* data ready? */
#define OS9_SS_SIZE  0x02 /* file size (X:U); set = truncate/extend */
#define OS9_SS_POS   0x05 /* file position (X:U) */
#define OS9_SS_EOF   0x06 /* at end of file? */
#define OS9_SS_DEVNM 0x0E /* device name */
#define OS9_SS_FD    0x0F /* file descriptor sector */

/* Error codes this header interprets itself (the rest are in os9.d). */
#define OS9_E_PTHFUL 0xC8
#define OS9_E_BPNUM  0xC9
#define OS9_E_BMODE  0xCB
#define OS9_E_MEMFUL 0xCF
#define OS9_E_UNKSVC 0xD0
#define OS9_E_EOF    0xD3
#define OS9_E_NES    0xD5
#define OS9_E_FNA    0xD6
#define OS9_E_BPNAM  0xD7
#define OS9_E_PNNF   0xD8
#define OS9_E_SLF    0xD9
#define OS9_E_CEF    0xDA
#define OS9_E_IBA    0xDB
#define OS9_E_HANGUP 0xDC
#define OS9_E_MNF    0xDD
#define OS9_E_DELSP  0xDF
#define OS9_E_IPRCID 0xE0
#define OS9_E_NOCHLD 0xE2
#define OS9_E_KWNMOD 0xE7
#define OS9_E_NEMOD  0xEA
#define OS9_E_BNAM   0xEB
#define OS9_E_NORAM  0xED
#define OS9_E_DNE    0xEE /* directory not empty */
#define OS9_E_NOTASK 0xEF
#define OS9_E_UNIT   0xF0
#define OS9_E_SECT   0xF1
#define OS9_E_WP     0xF2
#define OS9_E_CRC    0xF3
#define OS9_E_READ   0xF4
#define OS9_E_WRITE  0xF5
#define OS9_E_NOTRDY 0xF6
#define OS9_E_SEEK   0xF7
#define OS9_E_FULL   0xF8
#define OS9_E_BTYP   0xF9
#define OS9_E_DEVBSY 0xFA
#define OS9_E_DIDC   0xFB
#define OS9_E_LOCK   0xFC
#define OS9_E_SHARE  0xFD
#define OS9_E_DEADLK 0xFE

/* I$Open / I$Create access modes and file attributes (same bits). */
#define OS9_READ     0x01
#define OS9_WRITE    0x02
#define OS9_UPDATE   (OS9_READ | OS9_WRITE)
#define OS9_EXEC     0x04
#define OS9_PREAD    0x08
#define OS9_PWRITE   0x10
#define OS9_PEXEC    0x20
#define OS9_SHARE    0x40
#define OS9_DIR      0x80

/* The OS-9 error code of the last failed call (the C library's errno when
 * one is linked; libclang_rt.os9.a carries a weak definition for programs
 * without one).  Lives in the process data area like every other writable
 * object. */
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
#define __OS9_INPUTS(...) __VA_OPT__(__VA_ARGS__,)
#define OS9_SYSCALL(code, outputs, inputs)                                    \
  __asm__ __volatile__("os9 %c[__os9_code]"                                   \
                       : __OS9_UNPAREN outputs                                \
                       : __OS9_INPUTS inputs [__os9_code] "i"(code)           \
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

/* Every call that takes a path name leaves X pointing past the name it
 * parsed (ioman: `stx R$X,u`), so X is a read-write operand, not an input:
 * told otherwise, the compiler believes the pointer survives the call and
 * gives the next one a name that starts where the last one stopped. */

/* I$Open: A = mode, X = path name -> A = path number.  A and B are the only
 * byte registers the 6809 has, so a call that reads one and writes the other
 * value back into it says so ("+A") rather than naming an input and an output
 * separately -- two live byte values at once is more than the allocator can
 * always place. */
static inline int _os_open(const char *__name, int __mode, int *__pathp) {
  unsigned char __err;
  unsigned char __a = (unsigned char)__mode; /* in: mode, out: path number */
  unsigned char __b;                         /* out: error code */
  const char *__x = __name;                  /* in: the name, out: past it */
  OS9_SYSCALL(OS9_I_OPEN, ("=c"(__err), "+A"(__a), "=B"(__b), "+x"(__x)), ());
  if (__err)
    return __os9_fail(__b);
  *__pathp = __a;
  return 0;
}

/* I$Close: A = path number. */
static inline int _os_close(int __path) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_I_CLOSE, ("=c"(__err), "=B"(__ecode)),
              ("A"((unsigned char)__path)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* I$Create: A = mode (must include write), B = attributes, X = path name
 * -> A = path number.  Fails with OS9_E_CEF if the file exists. */
static inline int _os_create(const char *__name, int __mode, int __attrs,
                             int *__pathp) {
  unsigned char __err;
  /* A = mode, B = attributes going in; A = path number, B = the error code
   * coming out.  Named as the pair D rather than as two byte operands: with
   * the carry also an output, two tied byte operands leave the allocator
   * nothing to place them in (A and B are all the 6809 has). */
  unsigned __d = ((unsigned)(unsigned char)__mode << 8) |
                 (unsigned char)__attrs;
  const char *__x = __name;
  OS9_SYSCALL(OS9_I_CREATE, ("=c"(__err), "+d"(__d), "+x"(__x)), ());
  if (__err)
    return __os9_fail(__d & 0xFF);
  *__pathp = __d >> 8;
  return 0;
}

/* I$Delete: X = path name. */
static inline int _os_delete(const char *__name) {
  unsigned char __err, __ecode;
  const char *__x = __name;
  OS9_SYSCALL(OS9_I_DELETE, ("=c"(__err), "=B"(__ecode), "+x"(__x)), ());
  return __err ? __os9_fail(__ecode) : 0;
}

/* I$MakDir: B = attributes, X = path name. */
static inline int _os_makdir(const char *__name, int __attrs) {
  unsigned char __err, __ecode;
  const char *__x = __name;
  OS9_SYSCALL(OS9_I_MAKDIR, ("=c"(__err), "=B"(__ecode), "+x"(__x)),
              ("B"((unsigned char)__attrs)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* I$ChgDir: A = mode (OS9_READ/OS9_WRITE = data directory, OS9_EXEC =
 * execution directory), X = path name. */
static inline int _os_chgdir(const char *__name, int __mode) {
  unsigned char __err, __ecode;
  const char *__x = __name;
  OS9_SYSCALL(OS9_I_CHGDIR, ("=c"(__err), "=B"(__ecode), "+x"(__x)),
              ("A"((unsigned char)__mode)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* I$Dup: A = path number -> A = new path number. */
static inline int _os_dup(int __path, int *__newp) {
  unsigned char __err, __ecode, __new;
  OS9_SYSCALL(OS9_I_DUP, ("=c"(__err), "=B"(__ecode), "=A"(__new)),
              ("A"((unsigned char)__path)));
  if (__err)
    return __os9_fail(__ecode);
  *__newp = __new;
  return 0;
}

/* I$GetStt / I$SetStt with a buffer: A = path, B = function, X = buffer,
 * Y = byte count (SS.FD takes one; SS.Opt ignores it).  SS.Opt reads or
 * writes the path's 32-byte option section; SS.FD reads the first Y bytes
 * of the file descriptor sector. */
static inline int _os_getstt_buf(int __path, int __fn, void *__buf, int __n) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_I_GETSTT, ("=c"(__err), "=B"(__ecode)),
              ("A"((unsigned char)__path), "B"((unsigned char)__fn),
               "x"(__buf), "y"(__n)));
  return __err ? __os9_fail(__ecode) : 0;
}
static inline int _os_setstt_buf(int __path, int __fn, const void *__buf) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_I_SETSTT, ("=c"(__err), "=B"(__ecode)),
              ("A"((unsigned char)__path), "B"((unsigned char)__fn),
               "x"(__buf)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* I$GetStt with no operands beyond the function: SS.EOF (OS9_E_EOF when at
 * the end), SS.Ready (B = bytes available on success). */
static inline int _os_getstt(int __path, int __fn) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_I_GETSTT, ("=c"(__err), "=B"(__ecode)),
              ("A"((unsigned char)__path), "B"((unsigned char)__fn)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* Calls that carry a 32-bit file position in X:U (I$Seek, and SS.Size /
 * SS.Pos through I$GetStt / I$SetStt): U is the process data-area base, so
 * these are hand-written stubs in libclang_rt.os9.a that save and restore it
 * around the call.  `pos` is read by the seek and the set, written by the
 * get.  0 on success, else the OS-9 error code (also in errno). */
struct __os9_xu {
  unsigned char path;
  unsigned char fn;      /* SS.Size / SS.Pos; ignored by the seek */
  unsigned long pos;
};
int __os9_seek(struct __os9_xu *__p);
int __os9_getstt_xu(struct __os9_xu *__p);
int __os9_setstt_xu(struct __os9_xu *__p);

/* F$Link, F$Load and F$UnLink: a module comes back in U, which is the
 * process data base, so these are stubs in libclang_rt.os9.a too.
 *
 *   F$Link   A = type/language (0 = any), X = the module's name
 *   F$Load   A = type/language, X = a pathlist, relative to the execution
 *            directory -- and the module it loads is linked, so it is one
 *            of ours to give back
 *   F$UnLink U = the module header
 *
 * Both return the header in U and the entry point in Y, and leave X past
 * the name.  0 on success, else the OS-9 error code (also in errno). */
struct __os9_module {
  const char *name;    /* in: the module name, or a pathlist for the load */
  unsigned char type;  /* in: type/language byte, 0 for any */
  void *header;        /* out: the module header */
  void *entry;         /* out: its entry point */
};
int __os9_link(struct __os9_module *__m);
int __os9_load(struct __os9_module *__m);
int __os9_unlink(void *__header);

/* F$Mem: D = new total data-area size in bytes (0 = query) -> D = actual
 * size (whole pages), Y = upper bound of the area.  The kernel grows the area
 * at the top; it refuses (OS9_E_DELSP) to shrink below the stack. */
static inline int _os_mem(unsigned __size, unsigned *__sizep, void **__topp) {
  unsigned char __err, __ecode;
  void *__top;
  OS9_SYSCALL(OS9_F_MEM, ("=c"(__err), "=B"(__ecode), "+d"(__size), "=y"(__top)),
              ());
  if (__err)
    return __os9_fail(__ecode);
  if (__sizep)
    *__sizep = __size;
  if (__topp)
    *__topp = __top;
  return 0;
}

/* F$Time: X = 6-byte buffer -> year-1900, month, day, hour, minute, second. */
static inline int _os_time(unsigned char __buf[6]) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_F_TIME, ("=c"(__err), "=B"(__ecode)), ("x"(__buf)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* F$ID: -> A = process id, Y = user id. */
static inline int _os_id(int *__pidp, int *__userp) {
  unsigned char __err, __ecode, __pid;
  int __user;
  OS9_SYSCALL(OS9_F_ID, ("=c"(__err), "=B"(__ecode), "=A"(__pid), "=y"(__user)),
              ());
  if (__err)
    return __os9_fail(__ecode);
  if (__pidp)
    *__pidp = __pid;
  if (__userp)
    *__userp = __user;
  return 0;
}

/* F$Sleep: X = ticks (0 = indefinitely, 1 = give up the rest of the slice). */
static inline int _os_sleep(unsigned __ticks) {
  unsigned char __err, __ecode;
  OS9_SYSCALL(OS9_F_SLEEP, ("=c"(__err), "=B"(__ecode)), ("x"(__ticks)));
  return __err ? __os9_fail(__ecode) : 0;
}

/* The C runtime's view of the process, set up by the start-up code before
 * main(): the heap sbrk() hands out, and the program's own name (the
 * module name, argv[0]). */
extern char *__os9_heap_cur;   /* next free byte */
extern char *__os9_heap_end;   /* one past the last usable byte */
extern char  __os9_progname[]; /* NUL-terminated module name */

/* Out-of-line entry points, in libclang_rt.os9.a: _exit, and the
 * byte-count-returning _read/_write (-1 on failure, errno set) under their
 * POSIX-style and NitrOS-9 names (`$` in an identifier needs
 * -fdollars-in-identifiers, which the mc6809-unknown-os9 driver enables;
 * define OS9_NO_DOLLAR_NAMES to hide them).  A C library's read()/write()
 * are separate functions built on the shims. */
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
