# OS-9 Runtime Porting Notes

This directory is the LLVM landing area for OS-9 runtime support.  The
CMOC/Kreider support tree at `~/Projects/coco-shelf/cmoc_os9` is the
reference implementation, but its objects and assembly cannot be copied in
wholesale because CMOC and LLVM use different C ABIs and object/tool formats.

## System-Call Layer

`include/os9.h` owns the low-level OS-9 entry points.  A system call is one
`os9 <code>` instruction, so each is a static inline shim around a single
`OS9_SYSCALL(code, (outputs), (inputs))` -- inline asm whose register
marshalling is the compiler's job.  `syscalls.c` (linked from
`libclang_rt.os9.a`) adds `errno` and the out-of-line POSIX-style calls.

The direct shims mirror the CMOC low-level I/O helpers:

```c
int _os_write(int fd, const void *buf, int *countp);
int _os_writeln(int fd, const void *buf, int *countp);
int _os_read(int fd, void *buf, int *countp);
int _os_readln(int fd, void *buf, int *countp);
int _os_open(const char *name, int mode, int *pathp);
int _os_close(int fd);
```

They return `0` on success, or the OS-9 error code on failure (also stored
in `errno`), and update `*countp` with the actual transfer count on success.
EOF from `I$Read`/`I$ReadLn` is a successful zero-byte read.

The POSIX-style `_read` and `_write` wrappers stay byte-count based: they
return the actual byte count on success, `0` at EOF for `_read`, or `-1` with
`errno` set on error.

## Source Mapping

Initial reference files in `cmoc_os9`:

- `lib/io.as`: direct `_os_read`, `_os_readln`, `_os_write`,
  `_os_writeln`, and seek wrappers.
- `lib/syscommon.as`: `_oserr`, `_osret`, `_os9err`, and `_sysret` error
  helper conventions.
- `include/os9.d`: OS-9 syscall and error constants.
- `include/errno.h`, `include/unistd.h`, and higher-level `lib/*.as`/`lib/*.c`:
  future libc surface to port after the syscall layer is stable.

## Porting Rule

Port behavior and API shape, not generated artifacts.  Express an imported
routine as C over `OS9_SYSCALL` rather than as assembly; the compiler owns
the calling convention (first pointer/16-bit argument in `X`, first 8-bit in
`B`, the rest on the stack, 16-bit results in `X`).  A call that hands `U`
to the kernel needs a hand-written stub that saves and restores it.

Keep CMOC-specific direct-page/Y data-base assumptions out of the LLVM runtime.
Do not write through PC-relative addresses in OS-9 program modules: writable
runtime state belongs in the per-process data area addressed from `U`/`DP`,
not in the reentrant module body.
