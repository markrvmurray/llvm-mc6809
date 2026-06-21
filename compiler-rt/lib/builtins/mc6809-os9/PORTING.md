# OS-9 Runtime Porting Notes

This directory is the LLVM landing area for OS-9 runtime support.  The
CMOC/Kreider support tree at `~/Projects/coco-shelf/cmoc_os9` is the
reference implementation, but its objects and assembly cannot be copied in
wholesale because CMOC and LLVM use different C ABIs and object/tool formats.

## First Porting Layer

`syscalls.S` owns the low-level OS-9 entry points that Clang links from
`libclang_rt.os9.a`.

The current direct wrapper shape mirrors the CMOC low-level I/O helpers:

```c
int _os_write(int fd, const void *buf, int *countp);
int _os_writeln(int fd, const void *buf, int *countp);
int _os_read(int fd, void *buf, int *countp);
int _os_readln(int fd, void *buf, int *countp);
```

These wrappers return `0` on success, return the OS-9 error code on failure,
store that code in the private runtime errno slot in the process data area,
and update `*countp` with the actual transfer count on success.  EOF from
`I$Read`/`I$ReadLn` is treated as a successful zero-byte read.

The POSIX-style `_read` and `_write` wrappers stay byte-count based: they
return the actual byte count on success, `0` at EOF for `_read`, or `-1` with
the private runtime errno slot set on error.

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

Port behavior and API shape, not generated artifacts.  Each imported assembly
routine needs an LLVM ABI pass:

- first pointer or 16-bit argument arrives in `X`;
- first 8-bit argument arrives in `B`;
- additional arguments are stack slots after the return address;
- 16-bit and pointer returns leave through `X`;
- preserve callee-saved registers used by the wrapper.

Keep CMOC-specific direct-page/Y data-base assumptions out of the LLVM runtime.
Do not write through PC-relative addresses in OS-9 program modules: writable
runtime state belongs in the per-process data area addressed from `U`/`DP`,
not in the reentrant module body.
