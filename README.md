# LLVM-MC6809

An LLVM and clang fork that compiles C — and just enough C++ — for the
Motorola 6809 and the Hitachi 6309.

This is a project done in my own time, for education and entertainment. It
is a long way past the toy stage: the code generator is exercised by a
benchmark suite of several thousand programs at thirty target and
optimisation combinations, and the toolchain is packaged, checked and
released by CI.

## Try it without building anything

The [latest release](https://github.com/markrvmurray/llvm-mc6809/releases)
carries a self-contained toolchain for Linux (x86-64 and arm64) and macOS
(Apple silicon). Unpack it anywhere — nothing is baked in, so the directory
can be moved or renamed — and compile:

```sh
tar -xf mc6809-toolchain-*.tar.xz
export PATH=$PWD/mc6809-toolchain-*/bin:$PATH

mc6809-clang hello.c -o hello.elf          # bare metal
mc6809-os9-clang hello.c -o hello          # an OS-9 / NitrOS-9 module
mc6809-decb-clang hello.c -o HELLO.BIN     # a CoCo LOADM binary
```

No linker script, no include path, no cross-file. Add `-mcpu=hd6309` for
the 6309, and the right library variant is chosen for you.

### What is in the box

* **Three targets** — bare metal ELF, OS-9 program modules, and Disk
  Extended Color BASIC binaries.
* **A C library**, picolibc, in four variants per machine-backed target:
  6809 or 6309, integer-only or floating-point. The default is
  integer-only, which is smaller and much faster; ask for the
  floating-point one with `-mlibc=float` when you need `printf("%f")` or
  `sqrt`.
* **Floating point in software**, from Motorola's own MC6839 ROM — the
  IEEE 754 package Joel Boney wrote for the 6809 in 1980, which Motorola
  released into the public domain. I maintain a reconstruction of it at
  [markrvmurray/nfp09](https://github.com/markrvmurray/nfp09); the bundle
  ships the binary, and on OS-9 it is a loadable module the start-up code
  links against by name. See the
  [third-party note](llvm/docs/MC6809-Third-Party.md) for its provenance.
* **Bare-bones C++** — no standard library, but the C library under its C++
  names, so `<cstdio>`, `new` and `delete` work. Exceptions and RTTI are
  off, and say so at compile time rather than failing at link.
* `mc6809-run`, which picks a simulator by looking at what you built.

Everything is position-independent by default: binaries use PCR-relative
globals and load at any address, which is what OS-9 modules and floating
ROMs need. `-fno-pie` opts out for a fixed-address ROM.

## Reporting a bug

Please do — [open an issue](https://github.com/markrvmurray/llvm-mc6809/issues).
The two things that make a report immediately actionable are:

1. **The version banner**, `mc6809-clang --version`. It names the exact
   revision the toolchain was built from.
2. **The source, and the command line you ran.** A miscompile matters most:
   if a program builds but computes the wrong answer, that is the kind of
   bug worth interrupting anything else for.

A note on scope: wide characters and multibyte strings are deliberately
absent rather than missing, and the default C library formats no floating
point. Neither is a bug.

## Documentation

* [Using a toolchain](llvm/docs/MC6809-Toolchain-Usage.md) — the targets,
  the library variants, running what you built.
* [Rolling and installing one](llvm/docs/MC6809-Toolchain-Build.md) — how a
  release is built, packaged and checked, and what CI does.
* [OS-9 and NitrOS-9](llvm/docs/MC6809-OS9.md) and the
  [system calls](llvm/docs/MC6809-OS9-syscalls.md).

## Building from source

```sh
cmake -C clang/cmake/caches/MC6809.cmake -G Ninja -S llvm -B build
cmake --build build --target all
```

You will need [Ninja](https://ninja-build.org/) and a recent CMake; the
final link steps are memory-hungry. That cache file builds a development
compiler — Debug, with assertions. For a release build, and for rolling a
bundle of your own, see the
[build guide](llvm/docs/MC6809-Toolchain-Build.md).

The C library is a separate repository,
[markrvmurray/picolibc](https://github.com/markrvmurray/picolibc) on its
`mc6809-port` branch. The two are released together and carry matching
tags, so a given release of this compiler names the exact picolibc it was
built against.

## Provenance

Forked from [llvm-mos](https://github.com/llvm-mos/llvm-mos), which is
itself a fork of [LLVM](https://github.com/llvm/llvm-project), and the 6809
backend borrows a good deal of hard-won structure from the 6502 one. Same
licence as LLVM: Apache 2.0 with LLVM exceptions.
