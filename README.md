# LLVM-MC6809

LLVM-MC6809 is a LLVM fork supporting the 6x09 series of microprocessors.

This is a project done in my own time for education and entertainment purposes.

I use CLion as my development environment  - its great! My main development
machine is a laptop running macOS Sequoia.

## Design highlights

- **Position-independent by default (`-fPIE`)** — binaries built with default
  flags use PCR-relative globals (`R_MC6809_PCREL_16`) and load at any
  address. Designed for OS-9 / NitrOS-9 modules, ROMs that float across
  boards, and relocatable embedded use. Pass `-fno-pie` (or `-static`) to
  opt out for fixed-address ROMs. The compiler-rt runtime is being brought
  to the same posture so the entire toolchain ships PIE end-to-end; the
  integer helpers are already PIE-clean and the floating-point wrappers
  are next.

## Build instructions

To configure and build, try:

```
cd ~/git/llvm-mc6809
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug -C ../../clang/cmake/caches/MC6809.cmake -G Ninja -S ../../llvm -B .
cmake --build . --target all
```
You'll need [Ninja](https://ninja-build.org/) and a recent CMake installed; the
final link step is memory-hungry.

## picolibc port

A work-in-progress port of [picolibc](https://github.com/picolibc/picolibc)
to mc6809-unknown-elf lives on the `mc6809-port` branch of a separate
clone (typically `~/GitHub/picolibc`). The port consists of a meson
cross-file, a `picocrt/machine/mc6809/` startup stub, a
`libc/machine/mc6809/` directory, and a one-line patch to
`libc/include/machine/ieeefp.h` to declare the `__6809__` target as
big-endian. Build with:

```
cd ~/GitHub/picolibc
git checkout mc6809-port
meson setup builddir-mc6809 \
    --cross-file scripts/cross-clang-mc6809-unknown-elf.txt \
    -Dmultilib=false -Dsemihost=false -Dfake-semihost=false \
    -Dformat-default=minimal
ninja -C builddir-mc6809
```

The cross-file references the LLVM-MC6809 clang/llvm-ar/llvm-nm/llvm-strip
binaries by absolute path; edit it to match your build directory.

The port is incomplete — see the picolibc-related TODO bugs in
the project bug tracker memory for the current bug list.

### Bench & test harness

The picolibc fork carries the MC6809/HD6309 benchmark + correctness harness
that cross-builds picolibc with this compiler and runs the test suite on the
usim/MAME emulators, recording per-test cycles and pass/fail to a SQLite
ledger. See **`~/GitHub/picolibc/scripts/README-mc6809.md`**
(`bench-parallel.sh` being the orchestrator; ledger at
`~/Documents/mc6809-bench/results.sqlite`).
