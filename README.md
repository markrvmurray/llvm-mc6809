# LLVM-MC6809

LLVM-MC6809 is a LLVM fork supporting the 6x09 series of microprocessors.

This is a project done in my own time for education and entertainment purposes.

I use CLion as my development environment  - its great! My main development
machine is a laptop running macOS Sequoia.

To configure and build, try:

```
$ cd ~/git/llvm-mc6809/llvm
$ mkdir -p build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Debug -C ../clang/cmake/caches/MC6809.cmake -G Ninja -S ../llvm -B .
$ cmake --build . --target all -j 8
```

## picolibc port

A work-in-progress port of [picolibc](https://github.com/picolibc/picolibc)
to mc6809-unknown-elf lives on the `mc6809-port` branch of a separate
clone (typically `~/GitHub/picolibc`). The port consists of a meson
cross-file, a `picocrt/machine/mc6809/` startup stub, a
`libc/machine/mc6809/` directory, and a one-line patch to
`libc/include/machine/ieeefp.h` to declare the `__6809__` target as
big-endian. Build with:

```
$ cd ~/GitHub/picolibc
$ git checkout mc6809-port
$ meson setup builddir-mc6809 \
    --cross-file scripts/cross-clang-mc6809-unknown-elf.txt \
    -Dmultilib=false -Dsemihost=false -Dfake-semihost=false \
    -Dformat-default=minimal
$ ninja -C builddir-mc6809
```

The cross-file references the LLVM-MC6809 clang/llvm-ar/llvm-nm/llvm-strip
binaries by absolute path; edit it to match your build directory.

The port is incomplete — see the picolibc-related TODO bugs in
the project bug tracker memory for the current bug list.
