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
