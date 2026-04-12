#!/bin/sh
# sdcc2gas.sh — translate gcc6809 / SDCC assembler syntax to GAS
# (the syntax llvm-mc accepts).
#
# Legacy: only used by the gcc6809 interop tests (interop.s,
# calling-convention.s, calling-convention-16.s). The main codegen
# tests now use clang for harness compilation, which outputs GAS
# syntax directly.
#
# Usage: sdcc2gas.sh < gcc6809-output.s > gas-input.s
#
# Used by every C-harness execution test (codegen-*-gcc.s,
# codegen-picolibc.s, codegen-if.s, ...). When gcc6809 emits a new
# SDCC-ism that llvm-mc rejects, add a translation rule here rather
# than to every individual test's RUN line.
#
# Current rule set:
#   .module foo.c             →  drop entirely (informational only)
#   .area .text               →  .section .rom,"ax",@progbits
#   .area .bss                →  .section .bss,"aw",@nobits
#   .area .rodata             →  .section .rodata,"a",@progbits
#   _funcname (leading _)     →  funcname (drop SDCC's underscore prefix)
#   .blkb N                   →  .space N
#   zmb N (lwasm directive)   →  .space N

exec perl -ne '
    next if /^\s*\.module/;
    s/\.area\s+\.text/.section .rom,"ax",\@progbits/;
    s/\.area\s+\.bss/.section .bss,"aw",\@nobits/;
    s/\.area\s+\.rodata/.section .rodata,"a",\@progbits/;
    s/(?<![a-zA-Z0-9])_([a-zA-Z])/$1/g;
    s/\.blkb\s+(\d+)/.space $1/;
    s/\bzmb\s+(\d+)/.space $1/;
    print;
'
