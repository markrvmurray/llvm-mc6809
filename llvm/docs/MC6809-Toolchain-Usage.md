# Using the MC6809 toolchain

A short guide to the compiler once it is installed.  For building one, see
[MC6809-Toolchain-Build.md](MC6809-Toolchain-Build.md).

Put its `bin` on your PATH:

```sh
export PATH=/opt/mc6809/bin:$PATH
```

## Hello, world, three ways

```sh
mc6809-clang -mcrt0=semihost hello.c -o hello.elf  # bare metal, on a simulator
mc6809-os9-clang   hello.c -o hello       # an OS-9 / NitrOS-9 program module
mc6809-decb-clang  hello.c -o hello.bin   # a CoCo DECB binary (LOADM/EXEC)
```

`-mcrt0=semihost` on the first one is not optional if you mean to run it: the
default start-up is the one for hardware, where a program does not return
from `main`, so under a simulator it prints its output and then runs for
ever.  See [Choosing the start-up](#choosing-the-start-up).  OS-9 and DECB
need nothing said: one exits through the kernel, the other returns to BASIC.

Nothing else is needed: no `-L`, no `-l`, no linker script, no include path.
The compiler finds its own library beside itself.

`clang --target=mc6809-unknown-os9` and the like work just as well; the short
names are the same binary reading its own name.

To run one:

```sh
mc6809-run hello.elf          # picks a simulator by what the file is
```

It looks at the file — an ELF, an OS-9 module, a DECB binary — and hands it
to something that can run it, or says plainly that nothing here can.  The
simulators are separate programs (USim, MAME); the toolchain does not carry
them.

## The tools

`mc6809-clang`, `mc6809-os9-clang`, `mc6809-decb-clang`, and `mc6809-ar`,
`-ld`, `-nm`, `-objcopy`, `-objdump`, `-ranlib`, `-readelf`, `-size`,
`-strip`, `-run`.  They are the LLVM tools under names that say what they are
for.

## Choosing the machine

```sh
mc6809-clang -mcpu=hd6309 hello.c -o hello.elf
```

That selects the HD6309 instruction set **and** a C library built for it;
without it you get plain 6809 throughout.  `-mcpu=6309` means the same.  It
works for OS-9 as well as for bare metal:

```sh
mc6809-os9-clang -mcpu=hd6309 hello.c -o hello
```

**An OS-9 program built for the 6309 needs a NitrOS-9 built for the 6309.**
The linker marks the module `6309 Obj` in its header, and the kernel matches
a module by type *and* language, so a 6809 system does not find it at all:
you get `ERROR #234`, non-existent module, which reads like a missing file
and is really the kernel declining to run 6309 code on a 6809.  That is the
right answer — the alternative is an illegal instruction somewhere later —
but it is a puzzling one the first time.

## Choosing the start-up

A program on real hardware does not return from `main`, so the default
start-up does not call `exit()` — which under a simulator means the program
prints its output and then runs for ever.

```sh
mc6809-clang -mcrt0=semihost hello.c -o hello.elf   # exits when main returns
```

| `-mcrt0=` | what it is for |
|---|---|
| *(default)* | hardware: `main` is not expected to return |
| `semihost` | a simulator: `exit()` stops the machine |
| `hosted` | as semihost, with the full hosted start-up |
| `minimal` | no constructors |
| `none` | no start-up at all — the link warns that there is no `_start`, which is the point of it |

This applies to bare metal only.  OS-9 exits through the kernel and DECB
returns to BASIC; both do the right thing without being asked.

## What you get on each target

**Bare metal** is picolibc over a semihosting layer: stdio goes to the
simulator's console.  On real hardware you would replace that layer.

**OS-9** is a real program module.  It is given `argc`/`argv` from the shell's
parameter string, a heap that grows through `F$Mem`, and files:
`open`/`read`/`write`/`lseek`/`stat`/`opendir`/`rename`/`mkdir`/`rmdir`/
`getcwd`/`statvfs` all go to the operating system.  Floating point comes from
the MC6839 ROM, linked at run time as the `FPO9` module.  The module is named
after your output file, so `-o hello` produces a module called `hello`.

Two things differ between NitrOS-9 levels and will surprise you if they are
not expected:

* **Room.** A Level 2 task gets seven 8 KB blocks; a module over **57344
  bytes** does not fail to load, it wedges the boot.  Level 1 shares one 64 KB
  space with the system and says `E$MemFul` politely instead.
* **Two paths on one file do not see each other at Level 1.** A reader opened
  while a writer holds the file gets the sectors the file was allotted, not
  what was written.  Level 2 is coherent.  Write, close, then open behaves the
  same on both.

**DECB** is a `LOADM`/`EXEC` binary: the direct page and the program at
`$3F00`, the console through the BASIC ROM, and a return to BASIC when `main`
ends.  **It has never been run** — the format, the start-up and the library
are written and link, but no machine has executed one.  Treat it as a
starting point, not a finished target.

## Limits worth knowing

* **64 KB, all in.** Code, data and stack share one address space.  Programs
  that do not fit fail at link time, or on OS-9 when the kernel is asked for
  the memory.
* **`int` is 16 bits**, pointers are 16 bits, and `long` is 32.
* **No fork, exec, or signals** on any target, and none planned.
* **No wide characters or multibyte**, as a matter of policy — those tests are
  out of scope rather than pending.
* **The libraries are integer-only.**  Floating-point *arithmetic* works: the
  compiler calls into its runtime, and on OS-9 into the MC6839 ROM, so
  `(int)(3.5 * 2.0 + 1.0)` is 8 as it should be.  But `printf("%f", x)` prints
  the literal `*float*` — picolibc's way of saying the format was left out —
  and `<math.h>` has headers with nothing behind them, so `sqrt` fails to
  link.  `%lld`, `regcomp` and the other POSIX extensions are absent for the
  same reason: the shipped libraries are built small, and a floating-point
  variant is not among them yet.  If you need one today, build picolibc
  yourself with `-Dstdio-float=true -Dwant-libm=true` and link against that.
* **Position-independent by default.** Code is PC-relative, which is what OS-9
  modules need; `-fno-pie` gives absolute addressing for a ROM at a fixed
  address.

## When something will not fit

`-Os` is the default in the shipped libraries and usually the right flag for
your own code too.  `-flto` often wins several per cent more.  Beyond that,
the honest answer is often that the program is too big for the machine: the
bench keeps a list of test programs that do not fit at any setting.
