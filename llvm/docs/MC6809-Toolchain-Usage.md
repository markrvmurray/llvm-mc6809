# Using the MC6809 toolchain

A short guide to the compiler once it is installed.  For building one, see
[MC6809-Toolchain-Build.md](MC6809-Toolchain-Build.md).

Unpack it anywhere and put its `bin` on your PATH:

```sh
tar -xf mc6809-toolchain-<version>-<host>.tar.xz
export PATH=$PWD/mc6809-toolchain-<version>-<host>/bin:$PATH
```

On Windows the bundle is a `.tar.gz`; see [Windows](#windows).

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
simulator is a separate program, [USim](https://github.com/markrvmurray/usim)
(`usim09batch` for bare metal, `usim09pt` for NitrOS-9), which `mc6809-run`
expects on the PATH; the toolchain does not carry it.

An OS-9 module needs two more things, because it is run by booting NitrOS-9
and handing it the module:

* `run-mc6809-os9`, from the `scripts/` directory of
  [picolibc](https://github.com/markrvmurray/picolibc), on the PATH;
* a bootable NitrOS-9 disk image for the Pico-Thing machine USim simulates,
  and its boot firmware.  Name them with `NITROS9_BOOT_DSK` and
  `NITROS9_FIRMWARE`, or point `NITROS9_RECIPES` at a NitrOS-9 build tree
  and let `NITROS9_LEVEL` (1 or 2) and `NITROS9_CPU` (6809 or 6309) pick
  them.  The script's own header lists the rest.

A DECB binary needs a CoCo; see [Running one under MAME](#running-one-under-mame).

## The tools

`mc6809-clang`, `mc6809-os9-clang`, `mc6809-decb-clang`, and `mc6809-ar`,
`-ld`, `-nm`, `-objcopy`, `-objdump`, `-ranlib`, `-readelf`, `-size`,
`-strip`, `-run`.  They are the LLVM tools under names that say what they are
for.

## Choosing the machine

```sh
mc6809-clang -mcpu=hd6309 -mcrt0=semihost hello.c -o hello.elf
```

That selects the HD6309 instruction set **and** a C library built for it;
without it you get plain 6809 throughout.  `-mcpu=6309` means the same.  It
works for OS-9 as well as for bare metal:

```sh
mc6809-os9-clang -mcpu=hd6309 hello.c -o hello
```

**An OS-9 program built for the 6309 needs a NitrOS-9 built for the 6309** —
the `l<level>_6309` recipe, not the ordinary one.  Nothing marks the program
as needing it: a module's language byte says `6809 Obj` whatever the `-mcpu`
was, exactly as NitrOS-9's own 6309 build marks its own modules, because the
kernel matches a module by type *and* language and only ever asks for that
one.  Put a 6309 program on a 6809 system and you get an illegal instruction
when it reaches one, not a polite refusal.

## C++

`mc6809-clang++` and its `-os9-` and `-decb-` siblings compile C++, and an
ordinary hello world works:

```cpp
#include <cstdio>
int main() { std::printf("hello, world\n"); return 0; }
```

**There is no C++ standard library**, and there is not going to be one —
libc++ is far larger than a 64 KB address space.  What there is, is the
language over the C library:

* `<cstdio>`, `<cstdlib>`, `<cstring>`, `<cmath>`, `<cctype>`, `<ctime>`,
  `<cstdint>`, `<cstddef>`, `<cerrno>`, `<cassert>`, `<climits>`, `<cfloat>`,
  `<cstdarg>`, `<cinttypes>` and `<new>`.  Each is a thin cover: it includes
  the C header and brings its names into `namespace std`.  There is no
  `<vector>`, no `<string>`, no `<iostream>`.
* classes, templates, virtual functions, objects with constructors —
  including globals and function-local statics — `new` and `delete`.

**Exceptions and RTTI are off**, and asking for a `try` or a `dynamic_cast`
is an error where you write it rather than a mangled name at link time.
Unwinding wants tables and an unwinder, RTTI wants a type_info hierarchy, and
neither earns its space here.  `-fexceptions` or `-frtti` turn them back on
if you have brought your own runtime; this toolchain has none.

`new` returns null when it cannot allocate — there is nothing to throw — so
check the pointer, as you would in C.

## Floating point

The default libraries are integer-only — small, which is the right default on
a machine with 64 KB.  `-mlibc=float` asks for the ones that can format a
double and do maths:

```c
#include <math.h>
#include <stdio.h>
int main(void) {
  double x = 2.0;
  printf("product %f\n", x * 4.0);
  printf("sqrt %f\n", sqrt(16.0));
  return 0;
}
```

```sh
mc6809-clang -mlibc=float -mcrt0=semihost hello.c -o hello.elf
mc6809-run hello.elf
```

```
product 8.000000
sqrt 4.000000
```

Without it, `printf("%f", x)` prints the literal `*float*` — picolibc saying
the format was left out — and `sqrt` does not link.  No `-lm` is needed
either way: picolibc's maths lives in `libc.a`.  The flag selects headers as
well as libraries, because whether plain `printf` can format a double is
decided in `picolibc.h` at the time the library is built.

It combines with `-mcpu=hd6309`, so bare metal and OS-9 each have four
libraries: plain, 6309, floating point, and both.  DECB has only the first
two, and `-mlibc=float` there is an error.

A bare-metal program records in its ELF header which processor it needs, so
`mc6809-run` starts the right simulator without being told.  An OS-9 module
has nowhere to record that — its language byte says "6809 object" whatever
the code is, because that is the only language NitrOS-9 asks for — so a 6309
module is named as one: `mc6809-run --hd6309 program`, which boots the 6309
NitrOS-9 rather than the ordinary one.

The arithmetic itself is Motorola's MC6839 floating-point ROM — the one piece
of this toolchain not written for it, described in
[MC6809-Third-Party.md](MC6809-Third-Party.md) — and where it comes from
depends on the target:

* **Bare metal and DECB**: the 8 KB ROM is part of the compiler's runtime and
  is linked into your program — only if you use floating point, so a program
  that does none carries none.  Nothing to install.  On DECB that gives you
  the arithmetic, but not `printf("%f")` or the maths functions: there is no
  floating-point C library for it.
* **OS-9**: the ROM is the `FPO9` module, and the start-up code links it at
  run time.  **It must be on the machine**: already in memory, or loadable
  from the execution directory the program was run from.  A program that
  cannot find it prints `fpo9: no floating point module` and stops with the
  kernel's own error — which is as often "no room" as "no such module", since
  the module, your program and its data share one 64 KB space.  The file
  ships in the bundle at `lib/clang/<ver>/lib/mc6809-unknown-os9/FPO9`; copy
  it to the target like any other module.  `mc6809-run` does that part for
  you — it asks the compiler where its runtime is and puts the module on the
  disk beside your program.

  In practice OS-9 floating point wants Level 2: at Level 1 the `FPO9`
  module and a program that uses it seldom fit beside the system.

Expect it to be expensive: a hello-world printing two doubles came to 59,964
bytes of the 64 KB address space.

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

## Interrupt handlers

On bare metal a function can be an interrupt handler, and the start-up code
points the processor's vector at it when it has the right name:

```c
volatile unsigned char ticks;

__attribute__((interrupt)) void __irq_handler(void) {
  ticks++;
}

__attribute__((interrupt("firq"))) void __firq_handler(void) {
  /* ... */
}
```

A handler takes no arguments and returns nothing.  It returns with `RTI`, and
it preserves everything the interrupted code may be using: what the processor
did not stack for it, and the compiler's direct-page registers.  So it can
interrupt any compiled code and call any compiled function.

The attribute's argument names the interrupt: `"irq"`, `"nmi"`, `"swi"`,
`"swi2"`, `"swi3"` or `"firq"`.  Only `"firq"` changes the code.  A FIRQ entry
stacks just the program counter and the flags, so a FIRQ handler saves the
registers it uses itself; the others arrive with the whole register set
stacked, and the bare attribute means the same as naming any of them.

The start-up code holds the vector table, `$FFF0`-`$FFFF`.  Each vector goes
to a function of a fixed name, and a program installs a handler by defining
it:

| Vector  | Interrupt                         | Function          |
|---------|-----------------------------------|-------------------|
| `$FFF0` | 6309 illegal instruction, divide by zero | `__trap_handler` |
| `$FFF2` | SWI3                              | `__swi3_handler`  |
| `$FFF4` | SWI2                              | `__swi2_handler`  |
| `$FFF6` | FIRQ                              | `__firq_handler`  |
| `$FFF8` | IRQ                               | `__irq_handler`   |
| `$FFFA` | SWI                               | `__swi_handler`   |
| `$FFFC` | NMI                               | `__nmi_handler`   |
| `$FFFE` | Reset                             | `_start`          |

A vector with no handler goes to `__unhandled_interrupt`, which stops the
program: under a simulator it exits with status 99, and on hardware it loops.
The compiler uses SWI3 for `__builtin_trap`, and its default handler does the
same.  A machine that keeps its vectors somewhere else can move the table
with `-Wl,--defsym=__vectors=ADDRESS`.

What this covers, and what it does not:

* **Bare metal only.**  Under OS-9 the kernel fields the interrupt and calls a
  service routine as an ordinary subroutine, and Color BASIC keeps the vectors
  for itself; the attribute is an error for both.
* **6309: native mode, except for FIRQ.**  In emulation mode the processor
  does not stack W, and an IRQ, NMI or software-interrupt handler does not
  save it.  A FIRQ handler saves W itself and works in either mode.  The
  default start-up enters native mode.
* **No floating point in a handler.**  The floating-point routines keep their
  working state in fixed memory, so a handler that used them would corrupt a
  calculation it interrupted.
* **A handler that calls a function pays for it.**  It cannot know which
  direct-page registers the callee uses, so it saves all 24 bytes of them:
  about 130 bytes of code and 340 cycles (6809 timings) on top of a handler
  that calls nothing.  Keep a handler that must be fast free of calls, which includes
  multiplication, division and 32-bit arithmetic on the 6809.
* **`m6809_interrupt_norecurse` and `m6809_no_isr`** are not implemented and
  are errors.

A function that a handler calls may be running already when the interrupt
arrives.  The compiler allows for that: it keeps a function's locals in static
memory only when it can see every caller and none is a handler, which in an
ordinary compile means a `static` function reached from `main` alone.  With
`-flto` it sees the whole program and can do it more often.

## What you get on each target

**Bare metal** is picolibc over a semihosting layer: stdio goes to the
simulator's console.  On real hardware you would replace that layer.  The
machine the libraries are built for, which is the one USim simulates, is laid
out like this, from the top of memory down:

| Address | What is there |
|---|---|
| `$0000`-`$00FF` | the direct page: the compiler's registers at `$00`-`$1F`, `__directpage` variables after them |
| `$0100`-`$FFC3` | RAM: code, data, heap and stack |
| `$FFC4`-`$FFC5` | the console, an MC6850 ACIA |
| `$FFCB` | trace control (simulator only) |
| `$FFCC` | the halt port: a write stops the simulator, with that exit status |
| `$FFCD`-`$FFEF` | the reset entry |
| `$FFF0`-`$FFFF` | the vectors |

A program for other hardware replaces the console routines and links to its
own memory map.

**OS-9** is a real program module.  It is given `argc`/`argv` from the shell's
parameter string, a heap that grows through `F$Mem`, and files:
`open`/`read`/`write`/`lseek`/`stat`/`opendir`/`rename`/`mkdir`/`rmdir`/
`getcwd`/`statvfs` all go to the operating system.  Floating point comes from
the MC6839 ROM, linked at run time as the `FPO9` module.  The module is named
after your output file, so `-o hello` produces a module called `hello`.
Characters that some NitrOS-9 versions do not accept in a name are left out, so
`-o my-prog` produces `myprog`; `-Wl,--os9-name=NAME` sets the name outright.

Four things will surprise you if they are not expected:

* **`'\n'` is a carriage return** ($0D), as in OS-9's own C compilers,
  because OS-9 ends a line with one; `'\r'` is one too, and `'\l'` is a line
  feed.  Text files therefore hold CR, and a `switch` cannot have both
  `'\n'` and `'\r'` as cases.  `-fno-os9-newline` keeps `'\n'` a line feed;
  [MC6809-OS9.md](MC6809-OS9.md) has the details.
* **Room.** A Level 2 task gets seven 8 KB blocks; a module over **57344
  bytes** does not fail to load, it wedges the system as it loads.  Level 1
  shares one 64 KB space with the system and says `E$MemFul` politely
  instead.
* **Two paths on one file may not see each other** on NitrOS-9 up to v3.3.0.
  At Level 1 a reader opened while a writer holds the file sees only what has
  reached the disk, and the size the file had when it opened; at Level 2 the
  same happens after a write of a whole sector.  Write, close, then open
  behaves the same everywhere.  This is a fault in NitrOS-9's file manager,
  fixed on its `main` branch after v3.3.0 but not yet in a release.
* **A file name is cut short** at the first character NitrOS-9 up to v3.3.0
  does not accept, without an error, so a program can open or delete the
  wrong file.  Names of letters, digits, `_` and `.` are safe; this too is
  fixed on NitrOS-9's `main` branch.

**DECB** is a `LOADM`/`EXEC` binary: the direct page and the program at
`$3F00`, and the console through the BASIC ROM.  It runs — on a CoCo 3 under
MAME, `printf` and formatted output work — with two conditions:

```basic
CLEAR 200,&H3EFF        ' move BASIC below the program, before loading
LOADM"HELLO"
EXEC
```

**The `CLEAR` is not optional.**  Without it BASIC's variables, strings and
stack occupy the memory the program's heap grows into; the library refuses to
allocate rather than corrupt them, so anything needing `malloc` — including
`printf`'s buffer — fails.

`-mcpu=hd6309` works here too, and gets its own library; MAME's `coco3h` is
the machine to run it on.

### Running one under MAME

`mc6809-run` cannot: a CoCo is a different machine from the simulators it
knows.  What works, and the sharp edges found getting it to:

With `decb` from [ToolShed](https://github.com/n6il/toolshed):

```sh
decb dskini hello.dsk                            # a Disk BASIC disk
decb copy -2 -b hello.bin hello.dsk,HELLO.BIN    # -2 machine code, -b binary
mame coco3 -flop1 hello.dsk -autoboot_script drive.lua   # coco3h for a 6309
```

* **`coco3h` needs its own ROM name.**  MAME looks for `coco3h.zip` (or
  `coco.zip`) and will not fall back to `coco3.zip`, though the ROM is the
  same: link one to the other in `roms/`.
* **Do not type a long command with `-autoboot_command`.**  MAME's keyboard
  queue is fixed-size and a CoCo types a few characters a second, so
  everything after the first Enter is silently dropped — which looks exactly
  like a program that never ran.  Post one short line at a time from a Lua
  script (`-autoboot_script`) with `emu.wait` between them:

  ```lua
  local nat = manager.machine.natkeyboard
  emu.wait(6);  nat:post('CLEAR 200,&H3EFF\n')
  emu.wait(30); nat:post('LOADM"HELLO"\n')
  emu.wait(25); nat:post('EXEC\n')
  emu.wait(20)
  ```

* **Read the screen, not a screenshot.**  The text screen is at `$0400`,
  32×16, so a few lines of Lua turn it into text a script can check.  The
  VDG has no lowercase — BASIC stores it as inverse-video uppercase -- so
  case does not survive the trip.
* **BASIC does not echo the newline before running `EXEC`**, so a screen
  showing `EXEC` with no blank line beneath it may well be a program that has
  already run.
* **Rebuild the program after rebuilding the library.**  A stale binary on
  the floppy looks precisely like a fix that did not work.

There is no floating-point DECB library: the MC6839 ROM would be linked into
the program, and 8 KB is a lot to spend under BASIC without deciding to.

A program returns to BASIC by returning from `main`, and its output stays on
screen:

```
EXEC
BACK TO BASIC
OK
```

The start-up saves the stack and the direct page BASIC handed over and puts
both back, so the interpreter resumes with its own machine — which matters
more than it sounds: a program runs with `DP` pointing at its own page, and
Color BASIC reaches its working storage through `DP`.

## Debugging

`-g` gives the usual DWARF debug information, including the call-frame
information (`.debug_frame`) a debugger unwinds the stack by;
`-funwind-tables` gives it as `.eh_frame` instead.  The C library and
start-up code carry it too, checked for every routine when the bundle is
made.

**There is no debugger in the bundle yet.**  The system lldb on a Mac does
not accept an MC6809 file, and lldb built from this tree has nothing a user
can attach it to until USim's gdb-remote server is released.  Both are being
worked on.  Until then `mc6809-objdump -d -S` and `mc6809-readelf` are the
tools to look with.

## Windows

The Windows bundle has had much less testing than the others; treat it as
experimental.  It is a `.tar.gz`; unpack it with the `tar` that comes with
Windows 10 and later:

```bat
tar -xf mc6809-toolchain-<version>-windows-x86_64.tar.gz
```

Its tools' other names are hard links, which that `tar` keeps.  It compiles
and links for every target, but **it cannot run what it builds**: there is no
Windows simulator, and `mc6809-run` is a shell script.  Run programs on
another host or on the machine itself.

## Limits worth knowing

* **64 KB, all in.** Code, data and stack share one address space.  Programs
  that do not fit fail at link time, or on OS-9 when the kernel is asked for
  the memory.
* **`int` is 16 bits**, pointers are 16 bits, and `long` is 32.
* **No fork, exec, or signals** on any target, and none planned.
* **No C++ standard library** — see [C++](#c) above for what there is.
* **No wide characters or multibyte**, as a matter of policy — those tests are
  out of scope rather than pending.
* **The default libraries are integer-only**, and `%lld`, `regcomp` and the
  other POSIX extensions are absent as well — the shipped libraries are built
  small.  Floating point has a variant of its own: see
  [Floating point](#floating-point) above.  The others do not; if you need
  one, build picolibc with the option that provides it and link against that
  sysroot.
* **Position-independent by default.** Code is PC-relative, which is what OS-9
  modules need; `-fno-pie` gives absolute addressing for a ROM at a fixed
  address.
* **Optimisation levels:** `-O0`, `-O1`, `-O2`, `-Os` and `-Oz` are the ones
  supported.  The others are accepted and tested, but are not what this
  toolchain is tuned for.

## When something will not fit

`-Os` is the default in the shipped libraries and usually the right flag for
your own code too.  `-flto` often wins several per cent more.  Beyond that,
the honest answer is often that the program is too big for the machine:
some programs do not fit at any setting.
