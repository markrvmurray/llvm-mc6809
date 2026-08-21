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

```sh
mc6809-clang -mlibc=float -mcrt0=semihost hello.c -o hello.elf
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

It combines with `-mcpu=hd6309`, so there are four libraries per target:
plain, 6309, floating point, and both.

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
  that does none carries none.  Nothing to install.
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

## When something will not fit

`-Os` is the default in the shipped libraries and usually the right flag for
your own code too.  `-flto` often wins several per cent more.  Beyond that,
the honest answer is often that the program is too big for the machine: the
bench keeps a list of test programs that do not fit at any setting.
