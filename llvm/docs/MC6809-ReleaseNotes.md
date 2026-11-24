# llvm-mc6809 release notes

Each release has a section headed `## <version>`, which the release workflow
publishes on that release's GitHub page above the table of test results.  A
tag with no section here is not published.

## 1.1.0-rc2

The second candidate for llvm-mc6809 1.1.0.

### Windows

There is now a bundle for Windows on x86_64 as well as for Linux and macOS.
It holds the same compiler, tools and libraries, and is checked the same way,
except that no program is run, since there is no simulator for Windows yet:
it compiles and links every test program. A release does not wait for it, so
if the Windows bundle fails its checks, the release is published without it.

### Known issues

* NitrOS-9 up to its current release, v3.3.0, cuts a file name short at the
  first character it does not accept, without an error, so a program can open
  or delete the wrong file. Names made of letters, digits, `_` and `.` are
  safe on every version.
* On NitrOS-9 up to v3.3.0, a file open twice can hide what one path wrote
  from the other. On Level 1 the second path sees only what has reached the
  disk, and the file's size as it was when that path opened, and `fflush`
  does not change this; on Level 2 the same happens after a write of a whole
  sector. NitrOS-9's file manager is fixed on its `main` branch, after
  v3.3.0, and a system built from that behaves as POSIX expects on both
  levels.
* Programs built on Windows have to be run somewhere else: on a simulator on
  another host, or on real hardware.

## 1.1.0-rc1

The first candidate for llvm-mc6809 1.1.0: faster and smaller code, a good
deal of wrong code fixed, interrupt handlers, and the 6309 under OS-9 tested
as thoroughly as the 6809. LLVM and picolibc are updated to their current
upstream versions.

### Smaller and faster

Against 1.0, the C library for the 6809 at `-Os` is 25% smaller (141,195
bytes of code, from 188,946). On the test programs whose sources are
unchanged since 1.0, code takes 12-20% fewer cycles in all on the 6809 and
8-14% fewer on the 6309, at every optimisation level; a typical program is
16-24% faster on the 6809 and 15-22% on the 6309.

* Values kept in the direct-page registers are no longer saved and restored
  around every use that borrows an accumulator: a register is saved only
  where its value is read again, and two such uses in a row share one save.
* Fewer redundant loads and stores of the same memory, by one set of rules
  shared by the passes that remove them.
* `clr` in place of loading zero, and on the 6309 `aim`, `oim` and `eim`
  in place of loading, changing and storing a byte; `inc` and `dec` on
  memory on both processors.
* An 8-bit by 8-bit multiply uses `MUL`; epilogues end in `puls ...,pc`;
  `strcmp` works a byte at a time.
* A call's register arguments are computed after its stack arguments are
  stored, so the two no longer compete for the accumulators, and `memset`,
  `memcpy` and `memmove` are no longer made to keep a copy of the
  destination they hand back.
* `memset` stores a word at a time on the 6809 and uses `TFM` on the 6309.
* `__builtin_mul_overflow`, and with it `calloc`, `fread` and `fwrite`,
  checks for overflow with multiplies instead of a division: on the 6809
  about 800 cycles less a check at 16 bits and 3,600 at 32 (440 and 2,700
  on the 6309), in less code.
* A program that uses floating point carries only the parts of the
  floating-point support it calls, 240-664 bytes less than before.
* On OS-9 and in Disk BASIC programs, `-mcpu=hd6309` now links the 6309
  build of the compiler's runtime library (multiply, divide and block moves
  in single instructions); it was the 6809 one.

### Changed behaviour

* `malloc` and `realloc` no longer clear the memory they return, which C
  does not ask of them; `calloc` still does. Allocating is much faster as a
  result, but a program that read memory from `malloc` without writing it
  first, and relied on it being zero, now sees whatever was there.

### Wrong code fixed

* Floating point: `+inf` and `-inf` were treated as one unsigned infinity,
  so `sqrt(INFINITY)` gave NaN and `-INFINITY == INFINITY` was true;
  `x > NaN` and `x >= NaN` were true; a `double` converted to `float` below
  `FLT_MIN` became NaN; and `printf` could round a `float` up one digit
  early just below a power of ten (`%.7f` of 0.9999999f gave 1.0000000).
* `alignas` on a local variable was ignored.
* A library call made while another call's arguments were being placed --
  a 64-bit conversion, say -- could overwrite those arguments.
* On the 6309: sign-extending the low part of a loaded `long` read its high
  bytes; negating a value in E, F or W also added the DP register, which is
  not zero in an OS-9 program; and a failed OS-9 system call could look
  successful, so `fopen` for writing or appending, `statvfs`, `rmdir` and
  signal delivery all went wrong; and a carry or overflow flag kept as a
  value could be lost when it was given the E or F register.
* At `-O0`, a branch on the borrow of a subtraction whose result was unused
  tested a stale carry.
* A reload could be removed, or a value reused, across a store that wrote
  the same memory: a store through another pointer that may alias it, a
  store that overlapped it only in part, a 6309 `stq` or `sts`, or an
  access whose offset is a symbol (`tbl,x`) taken for one at offset 0.
* On a Level 1 OS-9 system, a program could get a heap smaller than the one
  it declared, depending only on where it was loaded.

### Compiler and assembler errors fixed

* The compiler no longer crashes on `__builtin_trap`, `-ftrapv` (both now
  call `abort()`), C23 `fminimum`, `fmaximum` and their `_num` forms, or an
  inline-assembly memory operand it has to find a register for.
* Programs using conversions from floating point to unsigned integers,
  `__builtin_popcount` on a `long`, or `fesetexcept` now link.
* The assembler reported nothing for a register-pair instruction (`tfr`,
  `exg`, `tfm`, `adcr` and the like) naming a register it did not know, and
  left the instruction out; it now reports an error. It accepts the 6309's
  zero register, written `0`.

### Interrupt handlers

On bare metal, `__attribute__((interrupt))` makes a function an interrupt
handler: it returns with `rti` and preserves everything the interrupted code
may be using, so it can interrupt any compiled code and call any compiled
function. `interrupt("firq")` makes a fast-interrupt handler, which saves the
registers it uses itself. The start-up code holds the vector table,
`$FFF0`-`$FFFF`; a program installs a handler by defining the function of the
vector's name (`__irq_handler`, `__firq_handler`, `__nmi_handler` and so on).
See "Interrupt handlers" in MC6809-Toolchain-Usage.md.

### C library

* picolibc is updated to its current version, which adds among other things
  `quick_exit`, `call_once`, `free_sized`, `free_aligned_sized` and
  `memalignment`, and a reworked `malloc`.
* `printf` and `scanf` take `%b` for binary, as C23 has it. The formatting
  code is a little larger and slower for it, in programs that use them.
* A program can declare the least heap it needs (`-Wl,--defsym=__heap_size_min=N`);
  if the heap it gets is smaller, the link fails rather than the program.

### How it is tested

As for 1.0, with the 6309 under OS-9 added: about 380 compiler tests, and
the test programs at 31 combinations of optimisation level, processor,
library and target, three of them on NitrOS-9 (Level 1 and Level 2 on the
6809, Level 2 on the 6309). Every test now runs on USim, 6309 included;
MAME remains available for the 6309.

### Known issues

* NitrOS-9 up to its current release, v3.3.0, cuts a file name short at the
  first character it does not accept, without an error, so a program can open
  or delete the wrong file. Names made of letters, digits, `_` and `.` are
  safe on every version.
* On NitrOS-9 Level 1, a file open twice does not show what one path wrote
  to the other: the second path sees only what has reached the disk, and the
  file's size as it was when that path opened. `fflush` does not change
  this. Level 1's file manager keeps no common view of a file's open paths;
  Level 2's does, and behaves as POSIX expects.
* There is no Windows bundle.

## 1.0

llvm-mc6809 1.0, the first release of a C compiler for the Motorola 6809 and
the Hitachi 6309 built on LLVM and clang.  The compiler and C library are
those of 1.0-rc3.

### A toolchain in one archive

Unpack it anywhere and compile: there is no linker script to write, no include
path and no cross file.  Bundles are built for Linux (x86-64 and arm64) and
macOS (Apple silicon), and can be moved or renamed after unpacking.

```sh
mc6809-clang hello.c -o hello.elf          # bare metal
mc6809-os9-clang hello.c -o hello          # an OS-9 / NitrOS-9 module
mc6809-decb-clang hello.c -o HELLO.BIN     # a CoCo LOADM binary
mc6809-run hello.elf                       # run it on a simulator
```

`mc6809-run` picks a simulator to suit what was built.  The simulators, USim
and MAME, are installed separately.

### Three targets

* **Bare metal**: ELF programs, position-independent by default, or at a
  fixed address with `-fno-pie` for a ROM.
* **OS-9**: program modules, run on NitrOS-9 Level 1 and Level 2, with
  `argc`/`argv`, a heap, and files and directories through the operating
  system.
* **Disk Extended Color BASIC**: `LOADM` binaries for the Color Computer.

### Both processors

`-mcpu=hd6309` generates code for the 6309's extra registers and
instructions, such as its W and Q registers and its block moves.  Each target
has a 6309 C library as well as a 6809 one, and a bare-metal 6309 program
starts the processor in native mode.

### A C library, and floating point

* **picolibc**, built for each target and processor.  The default library is
  integer-only, which is smaller and faster.  On bare metal and OS-9,
  `-mlibc=float` selects one that can format floating point and do maths.
* **IEEE 754 floating point**, single and double precision, from Motorola's
  MC6839 floating-point ROM, written in 1980 and since placed in the public
  domain.  It is linked into the program, or loaded as the `FPO9` module on
  OS-9.
* **C++** without a standard library: the C library under its C++ names, and
  `new` and `delete`.  Exceptions and RTTI are off.

### How it is tested

Every change is measured on about 280 test programs, each built at 30
combinations of optimisation level, processor, library and target, plus about
320 compiler tests.  The programs run on simulators of each machine, and on
NitrOS-9 itself booted in a simulator.  Each bundle in this release was
checked again after unpacking, somewhere it was never built.

### Known issues

* The last update of LLVM cost a little speed: code takes about 1% more cycles
  on the 6809 and 2-4% more on the 6309 than before it.
* NitrOS-9 up to its current release, v3.3.0, cuts a file name short at the
  first character it does not accept, without an error, so a program can open
  or delete the wrong file.  NitrOS-9's development branch now reports the
  error, but still ends a name at a character the shell uses, such as `#`,
  `;` or `|`.  Names made of letters, digits, `_` and `.` are safe on every
  version.
* There is no Windows bundle.

## 1.0-rc3

The third candidate for llvm-mc6809 1.0, the first release of a C compiler
for the Motorola 6809 and the Hitachi 6309 built on LLVM and clang.

### A toolchain in one archive

Unpack it anywhere and compile: there is no linker script to write, no include
path and no cross file.  Bundles are built for Linux (x86-64 and arm64) and
macOS (Apple silicon), and can be moved or renamed after unpacking.

```sh
mc6809-clang hello.c -o hello.elf          # bare metal
mc6809-os9-clang hello.c -o hello          # an OS-9 / NitrOS-9 module
mc6809-decb-clang hello.c -o HELLO.BIN     # a CoCo LOADM binary
mc6809-run hello.elf                       # run it on a simulator
```

`mc6809-run` picks a simulator to suit what was built.  The simulators, USim
and MAME, are installed separately.

### Three targets

* **Bare metal**: ELF programs, position-independent by default, or at a
  fixed address with `-fno-pie` for a ROM.
* **OS-9**: program modules, run on NitrOS-9 Level 1 and Level 2, with
  `argc`/`argv`, a heap, and files and directories through the operating
  system.
* **Disk Extended Color BASIC**: `LOADM` binaries for the Color Computer.

### Both processors

`-mcpu=hd6309` generates code for the 6309's extra registers and
instructions, such as its W and Q registers and its block moves.  Each target
has a 6309 C library as well as a 6809 one, and a bare-metal 6309 program
starts the processor in native mode.

### A C library, and floating point

* **picolibc**, built for each target and processor.  The default library is
  integer-only, which is smaller and faster.  On bare metal and OS-9,
  `-mlibc=float` selects one that can format floating point and do maths.
* **IEEE 754 floating point**, single and double precision, from Motorola's
  MC6839 floating-point ROM, written in 1980 and since placed in the public
  domain.  It is linked into the program, or loaded as the `FPO9` module on
  OS-9.
* **C++** without a standard library: the C library under its C++ names, and
  `new` and `delete`.  Exceptions and RTTI are off.

### How it is tested

Every change is measured on about 280 test programs, each built at 30
combinations of optimisation level, processor, library and target, plus about
320 compiler tests.  The programs run on simulators of each machine, and on
NitrOS-9 itself booted in a simulator.  Each bundle in this release was
checked again after unpacking, somewhere it was never built.

### Since rc2

* Built on upstream LLVM as of late October 2026, by way of llvm-mos.  That
  exposed about ten ways to stop the compiler or the link, and one start-up
  crash, all fixed.
* Bare-metal 6309 programs start in native mode.  Define
  `PICOLIBC_MC6809_EMULATION_MODE` to keep emulation mode.
* An OS-9 module named from its output file keeps only the characters every
  NitrOS-9 version accepts, so `-o my-prog` makes a module called `myprog`.

### Known issues

* Code takes more cycles than it did before the LLVM update: about 1% on the
  6809 and 2-4% on the 6309.
* NitrOS-9 up to its current release, v3.3.0, cuts a file name short at the
  first character it does not accept, without an error, so a program can open
  or delete the wrong file.  NitrOS-9's development branch now reports the
  error, but still ends a name at a character the shell uses, such as `#`,
  `;` or `|`.  Names made of letters, digits, `_` and `.` are safe on every
  version.
* There is no Windows bundle.
