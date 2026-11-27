# llvm-mc6809 release notes

Each release has a section headed `## <version>`, which the release workflow
publishes on that release's GitHub page above the table of test results.  A
tag with no section here is not published.

## 1.1.0-rc3

The third candidate for llvm-mc6809 1.1.0.  On OS-9, `'\n'` is now a
carriage return, the platform's line separator, and the C library writes
text lines the way OS-9 expects; the linker no longer corrupts pointers in
an OS-9 module built with debug information; and `-mlto-dp`, which could
miscompile, is refused until it is finished.  The driver also finds the
right library however the processor is spelled, DECB refuses a
floating-point library it does not have, and the documentation, the
bundle's own README included, has been checked against the code.  Apart
from the OS-9 newline, the code the compiler generates is unchanged from
rc2: every program the bench runs on bare metal takes exactly as many
cycles as it did.

**Upgrading from rc2**: rebuild OS-9 programs.  Their `'\n'` is now a
carriage return, so code that wanted a real line feed should say `'\l'`,
and code that compared a character with both `'\n'` and `'\r'` now tests
the same value twice: a `switch` with both as `case` labels no longer
compiles.  Rebuilding also relinks any OS-9 module that was linked with
debug information (`-g`), which the linker fix below needs.  Otherwise
nothing changes in how a program is built, unless it was built for DECB
with `-mlibc=float`.  That never gave a floating-point C library; it now
says so and stops, so drop the flag.  The same goes for `-mlto-dp` and
`-mreserve-dp`, which are now refused: see below.

### Changed behaviour

* For the OS-9 target, `'\n'` is a carriage return ($0D) in character and
  string literals of every width, as in OS-9's own C compilers: SCF ends a
  line on CR, `I$ReadLn` and `I$WritLn` work in CR-terminated records, and
  text files hold CR.  `'\r'` is a carriage return too, and the new
  `'\l'` names a line feed ($0A).  The C library is built the same way, so
  its lines end in CR with no translation.  `-fno-os9-newline` keeps `'\n'`
  a line feed; `-fos9-newline` asks for the OS-9 newline on another
  target.  An `asm` string keeps its line feeds, since it is assembler
  source.
* `-mlto-dp` and `-mreserve-dp`, which gave a link-time-optimised program
  bytes of the direct page for the compiler to use, are refused with an
  error.  Given those bytes, the compiler could move a function's stack
  slots into the direct page but go on addressing them on the stack, past
  the function's own frame, so the program silently overwrote its caller's
  stack (or, in an interrupt handler, the interrupted code's).  It could
  also link the same input into different programs from one run to the
  next.  They will come back once the direct-page allocation is finished.
* On OS-9 the C library writes a text stream with `I$WritLn`, so an SCF
  device applies its path's own settings, such as the line feed it adds
  after each carriage return, and a file on disk gets every byte as written.
  A stream opened with `b` in its mode, a stream with its own write
  function, and `write` itself still use `I$Write`.  Nothing in the library
  translates CR or LF.
* For the OS-9 target the compiler's rewrites of `printf` into `puts` and
  of `puts("")` into `putchar` follow the C library's newline, a carriage
  return, so they keep doing what the call would have done.

### Fixed

* An OS-9 module linked with debug information could hold wrong pointers in
  its initialised data.  The linker makes each such pointer relative to the
  data area or the module, so that the start-up code can rebase it, but a
  debug section could hide the word, which was then left as a raw address;
  the program saw a pointer to the wrong place, with no error.  Whether a
  word was hit depended only on the link's layout.  In the C library this
  silently lost everything written to `stderr` in some programs, and a
  function pointer could as well have jumped into nothing.  Every such
  pointer is now converted, and one the linker cannot place is an error.
  Bare-metal and DECB programs were not affected.
* `freopen` on a stream made with `funopen` passed the stream's cookie to
  `close` as though it were a file descriptor, instead of calling the
  stream's own close function, and left the stream marked as a cookie
  stream once it had been reopened on a file.  Every target was affected.
* `-mcpu=6809` found no library: it is accepted as the 6809, but the name
  went to the library rules as written, matched none of them, and the link
  failed.  Every spelling of each processor now chooses the same library.
* `-mlibc=float` for DECB quietly linked the integer C library, whose
  `printf` prints nothing for `%f`; there is no floating-point library for
  DECB.  It is now an error that says so.
* The bundle's README now shows the bare-metal example with
  `-mcrt0=semihost`, without which a program never ends on a simulator, and
  says what an OS-9 run needs and that nothing in a Windows bundle runs a
  program.  These release notes now ship in the bundle, beside the guides.

### Documentation

The guides were audited against the code and corrected throughout.  Among
what was wrong: the OS-9 guide said the heap could not grow and gave the
stack as 512 bytes (it grows, and the stack is 2 KB); the build guide
described the Windows bundle as a `.zip` that had never been tried; the
third-party note said an integer-only C library keeps the MC6839 ROM out of a
program, which only doing no floating-point arithmetic does.  The usage guide
now covers running an OS-9 module, the bare-metal memory map, debugging,
Windows, and the supported optimisation levels.

### How it is tested

As for rc2, with one change: each Linux and macOS bundle must now pass all 48
cases of its acceptance check that a runner can run, where before 39 were
enough.

Before this candidate was cut, the compiler and C library were run at every
level the bench covers, 20 on the 6809 and 8 on the 6309 under USim and the
three OS-9 builds on NitrOS-9 Level 1 and Level 2, with no failure and no
time-out, and the compiler's own 424 tests passed.  Every test that passed
in rc2's run passed again, two new tests of OS-9 text output passed at each
OS-9 level, and no bare-metal program's cycle count changed.

### Against 1.0, like for like

Measured on the rc2 compiler and C library, against 1.0's, on the test
programs whose own sources are unchanged since 1.0 (82 to 86 at each level,
counting those that pass in both).  rc3 gives exactly the same cycle counts,
so these figures hold for it. The C library's own changes are part of
what is measured. Both are measured on USim, the 6309 included.

| | Cycles in all | A typical program |
|---|---|---|
| 6809, `-O0` to `-Oz` | 12-20% fewer | 16-24% fewer |
| 6309, `-O2`, `-Og` and `-Os` (the 6309 levels the bench runs) | 8-13% fewer | 15-22% fewer |

"A typical program" is the geometric mean of each program's change, which a
few long-running tests do not dominate.

At every level at most three of those programs are slower, and at `-O0`,
`-Os` and `-Oz` on the 6809 none is. The one that is slower at most levels
is the test of the MC6839 floating-point calls, by 1-3% on the 6809 and 4-8%
on the 6309: those calls now find the ROM at run time, so they work wherever
the program is loaded. The others are slower by less than 3%, and only on
the 6309.

### Known issues

* NitrOS-9 up to its current release, v3.3.0, cuts a file name short at the
  first character it does not accept, without an error, so a program can open
  or delete the wrong file. Names made of letters, digits, `_` and `.` are
  safe on every version. This is fixed on NitrOS-9's `main` branch, after
  v3.3.0, though not yet in a release.
* On NitrOS-9 up to v3.3.0, a file open twice can hide what one path wrote
  from the other. On Level 1 the second path sees only what has reached the
  disk, and the file's size as it was when that path opened, and `fflush`
  does not change this; on Level 2 the same happens after a write of a whole
  sector. This too is fixed on NitrOS-9's `main` branch: a system built from
  it shows every path of a file what the others wrote, on both levels.
* NitrOS-9 Level 2 built from its `main` branch between 23 and 27 November
  2026 hangs just after it starts: a program that chains to a smaller one,
  as the start-up program does to the shell, fails.  Build from `main` as it
  is now, which has the fix.
* The Windows bundle has had much less testing than the others; treat it as
  experimental. Programs built on Windows have to be run somewhere else: on a
  simulator on another host, or on real hardware.
* There is no debugger in the bundle yet.

## 1.1.0-rc2

The second candidate for llvm-mc6809 1.1.0: a debugger can now follow the
stack through any code in the toolchain, and there is a Windows bundle.
The code the compiler generates is unchanged from rc1.

### Debugging

A debugger attached to a program (lldb or gdb, through a simulator's remote
stub) unwinds the stack by the call-frame information (CFI) the compiler and
assembler write. That information is now right everywhere, so a backtrace
shows the true chain of callers from any instruction:

* in compiled code at every optimisation level, with or without a frame
  pointer, including while a function pushes arguments or temporaries;
* in every hand-written routine in the C library, the compiler's runtime and
  the start-up code, which had none before;
* in interrupt handlers, which describe the frame the processor stacked on
  entry (12 bytes for an IRQ, NMI or SWI; 14 on a 6309 in native mode; 3 for
  a FIRQ), so a backtrace continues into the code that was interrupted;
* across saved registers: the CFI names the program counter as the return
  address and says where each preserved register was pushed, so a debugger
  shows a caller's own values, not the callee's.

The program's entry point is marked as the outermost frame, so a backtrace
ends there cleanly. Every bundle is checked for all of this before it is
packaged: a tool follows every path through every routine in its libraries
and start-up objects, and refuses a bundle whose CFI is wrong anywhere.

lldb itself is not yet part of the bundle.

### Windows

There is now a bundle for Windows on x86_64 as well as for Linux and macOS.
It holds the same compiler, tools and libraries.

**The Windows bundle has had much less testing than the Linux and macOS
ones; treat it as experimental.** Those are built from the same sources the
full benchmark suite runs against, thousands of programs run to completion
at 31 combinations of optimisation level, processor, library and target,
and their bundles run every test program on a simulator before release. On
Windows no program is run at all, since there is no simulator for Windows
yet: the compiler's own tests are run, and every test program is compiled
and linked, but nothing checks that what it builds works. It has been
through two complete runs of its checks, both shortly before this release.

A release does not wait for it, so if the Windows bundle fails its checks,
the release is published without it.

### Fixed

* A data word holding a distance to a symbol in another section, such as
  `.word target - .`, was linked as the target's address instead of the
  distance. This also made every `.eh_frame` entry (written with
  `-funwind-tables`) name the wrong code.
* The CoCo (DECB) start-up code saved and restored BASIC's stack through
  absolute addresses; it is now position-independent, like the code the
  compiler generates.

### Assembler and tools

* The assembler accepts Motorola's `,pcr` spelling of PC-relative indexing,
  as well as `,pc`. With `,pcr` the operand is the address to reach, as in
  Motorola's assembler, rather than the offset.
* `llvm-objdump` names a backward branch's target after the symbol it is in,
  and prints signed PC-relative offsets as signed.
* The assembler rejects an unknown register name in a CFI directive instead of
  writing a wrong register number.

### How it is tested

As for rc1. In addition, the compiler's own tests that need no simulator now
run on every host a bundle is built for, Windows included, and each bundle's
CFI is checked as above.

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
