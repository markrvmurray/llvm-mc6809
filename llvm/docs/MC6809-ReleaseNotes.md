# llvm-mc6809 release notes

Each release has a section headed `## <version>`, which the release workflow
publishes on that release's GitHub page above the table of test results.  A
tag with no section here is not published.

## 1.1.0-rc1

The first candidate for llvm-mc6809 1.1.0: faster and smaller code, wrong
code fixed, and interrupt handlers.

### Smaller and faster

Against 1.0, the C library for the 6809 at `-Os` is 22% smaller (146,820
bytes of code, from 188,946), and the test programs take 17-20% fewer cycles
on the 6809 at every level measured (`-O0`, `-O2`, `-Os`, `-Os` with LTO and
with floating point).

* Values kept in the direct-page registers are no longer saved and restored
  around every use that borrows an accumulator: a register is saved only
  where its value is read again.
* Fewer redundant loads and stores of the same memory, by one set of rules
  shared by the passes that remove them.
* `clr` in place of loading zero, and on the 6309 `aim`, `oim` and `eim`
  in place of loading, changing and storing a byte; `inc` and `dec` on
  memory on both processors.
* An 8-bit by 8-bit multiply uses `MUL`; epilogues end in `puls ...,pc`;
  `strcmp` works a byte at a time.

### Wrong code fixed

* On the 6309, sign-extending the low part of a loaded `long` read its high
  bytes.
* At `-O0`, a branch on the borrow of a subtraction whose result was unused
  tested a stale carry.
* A reload could be removed, or a value reused, across a store that wrote
  the same memory: a store through another pointer that may alias it, a
  store that overlapped it only in part, a 6309 `stq` or `sts`, or an
  access whose offset is a symbol (`tbl,x`) taken for one at offset 0.

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

* A program can declare the least heap it needs (`-Wl,--defsym=__heap_size_min=N`);
  if the heap it gets is smaller, the link fails rather than the program.

### Known issues

* NitrOS-9 up to its current release, v3.3.0, cuts a file name short at the
  first character it does not accept, without an error, so a program can open
  or delete the wrong file. Names made of letters, digits, `_` and `.` are
  safe on every version.
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
