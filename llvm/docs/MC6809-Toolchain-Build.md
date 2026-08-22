# Rolling and installing an MC6809 toolchain

How to turn this source tree into something a person can unpack and use, and
how to know whether what came out works.  For using the result, see
[MC6809-Toolchain-Usage.md](MC6809-Toolchain-Usage.md).

## What a bundle is

```
<prefix>/bin/                                 clang, lld, the llvm tools, and
                                              mc6809-* names for all of them
<prefix>/lib/clang/<ver>/                     compiler-rt, the OS-9 and DECB
                                              start-up code, their linker
                                              scripts, the MC6839 FP module
<prefix>/lib/clang-runtimes/<triple>/include  picolibc headers
<prefix>/lib/clang-runtimes/<triple>/lib      libc.a and its system layer
<prefix>/lib/clang-runtimes/<triple>/multilib.yaml   which library a link gets
<prefix>/lib/clang/<ver>/include/c++/          <cstdio> and friends: the C
                                              library under its C++ names
<prefix>/share/doc/mc6809/                    the guides, including the note
                                              on the MC6839 ROM
<prefix>/README.md                            what it is, in one page
<prefix>/VERSION                              the compiler's own version line
```

Three triples are built:

| triple | what it is | library |
|---|---|---|
| `mc6809-unknown-unknown` | bare metal, on a simulator or hardware | picolibc + `libsemihost` |
| `mc6809-unknown-os9` | an OS-9 / NitrOS-9 program module | picolibc + `libos9` |
| `mc6809-unknown-decb` | a CoCo Disk Extended Color BASIC binary | picolibc + `libdecb` |

**Nothing is baked in.** The driver works out where its sysroot, resource
directory and linker scripts are from the path of the binary that was
invoked, so a bundle can be moved, renamed or unpacked anywhere.  That is
also why the tools are copied and symlinked rather than wrapped: a shell
script that execs a clang from somewhere else sends the driver looking for a
sysroot beside *that* clang.

## Prerequisites

* An LLVM build of this tree.  **Use a Release build for anything anyone else
  will run**: the development Debug tree produces a bundle several times the
  size and a slow compiler, and is the default only because it is the one
  that is always there.  Configure a release build from its own cache file,
  which sets what differs from a working tree — Release, no assertions, no
  `git` on the version, and the release number.  **From the repository
  root**, since the paths are relative to it:

  ```sh
  cmake -C clang/cmake/caches/MC6809-Release.cmake -G Ninja -S llvm -B build-release
  cmake --build build-release --target all -j $(sysctl -n hw.ncpu)   # or -j N
  ```

  `build-release` at the root is git-ignored (`/build*`), as is
  `llvm/cmake-build-*`; `llvm/build-release` is **not**, and will litter
  `git status`.

  Measured on an eighteen-core Apple Silicon Mac, from nothing: **14 minutes
  at `-j 8`, 9 minutes at `-j 18`**, and a 2.6 GB build tree.  What comes out
  of it is a **234 MB** bundle, **45 MB** packed.

* A picolibc checkout beside this one (`../picolibc` by default), with meson
  and ninja available.
* For checking the result: `usim09batch` and `usim09pt` on the PATH, and a
  NitrOS-9 image tree (`NITROS9_RECIPES`, default
  `~/Documents/NitrOS-9/nitros9/recipes/picothing`).
* Optionally, for the DECB cases: a CoCo-capable MAME
  (`MC6809_MAMECOCO3=/path/to/mamecoco3-headless`) with Tandy's ROMs —
  including `coco3h.zip`, which MAME will not take from `coco3.zip` though
  the ROM is the same — and ToolShed's `decb` to write a Disk BASIC floppy.
  Without them those two cases say so rather than passing.


## Which version this is

Every tool answers `--version`, and the answer should say which llvm-mc6809
it is — LLVM's own version says which LLVM we forked and nothing about the
6809 work.  A release build reports:

```
clang --version     llvm-mc6809 clang version 24.0.0 (llvm-mc6809 1.0-rc1 <rev>)
ld.lld --version    llvm-mc6809 LLD 24.0.0 (<repository> <rev>)
llc --version       llvm-mc6809 LLVM version 24.0.0
```

The vendor — `llvm-mc6809` — comes from `PACKAGE_VENDOR`, which reaches all
three: clang and lld copy it into their own `CLANG_VENDOR` and `LLD_VENDOR`
the first time a tree is configured, and LLVM's tool printer uses it in place
of the `LLVM (http://llvm.org/):` line.

**The vendor is a name and must never contain a version number.**  Build
systems take the first dotted number in `clang --version` to be the
compiler's version — meson's `search_version` does exactly that — so
`llvm-mc6809 1.0 clang version 24.0.0` makes every meson project believe it
is looking at clang 1.0 and refuse to build (`ERROR: None of values ['c18']
are supported by the C compiler`).  That is why `Ubuntu clang version 18.1.3`
and `Apple clang version ...` are names.  The release number therefore goes
*after* the version, in the parentheses, through
`CLANG_REPOSITORY_STRING`.

Only clang carries the number.  `LLVM_FORCE_VC_REPOSITORY` would give it to
lld and the llvm tools as well, but forcing the repository without also
forcing the revision leaves the revision undefined and both tools then print
no revision at all — a bad trade, since the number is also in
`<prefix>/VERSION` and the bundle README while the revision is the only thing
that says which source built it.

**Editing it for a candidate**: one line in `MC6809-Release.cmake`,
`MC6809_RELEASE`, which reads `1.0-rc1`, then `1.0-rc2`, then `1.0`.  A
candidate is built exactly like the release it is a candidate for, so it is
the only thing that changes between them.

**A one-off build with a different label** cannot use `-DMC6809_RELEASE=`:
cmake reads the `-C` file before it applies any `-D`, so the string derived
from that line is already fixed.  Set the derived string directly:

```sh
cmake -C clang/cmake/caches/MC6809-Release.cmake \
    -DCLANG_REPOSITORY_STRING="llvm-mc6809 1.0-smoke" \
    -G Ninja -S llvm -B build-release
```

`-DPACKAGE_VENDOR=` does **not** do this job, and fails in the worst way: the
cache forces `CLANG_VENDOR`, so clang goes on reporting whatever
`MC6809_RELEASE` said while `llc` reports the `-D` value — a split identity,
with the meson-breaking shape on one side, and a bundle named after the wrong
release, since the roll script takes the version from clang.

A release build also drops LLVM's `git` suffix, so it reports `24.0.0` rather
than `24.0.0git`.  The suffix says "this is not a release of LLVM", which is
true of a working tree and noise on a release of this.  It returns by itself
in any development build, which uses `MC6809.cmake` and never reads the
release cache.

## Rolling one

```sh
scripts/roll-toolchain.sh --prefix ~/mc6809 \
    --llvm-build build-release \
    [--picolibc /path/to/picolibc] \
    [--tarball DIR]
```

The prefix is any directory you can write to; it is created.  **Pass
`--llvm-build` for anything you intend to ship**: without it the script uses
the development Debug tree, which is exactly what the prerequisites tell you
not to hand to anybody.  `--picolibc` defaults to `../picolibc`.

It stages the tools and the compiler's runtime directory, generates cross
files pointing at the staged compiler, builds picolibc for each triple **with
that compiler**, installs it with picolibc's own rules, writes the
`multilib.yaml` that chooses between library variants, and then checks the
result by compiling and running programs against it.  It exits non-zero if
those do not run.

From a release tree the roll takes about fifty seconds, nearly all of it the
eight picolibc builds, and the check that follows it another twenty; from the
development tree, several times that, because a Debug clang compiles slowly.

### Packaging

`--tarball DIR` turns the rolled prefix into something you can hand over, and
runs **only after the check has passed** — an archive of a bundle nobody has
run is not a release, it is a large file.  It:

* stages the guides into `<prefix>/share/doc/mc6809/`, so a bundle explains
  itself away from this source tree;
* writes `<prefix>/README.md` and `<prefix>/VERSION`, taking the version from
  the compiler's own banner rather than from a number kept in the script;
* packs `DIR/mc6809-toolchain-<version>-<host>.tar.xz`, which unpacks into a
  directory named for what it is whatever the prefix was called, with `xz`
  using every core;
* writes `DIR/mc6809-toolchain-<version>-<host>.tar.xz.sha256` beside it.

`xz` must be on the `PATH`; macOS `tar` will not make an `.xz` by itself.

**Then check the archive, not the prefix it came from.**  The roll checks the
directory it built; what you hand over is the tarball, and the two are only
the same if nothing went wrong in between.  Unpack it somewhere it was never
built and run the check there, which tests the archive's completeness and the
bundle's relocatability at once:

```sh
shasum -a 256 -c mc6809-toolchain-1.0-rc1-darwin-arm64.tar.xz.sha256
tar -xf mc6809-toolchain-1.0-rc1-darwin-arm64.tar.xz -C /tmp/unpacked
picolibc/scripts/check-mc6809-toolchain --corpus \
    /tmp/unpacked/mc6809-toolchain-1.0-rc1-darwin-arm64
```

### Library variants

The two targets with a machine to run on get four libraries each — the
processor and the floating-point question are independent, so the matrix is
2×2:

```
lib/clang-runtimes/<triple>/lib/          plain 6809, integer-only
lib/clang-runtimes/<triple>/hd6309/       HD6309, integer-only
lib/clang-runtimes/<triple>/fp/           plain 6809, floating point
lib/clang-runtimes/<triple>/hd6309-fp/    HD6309, floating point
```

DECB has the processor variant but not the floating-point one: there the
MC6839 ROM is linked into the program, and 8 KB of it in something that must
fit under BASIC is a decision to take deliberately rather than to ship four
ways.  A 6309 DECB program runs on MAME's `coco3h`, the CoCo 3 with the chip
swapped.

`-mcpu=hd6309` and `-mlibc=float` choose between them.  A variant directory
holds its libraries directly, with its headers in `include/` beside them —
the shape clang looks for when `multilib.yaml` names that directory.  Only a
variant whose headers really differ keeps a copy of them: the 6309 library is
the same C library built for another processor, so a second set would be a
megabyte saying so, while a floating-point one differs in the line of
`picolibc.h` that decides what `printf` can format.  DECB has the processor
variant but not this one: there the MC6839 ROM is linked into the program,
and 8 KB inside something that must fit under BASIC is a choice to make
deliberately.

`multilib.yaml` names the directories and the flags that choose them.  **Each
sysroot with a variant needs its own copy**: clang reads the file from the
sysroot it picked for the triple, and the file says nothing about which
triple that was.  Its
rules deliberately do **not** name the triple: the file already lives inside
a sysroot for one, and the short tool names compute different spellings of it
(`mc6809-clang` computes `mc6809`, not `mc6809-unknown-unknown`), so a rule
naming the triple would match none of them.

Adding a variant is a build and two lines of YAML — but each one is another
full picolibc build in the roll and another thing to be sure of, so they
should earn their place.  The flags a rule can match on are the target, the
relocation model, `-mcpu=` and whether the link is doing LTO.

### Adding a variant

A variant is a picolibc build plus a rule.  The floating-point ones are three
meson options — `-Dstdio-float=true`, `-Dwant-libm=true` and, easy to miss,
`-Dformat-default=double`, without which the float-capable code is built but
plain `printf` still resolves to the integer one — and a `multilib.yaml`
entry naming the flags that select them.

What made this one more than a build is worth knowing before adding the next:

* **A rule can only match on flags clang produces.** `getMultilibFlags` gives
  the target and the relocation model; `getMC6809MultilibFlags` adds `-mcpu`,
  `-flto`/`-fno-lto` and `-mlibc`.  Nothing clang works out for itself says
  whether a library can format a double, so the flag has to be asked for
  outright — hence `-mlibc=`, beside the `-mcrt0=` that picks a start-up.
* **Headers can differ, and here they do.** clang's multilib mechanism
  switches the library directory; the include path comes from the sysroot.
  `MC6809ToolChain::AddClangSystemIncludeArgs` now puts a selected variant's
  own `include/` ahead of the default, which is what makes `picolibc.h` —
  and so plain `printf` — come from the right build.

Two things that surprise people:

* **No `-lm`.**  picolibc's `libm.a` is an empty archive; the maths lives in
  `libc.a`, and the link works without it.
* **Size.**  A hello-world that prints two doubles came to 59,964 bytes of
  the 64 KB address space.  Floating point is affordable on this machine only
  in small doses.

### C++

There is no C++ standard library and no plan for one.  What the bundle
carries is `libclang_rt.cxx.a` per triple — the operator new and delete
family over malloc, `__cxa_pure_virtual`, and the guard functions for
function-local statics — and a directory of headers that give the C library
its C++ spelling.  Both are built by
`compiler-rt/lib/builtins/mc6809-cxx/`; the runtime is compiled freestanding,
since compiler-rt is built before a C library exists to include from.

Exceptions and RTTI are off by default, in `addExceptionArgs` and
`CalculateRTTIMode` where clang keeps such decisions, so that what is missing
is refused where it is written instead of at link time.

### The MC6839 ROM

Floating-point *arithmetic* does not come from picolibc at all: it comes from
Motorola's MC6839 floating-point ROM, and the bundle already carries it in
both of the forms it is needed in.

* **Bare metal and DECB**: the 8 KB ROM is assembled into the compiler's
  runtime (`compiler-rt/lib/builtins/mc6809/mc6839_rom.S`) and linked into
  the program, but only if the program does floating point at all — it is
  reached through a weak reference.  Nothing extra to install.
* **OS-9**: the same ROM as a loadable module, `FPO9`, staged in the
  compiler's resource directory (`lib/clang/<ver>/lib/mc6809-unknown-os9/`).
  A program links it at start-up, so **it has to be on the target machine**
  as well as in the bundle.

The ROM is Motorola's, written by Joel Boney in 1980 and documented as
released into the public domain by Motorola in 1988.  The provenance, the
chain of custody and the exact bytes are in
[MC6809-Third-Party.md](MC6809-Third-Party.md), which every bundle carries.

## Checking one

```sh
picolibc/scripts/check-mc6809-toolchain ~/mc6809
```

Twenty-five cases with a CoCo to hand, twenty-three without, and it prints
its own tally.  It compiles ordinary
programs — stdio, malloc, string, 32-bit arithmetic — with nothing but
`--target`, and runs each: bare metal on USim, OS-9 on a real NitrOS-9 boot.
Then each library variant twice over, that the link uses the variant's own
directory *and* that the program runs, since a default library would often
link and run anyway.  Then C++: one program per target using a global with a
constructor, `new`, virtual dispatch, a virtual destructor, a local static, a
template and the C++ headers all at once — each of those is easy to leave out
of a runtime and hard to notice missing — and that a `try` is refused at
compile time, since the alternative is a link failing on
`__cxa_allocate_exception`.  Then that DECB comes out in a LOADM envelope,
that a build system reading `clang --version` gets the compiler's version
rather than the release number, and that all six `mc6809-*-clang` and
`-clang++` names work.

Nothing in it names a library, a linker script or an include path: if the
driver cannot find its own sysroot, it fails.  The short names are a test in
their own right, which is why all six are built through — each computes its
own spelling of the triple, and both the sysroot and the resource directory
are named after that spelling.

This is **not** the picolibc test suite as that suite is normally run: a
normal run links the `libc.a` it has just built, so it exercises the compiler
and never the installed library.  `--corpus` closes the gap from the other
side — it takes the suite's programs as *source* and compiles each against
the bundle with one plain clang command:

```sh
picolibc/scripts/check-mc6809-toolchain --corpus ~/mc6809
```

Seventeen of them run and pass, four report picolibc's own "skip", and four
cannot link — the same on a Debug-rolled bundle and a release one:

```
not in this libc: test-regex(regcomp) test-time(__d_vfprintf)
                  test-time2(__d_vfprintf) test-wcsftime(wcsncmp)
```

That line is the point of the mode.  A link that fails **only** for a missing
symbol is a statement about what the bundle was built with — the four above
are the POSIX extensions, floating-point `printf` and wide characters, all
three switched off deliberately.  Any other build failure — a header not
found, no `libc` at all — is reported as a failure, because that is what a
broken bundle looks like.  The corpus runs bare metal only; the OS-9 half of
the library is covered by the bench's `os9` levels and the runtime suite.

If the simulators are not on the `PATH` the check stops and says which one is
missing.  It does not report the cases it could not run as failures, and does
not report them as passes either.

DECB is built and its LOADM envelope inspected, but not run — nothing here
can run a CoCo program, and the check says `not run` rather than implying
otherwise.

## Testing the wider suites against a bundle

`MC6809_TOOLCHAIN` points the picolibc harness and the bench at an installed
prefix instead of a build tree:

```sh
MC6809_TOOLCHAIN=~/mc6809 picolibc/scripts/gen-mc6809-cross.sh
MC6809_TOOLCHAIN=~/mc6809 picolibc/scripts/bench-parallel.sh --levels Os
```

Everything else follows from that: the harness takes its compiler from
meson's own record of the build, and the OS-9 runner finds the FP module by
asking the compiler where its resource directory is.

## What is deliberately outside the bundle

**The simulators.**  `mc6809-run` dispatches on a program's magic bytes and
runs it, but it runs it on `usim09batch` or `usim09pt` found on the `PATH`,
and neither is shipped here.  They are somebody else's programs with their own
build and their own release; copying binaries of them into our tarball would
make us the people who ship a stale USim.  The check says which one is missing
when it cannot find them, and the usage guide names where they come from.

## What is not done

* **DECB needs a CoCo you supply.**  The check runs it on both processors
  when one is available — set `MC6809_MAMECOCO3` to a `mamecoco3-headless`
  — and reports "not checked" when it is not, since a CoCo needs Tandy's
  ROMs and a separate MAME build, neither of which can ship here.  Without
  them the envelope is still verified.
* **No Homebrew formula, no installer, no signing or notarisation.**  There
  is a tarball and a checksum (see [Packaging](#packaging)); everything
  beyond that is unbuilt, and on macOS an unsigned binary downloaded from
  elsewhere will need Gatekeeper talked round.
* **One optimisation level.** Every library is built `-Os`.  An LTO variant
  would be two lines of YAML and another build — `-flto` is already among the
  flags a rule can match on — but nobody has measured whether it is worth it.
* **The default libraries are integer-only**, and the POSIX extensions, wide
  characters and `%lld` are absent from every variant: `-Dposix-extensions=
  false -Dmb-capable=false -Dio-long-long=false`.  Floating point is the one
  that has a variant and a flag; the rest would each want the same treatment,
  and none has been asked for.
