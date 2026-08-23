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
  will run**: the development Debug tree produces a bundle of about 4 GB and
  a slow compiler.  The Debug tree is the default only because it is the one
  that is always there.  Configure a release build from its own cache file,
  which sets the three things that differ from a working tree:

  ```sh
  cmake -C clang/cmake/caches/MC6809-Release.cmake -G Ninja -S llvm -B build-release
  cmake --build build-release --target all -j 8
  ```

## Which version this is

Every tool answers `--version`, and the answer should say which llvm-mc6809
it is — LLVM's own version says which LLVM we forked and nothing about the
6809 work.  One CMake variable, `PACKAGE_VENDOR`, reaches all three banners,
because clang and lld each default their own vendor string from it:

```
clang --version     llvm-mc6809 1.0-rc1 clang version 24.0.0 (repo revision)
ld.lld --version    llvm-mc6809 1.0-rc1 LLD 24.0.0 (repo revision)
llc --version       llvm-mc6809 1.0-rc1 LLVM version 24.0.0
```

The last replaces LLVM's `LLVM (http://llvm.org/):` line rather than adding
to it.

The number lives in exactly two lines of CMake, one per cache file:
`MC6809.cmake` says `1.0-dev`, and `MC6809-Release.cmake` says which
candidate or release this is — **edit that line for each one**: `1.0-rc1`,
`1.0-rc2`, then `1.0`.  A candidate is built exactly like the release it is a
candidate for, so it is the only thing that changes between them.
`-DPACKAGE_VENDOR=...` overrides it for a one-off, because `-D` is processed
after `-C`.

A release build also drops LLVM's `git` suffix, so it reports `24.0.0` rather
than `24.0.0git`.  The suffix says "this is not a release of LLVM", which is
true of a working tree and noise on a release of this; clang and lld still
print the repository and the revision they were built from.  It returns by
itself in any development build, which uses `MC6809.cmake` and never reads
the release cache.
* A picolibc checkout beside this one (`../picolibc` by default), with meson
  and ninja available.
* For checking the result: `usim09batch` and `usim09pt` on the PATH, and a
  NitrOS-9 image tree (`NITROS9_RECIPES`, default
  `~/Documents/NitrOS-9/nitros9/recipes/picothing`).

## Rolling one

```sh
scripts/roll-toolchain.sh --prefix /opt/mc6809 \
    [--llvm-build /path/to/llvm-release-build] \
    [--picolibc /path/to/picolibc]
```

It stages the tools and the compiler's runtime directory, generates cross
files pointing at the staged compiler, builds picolibc for each triple **with
that compiler**, installs it with picolibc's own rules, writes the
`multilib.yaml` that chooses between library variants, and then checks the
result by compiling and running programs against it.  It exits non-zero if
those do not run.

From the development tree it takes about two minutes.

### Library variants

Each triple gets one library, and the two targets with a machine to run on
get a second built for the 6309, which `-mcpu=hd6309` selects:

```
lib/clang-runtimes/mc6809-unknown-unknown/lib/          plain 6809
lib/clang-runtimes/mc6809-unknown-unknown/hd6309/lib/   HD6309
lib/clang-runtimes/mc6809-unknown-os9/lib/              plain 6809
lib/clang-runtimes/mc6809-unknown-os9/hd6309/lib/       HD6309
```

A 6309 variant only skips the headers, not the libraries: they are the same
headers, and a second copy is a megabyte saying so.  DECB has no variant —
nothing has run the first library, so a second is premature.

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

### Adding a floating-point variant

The shipped libraries are integer-only, so `printf("%f")` prints `*float*`
and `sqrt` does not link.  A variant that can do both is one picolibc build:

```sh
meson setup /tmp/fp-build ../picolibc \
    --cross-file ../picolibc/scripts/cross-clang-mc6809-unknown-elf.txt \
    -Dprefix=/opt/mc6809/lib/clang-runtimes/mc6809-unknown-unknown-fp \
    --libdir=lib --includedir=include \
    -Dprintf-aliases=false -Dposix-extensions=false -Dsearch-extensions=true \
    -Dmb-capable=false -Dio-long-long=false -Dtests=false -Doptimization=s \
    -Dformat-default=double -Dstdio-float=true -Dwant-libm=true
meson install -C /tmp/fp-build
```

Three options do the work: `-Dstdio-float=true` builds the double-capable
`printf`/`scanf`, `-Dwant-libm=true` builds the maths functions, and
**`-Dformat-default=double` is the one that is easy to miss** — without it
the float-capable code is built but plain `printf` still resolves to the
integer one.  For OS-9, add that triple's options (`-Dpicocrt=false
-Dsemihost=false -Dos-os9=true -Dposix-console=true`) and install into
`mc6809-unknown-os9-fp`.

**It is a separate sysroot, not a multilib directory under an existing one**,
and that is forced rather than chosen: the two builds install *different
headers*.  `picolibc.h` carries `#define __IO_DEFAULT 'd'` where the integer
build has `'i'`, and clang's multilib mechanism switches the library
directory only — the include path comes from `<sysroot>/include`.  On top of
that there is no flag for a rule to match on: `getMultilibFlags` offers
`-mcpu` and `-flto`, and neither describes what `printf` can format.  Making
this a variant the driver selects by itself needs both halves — a driver
option emitted into the multilib flags, and a per-variant include directory —
which is why the bundle does not ship one yet.

Two things that surprise people:

* **No `-lm`.**  picolibc's `libm.a` is an empty archive; the maths lives in
  `libc.a`, and the link works without it.
* **Size.**  A hello-world that prints two doubles came to 59,964 bytes of
  the 64 KB address space.  Floating point is affordable on this machine only
  in small doses.

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

The ROM is a reconstruction from published Motorola source rather than
something written here; whoever publishes a tarball should satisfy themselves
about redistributing it.

## Checking one

```sh
picolibc/scripts/check-mc6809-toolchain /opt/mc6809
```

Compiles ordinary programs — stdio, malloc, string, 32-bit arithmetic — with
nothing but `--target`, and runs each: bare metal on USim, OS-9 on a real
NitrOS-9 boot.  Nothing in it names a library, a linker script or an include
path; if the driver cannot find its own sysroot, it fails.  It then builds
through the short names (`mc6809-clang`, `mc6809-os9-clang`,
`mc6809-decb-clang`), which is a different test again: each computes its own
spelling of the triple, and both the sysroot and the resource directory are
named after it.

This is **not** the picolibc test suite as that suite is normally run: a
normal run links the `libc.a` it has just built, so it exercises the compiler
and never the installed library.  `--corpus` closes the gap from the other
side — it takes the suite's programs as *source* and compiles each against
the bundle with one plain clang command:

```sh
picolibc/scripts/check-mc6809-toolchain --corpus /opt/mc6809
```

Seventeen of them run and pass on a bundle rolled from the Debug tree, four
report picolibc's own "skip", and four cannot link:

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
MC6809_TOOLCHAIN=/opt/mc6809 picolibc/scripts/gen-mc6809-cross.sh
MC6809_TOOLCHAIN=/opt/mc6809 picolibc/scripts/bench-parallel.sh --levels Os
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

* **Nothing runs DECB.** The format, the start-up code and the library are
  written and link; no machine has executed any of it.  The other three
  libraries — bare metal, OS-9, and the 6309 variants of both — are each run
  by the check.
* **No packaging**: no tarball, no Homebrew formula, no installer.  A bundle
  is a directory; `tar` is left as an exercise until somebody wants one.
* **One optimisation level.** Every library is built `-Os`.  An LTO variant
  would be two lines of YAML and another build; nobody has measured whether
  it is worth it.
* **The libraries are integer-only.** `-Dstdio-float=false -Dwant-libm=false
  -Dposix-extensions=false -Dmb-capable=false -Dio-long-long=false`: no
  `libm`, no `%f` in `printf`, no `regcomp`, no wide characters, no `%lld`.
  Floating-point *arithmetic* works — the compiler emits calls into
  compiler-rt, and on OS-9 into the MC6839 ROM — but `printf("%f")` prints
  the literal `*float*`, which is picolibc saying so out loud.
  A floating-point variant is a second full build per triple, and there is no
  obvious flag for clang's multilib rules to select it by: `-mcpu` and
  `-flto` are all `getMultilibFlags` offers, and neither says anything about
  what `printf` can format.  That design question is what stands between here
  and shipping one.
