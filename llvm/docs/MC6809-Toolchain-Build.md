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
  that is always there.
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

Each triple gets one library, and bare metal gets a second built for the
6309, which `-mcpu=hd6309` selects:

```
lib/clang-runtimes/mc6809-unknown-unknown/lib/          plain 6809
lib/clang-runtimes/mc6809-unknown-unknown/hd6309/lib/   HD6309
```

`multilib.yaml` names the directories and the flags that choose them.  Its
rules deliberately do **not** name the triple: the file already lives inside
a sysroot for one, and the short tool names compute different spellings of it
(`mc6809-clang` computes `mc6809`, not `mc6809-unknown-unknown`), so a rule
naming the triple would match none of them.

Adding a variant is a build and two lines of YAML — but each one is another
full picolibc build in the roll and another thing to be sure of, so they
should earn their place.  The flags a rule can match on are the target, the
relocation model, `-mcpu=` and whether the link is doing LTO.

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

This is **not** the picolibc test suite, and cannot be: that suite links the
`libc.a` it has just built, so it exercises the compiler and never the
installed library.

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

## What is not done

* **Nothing runs DECB.** The format, the start-up code and the library are
  written and link; no machine has executed any of it.
* **No packaging**: no tarball, no Homebrew formula, no installer.  A bundle
  is a directory; `tar` is left as an exercise until somebody wants one.
* **One optimisation level.** Every library is built `-Os`.  An LTO variant
  would be two lines of YAML and another build; nobody has measured whether
  it is worth it.
