# MC6809 OS-9 Runtime Tests

An opt-in end-to-end suite for LLVM-generated OS-9 program modules.  Each
test case is compiled with the in-tree Clang for `mc6809-unknown-os9`, put on
a disk, run twice under a real NitrOS-9 boot, and its console output is
checked with FileCheck.  It is the gate for the OS-9 runtime (crt0, linker
script, system-call shims) and for OS-9-specific code generation.

Two ways to boot NitrOS-9 are supported.  The suite is skipped unless one is
fully configured.

## usim09pt (default; no ROMs needed)

NitrOS-9 Level 1 or Level 2 for the Pico-Thing, booted from an IDE image on
`usim09pt`, the test on a second IDE image, the console driven by scripted
input.  A test takes 2–3 s.

```sh
export MC6809_OS9_RUNTIME_TESTS=1
export NITROS9_USIM09PT=/path/to/usim/usim09pt
export NITROS9_BOOT_DSK=/path/to/nitros9/recipes/picothing/l2/NOS9_6809_L2_DEV_picothing_x0.dsk
export NITROS9_FIRMWARE=/path/to/nitros9/recipes/picothing/l2/rel_picothing.srec
llvm/build/bin/llvm-lit -sv compiler-rt/test/mc6809-os9-runtime
```

The boot image must be an IDE (flat) image, not the recipe's default floppy
image; build it in the recipe directory with

```sh
NITROS9DIR=/path/to/nitros9 make \
    DSKIMAGE=NOS9_6809_L2_DEV_picothing_x0.dsk OS9FORMAT_CMD="os9 format -e -l65536"
```

(`l1` for Level 1: same command with the L1 names.)  Knobs:
`NITROS9_INPUT_FIRST` (cycles before the first console byte, default 78M —
the login prompt must be up), `NITROS9_TIMEOUT` (instruction cap),
`NITROS9_WALL_SECONDS` (wall-clock cap, default 300).

## MAME coco3 (needs CoCo 3 ROMs)

```sh
export MC6809_OS9_RUNTIME_TESTS=1
export MAME_ROMPATH=/path/to/coco3/roms
export OS9_BOOT_DSK=/path/to/nitros9-coco3.dsk     # a bootable L2 floppy image
export MC6809_OS9_BACKEND=mame                       # if usim09pt is also configured
```

Optional: `MAME`, `MAME_SECONDS_TO_RUN`, `MAME_AUTOBOOT_DELAY`, `MAME_VISIBLE=1`.

## Common

`MC6809_OS9_CLANG`, `OS9_TOOL` (ToolShed `os9`), `FILECHECK` override the
tools; `KEEP_OS9_TEST_WORKDIR=1` keeps each test's working directory (module,
disks, console log).  Test cases are `TestCases/*.c` with a `RUN: %run_os9_case
%s OS9` line and `OS9:` FileCheck lines; the program is run twice, so expect
its output twice.

Do not commit ROMs or disk images to this tree.
