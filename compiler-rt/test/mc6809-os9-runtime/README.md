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
export NITROS9_RECIPES=/path/to/nitros9/recipes/picothing   # if not ~/Documents/...
llvm/build/bin/llvm-lit -sv compiler-rt/test/mc6809-os9-runtime
```

The boot image is `$NITROS9_RECIPES/l<level>_mega/`'s (default level 2,
`NITROS9_LEVEL` to change it), and the firmware is the newest
`rel_picothing.srec` in any recipe, since one REL boots every level.  Name
`NITROS9_BOOT_DSK` and `NITROS9_FIRMWARE` instead if they live somewhere else.
Without them the suite reports UNSUPPORTED rather than failing, so check that
it says how many tests ran.  `usim09pt` is taken only from
`NITROS9_USIM09PT`: one that happens to be on the `PATH` is not used.

The test disk is the IDE slave, so the boot image needs the `/J0` descriptor,
and an image without one counts as absent.  Only the mega images have it: Level
2 in the bootfile, Level 1 in `/Modules`, from where each run loads it.

A case that only means something at one level says so with a `// LEVEL: N`
line and requires that level's image (`// REQUIRES: nitros9-l1`); see
`TestCases/argvlvl1.c`, which checks the recovery of argv[0] from the module
header where the data area does not start at zero.

Build the images with NitrOS-9's `.github/workflows/nitros9.sh`.  Knobs:
`NITROS9_INPUT_FIRST` (cycles before the first console byte, default 78M —
the login prompt must be up), `NITROS9_TIMEOUT` (instruction cap),
`NITROS9_WALL_SECONDS` (wall-clock cap, default 300).

## MAME coco3 (needs CoCo 3 ROMs)

```sh
export MC6809_OS9_RUNTIME_TESTS=1                   # or MC6809_OS9_MAME_TESTS=1
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
its output twice.  Two more lines shape a case, each used once at most:
`// ARGS: one two` gives the program those command-line words, and
`// CFLAGS: -fno-builtin` adds compiler options to its build.

Do not commit ROMs or disk images to this tree.
