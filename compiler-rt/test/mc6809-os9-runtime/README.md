# MC6809 OS-9 MAME Runtime Tests

This is an opt-in integration suite for LLVM-generated OS-9 modules.  It
compiles each test with the in-tree Clang, copies the resulting module to a
temporary copy of a provided CoCo 3/NitrOS-9 boot disk, runs MAME headlessly,
extracts the test output with ToolShed `os9`, and checks that output with
FileCheck.

The suite is skipped unless all required environment is present:

```sh
export MC6809_OS9_MAME_TESTS=1
export MAME_ROMPATH=/path/to/coco3/roms
export OS9_BOOT_DSK=/path/to/l2_coco3_cmoc_os9.dsk
```

Optional overrides:

```sh
export MC6809_OS9_CLANG=/path/to/clang
export MAME=/path/to/mame
export OS9_TOOL=/path/to/os9
export FILECHECK=/path/to/FileCheck
export MAME_SECONDS_TO_RUN=120
export MAME_AUTOBOOT_DELAY=8
```

Run from the LLVM source root:

```sh
MC6809_OS9_MAME_TESTS=1 \
MAME_ROMPATH=/path/to/roms \
OS9_BOOT_DSK=/Users/boisy/Projects/coco-shelf/cmoc_os9/recipes/coco3/l2_coco3_cmoc_os9.dsk \
MC6809_OS9_CLANG=/Users/boisy/Projects/llvm-mc6809/llvm/build/bin/clang \
llvm/build/bin/llvm-lit -sv compiler-rt/test/mc6809-os9-runtime
```

Do not commit ROMs or generated disk images to this tree.
