# -*- Python -*-
#
# End-to-end tests for LLVM-built OS-9 program modules: each TestCase is
# compiled with the in-tree clang for mc6809-unknown-os9, put on a disk, run
# under a real NitrOS-9 boot, and its console output checked with FileCheck.
#
# Two ways to boot NitrOS-9 are supported; the suite is skipped unless one is
# fully configured (see README.md):
#
#   usim09pt (Pico-Thing, ROM-free, the default)
#     NITROS9_USIM09PT=/path/to/usim09pt
#     NITROS9_BOOT_DSK=/path/to/NOS9_6809_L2_DEV_picothing_x0.dsk (IDE image)
#     NITROS9_FIRMWARE=/path/to/rel_picothing.srec
#
#   MAME coco3 (needs CoCo 3 ROMs)
#     MAME_ROMPATH=/path/to/roms
#     OS9_BOOT_DSK=/path/to/nitros9-coco3.dsk   [MAME=/path/to/mame]
#
# and MC6809_OS9_RUNTIME_TESTS=1 to opt in (booting an OS per test is slow).

import os
import shutil
import subprocess
import sys

import lit.formats

config.name = "MC6809 OS-9 Runtime"
config.test_format = lit.formats.ShTest()
config.test_source_root = os.path.dirname(__file__)
config.suffixes = [".c", ".test"]

llvm_tools_dir = getattr(config, "llvm_tools_dir", None)

tool_dirs = []
if llvm_tools_dir:
    tool_dirs.append(llvm_tools_dir)
llvm_lit_path = shutil.which("llvm-lit")
if llvm_lit_path:
    tool_dirs.append(os.path.dirname(llvm_lit_path))
if sys.argv and sys.argv[0]:
    tool_dirs.append(os.path.dirname(os.path.abspath(sys.argv[0])))


def find_tool(env_name, tool_name):
    value = os.environ.get(env_name)
    if value:
        return value
    for tool_dir in tool_dirs:
        if not tool_dir:
            continue
        candidate = os.path.join(tool_dir, tool_name)
        if os.path.exists(candidate):
            return candidate
    return shutil.which(tool_name) or tool_name


def usable(path):
    return bool(path) and (shutil.which(path) is not None or os.path.exists(path))


clang = find_tool("MC6809_OS9_CLANG", "clang")
filecheck = find_tool("FILECHECK", "FileCheck")
os9 = find_tool("OS9_TOOL", "os9")
mame = find_tool("MAME", "mame")
usim09pt = os.environ.get("NITROS9_USIM09PT", shutil.which("usim09pt") or "")

config.substitutions.append(("%clang", clang))
config.substitutions.append(("%filecheck", filecheck))
config.substitutions.append(("%os9", os9))
config.substitutions.append(("%run_os9_case",
    os.path.join(config.test_source_root, "scripts", "run-os9-case.sh")))

enabled = os.environ.get("MC6809_OS9_RUNTIME_TESTS") == "1" or \
          os.environ.get("MC6809_OS9_MAME_TESTS") == "1"

has_mc6809_clang = False
if usable(clang):
    try:
        result = subprocess.run(
            [clang, "--target=mc6809-unknown-os9", "-x", "c", "-c", "-o", os.devnull, "-"],
            input=b"int x;\n",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        has_mc6809_clang = result.returncode == 0
    except OSError:
        has_mc6809_clang = False

# The images live in a known place under the NitrOS-9 recipes tree, one
# directory per level, so derive them the way run-mc6809-os9 does rather than
# making every caller name two paths.  Naming them still wins.
nitros9_recipes = os.environ.get(
    "NITROS9_RECIPES",
    os.path.expanduser("~/Documents/NitrOS-9/nitros9/recipes/picothing"))


def level_images(level):
    lvldir = os.path.join(nitros9_recipes, "l%s" % level)
    return (os.path.join(lvldir,
                         "NOS9_6809_L%s_DEV_picothing_x0.dsk" % level),
            os.path.join(lvldir, "rel_picothing.srec"))


nitros9_level = os.environ.get("NITROS9_LEVEL", "2")
derived_boot, derived_firmware = level_images(nitros9_level)
usim_boot = os.environ.get("NITROS9_BOOT_DSK") or derived_boot
usim_firmware = os.environ.get("NITROS9_FIRMWARE") or derived_firmware
usim_ready = usable(usim09pt) and os.path.isfile(usim_boot) and os.path.isfile(usim_firmware)

mame_rompath = os.environ.get("MAME_ROMPATH", "")
mame_boot = os.environ.get("OS9_BOOT_DSK", "")
mame_ready = usable(mame) and os.path.isdir(mame_rompath) and os.path.isfile(mame_boot)

backend = os.environ.get("MC6809_OS9_BACKEND", "")
if not backend:
    backend = "usim" if usim_ready else ("mame" if mame_ready else "")

configured = enabled and usable(os9) and has_mc6809_clang and (
    (backend == "usim" and usim_ready) or (backend == "mame" and mame_ready))

if configured:
    config.available_features.add("mc6809-os9-runtime")
    config.available_features.add("mc6809-os9-%s-runtime" % backend)
    # A case can ask for a level other than the one in force (`// LEVEL: 1`),
    # which is only honest if that level's images are actually here.
    for _lvl in ("1", "2"):
        _b, _f = level_images(_lvl)
        if os.path.isfile(_b) and os.path.isfile(_f):
            config.available_features.add("nitros9-l%s" % _lvl)
else:
    config.unsupported = True

config.environment["MC6809_OS9_BACKEND"] = backend
config.environment["MC6809_OS9_CLANG"] = clang
config.environment["OS9_TOOL"] = os9
config.environment["FILECHECK"] = filecheck
config.environment["MAME"] = mame
config.environment["NITROS9_USIM09PT"] = usim09pt
config.environment["NITROS9_RECIPES"] = nitros9_recipes
config.environment["NITROS9_BOOT_DSK"] = usim_boot
config.environment["NITROS9_FIRMWARE"] = usim_firmware

for env_name in [
    "NITROS9_BOOT_DSK",
    "NITROS9_FIRMWARE",
    "NITROS9_INPUT_FIRST",
    "NITROS9_TIMEOUT",
    "NITROS9_WALL_SECONDS",
    "MAME_ROMPATH",
    "OS9_BOOT_DSK",
    "MAME_SECONDS_TO_RUN",
    "MAME_AUTOBOOT_DELAY",
    "MAME_VISIBLE",
    "KEEP_OS9_TEST_WORKDIR",
]:
    if env_name in os.environ:
        config.environment[env_name] = os.environ[env_name]
