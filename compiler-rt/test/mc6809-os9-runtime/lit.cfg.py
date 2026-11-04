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
#     NITROS9_BOOT_DSK=/path/to/NOS9_6809_L2_DEV_picothing_mega.dsk
#                      (an IDE image with a /J0 descriptor)
#     NITROS9_FIRMWARE=/path/to/rel_picothing.srec
#
#   MAME coco3 (needs CoCo 3 ROMs)
#     MAME_ROMPATH=/path/to/roms
#     OS9_BOOT_DSK=/path/to/nitros9-coco3.dsk   [MAME=/path/to/mame]
#
# and MC6809_OS9_RUNTIME_TESTS=1 to opt in (booting an OS per test is slow).

import functools
import glob
import operator
import os
import shutil
import subprocess
import sys
import tempfile

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


def level_image(level):
    return os.path.join(nitros9_recipes, "l%s_mega" % level,
                        "NOS9_6809_L%s_DEV_picothing_mega.dsk" % level)


def newest_firmware():
    # One REL boots every level and processor, so any recipe's copy will do;
    # the newest is the one least likely to predate the images.
    found = glob.glob(os.path.join(nitros9_recipes, "*", "rel_picothing.srec"))
    return max(found, key=os.path.getmtime) if found else ""


def module_names(data):
    # A module header is $87 $CD, its size, its name's offset, and at byte 8 a
    # check that makes the first nine bytes XOR to $FF.  A name's last
    # character carries the high bit.
    names = []
    at = data.find(b"\x87\xcd")
    while at >= 0:
        header = data[at:at + 9]
        if len(header) == 9 and functools.reduce(operator.xor, header) == 0xFF:
            name_at = at + ((header[4] << 8) | header[5])
            name = bytearray()
            while name_at < len(data):
                name.append(data[name_at] & 0x7F)
                if data[name_at] & 0x80:
                    break
                name_at += 1
            names.append(name.decode("ascii", "replace"))
        at = data.find(b"\x87\xcd", at + 1)
    return names


def image_has_j0(image):
    # The test disk is the IDE slave, reached through the /J0 descriptor.
    # Level 2 mega images have it in the bootfile.  Level 1's module directory
    # has no room for it there, so those images keep it in /Modules, and
    # run-os9-case.sh loads it from there.
    if not (image and os.path.isfile(image) and usable(os9)):
        return False
    listing = subprocess.run([os9, "dir", image + ",Modules"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             check=False).stdout
    if b"j0_pt.dd" in listing:
        return True
    with tempfile.TemporaryDirectory() as tmp:
        bootfile = os.path.join(tmp, "OS9Boot")
        subprocess.run([os9, "copy", "-r", image + ",OS9Boot", bootfile],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       check=False)
        if not os.path.isfile(bootfile):
            return False
        with open(bootfile, "rb") as f:
            return "J0" in module_names(f.read())


nitros9_level = os.environ.get("NITROS9_LEVEL", "2")
usim_boot = os.environ.get("NITROS9_BOOT_DSK") or level_image(nitros9_level)
usim_firmware = os.environ.get("NITROS9_FIRMWARE") or newest_firmware()
usim_ready = usable(usim09pt) and image_has_j0(usim_boot) and os.path.isfile(usim_firmware)

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
    # which is only honest if that level's image can reach the test disk.
    for _lvl in ("1", "2"):
        _image = level_image(_lvl)
        if image_has_j0(_image):
            config.available_features.add("nitros9-l%s" % _lvl)
            config.environment["NITROS9_BOOT_DSK_L%s" % _lvl] = _image
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
