# -*- Python -*-

import os
import shutil
import subprocess
import sys

import lit.formats

config.name = "MC6809 OS-9 MAME Runtime"
config.test_format = lit.formats.ShTest(execute_external=True)
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

# Convenient for running this source-tree suite directly with
# llvm/build/bin/llvm-lit, without a configured compiler-rt lit site.
repo_root = os.path.abspath(os.path.join(config.test_source_root, "..", "..", ".."))
tool_dirs.append(os.path.join(repo_root, "llvm", "build", "bin"))

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

clang = find_tool("MC6809_OS9_CLANG", "clang")
filecheck = find_tool("FILECHECK", "FileCheck")
mame = find_tool("MAME", "mame")
os9 = find_tool("OS9_TOOL", "os9")

config.substitutions.append(("%clang", clang))
config.substitutions.append(("%filecheck", filecheck))
config.substitutions.append(("%mame", mame))
config.substitutions.append(("%os9", os9))
config.substitutions.append(("%run_os9_case",
    os.path.join(config.test_source_root, "scripts", "run-os9-case.sh")))

enabled = os.environ.get("MC6809_OS9_MAME_TESTS") == "1"
rompath = os.environ.get("MAME_ROMPATH")
boot_disk = os.environ.get("OS9_BOOT_DSK")
has_rompath = bool(rompath) and os.path.isdir(rompath)
has_boot_disk = bool(boot_disk) and os.path.isfile(boot_disk)
has_mame = shutil.which(mame) is not None or os.path.exists(mame)
has_os9 = shutil.which(os9) is not None or os.path.exists(os9)
has_mc6809_clang = False
if shutil.which(clang) is not None or os.path.exists(clang):
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

configured = (
    enabled
    and has_rompath
    and has_boot_disk
    and has_mame
    and has_os9
    and has_mc6809_clang
)

if configured:
    config.available_features.add("mc6809-os9-mame-runtime")
else:
    config.unsupported = True

config.environment["MAME"] = mame
config.environment["OS9_TOOL"] = os9
config.environment["MC6809_OS9_CLANG"] = clang
config.environment["FILECHECK"] = filecheck

for env_name in [
    "MAME_ROMPATH",
    "OS9_BOOT_DSK",
    "MAME_SECONDS_TO_RUN",
    "MAME_AUTOBOOT_DELAY",
    "MAME_VISIBLE",
    "KEEP_OS9_TEST_WORKDIR",
    "OS9_TEST_RUN_TWICE",
]:
    if env_name in os.environ:
        config.environment[env_name] = os.environ[env_name]
