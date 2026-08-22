# MC6809.cmake
# Configure llvm-mc6809 for building a distribution
# Usage for configuring:
#   cmake -C [path-to-this-file] ...

set(LLVM_TARGETS_TO_BUILD "" CACHE STRING "")
set(LLVM_EXPERIMENTAL_TARGETS_TO_BUILD "MC6809;MOS" CACHE STRING "")
set(LLVM_ENABLE_PROJECTS clang;lld;lldb CACHE STRING "")
# set(LLVM_ENABLE_LIBEDIT OFF CACHE BOOL "")
set(LLVM_ENABLE_LIBXML2 "OFF" CACHE STRING "")
set(LLVM_ENABLE_ZLIB "OFF" CACHE STRING "")
set(LLVM_ENABLE_ZSTD "OFF" CACHE STRING "")

# LLVM_ENABLE_RUNTIMES gates the `runtimes` external sub-project that
# builds compiler-rt's sanitizers, XRay, profile, etc. None of those
# target bare-metal MC6809 — their feature-detection probes try
# `-arch x86_64h`, `-miphoneos-version-min=…`, `-g` (link-required)
# through our cross-clang, all of which fail. Force this empty so the
# `runtimes` sub-project is never generated and `cmake --build . --target
# all` succeeds. Builtins use a SEPARATE path below (LLVM_BUILTIN_TARGETS)
# and remain unaffected — `get_compiler_rt_path` falls back to the source
# tree when LLVM_BUILTIN_TARGETS is set, so builtins still build.
# FORCE here overrides whatever CLion's project setup tries to set.
set(LLVM_ENABLE_RUNTIMES "" CACHE STRING "" FORCE)
set(LLVM_RUNTIME_TARGETS "" CACHE STRING "" FORCE)

# RT library / builtins cross-compilation: one builtins tree per triple.
# mc6809-unknown-unknown is bare metal (integer + MC6839 floating point);
# mc6809-unknown-os9 is the OS-9 program-module environment (integer
# builtins plus the OS-9 CRT, linker script and system-call header).
set(LLVM_BUILTIN_TARGETS "mc6809-unknown-unknown;mc6809-unknown-os9" CACHE STRING "")
set(BUILTINS_mc6809-unknown-unknown_COMPILER_RT_BAREMETAL_BUILD ON CACHE BOOL "")
# Bug #295: PIE end-to-end. The hand-written FP wrapper .S files now use
# PCR-relative loads (R_MC6809_PCREL_16) for the FPCB pointer; the C-built
# builtins are PIE-clean too. Compiler-rt builds with -fPIC enabled.
set(BUILTINS_mc6809-unknown-unknown_COMPILER_RT_BUILTINS_ENABLE_PIC ON CACHE BOOL "")
set(BUILTINS_mc6809-unknown-unknown_CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "")
set(BUILTINS_mc6809-unknown-unknown_CMAKE_SYSTEM_NAME Generic CACHE STRING "")
set(BUILTINS_mc6809-unknown-os9_COMPILER_RT_BAREMETAL_BUILD ON CACHE BOOL "")
set(BUILTINS_mc6809-unknown-os9_COMPILER_RT_BUILTINS_ENABLE_PIC ON CACHE BOOL "")
set(BUILTINS_mc6809-unknown-os9_CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "")
set(BUILTINS_mc6809-unknown-os9_CMAKE_SYSTEM_NAME Generic CACHE STRING "")

set(LLVM_DEFAULT_TARGET_TRIPLE "mc6809-unknown-unknown" CACHE STRING "")

# The following option is principally to reduce space on Github action runner
# builds. They make smaller, and possibly slower, releases; but the releases are
# already over 1GB without them on most platforms, and the compilers don't seem
# to be slow on MC6809-sized projects.  If you have more disk space, you may not
# need them.
set(CMAKE_BUILD_TYPE Debug CACHE STRING "CMake build type")

# Disable precompiled headers so that TableGen changes only rebuild the
# affected backend, not the entire tree.
set(CMAKE_DISABLE_PRECOMPILE_HEADERS ON CACHE BOOL "")

# disable lldb testing until the lldb tests stabilize
set(LLDB_INCLUDE_TESTS OFF CACHE BOOL "Include lldb tests")

# Ship the release with these tools
set(LLVM_INSTALL_TOOLCHAIN_ONLY ON CACHE BOOL "")
set(LLVM_TOOLCHAIN_TOOLS
  llvm-addr2line
  llvm-ar
  llvm-cxxfilt
  llvm-dwarfdump
  llvm-mc
  llvm-mlb
  llvm-nm
  llvm-objcopy
  llvm-objdump
  llvm-ranlib
  llvm-readelf
  llvm-readobj
  llvm-size
  llvm-strings
  llvm-strip
  llvm-symbolizer CACHE STRING "")

set(LLVM_DISTRIBUTION_COMPONENTS
  clang
  clang-format
  clang-resource-headers
  clang-refactor
  clang-scan-deps
  ${LLVM_TOOLCHAIN_TOOLS}
  CACHE STRING "")

# Add clang symlinks prefixed with mc6809-* to allow distinguishing a llvm-mc6809
# directory from a system clang directory.
set(CLANG_LINKS_TO_CREATE
  clang++
  clang-cpp
  mc6809-clang
  mc6809-clang++
  mc6809-clang-cpp)
set(CLANG_LINKS_TO_CREATE ${CLANG_LINKS_TO_CREATE}
  CACHE STRING "Clang symlinks to create during install.")

# Disable bindings since they can be problematic for the install pattern used along with llvm-mc6809-sdk.
set(LLVM_ENABLE_BINDINGS OFF CACHE BOOL "Build bindings.")
