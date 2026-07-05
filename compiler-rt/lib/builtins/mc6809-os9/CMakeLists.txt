# Bug #163 Phase 2: build + install the OS-9 / NitrOS-9 runtime
# artifacts so `clang -target mc6809-unknown-os9` works as a one-shot
# from the install tree (no per-test absolute paths).
#
# Artifacts produced into <resourcedir>/lib/mc6809-unknown-os9/:
#   - libclang_rt.os9.a   (crt0.o + syscalls.o + syscalls_c.o bundled)
#   - mc6809-os9.lds      (the flat-binary linker script)
#
# And into <resourcedir>/include/:
#   - os9.h               (C API for the syscall stubs)
#
# `cmake --build . --target all` triggers `mc6809-os9-runtime` via
# its `ALL` attribute, so a normal full build deposits these files.
# `cmake --install` (or `ninja install`) copies them to the install
# prefix's resourcedir.

set(_OS9_TRIPLE "mc6809-unknown-os9")
set(_OS9_BUILD_LIB_DIR
    "${COMPILER_RT_OUTPUT_LIBRARY_DIR}/${_OS9_TRIPLE}")
set(_OS9_BUILD_INC_DIR
    "${COMPILER_RT_OUTPUT_DIR}/include")

# Common flags for the standalone .S compiles.  -fdollars-in-identifiers
# is auto-passed by the driver, but spelling it here too is defensive.
set(_OS9_FLAGS
    --target=${_OS9_TRIPLE}
    -nostdinc -nostdlib
    -fdollars-in-identifiers)

set(_OS9_LIB_OUT "${_OS9_BUILD_LIB_DIR}/libclang_rt.os9.a")
set(_OS9_CRT0_OUT "${_OS9_BUILD_LIB_DIR}/crt0.o")
set(_OS9_SYSCALLS_OUT "${_OS9_BUILD_LIB_DIR}/syscalls.o")
set(_OS9_SYSCALLS_C_OUT "${_OS9_BUILD_LIB_DIR}/syscalls_c.o")
set(_OS9_LDS_OUT "${_OS9_BUILD_LIB_DIR}/mc6809-os9.lds")
set(_OS9_HDR_OUT "${_OS9_BUILD_INC_DIR}/os9.h")

# Inside the compiler-rt sub-project, ${CMAKE_C_COMPILER} is the
# already-built clang and ${CMAKE_AR} is llvm-ar (both passed in via
# -D from the outer LLVM cmake invocation).  We can't use
# $<TARGET_FILE:clang> here because those LLVM targets don't exist in
# the runtimes sub-project's namespace.

# crt0.o
add_custom_command(
    OUTPUT  "${_OS9_CRT0_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS}
            -c "${CMAKE_CURRENT_SOURCE_DIR}/crt0.S"
            -o "${_OS9_CRT0_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/crt0.S"
    COMMENT "[OS-9] Building crt0.o"
    VERBATIM
)

# syscalls.o
add_custom_command(
    OUTPUT  "${_OS9_SYSCALLS_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS}
            -c "${CMAKE_CURRENT_SOURCE_DIR}/syscalls.S"
            -o "${_OS9_SYSCALLS_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/syscalls.S"
    COMMENT "[OS-9] Building syscalls.o"
    VERBATIM
)

# syscalls_c.o — the C (inline-asm) leaf syscalls (_exit for now).
add_custom_command(
    OUTPUT  "${_OS9_SYSCALLS_C_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS} -Os
            -c "${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c"
            -o "${_OS9_SYSCALLS_C_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c"
    COMMENT "[OS-9] Building syscalls_c.o"
    VERBATIM
)

# Bundle into libclang_rt.os9.a.  We use llvm-ar with rcsD for a
# deterministic (-D = no timestamps) static archive.
add_custom_command(
    OUTPUT  "${_OS9_LIB_OUT}"
    COMMAND ${CMAKE_COMMAND} -E remove -f "${_OS9_LIB_OUT}"
    COMMAND ${CMAKE_AR} rcsD
            "${_OS9_LIB_OUT}"
            "${_OS9_CRT0_OUT}"
            "${_OS9_SYSCALLS_OUT}"
            "${_OS9_SYSCALLS_C_OUT}"
    DEPENDS "${_OS9_CRT0_OUT}"
            "${_OS9_SYSCALLS_OUT}"
            "${_OS9_SYSCALLS_C_OUT}"
    COMMENT "[OS-9] Bundling libclang_rt.os9.a"
    VERBATIM
)

# Stage the linker script + header into the resourcedir.
add_custom_command(
    OUTPUT  "${_OS9_LDS_OUT}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/mc6809-os9.lds"
            "${_OS9_LDS_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/mc6809-os9.lds"
    COMMENT "[OS-9] Staging mc6809-os9.lds"
)
add_custom_command(
    OUTPUT  "${_OS9_HDR_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_INC_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/include/os9.h"
            "${_OS9_HDR_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/os9.h"
    COMMENT "[OS-9] Staging os9.h"
)

add_custom_target(mc6809-os9-runtime ALL
    DEPENDS
        "${_OS9_LIB_OUT}"
        "${_OS9_LDS_OUT}"
        "${_OS9_HDR_OUT}"
)
add_dependencies(builtins mc6809-os9-runtime)

# Install rules — match the resourcedir layout so the in-tree paths
# and the installed paths are byte-identical relative to the prefix.
install(FILES "${_OS9_LIB_OUT}" "${_OS9_LDS_OUT}"
        DESTINATION "${COMPILER_RT_INSTALL_LIBRARY_DIR}/${_OS9_TRIPLE}"
        COMPONENT mc6809-os9-runtime)
install(FILES "${_OS9_HDR_OUT}"
        DESTINATION "${COMPILER_RT_INSTALL_INCLUDE_DIR}"
        COMPONENT mc6809-os9-runtime)
