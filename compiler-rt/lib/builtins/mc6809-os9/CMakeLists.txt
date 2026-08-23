# Build + install the OS-9 / NitrOS-9 runtime artifacts so
# `clang -target mc6809-unknown-os9` works as a one-shot from the
# install tree (no per-test absolute paths).
#
# Artifacts produced into <resourcedir>/lib/mc6809-unknown-os9/:
#   - libclang_rt.os9.a   (crt0.o + syscalls.o bundled)
#   - mc6809-os9.lds      (the program-module linker script)
#
# And into <resourcedir>/include/:
#   - os9.h               (the system-call shims and the C API)
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
set(_OS9_CRT1_OUT "${_OS9_BUILD_LIB_DIR}/crt1.o")
set(_OS9_LINK_OUT "${_OS9_BUILD_LIB_DIR}/os9-link.o")
set(_OS9_XU_OUT "${_OS9_BUILD_LIB_DIR}/os9-xu.o")
set(_OS9_LDS_OUT "${_OS9_BUILD_LIB_DIR}/mc6809-os9.lds")
set(_OS9_HDR_OUT "${_OS9_BUILD_INC_DIR}/os9.h")
set(_OS9_FP_OUT  "${_OS9_BUILD_LIB_DIR}/FPO9")

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

# syscalls.o -- errno and the POSIX-style entry points, C over the inline
# shims in include/os9.h.
add_custom_command(
    OUTPUT  "${_OS9_SYSCALLS_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS} -Os
            -I "${CMAKE_CURRENT_SOURCE_DIR}/include"
            -c "${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c"
            -o "${_OS9_SYSCALLS_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/syscalls.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/include/os9.h"
    COMMENT "[OS-9] Building syscalls.o"
    VERBATIM
)

# crt1.o -- the C half of start-up: heap claim (F$Mem), argc/argv, main.
add_custom_command(
    OUTPUT  "${_OS9_CRT1_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS} -Os
            -I "${CMAKE_CURRENT_SOURCE_DIR}/include"
            -c "${CMAKE_CURRENT_SOURCE_DIR}/crt1.c"
            -o "${_OS9_CRT1_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/crt1.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/include/os9.h"
    COMMENT "[OS-9] Building crt1.o"
    VERBATIM
)

# os9-xu.o -- the calls that carry a 32-bit position in X:U (save/restore U).
add_custom_command(
    OUTPUT  "${_OS9_XU_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS}
            -c "${CMAKE_CURRENT_SOURCE_DIR}/os9-xu.S"
            -o "${_OS9_XU_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/os9-xu.S"
    COMMENT "[OS-9] Building os9-xu.o"
    VERBATIM
)

# os9-link.o -- F$Link / F$Load / F$UnLink, which answer in U.
add_custom_command(
    OUTPUT  "${_OS9_LINK_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_OS9_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER} ${_OS9_FLAGS}
            -c "${CMAKE_CURRENT_SOURCE_DIR}/os9-link.S"
            -o "${_OS9_LINK_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/os9-link.S"
    COMMENT "[OS-9] Building os9-link.o"
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
            "${_OS9_CRT1_OUT}"
            "${_OS9_SYSCALLS_OUT}"
            "${_OS9_XU_OUT}"
            "${_OS9_LINK_OUT}"
    DEPENDS "${_OS9_CRT0_OUT}"
            "${_OS9_CRT1_OUT}"
            "${_OS9_SYSCALLS_OUT}"
            "${_OS9_XU_OUT}"
            "${_OS9_LINK_OUT}"
    COMMENT "[OS-9] Bundling libclang_rt.os9.a"
    VERBATIM
)

# The floating-point module, staged under the name OS-9 knows it by.  A
# program that does floating point links FPO9 at run time rather than
# carrying the ROM, so it has to be on the system: this is the copy to put
# on a disk, next to the linker script the same programs are built with.
add_custom_command(
    OUTPUT  "${_OS9_FP_OUT}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/../mc6809/Float09.bin"
            "${_OS9_FP_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../mc6809/Float09.bin"
    COMMENT "[OS-9] Staging FPO9"
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
        "${_OS9_FP_OUT}"
)
add_dependencies(builtins mc6809-os9-runtime)

# Install rules — match the resourcedir layout so the in-tree paths
# and the installed paths are byte-identical relative to the prefix.
install(FILES "${_OS9_LIB_OUT}" "${_OS9_LDS_OUT}" "${_OS9_FP_OUT}"
        DESTINATION "${COMPILER_RT_INSTALL_LIBRARY_DIR}/${_OS9_TRIPLE}"
        COMPONENT mc6809-os9-runtime)
install(FILES "${_OS9_HDR_OUT}"
        DESTINATION "${COMPILER_RT_INSTALL_INCLUDE_DIR}"
        COMPONENT mc6809-os9-runtime)
