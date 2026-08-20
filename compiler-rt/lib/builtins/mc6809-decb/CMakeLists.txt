# Build + install the DECB runtime artifacts so `clang -target
# mc6809-unknown-decb` works as a one-shot, the way the OS-9 triple does.
#
# Artifacts produced into <resourcedir>/lib/mc6809-unknown-decb/:
#   - crt0.o           the entry: direct page, zeroed areas, constructors,
#                      main, and RTS back to BASIC
#   - mc6809-decb.lds  the linker script, which wraps the body in the
#                      LOADM/EXEC envelope via OUTPUT_FORMAT(decb-program)
#
# There is no library yet: a program links its own code and whatever it
# brings.  A picolibc variant for the triple is separate work.

set(_DECB_TRIPLE "mc6809-unknown-decb")
set(_DECB_BUILD_LIB_DIR
    "${COMPILER_RT_OUTPUT_LIBRARY_DIR}/${_DECB_TRIPLE}")

set(_DECB_CRT0_OUT "${_DECB_BUILD_LIB_DIR}/crt0.o")
set(_DECB_LDS_OUT  "${_DECB_BUILD_LIB_DIR}/mc6809-decb.lds")

# -fno-builtin: the zero loops in crt0 are what they say they are, and must
# not become calls to a memset that nothing here provides.
add_custom_command(
    OUTPUT  "${_DECB_CRT0_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_DECB_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_C_COMPILER}
            --target=${_DECB_TRIPLE} -Os -nostdlib -nostdinc -fno-builtin
            -c "${CMAKE_CURRENT_SOURCE_DIR}/crt0.c"
            -o "${_DECB_CRT0_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/crt0.c"
    COMMENT "[DECB] Building crt0.o"
    VERBATIM
)

add_custom_command(
    OUTPUT  "${_DECB_LDS_OUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_DECB_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/mc6809-decb.lds" "${_DECB_LDS_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/mc6809-decb.lds"
    COMMENT "[DECB] Staging mc6809-decb.lds"
    VERBATIM
)

add_custom_target(mc6809-decb-runtime ALL
    DEPENDS "${_DECB_CRT0_OUT}" "${_DECB_LDS_OUT}"
)
add_dependencies(builtins mc6809-decb-runtime)

install(FILES "${_DECB_CRT0_OUT}" "${_DECB_LDS_OUT}"
        DESTINATION "${COMPILER_RT_INSTALL_LIBRARY_DIR}/${_DECB_TRIPLE}"
        COMPONENT mc6809-decb-runtime)
