# Bare-bones C++ for MC6809: what the language needs beyond the C library.
#
# Two artifacts, both built with the clang that was just built:
#
#   <resourcedir>/lib/<triple>/libclang_rt.cxx.a   operator new/delete,
#                                                  __cxa_pure_virtual, the
#                                                  guard functions
#   <resourcedir>/include/c++/                     <cstdio> and friends, thin
#                                                  covers over the C headers
#
# There is no C++ standard library here and no plan for one: libc++ is far
# larger than a 64K address space.  This is the language over picolibc.
#
# The headers are the same for every triple, so whichever build gets there
# first stages them; the library is per-triple because the link line for each
# target looks in its own directory.

set(_CXX_TRIPLE "${COMPILER_RT_DEFAULT_TARGET_TRIPLE}")
set(_CXX_BUILD_LIB_DIR "${COMPILER_RT_OUTPUT_LIBRARY_DIR}/${_CXX_TRIPLE}")
set(_CXX_BUILD_INC_DIR "${COMPILER_RT_OUTPUT_DIR}/include/c++")

set(_CXX_OBJ "${_CXX_BUILD_LIB_DIR}/cxx_runtime.o")
set(_CXX_LIB "${_CXX_BUILD_LIB_DIR}/libclang_rt.cxx.a")

# -nostdinc as well as -nostdinc++: this is built with compiler-rt, before a
# C library exists to include from, so the source declares the three C
# functions it uses and includes nothing.
add_custom_command(
    OUTPUT  "${_CXX_OBJ}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_CXX_BUILD_LIB_DIR}"
    COMMAND ${CMAKE_CXX_COMPILER} --target=${_CXX_TRIPLE}
            -nostdinc -nostdinc++ -fno-exceptions -fno-rtti -Os
            -c "${CMAKE_CURRENT_SOURCE_DIR}/cxx_runtime.cpp"
            -o "${_CXX_OBJ}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cxx_runtime.cpp"
    COMMENT "[C++] Building cxx_runtime.o for ${_CXX_TRIPLE}"
    VERBATIM
)

add_custom_command(
    OUTPUT  "${_CXX_LIB}"
    COMMAND ${CMAKE_AR} rcs "${_CXX_LIB}" "${_CXX_OBJ}"
    DEPENDS "${_CXX_OBJ}"
    COMMENT "[C++] Archiving libclang_rt.cxx.a for ${_CXX_TRIPLE}"
    VERBATIM
)

# The headers, copied whole: a directory of one-liners over the C library.
file(GLOB _CXX_HDRS "${CMAKE_CURRENT_SOURCE_DIR}/include/*")
set(_CXX_HDR_STAMP "${_CXX_BUILD_INC_DIR}/.stamp")
add_custom_command(
    OUTPUT  "${_CXX_HDR_STAMP}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_CXX_BUILD_INC_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_CXX_HDRS}
            "${_CXX_BUILD_INC_DIR}"
    COMMAND ${CMAKE_COMMAND} -E touch "${_CXX_HDR_STAMP}"
    DEPENDS ${_CXX_HDRS}
    COMMENT "[C++] Staging the C-library headers under their C++ names"
    VERBATIM
)

add_custom_target(mc6809-cxx-runtime ALL
                  DEPENDS "${_CXX_LIB}" "${_CXX_HDR_STAMP}")

install(FILES "${_CXX_LIB}"
        DESTINATION "${COMPILER_RT_INSTALL_LIBRARY_DIR}/${_CXX_TRIPLE}")
install(DIRECTORY "${_CXX_BUILD_INC_DIR}/"
        DESTINATION "${COMPILER_RT_INSTALL_PATH}/include/c++"
        PATTERN ".stamp" EXCLUDE)
