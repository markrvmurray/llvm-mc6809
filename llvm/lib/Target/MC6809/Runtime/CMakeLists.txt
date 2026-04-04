# Build libmc6809rt.a — hand-written assembly builtins for MC6809.
#
# Uses the just-built llvm-mc and llvm-ar to assemble and archive the
# runtime library, then installs it alongside libclang_rt.builtins.a.

set(MC6809RT_SRC ${CMAKE_CURRENT_SOURCE_DIR}/mc6809rt.s)
set(MC6809RT_INC ${CMAKE_CURRENT_SOURCE_DIR})
set(MC6809RT_OBJ ${CMAKE_CURRENT_BINARY_DIR}/mc6809rt.o)
set(MC6809RT_LIB ${CMAKE_CURRENT_BINARY_DIR}/libmc6809rt.a)

# Where clang expects to find runtime libraries for mc6809.
set(MC6809RT_INSTALL_DIR
  ${LLVM_LIBRARY_OUTPUT_INTDIR}/clang/${CLANG_VERSION_MAJOR}/lib/mc6809-unknown-unknown)

add_custom_command(
  OUTPUT ${MC6809RT_OBJ}
  COMMAND $<TARGET_FILE:llvm-mc> -triple=mc6809
    -I ${MC6809RT_INC}
    --filetype=obj
    -o ${MC6809RT_OBJ}
    ${MC6809RT_SRC}
  DEPENDS llvm-mc ${MC6809RT_SRC}
    ${MC6809RT_INC}/shiftqi3.inc
    ${MC6809RT_INC}/shifthi3.inc
    ${MC6809RT_INC}/shiftsi3.inc
    ${MC6809RT_INC}/mulhi3.inc
    ${MC6809RT_INC}/mulsi3.inc
    ${MC6809RT_INC}/divhi.inc
    ${MC6809RT_INC}/divsi.inc
  COMMENT "Assembling MC6809 runtime library"
)

add_custom_command(
  OUTPUT ${MC6809RT_LIB}
  COMMAND $<TARGET_FILE:llvm-ar> rcs ${MC6809RT_LIB} ${MC6809RT_OBJ}
  DEPENDS llvm-ar ${MC6809RT_OBJ}
  COMMENT "Archiving MC6809 runtime library"
)

add_custom_target(mc6809rt ALL DEPENDS ${MC6809RT_LIB})

# Install to the clang resource directory.
add_custom_command(TARGET mc6809rt POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${MC6809RT_INSTALL_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy ${MC6809RT_LIB} ${MC6809RT_INSTALL_DIR}/
  COMMENT "Installing libmc6809rt.a"
)
