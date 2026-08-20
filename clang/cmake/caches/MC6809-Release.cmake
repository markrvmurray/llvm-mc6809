# MC6809-Release.cmake
# Configure llvm-mc6809 for a build somebody else will use.
#   cmake -C [path-to-this-file] -G Ninja -S llvm -B build
#
# The development cache builds Debug with assertions and calls itself
# `1.0-dev`, which is what a working tree should say.  A release, and a
# candidate for one, says something different; the differences live here
# rather than as flags somebody has to remember.
include(${CMAKE_CURRENT_LIST_DIR}/MC6809.cmake)

set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
set(LLVM_ENABLE_ASSERTIONS OFF CACHE BOOL "" FORCE)

# **Edit this line for each candidate**: 1.0-rc1, 1.0-rc2, then 1.0.  A
# candidate is built exactly like the release it is a candidate for, so the
# only thing that changes between them is this string.  `-DPACKAGE_VENDOR=...`
# on the command line overrides it, because -D is processed after -C.
set(PACKAGE_VENDOR "llvm-mc6809 1.0-rc1" CACHE STRING "" FORCE)

# LLVM appends `git` to its version to say "this is not a release of LLVM".
# On a release of *this* it is noise: the release number is in the vendor
# string beside it, and clang and lld still print the repository and the
# revision they were built from.  It comes back by itself in any development
# build, which uses MC6809.cmake and never reads this file.
set(LLVM_VERSION_SUFFIX "" CACHE STRING "" FORCE)
