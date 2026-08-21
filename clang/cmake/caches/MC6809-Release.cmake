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
# only thing that changes between them is this string.
#
# It cannot be overridden with `-DMC6809_RELEASE=`: cmake reads this file
# before it applies any -D, so the two strings below are already derived from
# whatever this line said.  A one-off build with a different number sets those
# two directly instead:
#
#   -DCLANG_REPOSITORY_STRING="llvm-mc6809 X" -DLLVM_FORCE_VC_REPOSITORY="llvm-mc6809 X"
set(MC6809_RELEASE "1.0-rc1" CACHE STRING "Which llvm-mc6809 release this is")

# Forced, and all three of them: clang and lld copy PACKAGE_VENDOR into their
# own cache entries the first time a tree is configured and keep what they
# copied, so setting only PACKAGE_VENDOR leaves a carried-over cache saying
# something else.  A vendor with a number in it is the one thing that must
# never reach a release (see below).
set(PACKAGE_VENDOR "llvm-mc6809" CACHE STRING "" FORCE)
set(CLANG_VENDOR "llvm-mc6809" CACHE STRING "" FORCE)
set(LLD_VENDOR "llvm-mc6809" CACHE STRING "" FORCE)

# The number goes *after* the version, in the parentheses where the
# repository and revision are printed, and never before it: a build system
# reads the first dotted number in the banner as the compiler's version, so a
# number ahead of "clang version 24.0.0" makes meson see clang 1.0 and refuse
# to build anything.  So:
#
#   clang --version   llvm-mc6809 clang version 24.0.0 (llvm-mc6809 1.0-rc1 rev)
#   ld.lld --version  llvm-mc6809 LLD 24.0.0 (<repository> rev)
#
# Only clang carries the number, and that is deliberate.  The obvious way to
# give it to lld and the llvm tools as well is LLVM_FORCE_VC_REPOSITORY --
# but forcing the repository without also forcing the revision makes
# GenerateVersionFromVCS leave the revision undefined, and both tools then
# print no revision at all.  Trading the revision for the release number is a
# bad bargain: the number is in clang's banner, in <prefix>/VERSION and in
# the bundle README, while the revision is the only thing that says which
# source built this.
set(CLANG_REPOSITORY_STRING "llvm-mc6809 ${MC6809_RELEASE}" CACHE STRING "" FORCE)

# LLVM appends `git` to its version to say "this is not a release of LLVM".
# On a release of *this* it is noise: the release number is in the vendor
# string beside it, and clang and lld still print the repository and the
# revision they were built from.  It comes back by itself in any development
# build, which uses MC6809.cmake and never reads this file.
set(LLVM_VERSION_SUFFIX "" CACHE STRING "" FORCE)
