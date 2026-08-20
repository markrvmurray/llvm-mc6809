#!/bin/sh
# roll-toolchain.sh — build an MC6809 toolchain someone can unpack and use.
#
#   scripts/roll-toolchain.sh --prefix DIR [--llvm-build DIR] [--picolibc DIR]
#
# Produces:
#
#   <prefix>/bin/                                  clang, lld, the llvm tools
#   <prefix>/lib/clang/<ver>/                      compiler-rt + the OS-9 CRT,
#                                                  linker script and FP module
#   <prefix>/lib/clang-runtimes/<triple>/include   picolibc headers
#   <prefix>/lib/clang-runtimes/<triple>/lib       libc.a and its system layer
#
# The driver finds all of that relative to the binary that was invoked, so the
# result can be moved, renamed or unpacked anywhere; nothing is baked in.
#
# Stage from a **Release** LLVM build for anything anyone else will use: the
# development Debug tree makes a bundle of about a gigabyte and a slow
# compiler.  It is the default only because it is the one that is always
# there.
set -eu

REPO=$(cd "$(dirname "$0")/.." && pwd)
PREFIX=
LLVM_BUILD=$REPO/llvm/cmake-build-debug-system
PICOLIBC=$(cd "$REPO/.." && pwd)/picolibc

while [ $# -gt 0 ]; do
  case "$1" in
    --prefix)      PREFIX=$2; shift 2 ;;
    --llvm-build)  LLVM_BUILD=$2; shift 2 ;;
    --picolibc)    PICOLIBC=$2; shift 2 ;;
    -h|--help)     sed -n '2,24p' "$0"; exit 0 ;;
    *) echo "$0: unknown option $1" >&2; exit 2 ;;
  esac
done
[ -n "$PREFIX" ] || { echo "$0: --prefix is required" >&2; exit 2; }
[ -x "$LLVM_BUILD/bin/clang" ] || { echo "$0: no clang in $LLVM_BUILD/bin" >&2; exit 2; }
[ -f "$PICOLIBC/meson.build" ] || { echo "$0: no picolibc at $PICOLIBC" >&2; exit 2; }

say() { echo "==[ $(date '+%H:%M:%S') $* ]=="; }

# ---------------------------------------------------------------- the tools
# What a person invokes, plus what clang invokes for them.  These are copies:
# clang resolves its own path to the real binary, so a symlink into a build
# tree would send the driver looking for its sysroot there instead of here.
say "staging tools from $LLVM_BUILD"
mkdir -p "$PREFIX/bin" "$PREFIX/lib"
for f in "$LLVM_BUILD"/bin/clang-* "$LLVM_BUILD"/bin/lld \
         "$LLVM_BUILD"/bin/llvm-ar "$LLVM_BUILD"/bin/llvm-nm \
         "$LLVM_BUILD"/bin/llvm-objcopy "$LLVM_BUILD"/bin/llvm-objdump \
         "$LLVM_BUILD"/bin/llvm-ranlib "$LLVM_BUILD"/bin/llvm-readelf \
         "$LLVM_BUILD"/bin/llvm-size "$LLVM_BUILD"/bin/llvm-strip; do
  [ -f "$f" ] && [ ! -L "$f" ] && cp -p "$f" "$PREFIX/bin/"
done
( cd "$PREFIX/bin"
  clangbin=$(ls clang-* 2>/dev/null | head -1)
  [ -n "$clangbin" ] && ln -sf "$clangbin" clang
  [ -f lld ] && ln -sf lld ld.lld
  true )

say "staging the compiler's own runtime directory"
resdir=$("$LLVM_BUILD/bin/clang" -print-resource-dir)
mkdir -p "$PREFIX/lib/clang"
cp -R "$resdir" "$PREFIX/lib/clang/"

# The staged compiler has to be the one that finds it, or the bundle is not
# self-contained and every later step would be testing the build tree.
staged_res=$("$PREFIX/bin/clang" -print-resource-dir)
case "$staged_res" in
  "$PREFIX"/*|/private"$PREFIX"/*) : ;;
  *) echo "$0: the staged clang looks outside the prefix ($staged_res)" >&2
     exit 1 ;;
esac

# ------------------------------------------------------------- the library
# Built by the staged compiler and installed by picolibc's own rules, which
# already put headers in include/ and libraries in lib/.  Tests are off: a
# bundle does not need them, and some of them deliberately do not link.
say "generating cross files against $PREFIX"
MC6809_TOOLCHAIN=$PREFIX "$PICOLIBC/scripts/gen-mc6809-cross.sh" >/dev/null
trap '"$PICOLIBC/scripts/gen-mc6809-cross.sh" >/dev/null 2>&1 || true' EXIT

COMMON="-Dformat-default=integer -Dprintf-aliases=false -Dstdio-float=false
        -Dposix-extensions=false -Dsearch-extensions=true -Dwant-libm=false
        -Dmb-capable=false -Dio-long-long=false -Dtests=false
        -Doptimization=s"

# build_variant TRIPLE SUBDIR CROSS [meson options...]
#   SUBDIR is "" for a triple's default library, or the multilib directory
#   under it -- `hd6309`, say -- for one a rule in multilib.yaml selects.
build_variant() {
  triple=$1; subdir=$2; cross=$3; shift 3
  sysroot=$PREFIX/lib/clang-runtimes/$triple${subdir:+/$subdir}
  build=$PREFIX/.build-$triple${subdir:+-$subdir}
  say "building picolibc for $triple"
  rm -rf "$build" "$sysroot"
  # shellcheck disable=SC2086
  meson setup "$build" "$PICOLIBC" --cross-file "$PICOLIBC/scripts/$cross" \
      -Dprefix="$sysroot" --libdir=lib --includedir=include \
      $COMMON "$@" >"$build.log" 2>&1 \
    || { echo "$0: meson setup failed for $triple; see $build.log" >&2; exit 1; }
  meson install -C "$build" --quiet >>"$build.log" 2>&1 \
    || { echo "$0: install failed for $triple; see $build.log" >&2; exit 1; }
  # A multilib variant shares the triple's headers: they are the same
  # headers, and a second copy is a megabyte saying so.
  [ -n "$subdir" ] && rm -rf "$sysroot/include"
  rm -rf "$build" "$build.log"
}

# The default library for each triple, and one variant: a bare-metal library
# built for the 6309, which multilib.yaml below selects when -mcpu says so.
# Every further variant is another full picolibc build here and another cell
# to be sure of, so they earn their place one at a time.
build_variant mc6809-unknown-unknown "" cross-clang-mc6809-unknown-elf.txt
build_variant mc6809-unknown-os9 "" cross-clang-mc6809-unknown-os9.txt \
    -Dpicocrt=false -Dsemihost=false -Dos-os9=true -Dposix-console=true

# DECB: a program EXECed from Disk Extended Color BASIC.  Its console is the
# ROM's, and nothing in this tree can run one -- the check below builds these
# and looks at the envelope rather than pretending to test them.
build_variant mc6809-unknown-decb "" cross-clang-mc6809-unknown-decb.txt \
    -Dpicocrt=false -Dsemihost=false -Dos-decb=true -Dposix-console=true

build_variant mc6809-unknown-unknown hd6309 cross-clang-mc6809-unknown-elf.txt \
    -Dc_args=-mcpu=hd6309 -Dcpp_args=-mcpu=hd6309 \
    -Dc_link_args=-mcpu=hd6309 -Dcpp_link_args=-mcpu=hd6309

# What chooses between them.  clang puts <sysroot>/<Dir> on the library path
# ahead of the default, so Dir names the directory holding the libraries and
# the default stays behind it as the fallback.
say "writing multilib.yaml"
cat > "$PREFIX/lib/clang-runtimes/mc6809-unknown-unknown/multilib.yaml" <<'YAML'
# Which library a link gets.  The flags a rule can match on are the ones
# clang works out for this target: the triple, the relocation model, -mcpu=
# and whether the link is doing whole-program optimisation.
MultilibVersion: 1.0

Variants:
# The default library needs a rule of its own, or a plain build matches
# nothing and clang says so on every compile.
- Dir: lib
  Flags: [--target=mc6809-unknown-unknown, -mcpu=mc6809]
- Dir: hd6309/lib
  Flags: [--target=mc6809-unknown-unknown, -mcpu=hd6309]

Mappings: []
YAML

# ------------------------------------------------------------------- check
# A bundle that has not compiled and run a program is not known to work.
say "checking the result"
if "$PICOLIBC/scripts/check-mc6809-toolchain" "$PREFIX"; then
  say "rolled: $PREFIX ($(du -sh "$PREFIX" | cut -f1))"
else
  echo "$0: rolled, but it does not pass its own check" >&2
  exit 1
fi
