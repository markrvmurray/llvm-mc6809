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
         "$LLVM_BUILD"/bin/llvm-readobj "$LLVM_BUILD"/bin/llvm-size \
         "$LLVM_BUILD"/bin/llvm-strip; do
  [ -e "$f" ] || continue
  base=$(basename "$f")
  if [ -L "$f" ]; then
    # Upstream ships several of these as links to one binary -- ranlib is
    # llvm-ar, strip is llvm-objcopy, readelf is llvm-readobj -- and they
    # behave differently depending on the name they were called by.  Copy
    # the link, not another copy of the binary.
    ln -sf "$(basename "$(readlink "$f")")" "$PREFIX/bin/$base"
  else
    cp -p "$f" "$PREFIX/bin/"
  fi
done
( cd "$PREFIX/bin"
  clangbin=$(ls clang-* 2>/dev/null | head -1)
  [ -n "$clangbin" ] && ln -sf "$clangbin" clang
  [ -f lld ] && ln -sf lld ld.lld
  true )

# Names for the target, so nobody has to spell a triple to compile a program.
# Symlinks, not wrapper scripts: clang resolves its own path to the real
# binary before working out where its sysroot is, so a script that execs a
# clang from elsewhere would send it looking beside *that* one.
say "naming the tools for the target"
( cd "$PREFIX/bin"
  for t in clang ld.lld llvm-ar llvm-nm llvm-objcopy llvm-objdump \
           llvm-ranlib llvm-readelf llvm-size llvm-strip; do
    [ -e "$t" ] || continue
    short=${t#llvm-}
    short=${short%.lld}
    ln -sf "$t" "mc6809-$short"
  done
  # And one per target, which the driver reads off its own name.
  for target in os9 decb; do
    ln -sf clang "mc6809-$target-clang"
  done )

# Running one.  The simulators are not part of this bundle -- they are other
# people's programs -- so this finds one and says plainly when it cannot.
cat > "$PREFIX/bin/mc6809-run" <<'RUNNER'
#!/bin/sh
# mc6809-run — run a program built by this toolchain, on whatever simulator
# is to hand.  Which one depends on what the file is, and the file says:
#
#   7f 45 4c 46   an ELF: bare metal, on usim09batch
#   87 cd         an OS-9 program module: NitrOS-9, on usim09pt
#   00            a DECB binary: a CoCo, which nothing here can be
set -eu
[ $# -ge 1 ] || { echo "usage: mc6809-run PROGRAM [args...]" >&2; exit 2; }
prog=$1; shift
magic=$(od -An -N2 -tx1 "$prog" | tr -d ' \n')
case "$magic" in
  7f45)
    command -v usim09batch >/dev/null 2>&1 || {
      echo "mc6809-run: usim09batch is not on the PATH; it is part of USim," >&2
      echo "            not of this toolchain." >&2
      exit 127; }
    exec usim09batch "$prog" "$@" ;;
  87cd)
    command -v run-mc6809-os9 >/dev/null 2>&1 || {
      echo "mc6809-run: an OS-9 module needs run-mc6809-os9 and a NitrOS-9" >&2
      echo "            image; both live with picolibc and USim, not here." >&2
      exit 127; }
    exec run-mc6809-os9 "$prog" "$@" ;;
  00*)
    echo "mc6809-run: this is a DECB binary, for a CoCo.  Nothing here can" >&2
    echo "            run one: LOADM it on a machine or in an emulator." >&2
    exit 127 ;;
  *)
    echo "mc6809-run: $prog is not a program this toolchain builds" >&2
    exit 2 ;;
esac
RUNNER
chmod +x "$PREFIX/bin/mc6809-run"

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
# What this bundle is, taken from the compiler rather than from a number kept
# here: two records of a version disagree eventually, and the one the user
# gets is the one the compiler prints.
"$PREFIX/bin/clang" --version | head -1 > "$PREFIX/VERSION"
say "this is $(cat "$PREFIX/VERSION")"

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

# The default library for each triple, and a 6309 variant for the two targets
# that have a machine to run on.  multilib.yaml below picks a variant when
# -mcpu says so.  Every further variant is another full picolibc build here
# and another cell to be sure of, so they earn their place one at a time.
OS9_OPTS="-Dpicocrt=false -Dsemihost=false -Dos-os9=true -Dposix-console=true"
HD6309_OPTS="-Dc_args=-mcpu=hd6309 -Dcpp_args=-mcpu=hd6309
             -Dc_link_args=-mcpu=hd6309 -Dcpp_link_args=-mcpu=hd6309"

# shellcheck disable=SC2086
build_variant mc6809-unknown-unknown "" cross-clang-mc6809-unknown-elf.txt
# shellcheck disable=SC2086
build_variant mc6809-unknown-os9 "" cross-clang-mc6809-unknown-os9.txt $OS9_OPTS

# DECB: a program EXECed from Disk Extended Color BASIC.  Its console is the
# ROM's, and nothing in this tree can run one -- the check below builds these
# and looks at the envelope rather than pretending to test them.
build_variant mc6809-unknown-decb "" cross-clang-mc6809-unknown-decb.txt \
    -Dpicocrt=false -Dsemihost=false -Dos-decb=true -Dposix-console=true

# NitrOS-9 runs on a 6309 as readily as bare metal does -- a CoCo 3 with the
# chip swapped is the usual way people meet one -- so the OS-9 sysroot gets
# the same variant.  DECB has no second library: nothing has run the first.
# shellcheck disable=SC2086
build_variant mc6809-unknown-unknown hd6309 cross-clang-mc6809-unknown-elf.txt \
    $HD6309_OPTS
# shellcheck disable=SC2086
build_variant mc6809-unknown-os9 hd6309 cross-clang-mc6809-unknown-os9.txt \
    $OS9_OPTS $HD6309_OPTS

# What chooses between them.  clang puts <sysroot>/<Dir> on the library path
# ahead of the default, so Dir names the directory holding the libraries and
# the default stays behind it as the fallback.  Each sysroot that has a
# variant needs its own copy: the file is read from the sysroot clang picked
# for the triple, and says nothing about which triple that was.
write_multilib() {
cat > "$PREFIX/lib/clang-runtimes/$1/multilib.yaml" <<'YAML'
# Which library a link gets.  A rule matches when its flags are among the ones
# clang worked out for the link.
#
# The triple is deliberately not one of them: this file already lives inside
# the sysroot for a triple, and naming it here would only mean spelling every
# way of writing that triple -- `mc6809-clang` computes `mc6809`, the long
# name computes `mc6809-unknown-unknown`, and a rule naming one would not
# match the other.
MultilibVersion: 1.0

Variants:
# The default needs a rule of its own, or a plain build matches nothing and
# clang says so on every compile.
- Dir: lib
  Flags: [-mcpu=mc6809]
- Dir: hd6309/lib
  Flags: [-mcpu=hd6309]

Mappings: []
YAML
}

say "writing multilib.yaml"
write_multilib mc6809-unknown-unknown
write_multilib mc6809-unknown-os9

# `mc6809-clang` computes the triple `mc6809`, not `mc6809-unknown-unknown`,
# and the sysroot is named after the triple -- so without these the friendly
# name would find no library and say nothing about it.  `mc6809-elf` is the
# other spelling people reach for.
say "linking the bare-metal directories under the names the short forms compute"
( cd "$PREFIX/lib/clang-runtimes"
  for alias in mc6809 mc6809-unknown-unknown-elf mc6809-elf; do
    [ -e "$alias" ] || ln -sf mc6809-unknown-unknown "$alias"
  done )
# The compiler's own runtime directory is named after the triple as well, so
# the short name has to find the builtins there too.
( cd "$PREFIX"/lib/clang/*/lib 2>/dev/null || exit 0
  for alias in mc6809 mc6809-unknown-unknown-elf mc6809-elf; do
    [ -e "$alias" ] || ln -sf mc6809-unknown-unknown "$alias"
  done )

# ------------------------------------------------------------------- check
# A bundle that has not compiled and run a program is not known to work.
say "checking the result"
"$PICOLIBC/scripts/check-mc6809-toolchain" "$PREFIX"
case $? in
  0) say "rolled: $PREFIX ($(du -sh "$PREFIX" | cut -f1))" ;;
  # The check says 2 when it could not run -- no simulator, no NitrOS-9 image.
  # That is not a bundle that failed; it is a bundle nobody has tried yet, and
  # calling it either would be wrong.  Still not a success: a release that has
  # never run a program is not one.
  2) echo "$0: rolled, but nothing here could run it -- see above." >&2
     echo "  The bundle is at $PREFIX; it is untested, not broken." >&2
     exit 2 ;;
  *) echo "$0: rolled, but it does not pass its own check" >&2
     exit 1 ;;
esac
