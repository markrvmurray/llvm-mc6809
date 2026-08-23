#!/usr/bin/env bash
# Compile one OS-9 test case, run it twice under a real NitrOS-9 boot and
# FileCheck the console output. See ../lit.cfg.py for the environment.
set -euo pipefail

usage() {
  echo "usage: run-os9-case.sh <source.c> <check-prefix>" >&2
  exit 2
}

[ "$#" -eq 2 ] || usage

src=$1
check_prefix=$2

: "${MC6809_OS9_CLANG:=clang}"
: "${OS9_TOOL:=os9}"
: "${FILECHECK:=FileCheck}"
: "${MC6809_OS9_BACKEND:=usim}"

case_name=$(basename "$src")
case_name=${case_name%.*}
case_name=$(printf '%s' "$case_name" | tr -cd 'A-Za-z0-9_')
[ -n "$case_name" ] || case_name=testcase
# The floating-point module.  A program that does any floating point links
# FPO9 at start-up, loading it from the execution directory when it is not
# already in memory, so it goes on the disk beside the test.  The compiler
# stages it next to the OS-9 linker script.
if [ -z "${MC6809_FP_MODULE:-}" ]; then
  MC6809_FP_MODULE=$("$MC6809_OS9_CLANG" -target mc6809-unknown-os9 \
                     -print-resource-dir 2>/dev/null)/lib/mc6809-unknown-os9/FPO9
fi

# Command-line words for the program: an optional `// ARGS: ...` line.
case_args=$(sed -n 's|^// *ARGS: *||p' "$src" | head -1)

# A case may name the level it wants (`// LEVEL: 1`) when what it checks only
# shows up at that level; the images come from the recipes tree, one directory
# per level.  Without the line, whatever the caller configured stands.
case_level=$(sed -n 's|^// *LEVEL: *||p' "$src" | head -1)
if [ -n "$case_level" ]; then
  : "${NITROS9_RECIPES:=$HOME/Documents/NitrOS-9/nitros9/recipes/picothing}"
  _lvldir="$NITROS9_RECIPES/l$case_level"
  NITROS9_BOOT_DSK="$_lvldir/NOS9_6809_L${case_level}_DEV_picothing_x0.dsk"
  NITROS9_FIRMWARE="$_lvldir/rel_picothing.srec"
  export NITROS9_BOOT_DSK NITROS9_FIRMWARE
fi
workdir=$(mktemp -d "${TMPDIR:-/tmp}/mc6809-os9-runtime.XXXXXX")
if [ "${KEEP_OS9_TEST_WORKDIR:-0}" != 1 ]; then
  trap 'rm -rf "$workdir"' EXIT
else
  echo "workdir=$workdir" >&2
fi

module="$workdir/$case_name"
out="$workdir/$case_name.out"

# A case may name compiler options (`// CFLAGS: -Os`).  Nothing needs them
# to be exercised -- the default is no optimisation, which is the harder
# case for the backend and worth keeping that way.
case_cflags=$(sed -n 's|^// *CFLAGS: *||p' "$src" | head -1)

"$MC6809_OS9_CLANG" -target mc6809-unknown-os9 ${case_cflags:-} "$src" -o "$module"

#-----------------------------------------------------------------------------
# usim09pt: NitrOS-9 Level 1/2 (Pico-Thing) booted from an IDE image on i0,
# the test on a fresh IDE image on j0, the console driven with --input.
#-----------------------------------------------------------------------------
run_usim() {
  : "${NITROS9_USIM09PT:?need NITROS9_USIM09PT}"
  : "${NITROS9_BOOT_DSK:?need NITROS9_BOOT_DSK}"
  : "${NITROS9_FIRMWARE:?need NITROS9_FIRMWARE}"
  # Cycles before the first input byte (the login prompt is up by ~78M),
  # instruction cap for the whole run, and the wall-clock cap.
  : "${NITROS9_INPUT_FIRST:=78000000}"
  : "${NITROS9_TIMEOUT:=600000000}"
  : "${NITROS9_WALL_SECONDS:=300}"

  local boot="$workdir/boot.dsk" disk="$workdir/test.dsk" log="$workdir/usim.log"
  cp "$NITROS9_BOOT_DSK" "$boot"
  "$OS9_TOOL" format "$disk" -e -l4096 -q -nTEST >/dev/null
  "$OS9_TOOL" makdir "$disk,CMDS" >/dev/null
  "$OS9_TOOL" copy -r "$module" "$disk,CMDS/$case_name" >/dev/null
  "$OS9_TOOL" attr -q -e -pe -pr "$disk,CMDS/$case_name" >/dev/null
  if [ -f "$MC6809_FP_MODULE" ]; then
    "$OS9_TOOL" copy -r "$MC6809_FP_MODULE" "$disk,CMDS/FPO9" >/dev/null
    "$OS9_TOOL" attr -q -e -pe -pr "$disk,CMDS/FPO9" >/dev/null
  fi

  # Empty user name logs in; the program runs twice (with the case's
  # `// ARGS:` words, if any); the echo is the sentinel that says the shell
  # got its prompt back.
  local cmd="$case_name"
  [ -n "$case_args" ] && cmd="$case_name $case_args"
  local input="\\r\\r\\rchx /j0/cmds\\r$cmd\\r$cmd\\recho CIDONE\\r"

  (cd "$workdir" && "$NITROS9_USIM09PT" --timeout="$NITROS9_TIMEOUT" \
      --input-first="$NITROS9_INPUT_FIRST" --input-step=500000 \
      --input="$input" -f "$workdir/picothing.fram" \
      -d "$boot" -D "$disk" "$NITROS9_FIRMWARE" >"$log" 2>&1) &
  local pid=$! waited=0
  while kill -0 "$pid" 2>/dev/null; do
    if grep -aq 'CIDONE' "$log" 2>/dev/null; then
      break
    fi
    if [ "$waited" -ge "$NITROS9_WALL_SECONDS" ]; then
      break
    fi
    sleep 2
    waited=$((waited + 2))
  done
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true

  if ! grep -aq 'CIDONE' "$log"; then
    echo "NitrOS-9 run did not reach the sentinel" >&2
    echo "----- usim.log (tail) -----" >&2
    tail -c 4000 "$log" | tr -d '\000' >&2 || true
    echo "---------------------------" >&2
    exit 1
  fi
  # The console output after the first command echo, without the NUL bytes
  # the console emits between lines.
  tr -d '\000\r' <"$log" | sed -n "/chx \/j0\/cmds/,\$p" >"$out"
}

#-----------------------------------------------------------------------------
# MAME coco3 (needs CoCo 3 ROMs): the test on a copy of a NitrOS-9 floppy,
# started from a startup file, its output collected in a file on the disk.
#-----------------------------------------------------------------------------
run_mame() {
  : "${MAME:=mame}"
  : "${MAME_ROMPATH:?need MAME_ROMPATH}"
  : "${OS9_BOOT_DSK:?need OS9_BOOT_DSK}"

  local disk="$workdir/test.dsk" startup="$workdir/startup" mame_log="$workdir/mame.log"
  cp "$OS9_BOOT_DSK" "$disk"
  "$OS9_TOOL" copy -r "$module" "$disk,CMDS/$case_name" >/dev/null
  "$OS9_TOOL" attr -q -e -pe -pr "$disk,CMDS/$case_name" >/dev/null

  {
    printf 'link shell\n'
    printf 'load utilpak1 %s\n' "$case_name"
    printf 'echo * %s *\n' "$case_name"
    printf 'shell %s %s >>>-%s.out\n' "$case_name" "$case_args" "$case_name"
    printf 'shell %s %s >>>+%s.out\n' "$case_name" "$case_args" "$case_name"
    printf 'echo CIDONE >>>-zzdone.out\n'
  } >"$startup"

  "$OS9_TOOL" copy -l -r "$startup" "$disk,startup" >/dev/null
  "$OS9_TOOL" attr -q -npe -npw -pr -ne -w -r "$disk,startup" >/dev/null

  local mame_args=(
    coco3 -rompath "$MAME_ROMPATH" -skip_gameinfo
    -ext fdc -ext:fdc:wd17xx:0 525qd -flop1 "$disk"
    -sound none
    -cfg_directory "$workdir/cfg"
    -nvram_directory "$workdir/nvram"
    -snapshot_directory "$workdir/snap"
    -comment_directory "$workdir/cmt"
    -autoboot_delay "${MAME_AUTOBOOT_DELAY:-8}"
    -autoboot_command "DOS\n"
    -seconds_to_run "${MAME_SECONDS_TO_RUN:-120}"
  )

  local mame_rc
  if [ "${MAME_VISIBLE:-0}" = 1 ]; then
    mame_args+=(-window -nothrottle)
    "$MAME" "${mame_args[@]}" 2>&1 | tee "$mame_log"
    mame_rc=${PIPESTATUS[0]}
  else
    mame_args+=(-video none -nothrottle)
    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
      "$MAME" "${mame_args[@]}" >"$mame_log" 2>&1
    mame_rc=$?
  fi

  if [ "$mame_rc" -ne 0 ]; then
    echo "MAME exited with status $mame_rc" >&2
    echo "----- mame.log -----" >&2
    cat "$mame_log" >&2 || true
    echo "--------------------" >&2
    exit "$mame_rc"
  fi

  if ! "$OS9_TOOL" dir "$disk" 2>/dev/null | grep -q zzdone; then
    echo "MAME run did not produce zzdone sentinel" >&2
    echo "workdir=$workdir" >&2
    "$OS9_TOOL" dir "$disk" >&2 || true
    if "$OS9_TOOL" dir "$disk" 2>/dev/null | grep -q "$case_name.out"; then
      echo "----- $case_name.out -----" >&2
      "$OS9_TOOL" list "$disk,$case_name.out" >&2 || true
      echo "--------------------------" >&2
    fi
    exit 1
  fi

  "$OS9_TOOL" list "$disk,$case_name.out" >"$out"
}

case "$MC6809_OS9_BACKEND" in
  usim) run_usim ;;
  mame) run_mame ;;
  *) echo "unknown MC6809_OS9_BACKEND '$MC6809_OS9_BACKEND'" >&2; exit 2 ;;
esac

"$FILECHECK" "$src" --check-prefix="$check_prefix" <"$out"
