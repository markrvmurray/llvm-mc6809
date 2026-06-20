#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run-os9-case.sh <source.c> <check-prefix>" >&2
  exit 2
}

[ "$#" -eq 2 ] || usage

src=$1
check_prefix=$2

: "${MC6809_OS9_CLANG:=clang}"
: "${MAME:=mame}"
: "${OS9_TOOL:=os9}"
: "${FILECHECK:=FileCheck}"
: "${MAME_ROMPATH:?need MAME_ROMPATH}"
: "${OS9_BOOT_DSK:?need OS9_BOOT_DSK}"

case_name=$(basename "$src")
case_name=${case_name%.*}
case_name=$(printf '%s' "$case_name" | tr -cd 'A-Za-z0-9_')
[ -n "$case_name" ] || case_name=testcase
workdir=$(mktemp -d "${TMPDIR:-/tmp}/mc6809-os9-runtime.XXXXXX")
trap 'rm -rf "$workdir"' EXIT

module="$workdir/$case_name"
disk="$workdir/test.dsk"
startup="$workdir/startup"
out="$workdir/$case_name.out"

"$MC6809_OS9_CLANG" -target mc6809-unknown-os9 "$src" -o "$module"

cp "$OS9_BOOT_DSK" "$disk"
"$OS9_TOOL" copy -r "$module" "$disk,CMDS/$case_name" >/dev/null
"$OS9_TOOL" attr -q -e -pe -pr "$disk,CMDS/$case_name" >/dev/null

{
  printf 'echo * %s *\n' "$case_name"
  printf 'link shell\n'
  printf 'load utilpak1\n'
  printf 'shell %s >>>-%s.out\n' "$case_name" "$case_name"
  printf 'echo CIDONE >>>-zzdone.out\n'
} >"$startup"

"$OS9_TOOL" copy -l -r "$startup" "$disk,startup" >/dev/null
"$OS9_TOOL" attr -q -npe -npw -pr -ne -w -r "$disk,startup" >/dev/null

SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  "$MAME" coco3 -rompath "$MAME_ROMPATH" -skip_gameinfo \
    -ext fdc -ext:fdc:wd17xx:0 525qd -flop1 "$disk" \
    -video none -sound none -nothrottle \
    -cfg_directory "$workdir/cfg" \
    -nvram_directory "$workdir/nvram" \
    -snapshot_directory "$workdir/snap" \
    -comment_directory "$workdir/cmt" \
    -autoboot_delay "${MAME_AUTOBOOT_DELAY:-8}" \
    -autoboot_command "DOS\n" \
    -seconds_to_run "${MAME_SECONDS_TO_RUN:-120}" >/dev/null 2>&1

if ! "$OS9_TOOL" dir "$disk" 2>/dev/null | grep -q zzdone; then
  echo "MAME run did not produce zzdone sentinel" >&2
  "$OS9_TOOL" dir "$disk" >&2 || true
  if "$OS9_TOOL" dir "$disk" 2>/dev/null | grep -q "$case_name.out"; then
    echo "----- $case_name.out -----" >&2
    "$OS9_TOOL" list "$disk,$case_name.out" >&2 || true
    echo "--------------------------" >&2
  fi
  exit 1
fi

"$OS9_TOOL" list "$disk,$case_name.out" >"$out"
"$FILECHECK" "$src" --check-prefix="$check_prefix" <"$out"
