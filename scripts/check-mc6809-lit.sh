#!/bin/sh
# check-mc6809-lit.sh — run every MC6809 lit test a built tree can run.
#
#   scripts/check-mc6809-lit.sh BUILD_DIR [MIN_PASS [EXTRA_TEST_PATH...]]
#
# MIN_PASS defaults to what a tree built without lldb passes with no
# simulator asked for (313 when this was written; a tree with lldb passes
# two more).  Raise it as tests are added; pass 0 to check a partial build.
# EXTRA_TEST_PATHs are run with the rest: the bench adds the OS-9 runtime
# suite, which boots NitrOS-9 and has its own opt-in.
#
# The one list of the fork's own lit tests, wherever they live (llvm, clang,
# lld), so that CI, each release host and a developer run the same set.
#
# Tests that run programs on a simulator are not run unless asked for, the
# way their lit configs ask: USIM09BATCH (and MAMELLVM6309 for the 6309 on
# MAME) in the environment.  Without them those tests report UNSUPPORTED,
# as do the lldb tests in a tree built without lldb and the tests that want
# an assertions build in one without.
#
# Exit status 0 means no test failed, none was unresolved or timed out or
# unexpectedly passed, and at least MIN_PASS passed: a suite that stops
# being discovered must not read as a pass.
# The full lit output goes to $LIT_LOG (default ./lit.log); a summary goes
# to $GITHUB_STEP_SUMMARY when that is set.
set -eu

usage() { echo "usage: $0 BUILD_DIR [MIN_PASS [EXTRA_TEST_PATH...]]" >&2; exit 2; }
[ $# -ge 1 ] || usage
build=$1; shift
min_pass=${1:-313}; [ $# -gt 0 ] && shift
log=${LIT_LOG:-lit.log}
case $log in /*|[A-Za-z]:*) ;; *) log="$PWD/$log" ;; esac
src=$(cd "$(dirname "$0")/.." && pwd)

# On Windows the build has bin/llvm-lit.py and no bin/llvm-lit.
if [ -f "$build/bin/llvm-lit" ]; then
  lit="$build/bin/llvm-lit"
elif [ -f "$build/bin/llvm-lit.py" ]; then
  lit="${PYTHON:-python} $build/bin/llvm-lit.py"
else
  echo "$0: no llvm-lit in $build/bin" >&2; exit 2
fi

cd "$src"
set +e
# shellcheck disable=SC2086 # $lit may be "python .../llvm-lit.py"
$lit --no-progress-bar -v \
  llvm/test/CodeGen/MC6809 \
  llvm/test/MC/MC6809 \
  llvm/test/MC/Disassembler/MC6809 \
  llvm/test/DebugInfo/MC6809 \
  llvm/test/LTO/Resolution/MC6809 \
  llvm/test/tools/llvm-objdump/MC6809 \
  llvm/test/tools/mc6809-cfi \
  clang/test/CodeGen/MC6809 \
  clang/test/Driver/mc6809*.c \
  clang/test/Preprocessor/mc6809*.c \
  clang/test/Sema/asm-mc6809-*.c \
  clang/test/Sema/mc6809-*.c \
  clang/test/Misc/pragma-attribute-supported-attributes-list.test \
  lld/test/ELF/mc6809-*.s \
  lld/test/ELF/lto/mc6809-*.test \
  "$@" \
  > "$log" 2>&1
lit_status=$?
set -e

count() { # LABEL -- the count lit printed for LABEL, or 0
  n=$(sed -n "s/^ *$1 *: *\([0-9]*\).*/\1/p" "$log" | tail -1)
  echo "${n:-0}"
}
passed=$(count Passed)
failed=$(count Failed)
unsupported=$(count Unsupported)
xfail=$(count 'Expectedly Failed')
xpass=$(count 'Unexpectedly Passed')
unresolved=$(count Unresolved)
timedout=$(count 'Timed Out')

summary() {
  echo "### MC6809 lit tests"
  echo
  echo "| | |"
  echo "|---|---|"
  echo "| passed | $passed |"
  echo "| failed | $failed |"
  echo "| expectedly failed | $xfail |"
  echo "| unexpectedly passed | $xpass |"
  echo "| unresolved | $unresolved |"
  echo "| timed out | $timedout |"
  echo "| unsupported (simulator not asked for, no lldb, or no assertions) | $unsupported |"
}
summary
[ -n "${GITHUB_STEP_SUMMARY:-}" ] && summary >> "$GITHUB_STEP_SUMMARY"

bad=0
if [ $((failed + xpass + unresolved + timedout)) -ne 0 ]; then
  grep -E '^(FAIL|XPASS|UNRESOLVED|TIMEOUT): ' "$log" || true
  echo "error: lit reported failures" >&2; bad=1
fi
if [ "$passed" -lt "$min_pass" ]; then
  echo "error: only $passed tests passed, expected at least $min_pass -- tests are disappearing rather than failing" >&2
  bad=1
fi
# A lit that failed for a reason not counted above (it died, or a test it
# could not even start) passes its own status on.
if [ "$bad" -eq 0 ] && [ "$lit_status" -ne 0 ]; then
  tail -20 "$log"
  echo "error: lit exited $lit_status" >&2
  exit "$lit_status"
fi
exit "$bad"
