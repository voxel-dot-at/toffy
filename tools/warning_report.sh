#!/usr/bin/env bash
#
# warning_report.sh — count compiler warnings per area from a build log.
#
#   tools/warning_report.sh build.log        # or: ... - for stdin
#   cmake --build build -j"$(nproc)" 2>&1 | tools/warning_report.sh -
#
# Why this exists (P3-11): a full build of this tree emits 28 warnings and
# nothing has ever looked at them. modules/core is compiled -Werror since P3-4,
# so the only way the other 27 go down is if somebody sees them. This script
# makes the number visible on every CI run and holds the line where holding it
# is already achievable.
#
# Scope, stated per DOD 0 rule 9 (a check that cannot fail is not a gate, and a
# check whose scope is unstated cannot be trusted):
#
#   * Input is a *build log*, not the source tree. A warning is attributed to the
#     file the compiler named, which for a warning inside a header is the header,
#     not the translation unit. Warnings are counted once per emitted line, so a
#     header included by 10 TUs counts 10 times — that is what a build emits and
#     what a developer sees, so that is what is reported.
#   * Buckets are the areas that ship in libtoffy.so (modules/core,
#     modules/filters, modules/bta, libraries) plus apps/. Anything the compiler
#     names outside the repository (system headers, /usr/include, generated
#     protobuf-style code) goes to `external/` and never gates.
#   * Only modules/core gates, and only because P3-4 made 0 warnings reachable
#     there. Gating modules/filters would fail the build on 20-odd pre-existing
#     warnings and get this job disabled instead of acted on.
#
# Exit status: 1 if any warning is attributed to modules/core, else 0.
# CORE_WERROR=OFF builds still get the number printed; the gate is here as well
# as in the compiler flags so that the escape hatch stays visible.

set -uo pipefail

log=${1:--}
if [ "$log" = "-" ]; then
  data=$(cat)
else
  [ -r "$log" ] || { echo "warning_report.sh: cannot read '$log'" >&2; exit 2; }
  data=$(cat "$log")
fi

# Repository root, so absolute paths in the log become repo-relative. Empty when
# this runs outside a checkout, in which case paths are used as the compiler
# printed them.
root=$(git rev-parse --show-toplevel 2>/dev/null || true)

warn_lines=$(printf '%s\n' "$data" | grep -E ':[0-9]+:[0-9]+: warning:' || true)
total=$(printf '%s\n' "$warn_lines" | grep -c . || true)

# A report over an up-to-date build is vacuous: nothing was compiled, so nothing
# could warn, and "0 warnings" would be read as "clean tree" by whoever reads the
# log next. Say which of the two it is (DOD 0 rule 9: a check that cannot fail is
# not a gate, so do not let one pretend to have passed).
compiled=$(printf '%s\n' "$data" | grep -cE '\] Building (C|CXX|OBJC)' || true)

echo "warning report"
echo "=============="

if [ "$compiled" -eq 0 ]; then
  echo "  no compilation records in this log — the build was up to date, or the"
  echo "  log is not a build log. This report is vacuous, not a pass."
  exit 0
fi

if [ "$total" -eq 0 ]; then
  echo "  0 warnings across $compiled compiled units"
  exit 0
fi

table=$(printf '%s\n' "$warn_lines" | sed -E 's/:[0-9]+:[0-9]+: warning:.*$//' | while read -r path; do
  [ -n "$path" ] || continue
  if [ -n "$root" ]; then
    path=${path#"$root"/}
  fi
  # strip any leading ./ or an absolute path we could not make relative
  path=${path#./}
  case "$path" in
    modules/core/*)    echo "modules/core" ;;
    modules/filters/*) echo "modules/filters" ;;
    modules/bta/*)     echo "modules/bta" ;;
    libraries/*)       echo "libraries" ;;
    apps/*)            echo "apps" ;;
    */*)               echo "external/" ;;
    *)                 echo "external/" ;;
  esac
done | sort | uniq -c | sort -rn)

printf '%s\n' "$table" | while read -r n area; do
  [ -n "${n:-}" ] || continue
  printf '  %4s  %s\n' "$n" "$area"
done
echo "  ----"
printf '  %4s  total, over %s compiled units\n' "$total" "$compiled"

core=$(printf '%s\n' "$table" | awk '$2=="modules/core"{print $1; found=1} END{if(!found) print 0}')

echo
echo "worst files:"
printf '%s\n' "$warn_lines" | sed -E 's/:[0-9]+:[0-9]+: warning:.*$//' | while read -r path; do
  [ -n "$path" ] || continue
  if [ -n "$root" ]; then path=${path#"$root"/}; fi
  echo "${path#./}"
done | sort | uniq -c | sort -rn | head -8 | sed 's/^/  /'

echo
echo "by message:"
printf '%s\n' "$warn_lines" | sed -E 's/.* warning: //' | sed -E 's/ \[-W/|[-W/' \
  | awk -F'|' '{print $2}' | tr -d '[]' | sort | uniq -c | sort -rn | head -10 | sed 's/^/  /'

if [ "${core:-0}" -gt 0 ]; then
  echo
  echo "::error::modules/core emitted $core warning(s); it is compiled -Werror (P3-4)." \
       "Fix them, or configure with -DCORE_WERROR=OFF and say why in the PR."
  exit 1
fi

echo "modules/core: 0 warnings (gated). The rest are reported, not gated — see"
echo "cleanup/plan/second-pass.md P3-11 and cleanup/counters.md 15/15a."
exit 0
