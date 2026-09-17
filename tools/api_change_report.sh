#!/usr/bin/env bash
#
# Report public API changes between a base ref and HEAD.
#
# The library version *and* SOVERSION are derived from `git describe`
# (CMakeLists.txt:32, :357), so an API change that is not accompanied by a new
# tag ships under the previous SOVERSION. Downstream binaries then fail to
# resolve symbols at load time while the SONAME still claims the old version.
# This script makes that visible and forces the version decision.
#
# Usage: tools/api_change_report.sh [BASE_REF]
#        BASE_REF defaults to the most recent tag reachable from HEAD.
#
# Exit status:
#   0  no public header changed
#   1  public headers changed -- a version/tag decision is required
#   2  could not determine a base ref
set -uo pipefail

BASE="${1:-$(git describe --tags --abbrev=0 2>/dev/null || true)}"
if [ -z "${BASE}" ]; then
    echo "api_change_report: no base ref given and no tag reachable from HEAD" >&2
    exit 2
fi

# Validate the base ref. Without this a typo'd ref makes `git diff` fail, the
# changed list comes back empty and the script would report "no API changes" --
# failing open, which is the one thing a gate must never do.
if ! git rev-parse --verify --quiet "${BASE}^{commit}" >/dev/null; then
    echo "api_change_report: '${BASE}' is not a valid commit" >&2
    exit 2
fi

# Public API is everything installed under an include/ directory.
if ! CHANGED=$(git diff --name-only "${BASE}..HEAD" -- '*/include/*.hpp' '*/include/*.h' 2>/dev/null); then
    echo "api_change_report: git diff against ${BASE} failed" >&2
    exit 2
fi

if [ -z "${CHANGED}" ]; then
    echo "No public API changes between ${BASE} and HEAD."
    exit 0
fi

echo "Public headers changed between ${BASE} and $(git rev-parse --short HEAD):"
echo "${CHANGED}" | sed 's/^/  /'
echo
echo "Changed declarations:"
for f in ${CHANGED}; do
    # Advisory detail only. -w drops pure reindentation; the greps drop diff
    # headers, comments, blank lines, preprocessor lines and brace-only lines,
    # which are not signatures. Text diffing cannot be exact -- the exit code
    # is the contract, this listing is a hint.
    decls=$(git diff -U0 -w "${BASE}..HEAD" -- "${f}" \
        | grep -E '^[+-]' \
        | grep -vE '^(\+\+\+|---)' \
        | grep -vE '^[+-][[:space:]]*(\*|/\*|//|\*/|@)' \
        | grep -vE '^[+-][[:space:]]*#' \
        | grep -vE '^[+-][[:space:]]*[{};]+[[:space:]]*(//.*)?$' \
        | grep -vE '^[+-][[:space:]]*$' || true)
    if [ -n "${decls}" ]; then
        echo "  ${f}"
        echo "${decls}" | sed 's/^/    /'
    fi
done

cat <<'EOF'

A public header changed. Before this ships:
  1. Decide whether the change is source- and ABI-compatible.
  2. Bump the version tag. Because the version comes from `git describe`, an
     untagged API change keeps the previous SOVERSION and breaks downstream
     binaries silently at load time.
  3. Add a deprecated alias, or state that this is a breaking release.
See DOD.md 2.4.
EOF
exit 1
