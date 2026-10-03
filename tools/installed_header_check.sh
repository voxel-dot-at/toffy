#!/usr/bin/env bash
#
# Compile-check every installed public header: standalone, and included twice.
#
# `make install` ships whatever is under each */include/ directory, and nothing in the
# build ever included most of those files: a header whose only consumer was a filter that
# was later deleted keeps shipping, keeps rotting, and stays invisible because the library
# still builds. Three installed headers under toffy/web/ reached that state and were only
# found by hand (cleanup/findings/build.md N1); two more in libraries/graphs/ were found
# the same way while deleting them. This is that hand check, run on every CI build.
#
# Two passes, because they catch different bugs:
#
#   once   - the header as the *only* include of a translation unit, which is what a
#            downstream user does. A header that compiles only after some other header
#            happened to pull in its dependencies is broken for that user.
#   twice  - the same header included twice in one translation unit. Without an include
#            guard, a typedef or a struct definition is a redefinition error, and every
#            build in this repository can include such a header exactly once and stay
#            green. toffy/bta/FrameHeader.hpp was one of those (X2b).
#
# Failure classification is the whole design. A missing header is only a finding when the
# missing file is ours:
#
#   toffy/…, toffy_web/…       -> BROKEN. A public header including a toffy header that
#                                 does not exist cannot be included by anyone. This is the
#                                 N1 bug class.
#   opencv2/…, pcl/…, boost/…  -> SKIPPED. The dependency is not installed in this
#                                 configuration; the header is not being tested, and the
#                                 run says so instead of failing.
#   any other compiler error   -> BROKEN. Undeclared identifier, missing include, …
#
# Usage: tools/installed_header_check.sh <include_dir> [extra compiler flags…]
#   e.g. tools/installed_header_check.sh /tmp/ti/usr/local/include $(pkg-config --cflags opencv4)
#
# Environment:
#   HEADER_CHECK_SINGLE=1   run the once-pass only (the double pass doubles the runtime)
#
# Exit status:
#   0  every header that could be tested compiles, once and twice
#   1  at least one header is broken
#   2  usage error / nothing was tested (a check that tests nothing is not a gate)

set -uo pipefail

INC="${1:-}"
if [ -z "${INC}" ] || [ ! -d "${INC}" ]; then
    echo "usage: ${0} <include_dir> [extra compiler flags…]" >&2
    exit 2
fi
shift
EXTRA="$*"
CXX="${CXX:-g++}"
STD="${CXXSTD:--std=c++17}"
SINGLE="${HEADER_CHECK_SINGLE:-0}"

tmpdir=$(mktemp -d)
trap 'rm -rf "${tmpdir}"' EXIT

total=0
ok=0
skipped=0
broken=0
: > "${tmpdir}/broken"
: > "${tmpdir}/skipped"

# compile_tu <file> -> 0 ok, 1 broken, 2 skipped (missing third-party dependency)
compile_tu() {
    local tu="$1"
    if "${CXX}" ${STD} -fsyntax-only -I"${INC}" ${EXTRA} "${tu}" \
           > "${tmpdir}/err" 2>&1; then
        return 0
    fi
    # A missing include is reported as "fatal error: <file>: No such file or directory".
    local missing
    missing=$(grep -m1 -oE 'fatal error: [^:]+: No such file or directory' "${tmpdir}/err" \
                | sed -E 's/^fatal error: //; s/: No such file or directory$//')
    if [ -n "${missing}" ] && ! printf '%s' "${missing}" | grep -qE '^(toffy|toffy_)'; then
        return 2
    fi
    return 1
}

for h in $(find "${INC}" \( -name '*.hpp' -o -name '*.h' \) | sort); do
    rel="${h#"${INC}"/}"
    total=$((total + 1))

    for pass in once twice; do
        [ "${pass}" = "twice" ] && [ "${SINGLE}" = "1" ] && continue

        if [ "${pass}" = "once" ]; then
            printf '#include <%s>\n' "${rel}" > "${tmpdir}/tu.cpp"
        else
            printf '#include <%s>\n#include <%s>\n' "${rel}" "${rel}" > "${tmpdir}/tu.cpp"
        fi

        compile_tu "${tmpdir}/tu.cpp"
        rc=$?
        if [ "${rc}" -eq 0 ]; then
            ok=$((ok + 1))
            continue
        fi
        if [ "${rc}" -eq 2 ]; then
            skipped=$((skipped + 1))
            printf '  %-52s [%s] missing dependency: %s\n' "${rel}" "${pass}" \
                "$(grep -m1 -oE 'fatal error: [^:]+: No such file or directory' "${tmpdir}/err" \
                    | sed -E 's/^fatal error: //; s/: No such file or directory$//')" >> "${tmpdir}/skipped"
            continue
        fi

        broken=$((broken + 1))
        if [ "${pass}" = "twice" ]; then
            printf '  %s  [%s: no include guard?]\n' "${rel}" "${pass}" >> "${tmpdir}/broken"
        else
            printf '  %s  [%s]\n' "${rel}" "${pass}" >> "${tmpdir}/broken"
        fi
        sed 's/^/      /' "${tmpdir}/err" | grep -E 'error:|fatal error:' | head -3 >> "${tmpdir}/broken"
    done
done

passes=2
[ "${SINGLE}" = "1" ] && passes=1
echo "Installed headers compiled: ${total} headers x ${passes} pass(es) = $((total * passes)) checks"
echo "  ${ok} ok, ${skipped} skipped, ${broken} broken"
echo "  (scope: ${INC}; skipped means a third-party dependency is absent from this"
echo "   configuration, so that header was not tested)"

if [ "${skipped}" -gt 0 ]; then
    echo
    echo "Not tested (dependency not installed):"
    cat "${tmpdir}/skipped"
fi

if [ "${broken}" -gt 0 ]; then
    echo
    echo "BROKEN — installed headers a user cannot rely on:"
    cat "${tmpdir}/broken"
    echo
    echo "A [twice] failure means the header has no include guard: it compiles for the one"
    echo "translation unit that happens to include it once and breaks for the next one."
    echo "See cleanup/plan/controller-extraction.md (X2, X2b) and cleanup/findings/build.md N1."
    exit 1
fi

# A check that tested nothing reports green, which is the failure mode this script exists
# to catch. If everything was skipped the run proves nothing.
if [ "${ok}" -eq 0 ]; then
    echo "::error::no installed header could be compiled - nothing was checked" >&2
    exit 2
fi

exit 0
