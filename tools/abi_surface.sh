#!/usr/bin/env bash
#
# Diff libtoffy's exported ABI surface against the recorded baseline.
#
# Why this exists, and why now. SOVERSION is the major alone
# (`set_property(TARGET toffy PROPERTY SOVERSION ${PROJECT_VERSION_MAJOR})`), so
# `libtoffy.so.2` is accepted by every consumer built against any 2.x. While the
# SONAME carried the full version the loader refused a library it had not been
# linked against — a loud failure at exec, and the reason the rule was "one clean
# rebuild of the whole chain, never a partial field deployment". That net is gone
# on purpose, so that an ABI-compatible patch can be dropped into a cabinet
# without relinking the world; what enforces the rule now is this check plus the
# release decision. The rule: **a change to a return type, to a vtable, or to the
# layout of a public type is a new major.**
#
# What this catches: a removed exported symbol, or a changed parameter list /
# cv-qualifier — anything that moves the mangled name. That is the class of
# `v1.10.0`, which deleted `toffy::Event` and the virtual `Filter::processEvent()`
# (one vtable slot shorter) while the SONAME still advertised 1.7.1; see
# cleanup/dod/stage-gates.md §2.4. Run against a library that is not the one this
# baseline was blessed from, it fails — including a BTA-off build of toffy, which
# is a smaller library and should not be mistaken for a drop-in.
#
# What this cannot catch, and no symbol diff can: a changed **return type** (the
# Itanium ABI does not mangle it — `createFilter` returning `FilterPtr` by value
# kept a byte-identical name and shifted every argument register by one), a
# vtable's *contents*, or the layout of a struct. `tools/api_change_report.sh` is
# the check for those: it fails on any changed installed header and forces the
# version decision out loud.
#
# Usage:
#   tools/abi_surface.sh --library <path/to/libtoffy.so[.X.Y.Z]> [options]
#   tools/abi_surface.sh --build-dir <cmake build dir>        [options]
#
# Options:
#   --baseline <file>   recorded surface (default: ci/abi-surface.txt)
#   --bless             write the current surface to the baseline instead of
#                       comparing — the step that re-opens the gate, so do it in
#                       a commit that says which major it is for
#   --quiet             only the verdict
#
# Exit status:
#   0  surface matches the baseline (additions are reported and allowed)
#   1  removals or signature changes — an ABI break: ship it as a new major
#   2  the check did not run: no library, no baseline, or an empty surface.
#      An empty surface is never a pass; that is the P0-6 lesson, where a
#      link-surface check measured the wrong artifact, found zero symbols, and
#      reported green for as long as it existed.
set -uo pipefail

LIB=""
BUILD_DIR=""
BASELINE="ci/abi-surface.txt"
BLESS=0
QUIET=0

while [ $# -gt 0 ]; do
    case "$1" in
        --library)  LIB="$2"; shift 2 ;;
        --build-dir) BUILD_DIR="$2"; shift 2 ;;
        --baseline) BASELINE="$2"; shift 2 ;;
        --bless)    BLESS=1; shift ;;
        --quiet)    QUIET=1; shift ;;
        -h|--help)  sed -n '2,50p' "$0"; exit 0 ;;
        *) echo "abi_surface: unknown argument '$1'" >&2; exit 2 ;;
    esac
done

note() { [ "$QUIET" -eq 1 ] || echo "$@"; }

# --------------------------------------------------------------- find the library --
if [ -z "$LIB" ] && [ -n "$BUILD_DIR" ]; then
    # The real file, not the `libtoffy.so` development symlink: nm on the symlink
    # follows to the same file, but picking the longest name is what makes the
    # report say which build this came from.
    LIB=$(find "$BUILD_DIR" -maxdepth 1 -name 'libtoffy.so.*' -type f -printf '%T@ %p\n' \
            2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)
fi
if [ -z "$LIB" ] || [ ! -f "$LIB" ]; then
    echo "abi_surface: no libtoffy found (use --library <file> or --build-dir <dir>)" >&2
    exit 2
fi

if ! command -v nm >/dev/null 2>&1; then
    echo "abi_surface: nm not on PATH, cannot read the symbol table" >&2
    exit 2
fi

# ------------------------------------------------------------------- the surface --
# Strong defined symbols in toffy's own namespace, mangled.
#
#   * mangled, not demangled: the mangled name *is* the ABI contract, and it is
#     unambiguous where a demangled one is not. Failures are reported demangled.
#   * strong only (T text, D/B initialised and uninitialised data). Weak symbols
#     come and go with the inliner and the template instantiator — diffing them
#     buries the real change in noise, which is what cleanup/dod/stage-gates.md
#     §2.4 warns about. Vtable and typeinfo objects are weak and therefore out;
#     the method whose deletion changes the vtable is strong, and that is the
#     symbol the deletion removes.
#   * `5toffy` is the mangled length-prefix form of the namespace, so it matches
#     `toffy::…` and nothing else that can appear in this library.
surface_of() {
    nm -D --defined-only "$1" 2>/dev/null \
        | awk '$2 ~ /^[TDB]$/ && /5toffy/ {print $3}' \
        | sort -u
}

CURRENT=$(surface_of "$LIB")
N=$(printf '%s\n' "$CURRENT" | grep -c . || true)

if [ "$N" -eq 0 ]; then
    echo "abi_surface: 0 toffy symbols in '$LIB' — the check did not run." >&2
    echo "  Either this is not libtoffy, or it was built with the symbols stripped." >&2
    echo "  A surface of zero is never a pass." >&2
    exit 2
fi

if [ "$BLESS" -eq 1 ]; then
    mkdir -p "$(dirname "$BASELINE")"
    {
        echo "# Recorded ABI surface of libtoffy — the baseline tools/abi_surface.sh diffs against."
        echo "#"
        echo "# One mangled, strong (T/D/B) toffy:: symbol per line, as exported by the shared"
        echo "# library. DO NOT HAND-EDIT: regenerate with"
        echo "#   tools/abi_surface.sh --build-dir <build> --baseline ci/abi-surface.txt --bless"
        echo "# and read the diff before committing it."
        echo "#"
        echo "# Removals and signature changes here are ABI breaks. Since SOVERSION is the major"
        echo "# alone, an ABI break shipped under an unchanged major loads into every consumer"
        echo "# built against that major and misbehaves there — the symbol diff is the only thing"
        echo "# that says so before the field does. Re-blessing IS the version decision: do it in"
        echo "# a commit that names the new major, or do not do it."
        echo "# Return-type, vtable-layout and struct-layout changes are NOT in this file and"
        echo "# cannot be; tools/api_change_report.sh covers those."
        echo "# Blessed from: $(basename "$LIB") ($(git -C "$(dirname "$0")/.." rev-parse --short HEAD 2>/dev/null || echo 'not a git checkout'))"
        printf '%s\n' "$CURRENT"
    } > "$BASELINE"
    echo "abi_surface: blessed $N symbols into $BASELINE"
    exit 0
fi

if [ ! -f "$BASELINE" ]; then
    echo "abi_surface: no baseline at '$BASELINE' — cannot compare." >&2
    echo "  Run with --bless to create it (and treat that as the ABI decision it is)." >&2
    exit 2
fi

BASE=$(grep -vE '^[[:space:]]*(#|$)' "$BASELINE" | sort -u)
NBASE=$(printf '%s\n' "$BASE" | grep -c . || true)

REMOVED=$(comm -23 <(printf '%s\n' "$BASE") <(printf '%s\n' "$CURRENT"))
ADDED=$(comm -13 <(printf '%s\n' "$BASE") <(printf '%s\n' "$CURRENT"))
NREM=$(printf '%s\n' "$REMOVED" | grep -c . || true)
NADD=$(printf '%s\n' "$ADDED" | grep -c . || true)

note "abi_surface: $LIB — $N symbols against a baseline of $NBASE"

if [ "$NADD" -gt 0 ]; then
    # Additions are allowed: a release that adds API is not an ABI break. They are
    # reported so a blessed baseline grows knowingly rather than silently — the
    # same rule the consumer link-surface gate runs on.
    note "  $NADD symbol(s) added since the baseline (allowed):"
    printf '%s\n' "$ADDED" | grep . | head -20 | c++filt | sed 's/^/    /'
    [ "$NADD" -gt 20 ] && note "    … and $((NADD - 20)) more"
fi

if [ "$NREM" -gt 0 ]; then
    echo "  $NREM symbol(s) REMOVED or CHANGED since the baseline — this is an ABI break:"
    printf '%s\n' "$REMOVED" | grep . | c++filt | sed 's/^/    /'
    echo
    echo "  SOVERSION is the major alone, so a consumer linked against this major"
    echo "  will LOAD a library with these symbols missing and fail somewhere far"
    echo "  from here. Either restore them, or ship this as a new major:"
    echo "    - cut the release with the next major version (the SONAME moves with it),"
    echo "    - re-bless this baseline in the same commit, so the next diff is clean."
    echo "  A parameter or qualifier change shows up here as one removal plus one"
    echo "  addition; read them together."
    exit 1
fi

note "  ABI surface matches the baseline."
exit 0
