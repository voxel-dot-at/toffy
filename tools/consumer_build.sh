#!/usr/bin/env bash
#
# consumer_build.sh — build a consumer repository against an installed toffy,
# and decide whether the result is the result we said to expect.
#
#   tools/consumer_build.sh \
#     --toffy-prefix /path/to/toffy-install \
#     --repo https://github.com/voxel-dot-at/toffy-oatpp.git \
#     --ref  main \
#     # HTTPS, not SSH: the CI runner clones anonymously and has no key. This
#     # example used the SSH URL and the workflow copied it, which is how both
#     # cells died with rc 2 ("could not clone") on 2026-10-30 — the third bug in
#     # that job. --repo is passed straight to git, so it must be a URL the
#     # machine running this can actually read.
#     --build-type Release \
#     [--work /tmp/consumer] [--allowlist ci/consumer-expected-failures.txt] \
#     [--link-surface ci/consumer-link-surface.txt] [--name toffy-oatpp] \
#     [--link-target <glob>] ...   # artifact(s) the link surface is read from.
#       Default 'lib*.so*', which is right for a library and wrong for an
#       application: becom-knapp-wurthbox builds `boxes` and
#       `compute_single_box` and no toffy-facing shared library at all, so the
#       default finds only libboxes-lic.so — 0 toffy symbols, a vacuous PASS.
#       Name the executables for an app: --link-target boxes --link-target
#       compute_single_box.
#
# Why this exists (P0-2). Every error recorded in the consumer upgrade notes was
# introduced by a toffy commit and sat undetected because no job anywhere
# configures a consumer: toffy's own CI compiles toffy, and a toffy that compiles
# against its own headers proves nothing about a library that includes them from
# an install prefix. This script is the missing edge. It is a script rather than
# YAML so that it can be run — and was — on a developer machine before it was
# ever trusted to gate a push.
#
# The judgement it makes is the whole point, and it is not "did the build pass":
#
#   * a build that fails with an error NOT on the allowlist is new breakage, and
#     this is the only automated thing standing between that commit and a
#     consumer build three repositories away;
#   * a build that PASSES while its allowlist entries are still in the file is
#     also an error — the list is the progress metric, and a list nobody deletes
#     is a list that stops meaning anything (that is how "expected failures"
#     jobs die: not disabled, just quietly stale);
#   * a build that passes but links against toffy symbols it did not before has
#     started depending on something new, which no compiler warning and no green
#     test suite will report. The link surface is diffed for *additions* only:
#     dropping coupling is not a regression.
#
# Failure modes are deliberately loud. Anything the script cannot do — no cmake,
# clone refused, consumer produced no shared library to inspect — exits 2
# ("could not run"), never 0: a gate that fails open is worse than no gate,
# because it is cited as evidence.
#
# Exit status:
#   0  result matches the recorded expectation
#   1  new breakage, stale allowlist, or new link coupling
#   2  usage or environment error (the check did not run)

set -uo pipefail

# ---------------------------------------------------------------- invocation --

usage() {
  sed -n '3,36p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
}

TOFFY_PREFIX="" CONSUMER_REPO="" CONSUMER_REF="" BUILD_TYPE="Release"
WORK="" ALLOWLIST="" LINKSURF="" NAME="" KEEP=0
LINK_TARGETS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --toffy-prefix) TOFFY_PREFIX="${2:-}"; shift 2 ;;
    --repo)         CONSUMER_REPO="${2:-}"; shift 2 ;;
    --ref)          CONSUMER_REF="${2:-}"; shift 2 ;;
    --build-type)   BUILD_TYPE="${2:-}"; shift 2 ;;
    --work)         WORK="${2:-}"; shift 2 ;;
    --allowlist)    ALLOWLIST="${2:-}"; shift 2 ;;
    --link-surface) LINKSURF="${2:-}"; shift 2 ;;
    --name)         NAME="${2:-}"; shift 2 ;;
    --link-target)  LINK_TARGETS+=("${2:-}"); shift 2 ;;
    --keep)         KEEP=1; shift ;;
    -h|--help)      usage ;;
    *) echo "consumer_build.sh: unknown argument '$1'" >&2; usage ;;
  esac
done

[ -n "$TOFFY_PREFIX" ]   || { echo "consumer_build.sh: --toffy-prefix is required" >&2; exit 2; }
[ -n "$CONSUMER_REPO" ]  || { echo "consumer_build.sh: --repo is required" >&2; exit 2; }
[ -n "$CONSUMER_REF" ]   || { echo "consumer_build.sh: --ref is required" >&2; exit 2; }
case "$BUILD_TYPE" in Release|Debug) ;; *)
  echo "consumer_build.sh: --build-type must be Release or Debug (got '$BUILD_TYPE')" >&2
  exit 2 ;;
esac

# Both build types are required by the gate, and not for completeness theatre:
# the consumers add -Wall -ggdb -O0 unconditionally and -O2 again in Release, so
# the Debug build is the one where the compiler still has something to say.
command -v cmake >/dev/null || { echo "consumer_build.sh: cmake not found" >&2; exit 2; }
command -v git   >/dev/null || { echo "consumer_build.sh: git not found" >&2; exit 2; }
[ -d "$TOFFY_PREFIX" ] || {
  echo "consumer_build.sh: toffy prefix '$TOFFY_PREFIX' does not exist" >&2; exit 2; }
[ -n "$(ls -d "$TOFFY_PREFIX"/lib/cmake/toffy 2>/dev/null)" ] || {
  echo "consumer_build.sh: '$TOFFY_PREFIX' has no lib/cmake/toffy — that is not an" >&2
  echo "  installed toffy. Install one first: cmake --install build --prefix <prefix>." >&2
  exit 2; }

# Default: the consumer's own shared library. See the --link-target note at the
# top — an application has to say what to look at instead.
[ ${#LINK_TARGETS[@]} -gt 0 ] || LINK_TARGETS=('lib*.so*')

NAME="${NAME:-$(basename "$CONSUMER_REPO" .git)}"
LABEL="${NAME}@${CONSUMER_REF}"
WORK="${WORK:-$(mktemp -d "${TMPDIR:-/tmp}/consumer-build-XXXXXX")}"
SRC="$WORK/src"
BUILD="$WORK/build-$BUILD_TYPE"
CFG_LOG="$WORK/configure-$BUILD_TYPE.log"
BUILD_LOG="$WORK/compile-$BUILD_TYPE.log"

mkdir -p "$WORK" || { echo "consumer_build.sh: cannot create '$WORK'" >&2; exit 2; }

# ------------------------------------------------------------------ summary ---
# Written as we go so that an early exit still leaves a report behind. On GitHub
# it lands in the job summary; locally it goes to stdout, which is the same
# audience.
SUMMARY=""
note() {
  SUMMARY="${SUMMARY}${1}
"
  echo "${1}"
}
flush_summary() {
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    {
      echo "### Consumer build: \`${LABEL}\` (${BUILD_TYPE})"
      echo
      printf '%s' "$SUMMARY"
    } >> "$GITHUB_STEP_SUMMARY"
  fi
}

# -------------------------------------------------------------------- clone ---

TOFFY_SO=$(ls "$TOFFY_PREFIX"/lib/ 2>/dev/null | grep -m1 -oE 'libtoffy\.so\..*' || true)
if [ -z "$TOFFY_SO" ]; then
  note "- toffy: \`$TOFFY_PREFIX\` — **no libtoffy.so.* in its lib/**: the prefix has the"
  note "  CMake package but not the library. That is a half-installed prefix; the consumer"
  note "  may still configure and then fail to link, which is not what this job tests."
else
  note "- toffy: \`$TOFFY_PREFIX\` (\`$TOFFY_SO\`)"
fi
note "- consumer: \`$CONSUMER_REPO\` @ \`$CONSUMER_REF\`"

rm -rf "$SRC"
if ! git clone --quiet --no-checkout "$CONSUMER_REPO" "$SRC" 2>"$WORK/clone.log"; then
  note "- **could not clone** \`$CONSUMER_REPO\` — see the log below"
  sed 's/^/    /' "$WORK/clone.log"
  flush_summary
  exit 2
fi
# Fetch the ref we were asked for. A branch that has been deleted since the last
# green run must fail here, loudly, and not silently build main instead.
#
# No `--depth 1`, and the previous version of this line had one. The clone above
# is a full clone, so the depth bought nothing; what it did was move the shallow
# boundary in front of the consumer's own tags, so `git describe --tags` inside
# the consumer's CMake found nothing and the build produced
# `libtoffy_oatpp.so.-128-NOTFOUND.-128-NOTFOUND.-128-NOTFOUND` while the cell
# reported PASS. Measured: after the clone, `git describe --tags` says
# `v0.1.0-3-g6f44c36`; after the depth-1 fetch it says "No tags can describe".
# This is the bug consumer-build.yml documents at length for toffy's own
# checkout — it was only ever fixed on the toffy side of the job, and the
# consumer side of the same job kept it.
if ! git -C "$SRC" fetch --quiet origin "$CONSUMER_REF" 2>>"$WORK/clone.log"; then
  note "- **ref \`$CONSUMER_REF\` does not exist on the consumer's origin** — this is not"
  note "  a toffy breakage, it is the gate being pointed at nothing. Fix the ref."
  sed 's/^/    /' "$WORK/clone.log"
  flush_summary
  exit 2
fi
git -C "$SRC" checkout --quiet FETCH_HEAD || { echo "checkout failed" >&2; exit 2; }
CONSUMER_SHA="$(git -C "$SRC" rev-parse --short FETCH_HEAD)"
note "- consumer commit: \`$CONSUMER_SHA\`"
# Reported, not gated: a version that did not parse is the consumer's business
# (or this script's, which is why the depth-1 fetch above is gone). It is said
# out loud because the alternative is a green cell whose artifact is named
# `.so.-128-NOTFOUND…` — and the upgrade notes' one-line sanity check for a bad
# install is a grep for exactly that string. The link surface is unaffected;
# what is not being tested is the consumer's version machinery, and a real
# install of an unversioned tree is refused by find_package at any version.
CONSUMER_DESCRIBE="$(git -C "$SRC" describe --tags 2>/dev/null || true)"
if [ -z "$CONSUMER_DESCRIBE" ]; then
  note "- **no tag describes \`$CONSUMER_SHA\`** — the consumer's \`git_describe\` will "
  note "  produce a NOTFOUND version, so its library name, SONAME and package config "
  note "  version are all junk. The link surface below is still a valid measurement; "
  note "  the version gate is not being exercised. Either the repository has no tags, "
  note "  or this checkout does not have them."
else
  note "- consumer version: \`$CONSUMER_DESCRIBE\`"
fi

# ------------------------------------------------------------- configure+build --

CFG_RC=0
cmake -S "$SRC" -B "$BUILD" \
      -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DCMAKE_PREFIX_PATH="$TOFFY_PREFIX" >"$CFG_LOG" 2>&1 || CFG_RC=$?
# -k: collect every error rather than the first one. The allowlist is only
# meaningful if the list of errors it is compared against is the whole list.
BUILD_RC=0
[ "$CFG_RC" -eq 0 ] && {
  cmake --build "$BUILD" -j "$(nproc)" -- -k >"$BUILD_LOG" 2>&1 || BUILD_RC=$?
}

STATUS="ok"
[ "$CFG_RC" -ne 0 ]   && STATUS="configure-failed"
[ "$CFG_RC" -eq 0 ] && [ "$BUILD_RC" -ne 0 ] && STATUS="build-failed"

# ------------------------------------------------------------- allowlist ------
# Format of the allowlist file: one entry per line,
#
#     <ref-glob>|<substring of an error line>
#
# A substring rather than a regex: entries have to stay readable by the person
# deleting them, and every error signature so far is a fixed sentence. Blank
# lines and #-comments are ignored. Entries are matched against normalised error
# lines (see normalise below), so they must be written against that form too —
# which is exactly what the "unmatched" report prints, so a new entry can be
# pasted in from a failing run.

normalise() {
  # Strip the compiler's file:line:col prefix, collapse whitespace, and cut the
  # rest. Without stripping the prefix an entry would name a line number and go
  # stale the moment anything above it moved — the failure mode this whole
  # repository is about.
  sed -E \
    -e 's/^[[:space:]]*//' \
    -e 's/^([A-Za-z0-9_+./-]+):[0-9]+:[0-9]+: //' \
    -e 's/^([A-Za-z0-9_+./-]+):[0-9]+: //' \
    -e 's/[[:space:]]+/ /g' \
    -e 's/ ?\.\.\.$//' \
  | cut -c1-200 \
  | sort -u
}

error_lines() {
  # Three sources, deduplicated: the same line can be caught by more than one
  # pattern, and the report reads one entry per line.
  {
    # CMake's configure errors and the compiler's errors.
    cat "$CFG_LOG" "$BUILD_LOG" 2>/dev/null \
      | grep -E '(^|[[:space:]])error:'
    # find_package version-gate failures are not "error:" lines in the log body;
    # CMake writes "Could not find a configuration file ...". They are the single
    # most common consumer breakage in this upgrade, so they count.
    cat "$CFG_LOG" 2>/dev/null \
      | grep -E 'Could not find a configuration file for package'
    # Linker diagnostics, which the `error:` grep above does not see — and this
    # gap hid a real breakage. A failed link ends with
    #
    #     collect2: error: ld returned 1 exit status
    #
    # which *does* match `error:`, and was the only line that did: what is actually
    # wrong is on the lines above it, `undefined reference to …` / `cannot find -l…`.
    # Run 41354081913 reported that one line for toffy-oatpp in both configurations
    # — "1 error NOT on the allowlist: collect2: error: ld returned 1 exit status" —
    # and no reader could tell a missing symbol from a missing library from an OOM
    # kill. The symbol name is the entire message here, so it is collected.
    cat "$BUILD_LOG" 2>/dev/null \
      | grep -E "undefined reference to|undefined symbol:|cannot find -l|cannot find library|DSO missing from command line|error: ld returned"
  } | normalise
}

MATCHED="" UNMATCHED=""
if [ -n "$ALLOWLIST" ] && [ -f "$ALLOWLIST" ]; then
  ENTRIES=$(grep -vE '^[[:space:]]*(#|$)' "$ALLOWLIST" | grep -F '|' || true)
else
  ENTRIES=""
fi

ERRORS=$(error_lines)

if [ "$STATUS" != "ok" ]; then
  if [ -z "$(printf '%s' "$ERRORS" | tr -d '[:space:]')" ]; then
    # Failed, but nothing we recognised as an error line. Never treat this as
    # "no new errors": that is precisely how a gate fails open.
    note "- **failed (\`$STATUS\`) with no recognised error line** — cannot classify,"
    note "  treating as new breakage. Tail of the log:"
    tail -25 "$CFG_LOG" "$BUILD_LOG" 2>/dev/null | sed 's/^/    /'
    flush_summary
    exit 1
  fi
  while IFS= read -r err; do
    [ -n "$err" ] || continue
    hit=""
    while IFS='|' read -r refglob pat; do
      [ -n "$pat" ] || continue
      case "$LABEL" in $refglob) hit="$pat" ;; esac
    done <<< "$ENTRIES"
    if [ -n "$hit" ]; then MATCHED="${MATCHED}${err}
"; else UNMATCHED="${UNMATCHED}${err}
"; fi
  done <<< "$ERRORS"
fi

# ------------------------------------------------------------- link surface ---
# nm -D --undefined-only on the built consumer, filtered to toffy's own symbols.
# Additions are new coupling; removals are not reported as a problem.
LINK_NOTE=""
if [ "$STATUS" = "ok" ]; then
  # Artifacts to read. Every --link-target is searched and every match is
  # inspected: a consumer that ships two executables binds to toffy twice, and
  # reading only the first would report a surface that is a subset of the truth.
  ARTIFACTS=""
  for pat in "${LINK_TARGETS[@]}"; do
    hit=$(find "$BUILD" -maxdepth 3 \( -type f -o -type l \) -name "$pat" 2>/dev/null \
            | grep -v '\.so\.[0-9]*\.[0-9]*\.[0-9]*$' | sort)
    [ -n "$hit" ] || hit=$(find "$BUILD" \( -type f -o -type l \) -name "$pat" 2>/dev/null | sort)
    if [ -z "$hit" ]; then
      note "- link surface: no artifact matching \`$pat\` under \`$BUILD\` — the surface"
      note "  could not be read, so this run does not prove what the gate claims."
      flush_summary
      exit 2
    fi
    ARTIFACTS="${ARTIFACTS}${hit}
"
  done
  ARTIFACTS=$(printf '%s\n' "$ARTIFACTS" | grep . | sort -u)
  ARTIFACT_NAMES=$(printf '%s\n' "$ARTIFACTS" | while read -r f; do basename "$f"; done \
                    | sort -u | paste -sd, -)

  # link_normalise: demangled names carry the standard library's own ABI tags
  # (libstdc++ spells std::string as std::__cxx11::string). Those are not
  # coupling and not toffy's, and leaving them in means a compiler upgrade
  # reports all 16 symbols as new — which is how a check like this gets muted.
  link_normalise() { sed -e 's/std::__cxx11::/std::/g'; }
  CURRENT=$(while read -r f; do nm -D --undefined-only "$f" 2>/dev/null; done <<< "$ARTIFACTS" \
              | awk '$1=="U"{print $2}' | c++filt 2>/dev/null | grep -E 'toffy::' \
              | link_normalise | sort -u)
  CURRENT_COUNT=$(printf '%s\n' "$CURRENT" | grep -c . || true)
  note "- link surface: \`${ARTIFACT_NAMES}\` — **${CURRENT_COUNT}** toffy symbols"
  # A surface of zero is not a result, it is the absence of one. This is the
  # shape the gate took on 2026-11-01 for boxes: the default target matched
  # libboxes-lic.so — the licence checker, which has no business calling toffy —
  # and the cell reported PASS with 0 symbols measured, against a baseline that
  # had no `boxes` lines either, so the diff was empty in both directions.
  # Zero means "looked in the wrong place", and rc 2 is what this script says
  # about a check that did not run: a gate that passes vacuously is worse than
  # no gate, because it gets cited as evidence.
  if [ "$CURRENT_COUNT" -eq 0 ]; then
    note "- **the artifacts inspected reference no toffy symbol at all.** Either this"
    note "  consumer does not link toffy — in which case it is not a consumer cell — or"
    note "  \`--link-target\` names the wrong artifact. For an application, name its"
    note "  executables (\`--link-target boxes --link-target compute_single_box\`)."
    flush_summary
    exit 2
  fi
  if [ -n "$LINKSURF" ] && [ -f "$LINKSURF" ]; then
    BASELINE=$(grep -E "^${NAME}([[:space:]]|@)" "$LINKSURF" 2>/dev/null \
                 | sed -E "s/^${NAME}([[:space:]]|@[^[:space:]]*)?[[:space:]]+//" \
                 | link_normalise | sort -u)
    NEW=$(comm -13 <(printf '%s\n' "$BASELINE") <(printf '%s\n' "$CURRENT") | grep . || true)
    GONE=$(comm -23 <(printf '%s\n' "$BASELINE") <(printf '%s\n' "$CURRENT") | grep . || true)
    if [ -n "$NEW" ]; then
      LINK_NOTE="new"
      note "- **new toffy coupling** — these symbols are used but were not in"
      note "  \`$(basename "$LINKSURF")\`:"
      printf '%s\n' "$NEW" | sed 's/^/      - `/' | sed 's/$/`/'
    else
      note "- link surface matches the recorded baseline"
    fi
    if [ -n "$GONE" ]; then
      note "- coupling *removed* (fine, but update the baseline when convenient):"
      printf '%s\n' "$GONE" | sed 's/^/      - `/' | sed 's/$/`/'
    fi
  else
    note "- no baseline file, so the surface is reported but not gated"
  fi
fi

# ------------------------------------------------------------------ verdict ---

RC=0
case "$STATUS" in
  ok)
    # Which entries in the allowlist apply to this consumer@ref? Bash's `case`
    # globbing is the matcher used in the failure path above, so it is the
    # matcher here too — one rule, one implementation.
    APPLICABLE=""
    while IFS='|' read -r refglob pat; do
      [ -n "$pat" ] || continue
      case "$LABEL" in $refglob) APPLICABLE="${APPLICABLE}${pat}
" ;; esac
    done <<< "$ENTRIES"
    N_APPLICABLE=$(printf '%s\n' "$APPLICABLE" | grep -c . || true)
    if [ "$N_APPLICABLE" -gt 0 ]; then
      if [ "$N_APPLICABLE" -eq 1 ]; then PLURAL=y; else PLURAL=ies; fi
      note "- **the build is GREEN, and the allowlist still claims this consumer is"
      note "  broken ($N_APPLICABLE entr$PLURAL).**"
      note "  Delete them: \`$ALLOWLIST\`. A stale expected-failure list is how this job"
      note "  stops meaning anything — the list is the progress metric, and the metric"
      note "  only shrinks if somebody notices it has."
      RC=1
    elif [ -n "$LINK_NOTE" ]; then
      note "- verdict: **FAIL** — new coupling (see above)"
      RC=1
    else
      note "- verdict: **PASS** — builds, links, and no new coupling against this toffy"
    fi
    ;;
  *)
    N_UNMATCHED=$(printf '%s\n' "$UNMATCHED" | grep -c . || true)
    N_MATCHED=$(printf '%s\n' "$MATCHED" | grep -c . || true)
    if [ "$N_UNMATCHED" -gt 0 ]; then
      note "- **$N_UNMATCHED error(s) NOT on the allowlist** — this is the case the job"
      note "  exists for. A toffy commit just broke \`$LABEL\`:"
      printf '%s\n' "$UNMATCHED" | grep . | sed 's/^/      - `/' | sed 's/$/`/'
      note ""
      note "  If this is a real, accepted change on the consumer side, add those lines to"
      note "  \`$(basename "${ALLOWLIST:-ci/consumer-expected-failures.txt}")\` — and say in the"
      note "  commit which toffy change made them necessary, because that is the sentence"
      note "  the next person reading this job will need."
      RC=1
    else
      note "- **KNOWN BROKEN, as recorded**: $N_MATCHED error(s), all on the allowlist,"
      note "  nothing new. This is not a pass; it is the failure we already knew about."
      note ""
      note "  Entries matched:"
      printf '%s\n' "$MATCHED" | grep . | sed 's/^/      - `/' | sed 's/$/`/'
    fi
    ;;
esac

flush_summary
if [ "$KEEP" -eq 0 ]; then
  rm -rf "$WORK"
else
  note "- work tree kept at \`$WORK\`"
fi
exit "$RC"
