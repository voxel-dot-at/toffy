# Definition of Done

Completion criteria for the `modules/core` cleanup. Companion to
[`CLEANUP_PLAN.md`](CLEANUP_PLAN.md) (items), [`CLEANUP_SEQUENCE.md`](CLEANUP_SEQUENCE.md)
(PR order) and [`PROJECT_INFO.md`](PROJECT_INFO.md) (evidence).

Three levels: **per-PR** (every PR), **stage** (extra gates for particular kinds of PR), and
**program** (when the cleanup as a whole is finished).

A PR is not done because the code compiles. It is done when the thing it claims to fix
cannot silently come back.

---

## 1. Per-PR — applies to every PR in the programme

1. **Builds in all four dependency configurations**, not just the developer's. This gate is
   now **green in all four cells**, which it was not when this document was written:

   | `PCL_FOUND` | `HAS_BTA` | status | how to reproduce |
   |---|---|---|---|
   | on | on | ✅ builds, 8/8 ctest | default (needs the bta SDK) |
   | on | off | ✅ builds, 8/8 ctest | `-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON` |
   | off | on | ✅ builds, 8/8 ctest | `-DWITHOUT_PCL=ON` |
   | off | off | ✅ builds, 8/8 ctest | both flags |

   The original note blamed only the unguarded PCL typedefs in `frame.hpp`. That was the
   first blocker of several, and the interesting part is that the typedefs had never been
   *seen* to fail: two CMake `if( ${VAR} )` clauses expanded to nothing when the dependency
   was absent, so configuration aborted before reaching the compiler. Fixing the header
   exposed the CMake bugs, which exposed an empty `add_library(toffy_3d OBJECT "")`, which
   exposed `exportcloud.hpp` holding a `pcl::PCDWriter` by value and an unnecessary
   `<pcl/io/pcd_io.h>` in `exportcsv.hpp`. Separately, `add_subdirectory(bta)` was
   unconditional, so every BTA-off configuration failed to link with ~162 undefined `BTA*`
   references.

   A PCL-less build is not optional to check: it is the configuration most downstream
   packagers will hit first. Note that CI can only ever cover the **BTA-off** axis (the SDK
   is proprietary), so the two BTA-on cells still require a machine that has it.

2. **CI is green, and CI actually builds and tests.** This gate now exists:
   `.github/workflows/ci.yml` configures, compiles and runs `ctest` on every push and PR,
   as a PCL-on/PCL-off matrix plus a separate ASan/UBSan/LSan job. The original wording —
   that `.github/` held only Codacy, Flawfinder and Dependabot and nothing compiled
   anything — no longer applies.

   Two limits to keep in mind when citing a green run:
   - Runners cannot have the proprietary Becom bta SDK, so CI only ever covers the
     **BTA-off** axis. A BTA-on build still has to be checked on a machine that has the SDK.
   - CI does **not** build the Doxygen docs, so broken `\ref`s and `\todo` drift are not
     caught by anything.

3. **`ctest` passes**, and any new test fails on the parent commit for the right reason.
   For a fix PR, paste the pre-fix failure into the PR description. A test that has never
   been observed to fail is not a regression fence.

4. **No new compiler warnings** under the existing `-Wall -Wextra`
   (`CMakeLists.txt:207`) in any file the PR touches. Do not enable `-Werror` globally — the
   pre-existing count would fail the build — but do not add to it either.

5. **One numbered plan item per commit, one PR-stage per group of commits.** The plan's
   ground rule: never mix a public rename with a behaviour change in one commit.

6. **Docs updated in the same PR** as the behaviour or API change. If a PR changes what a
   documented function returns, the doc comment changes in that PR — not later. Several
   findings exist precisely because the docs and the code disagreed (`findPos`, `remove`).

7. **No debug output added to library code.** `modules/` must not write to `std::cout`;
   use `BOOST_LOG_TRIVIAL`.

8. **Revertable in isolation.** If reverting the PR also requires reverting another, say so
   in the PR body and reconsider the split.

---

## 2. Stage gates — extra criteria by kind of PR

Only the gates matching the PR kind apply.

### 2.1 Correctness PRs (PRs 3–6, the `P0` items)

- The regression test is committed **before** the fix commit, and the PR body shows it
  failing on the parent commit.
- PR 3 (`_pipe` mutation: `remove()`, `findPos()`, `stop()`) additionally passes an
  **ASan/UBSan** build. That path was live undefined behaviour, so a green normal build
  proved very little. This is now automated: the `sanitizers` job in
  `.github/workflows/ci.yml` runs the whole suite under ASan + UBSan + LeakSanitizer with
  `detect_leaks=1` on every push, so it no longer has to be run and pasted by hand.
- `stop()` is asserted to actually stop: a test drives a bank, calls `stop()`, and asserts
  every child reports `filterIdle`. **Done** — `FilterBankStop.StopStopsChildren`.
- The other PR-3 fixes are pinned too: `FilterBankRemove.ByIndexRemovesTheElementAtThatIndex`,
  `FilterBankRemove.MissingNameLeavesBankIntact`, `FilterBankFindPos.MissingNameYieldsNoValue`.
  Each was confirmed to fail on its parent commit before the fix landed.

### 2.2 Threading PR (PR 12)

- **TSan clean** on a run that exercises `ParallelFilter` lanes, and **ASan clean** on the
  same run. A green `ctest` is weak evidence for a data race; a green TSan run is not.
- The stop path is exercised repeatedly (≥100 iterations of start/stop) rather than once —
  the `keepRunning` race and the `join()`-on-non-joinable-thread bug are both timing-dependent.
- No `boost::thread` object is ever assigned while joinable.

### 2.3 Ownership PR (PR 10, the keystone)

- A full `toffyRunner` session under ASan/valgrind reports **zero leaks and zero
  double-frees**, including shutdown.
- `grep -rn "delete " modules/core/src` returns no `delete` of a `Filter`.
- The name-based teardown is gone: no `deleteFilter(name())` call remains in
  `clearBank()` or `~Controller`.
- `FilterPtr` is used consistently — the factory and `_pipe` no longer hold raw `Filter*`.

### 2.4 API-changing PRs (PRs 16, 17)

- Every renamed or removed public symbol has a deprecated alias, or an explicit note that
  this is a breaking release.
- `modules/filters`, `modules/bta` and `apps/` all still compile — an API change that only
  builds `modules/core` is not done.
- The Doxygen comment for each changed symbol is updated in the same PR (see 1.6).
- **A new version tag is pushed.** This is not ceremony: the library version *and*
  `SOVERSION` are derived from `git describe --tag` (`CMakeLists.txt:32`, `:372`). An
  API change with no new tag therefore ships under the previous `SOVERSION`, so the
  SONAME keeps advertising the old release while the exported mangled symbols have
  already changed — downstream binaries then fail at load time against a library that
  claims to be the version they linked.

  Two worked examples, both real:
  - **`v1.7.0`** — exists only to mark the `findPos()` / `remove()` signature change.
  - **`v1.10.0`** — marks the deletion of `toffy::Event` and the removal of the public
    virtual `Filter::processEvent()`, which shrinks `Filter`'s vtable by one slot. Before
    the tag, this tree built `libtoffy.so.1.7.1` with SONAME `libtoffy.so.1.7.1` while
    exporting **none** of the `Event`/`processEvent` symbols the real 1.7.1 exported — the
    hazard above, demonstrated with `readelf` rather than asserted. After the tag the SONAME
    is `libtoffy.so.1.10.0` and a stale binary refuses to load.

  Numbering note: `1.10.0` rather than the convention-implied `1.8.0`, so the number does
  not sort below `v1.9.0`, which exists on `origin/next`. That branch diverged from
  `b6d7665` and is not an ancestor of the cleanup line, so the two schemes still need
  reconciling when they meet.

  Also worth knowing before the next bump: `SOVERSION` is set to the **full** version, not
  the major alone, so every patch bump changes the SONAME and forces a downstream relink.
  That is why version decisions here cannot be routine.
- Run `tools/api_change_report.sh [BASE]` and put its output in the PR description. It
  exits 1 when any installed header under `*/include/` changed, 2 if the base ref is
  invalid (it fails closed on purpose), and 0 otherwise. The declaration listing is
  advisory — text diffing cannot be exact — but the exit code is the gate.

```sh
tools/api_change_report.sh "$(git describe --tags --abbrev=0)"   # exit 1 => tag required
```

### 2.5 Layering PR (PR 13)

- `grep -rn "include <toffy/viewers" modules/core` returns **nothing**: core must not include
  from the viewers module.
- `grep -rn "bta" modules/core/include/toffy` returns nothing after `btaFrame.hpp` moves.
- The link line for the core library no longer references the viewers library.

### 2.6 Formatting PR (PR 18)

- The diff is provably whitespace-only:
  `git diff --ignore-all-space --exit-code <base> HEAD` exits **0**.
- The commit SHA is added to `.git-blame-ignore-revs`.
- The CI format check is added **in this same PR**, so the tree cannot drift afterwards.

---

## 3. Programme — when the cleanup is finished

The cleanup is done when all of the following hold. The **now** column is the measured
baseline on the current tree, so each target is checkable rather than aspirational.

Re-measured on the current tree (`v1.10.0`, commit `8d4c306`; the whole matrix below was
re-run from scratch there and every counter reproduced exactly). Four criteria are now met;
the mechanical counters are unchanged because the work since P0 has been correctness, build
and CI rather than cleanup.

| # | Criterion | Now | Target | |
|---|---|---|---|---|
| 1 | All 12 `P0` correctness items closed, each with a regression test | **12/12** | 12/12 | ✅ |
| 2 | All 16 plan items closed, or explicitly rejected with a written rationale | **3/16** (`P1-1`, `P2-8`, `P2-14`; `P2-16` 4-of-5, `P2-3` core-only) | 16/16 | |
| 3 | CI configures, builds and runs `ctest` on every PR | matrix + sanitizers | required | ✅ |
| 4 | Builds in all four `PCL_FOUND`/`HAS_BTA` combinations | **4/4** | 4/4 | ✅ |
| 5 | Debug prints in library code (`modules/`) — **`std::cout` *and* bare `cout`** | **129** (29 `std::cout` + 100 bare `cout`); **0 in `modules/core`** | 0 | |
| 6 | `#warning` directives | **0** (was 1) | 0 | ✅ |
| 7 | `DLLExport` occurrences (vestigial macro) | **0** live (was 22) | 0 | ✅ |
| 8 | Headers defining `RAWFILE` | **1** (was 3) | 1 or 0 | ✅ |
| 9 | `#include <toffy/viewers/...>` from `modules/core` | 1 | 0 | |
| 10 | `interprocess` primitives in core | 2 | 0 | |
| 11 | Hard-coded type branches in `createFilter()` | 37 | 0 | |
| 12 | `@todo` markers in `modules/core` | 21 (was 23; `filterThread.hpp` still 10) | ≤ 5, and **0** in `filterThread.hpp` | |
| 13 | `delete` of a `Filter` outside its owner | present | 0 | |
| 14 | `#ifdef MSVC` branches that are neither compiled nor tested | 19 (5 `#if`, 12 `#ifdef`, 2 `#ifndef`) | 0 | |
| 15 | Compiler warnings in `modules/core` under `-Wall -Wextra` | **3** with PCL on, **2** with `-DWITHOUT_PCL=ON` — all one `-Woverloaded-virtual=` on the `filter()` const-overload trap | 0, with `-Werror` on that directory | |
| 16 | Docs describing behaviour that does not exist | ≥ 3 (the Doxygen `use.dox` still documents the removed `minimal_toffy` / web UI) | 0 | |

Criterion 15 is now counted rather than uncounted, and is worth reading carefully: the 3
warnings are a single root cause, the `filter()` const/non-const overload trap, which is
plan item `P2-12`. Fixing that item should take core to **zero** warnings, which is what
makes `-Werror` on `modules/core` achievable. That is the cheapest remaining win on this
table.

The count is **configuration-dependent**: one of the three overload sites sits behind a PCL
guard, so a `-DWITHOUT_PCL=ON` build reports 2, not 3. The 3 above is the default (PCL-on)
figure. Quote the configuration alongside the number, or a PCL-less CI job will look like it
fixed a warning that is still there.

Build-flag work landed since this table was first drawn (`P2-16`): the global
`add_definitions(-O2 -fPIC)` and the redundant `add_definitions(-Wall)` are gone, and so is
the dead `#if (BOOST_VERSION > 105500)` branch. Removing the global `-O2` also restored
Release's `-O3`, which that flag had been silently overriding — see `P2-16`. It changed no
warning counts (verified against a `HEAD` worktree: 3 before, 3 after with PCL on; 2 and 2
without).

### Verification block

Copy-pasteable; each command must return nothing (or the stated value) when the programme is
complete.

Comments in the tree now name the things that used to be there ("DLLExport removed…"), so
every check below strips comment lines first. Without that, items 6 and 7 report the cleanup
notes rather than the code — they currently return 1 and 8 hits respectively, all comments.

`NC='grep -vE ":\s*(//|\*|/\*)"'` — set this up once:

```sh
NC() { grep -vE ':[0-9]+:[[:space:]]*(//|\*|/\*)'; }

# 5 — no debug output in library code.
# Counting only `std::cout` UNDERSTATES this by ~3x: most of modules/ does
# `using namespace std;`, so the prints are bare `cout <<`. Count both.
grep -rnE '(^|[^:a-zA-Z_.])(std::)?cout *<<' modules/ --include=*.cpp --include=*.hpp | NC

# 5b — core specifically (currently 0; use this to see per-file remainder elsewhere)
grep -rnE '(^|[^:a-zA-Z_.])(std::)?cout *<<' modules/ --include=*.cpp --include=*.hpp | NC \
  | awk -F: '{print $1}' | sort | uniq -c | sort -rn

# 6 — no build noise
grep -rn "#warning" modules/ | NC

# 7 — vestigial export macro removed
grep -rn "DLLExport" modules/ | NC

# 8 — RAWFILE defined in at most one place
grep -rn "define RAWFILE" modules/

# 9 — core must not depend on the viewers module
grep -rn "include <toffy/viewers" modules/core

# 10 — no inter-process primitives in a single-process pipeline
grep -rn "interprocess" modules/core

# 14 — dead Windows branches. The pattern must cover #if, #ifdef AND #ifndef;
# a pattern matching only "#if"/"#ifdef" under-counts by 2 (17 instead of 19).
grep -rnE '^[[:space:]]*#[[:space:]]*if(n?def)?[[:space:]].*MSVC' modules/ | wc -l

# 11 — factory is a lookup, not a type list
grep -c "else if (type ==" modules/core/src/filterfactory.cpp   # expect 0

# 12 — documentation debt
grep -rc "@todo" modules/core | grep -v ":0$"

# 3 — tests exist and pass
ctest --test-dir build --output-on-failure

# 4 — all four dependency configurations build and pass. Each must print 8/8.
for f in "" "-DWITHOUT_PCL=ON" "-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON" \
         "-DWITHOUT_PCL=ON -DCMAKE_DISABLE_FIND_PACKAGE_bta=ON"; do
  d=$(mktemp -d); cmake -S . -B "$d" $f >/dev/null 2>&1 \
    && cmake --build "$d" -j"$(nproc)" >/dev/null 2>&1 \
    && (cd "$d" && ctest 2>&1 | grep -E "tests passed|tests failed")
  rm -rf "$d"
done

# 15 — core warnings. Expect 0 once P2-12 (the filter() overload trap) lands.
cd build && touch ../modules/core/src/*.cpp && make toffy_core 2>&1 | grep -ci warning

# 2.4 — an API change since the last tag requires a new tag. Exit 1 => tag required.
tools/api_change_report.sh "$(git describe --tags --abbrev=0)"

# 2.4 — the tag actually reached the ABI: SONAME must match the tag.
git describe --tags --abbrev=0
readelf -d build/libtoffy.so | grep -i soname
```

Two gotchas when running the block above, both encountered for real:

- **The SONAME check needs a freshly configured build directory.** CMake resolves
  `git describe` at *configure* time and bakes the result into `SOVERSION`, so an existing
  build directory keeps producing the old SONAME after a new tag is created — running this
  in a `build/` configured before `v1.10.0` prints `libtoffy.so.1.7.1` and looks like the
  tag failed. Re-run `cmake -S . -B build` (or use a clean directory) first.
- **`api_change_report.sh` compares against the most recent tag**, so once the tag for the
  current work exists it correctly returns 0. It is a *pre*-tag gate: run it before tagging,
  not after, or it will tell you nothing.

### Explicitly out of scope

Not part of "done", so that the finish line stays reachable:

- Rewriting `modules/filters`, `modules/bta` or `apps/`. They must still *compile* (2.4), but
  their internal quality is a separate programme.
- Replacing Boost.Log, Boost.property_tree or Boost.Thread.
- Any change to the XML configuration format. Fixes preserve existing configs.
- New features, including a real event bus — `P2-14` may legitimately *delete* the stub.
- Formatting anything outside `modules/core` (PR 18 is scoped to core on purpose).

### The one-line version

**Done means: the P0 defects are closed by tests that failed before the fix, CI builds and
tests the library in every dependency configuration, and the mechanical checks above return
zero.**

Progress against that sentence: the first two clauses are met — P0 is 12/12 with
regression tests observed failing beforehand, and CI builds and tests in every dependency
configuration that a runner can reach. The third is not: criteria 5–14 and 16 are
essentially untouched, because the work so far went into correctness, the build matrix and
CI rather than into cleanup. The remaining cleanup is dominated by four structural items —
`P2-4` (the 37-branch factory), `P2-5` (one owner per `Filter`), `P2-7` (threading) and
`P2-3` (the 42 `std::cout` sites) — and `P2-5` is the keystone that the other three get
easier behind.
