# Definition of done — the programme

The cleanup is done when all of the following hold. Each criterion is a **target**; the
measured value of every one of them is in [`../counters.md`](../counters.md), which is
also where the commands live. This file does not restate numbers, because the last three
copies of this table disagreed with each other about `19` vs `12`, `21` vs `20` and `42`
vs `128` for the same greps.

| # | Criterion | Target |
|---|---|---|
| 1 | All 12 `P0` correctness items closed, each with a regression test | 12/12 |
| 2 | All 16 `P1`/`P2` items closed, or explicitly rejected with a written rationale | 16/16 |
| 3 | CI configures, builds and runs `ctest` on every PR | required |
| 4 | Builds in all four `PCL_FOUND`/`HAS_BTA` combinations | 4/4 |
| 5 | Debug prints in library code — `std::cout` **and** bare `cout`, `modules/` **and** `libraries/` | 0 |
| 6 | `#warning` directives | 0 |
| 7 | `DLLExport` occurrences (vestigial macro), tree-wide | 0 |
| 8 | Headers defining `RAWFILE` | ≤ 1 |
| 8b | `WIN` / `UNIX` defines in public headers, tree-wide | 0 |
| 9 | `#include <toffy/viewers/...>` from `modules/core` | 0 |
| 10 | `interprocess` primitives in core | 0 |
| 11 | Hard-coded type branches in `createFilter()` | 0 |
| 12 | `@todo` markers in `modules/core` | ≤ 5, and **0** in `filterThread.hpp` |
| 13 | `delete` of a `Filter` outside its one owner | 0 |
| 14 | `MSVC` branches that are neither compiled nor tested, tree-wide | 0 |
| 15 | Warnings in `modules/core` under `-Wall -Wextra` | 0, with `-Werror` on that directory |
| 16 | Docs (and installed headers) describing a product that does not exist | 0 |
| 17 | Installed public headers that do not compile | 0 |
| 18 | Public headers including the CMake-generated `toffy_export.h` | 0 |
| 19 | CMake `if( ${VAR} )` sites | 0 |

Met today: 1, 3, 4, 6, 7, 8, 8b and 15 — and 5 inside `modules/core` only, with 225 prints
still standing tree-wide. Two of those greens arrived late and were false for a while: 7 and
8b read 0 while the pattern sat in `libraries/sensor/`, outside the greps' scope, until
`P3-1` deleted the orphan. That is the difference between a criterion and a grep — the scope
is part of the claim ([`../dod/README.md`](../dod/README.md)).

**Criterion 15 is met, and the blocker it was gated on was wrong.** The three warnings were
one root cause, but it was `Mux` hiding `Filter::filter` by declaring a different signature,
not the `P2-12` overload trap: a single `using Filter::filter;` line took core to zero
(`P3-4`, landed). `toffy_core` now compiles `-Werror`, so the criterion holds itself — the
count cannot drift upward again without a build failure. Measured at 0 in all four
`PCL_FOUND` × `HAS_BTA` configurations, which also retires the configuration-dependence
warning this criterion used to carry: at zero, "2 in a PCL-less build" has nowhere to hide.
The same line removed a fourth warning outside core, in `3d/muxMerge.hpp`.

**Criterion 5 has two blind spots stacked on it.** Counting only `std::cout` misses the
bare `cout <<` sites that most of the tree writes (`using namespace std;`), which was a
factor of ~3. Scoping the grep to `modules/` misses `libraries/`, which links into the same
`libtoffy.so` — another 97 sites. A criterion that can be green while 225 prints ship is
not a criterion.

## Verification block

Every command below must return nothing (or the stated value) when the programme is
complete. The measurement commands — including the comment-stripping helper `NC` that the
macro checks need, because the tree's comments name the macros that were removed — are in
[`../counters.md`](../counters.md). What follows are the *gates*.

```sh
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

# 15 — core warnings. 0 since P3-4; toffy_core builds -Werror, so a nonzero count here
#      is a build failure rather than something to note in a commit message.
cd build && touch ../modules/core/src/*.cpp && make toffy_core 2>&1 | grep -ci warning

# 2.4 — an API change since the last tag requires a new tag. Exit 1 => tag required.
tools/api_change_report.sh "$(git describe --tags --abbrev=0)"

# 2.4 — the tag actually reached the ABI: SONAME must match the tag.
git describe --tags --abbrev=0
readelf -d build/libtoffy.so | grep -i soname

# 2.4 — a change that claims to be ABI-neutral must prove it. Snapshot the exported
# symbol set before and after and diff; this is how P3-10 (dropping TOFFY_EXPORT) was
# shown to be a no-op rather than an assertion.
readelf --dyn-syms -W build/libtoffy.so | awk '$7!="UND"{print $8}' | sort > /tmp/syms.txt
```

Three gotchas, all encountered for real:

- **The SONAME check needs a freshly configured build directory.** CMake resolves
  `git describe` at *configure* time and bakes the result into `SOVERSION`, so an existing
  build directory keeps producing the old SONAME after a new tag — running this in a
  `build/` configured before `v1.10.0` prints `libtoffy.so.1.7.1` and looks like the tag
  failed. Re-run `cmake -S . -B build` (or use a clean directory) first.
- **`api_change_report.sh` compares against the most recent tag**, so once the tag for the
  current work exists it correctly returns 0. It is a *pre*-tag gate: run it before
  tagging, not after, or it will tell you nothing.
- **A check scoped to `modules/` is a claim about `modules/`.** Two criteria reported 0 for
  weeks with the pattern sitting in `libraries/`, and criterion 5 hides 97 more prints the
  same way. State the scope in the check, not in prose next to it.

## Explicitly out of scope

Not part of "done", so that the finish line stays reachable:

- Rewriting `modules/filters`, `modules/bta`, `libraries/` or `apps/`. They must still
  *compile* (`dod/stage-gates.md` §2.4), and their debug output and warnings count against
  criteria 5 and 15 because they ship in `libtoffy.so` — but their internal quality is a
  separate programme.
- Replacing Boost.Log, Boost.property_tree or Boost.Thread.
- Any change to the XML configuration format. Fixes preserve existing configs.
- New features, including a real event bus — `P2-14` may legitimately *delete* the stub.
- Formatting anything outside `modules/core` (`P2-15` is scoped to core on purpose).

## The one-line version

**Done means: the `P0` defects are closed by tests that failed before the fix, CI builds and
tests the library in every dependency configuration, and the mechanical checks return zero
across the whole tree.**

Progress against that sentence: the first two clauses are met — `P0` is 12/12 with
regression tests observed failing beforehand, and CI builds and tests in every dependency
configuration a runner can reach. The third is not. The remaining cleanup is dominated by
four structural items — `P2-4` (the 37-branch factory), `P2-5` (one owner per `Filter`),
`P2-7` (threading) and `P2-3` (the 225 debug prints) — and `P2-5` is the keystone the other
three get easier behind. Alongside them sit the `P3` items, four of which are about fifteen
lines and each of which removes a false green.
