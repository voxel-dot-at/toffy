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

1. **Builds in all four dependency configurations**, not just the developer's:

   | `PCL_FOUND` | `HAS_BTA` | note |
   |---|---|---|
   | on | on | the reference build |
   | on | off | default |
   | off | on | |
   | off | off | **currently broken** — the PCL typedefs at `frame.hpp:49-50` are outside the `#if PCL_FOUND` guard that covers the includes at `:27-30` |

   A PCL-less build is not optional to check: it is the configuration most downstream
   packagers will hit first.

2. **CI is green, and CI actually builds and tests.** Today `.github/` contains only
   `codacy.yml`, `flawfinder.yml` and `dependabot.yml` — two static-analysis bots and a
   dependency updater. There is **no workflow that configures, compiles or runs anything**.
   Until PR 1 lands, "CI green" is not evidence of anything and must not be cited as such.

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
  **ASan/UBSan** build. That path is live undefined behaviour today, so a green normal build
  proves very little — run the bank add/remove/`clearBank()` sequence under ASan and show
  the output.
- `stop()` is asserted to actually stop: a test drives a bank, calls `stop()`, and asserts
  every child reports `filterIdle`. Today it reports `filterRunning` forever.

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

| # | Criterion | Now | Target |
|---|---|---|---|
| 1 | All 12 `P0` correctness items closed, each with a regression test | 1/12 | 12/12 |
| 2 | All 16 plan items closed, or explicitly rejected with a written rationale | 0/16 | 16/16 |
| 3 | CI configures, builds and runs `ctest` on every PR | none | required |
| 4 | Builds in all four `PCL_FOUND`/`HAS_BTA` combinations | PCL-off broken | 4/4 |
| 5 | `std::cout` in library code (`modules/`) | 42 | 0 |
| 6 | `#warning` directives | 1 | 0 |
| 7 | `DLLExport` occurrences (vestigial macro) | 22 | 0 |
| 8 | Headers defining `RAWFILE` | 3 | 1 or 0 |
| 9 | `#include <toffy/viewers/...>` from `modules/core` | 1 | 0 |
| 10 | `interprocess` primitives in core | 2 | 0 |
| 11 | Hard-coded type branches in `createFilter()` | 37 | 0 |
| 12 | `@todo` markers in `modules/core` | 23 | ≤ 5, and **0** in `filterThread.hpp` |
| 13 | `delete` of a `Filter` outside its owner | present | 0 |
| 14 | `#ifdef MSVC` branches that are neither compiled nor tested | present | 0 |
| 15 | Compiler warnings in `modules/core` under `-Wall -Wextra` | uncounted | 0, with `-Werror` on that directory |
| 16 | Docs describing behaviour that does not exist | ≥ 3 | 0 |

### Verification block

Copy-pasteable; each command must return nothing (or the stated value) when the programme is
complete.

```sh
# 5 — no debug output in library code
grep -rn "std::cout" modules/ --include=*.cpp --include=*.hpp

# 6 — no build noise
grep -rn "#warning" modules/

# 7 — vestigial export macro removed
grep -rn "DLLExport" modules/

# 8 — RAWFILE defined in at most one place
grep -rn "define RAWFILE" modules/

# 9 — core must not depend on the viewers module
grep -rn "include <toffy/viewers" modules/core

# 10 — no inter-process primitives in a single-process pipeline
grep -rn "interprocess" modules/core

# 11 — factory is a lookup, not a type list
grep -c "else if (type ==" modules/core/src/filterfactory.cpp   # expect 0

# 12 — documentation debt
grep -rc "@todo" modules/core | grep -v ":0$"

# 3 — tests exist and pass
ctest --test-dir build --output-on-failure
```

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
