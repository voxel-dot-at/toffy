# Toffy `modules/core` — PR Sequence & Closing Notes

Ordering companion to [`CLEANUP_PLAN.md`](CLEANUP_PLAN.md) (the numbered items) and
[`PROJECT_INFO.md`](PROJECT_INFO.md) (the evidence). Item references use the plan's own
convention: `P0-2` is item 2 of the P0 table, `P2-7` is item 7 under P2.

This file only answers *what order, and why*. Nothing here re-states the defects.

---

## Where the sequence stands

Re-checked against `8d4c306`. The table below is the plan; this is what has actually landed.

| PR | State | Evidence |
|---|---|---|
| **1** test harness + CI | ✅ **done** | `enable_testing()` (`CMakeLists.txt:420`), 7 `add_test()` targets, `.github/workflows/ci.yml` (matrix + sanitizers) |
| **2** build flags | 🟡 **4 of 5** | C++17 bumped; global `-O2` removed (Debug now `-O0`, Release back to `-O3`); redundant `-Wall` gone; dead `BOOST_VERSION` branch gone. Only `-Werror` left, blocked on `P2-12` |
| **7a** dead macros | ✅ **done** | 22 `DLLExport` → `TOFFY_EXPORT` or deleted; `WIN`/`UNIX` removed; `RAWFILE` 3 headers → 1 |
| **3** `stop()`/`remove()`/`findPos()` | ✅ **done** | `std::optional<int> findPos`, `_pipe.begin() + i`, `stop()` calls `stop()`; 4 tests pin it |
| **4** member init + null checks | ✅ **done** | in-class initialisers in `filter.hpp`; `Controller` ctor throws |
| **5** `Frame` metadata | ✅ **done** | `operator= = default`; all three maps handled together |
| **6** `creators.find()`, `Player::stop()`, `get_child` | ✅ **done** | `Player::stop()` defined at `player.cpp:122` |
| **7** debug leftovers | 🟡 **core done** | `modules/core`: 0 debug prints, 0 `#warning`, 0 `DLLExport`, `RAWFILE` down to 1 header, `WIN`/`UNIX` gone. **129 prints remain in `filters`/`bta`** — the real figure, once bare `cout` is counted (the old "42" missed them) |
| **8** Windows paths | ❌ not started | 19 `MSVC` preprocessor branches still present |
| **9** factory → registration | ❌ not started | 37 `else if (type ==` branches remain |
| **10** one owner per `Filter` | ❌ not started | `_pipe` and the factory still hold raw `Filter*` |
| **11** collapse run methods | ❌ not started | `stedBackward()`, `CERROR` still in the public API |
| **12** threading | ❌ not started | `bool keepRunning` (`filterThread.hpp:85`), 2 `interprocess` uses |
| **13** module layering | ❌ not started | `#include <toffy/viewers/imageview.hpp>` still in `controller.cpp` |
| **14** plugin loaders | ❌ not started | all three `::loadPlugins` still exist (`controller.cpp:329`, `filterbank.cpp:405`, `player.cpp:132`) |
| **15** logging policy | ❌ not started | `setLoggingLvl()` still on the hot path |
| **16** `Frame` API + `filter()` overload | ❌ not started | `getSertMatPtr` remains; the 3 `-Woverloaded-virtual=` warnings are this trap |
| **17** delete `Event` | ✅ **done** | `event.hpp`/`event.cpp` gone; tagged `v1.10.0` (local, **not pushed**) |
| **18** `clang-format` | ❌ not started | 11 of 20 core files still contain hard tabs |

**Six of eighteen PRs are done** (1, 3, 4, 5, 6, 17), plus 4 of the 5 sub-items of PR 2. The
whole P0 block, the test/CI fence and the build-flag cleanup are in place, which was the point
of the critical path's first stretch — PR 2's `-O2` removal in particular was what made the
concurrency work in PR 12 diagnosable at all. Everything from PR 7 onward — the structural
work — is untouched, which still matches the `2/16` figure in `DOD.md` (PR 2 counts as open
until `-Werror` lands).

---

## Recommended PR order

| PR | Contents | Depends on | Why here |
|---|---|---|---|
| **1** | `P1-1` test harness: `enable_testing()`, one `ctest` target, CI job that builds and runs it | — | Nothing else can be verified today. Every later PR needs this fence. |
| **2** | `P2-16` build flags: drop `-O2` from Debug, delete redundant `-Wall`, decide C++14 vs C++17 | 1 | Two reasons it is early: `-O2` in Debug makes the concurrency work in `P2-7` undiagnosable, and the standard decision changes the *shape* of `P2-11` and `P2-13`. Deciding late means doing them twice. |
| **3** | `P0-1`…`P0-4` `FilterBank::stop()`, `remove(size_t)`, `remove(string)`, `findPos()` | 1 | Active memory corruption. Highest severity, and a corrupted `_pipe` invalidates the evidence for other findings. |
| **4** | `P0-5` `Filter` member initialisation + `P0-7`/`P0-8`/`P0-9` null checks | 1 | Uninitialised `_bank` is a jump to an arbitrary address; cheap, independent, and makes every later debugging session trustworthy. |
| **5** | `P0-6` `Frame` `meta`/`desc` consistency | 1 | Self-contained; `Frame` is touched by almost everything after it, so stabilise it first. |
| **6** | `P0-10`, `P0-11`, `P0-12` — `creators.find()`, `Player::stop()`, guarded `get_child` | 1 | Small leftovers; finish P0 so the label stays meaningful. |
| **7** | `P2-3` debug leftovers and dead code | 3–6 | Removes code that would otherwise be copied into the structural PRs. Must come *after* P0 so the P0 diffs stay small and reviewable. |
| **8** | `P2-2` Windows paths — fix or delete | 7 | Deleting dead `#ifdef MSVC` branches shrinks `loadPlugin()` before `P2-10` consolidates it. |
| **9** | `P2-4` `createFilter()` → `registerCreator()` | 7 | Mechanical and independent. Do it before `P2-5` so the ownership change touches one map lookup instead of ~30 branches. |
| **10** | `P2-5` one owner per `Filter` (`FilterPtr`) | 9 | The keystone. `P2-9`'s `getFilter` fix and `P2-7`'s `FilterThread` ownership both fall out of this decision, so it must land before them. |
| **11** | `P2-6` + `P2-13` collapse the four run methods, unify state, rename typos | 10 | Tightly coupled — the run methods are where `Controller::state` is read and where banks are manipulated. One PR, no renames mixed with behaviour (plan ground rule). |
| **12** | `P2-7` threading: `atomic<bool>`, replace `interprocess_semaphore`, `FilterThread` ownership | 10 | Needs the ownership model settled. Run this PR under TSan/ASan (see Definition of done). |
| **13** | `P2-9` module layering: drop the `imageview.hpp` dependency, move `btaFrame.hpp`, `getFilter` against `_pipe`, private `Controller` members | 10 | `getFilter` can only be reimplemented against `_pipe` once `_pipe` owns. |
| **14** | `P2-10` consolidate the three plugin loaders | 8 | Easier once the dead `MSVC` branches are gone. |
| **15** | `P2-8` logging policy out of `Filter` | 6 | Independent, but touches the hot path — land after the correctness work so perf changes are not confused with fixes. |
| **16** | `P2-11` `Frame` accessor API, `P2-12` `filter()` const overload | 2, 5 | Both are API changes; `P2-11` wants the C++17 decision from PR 2, and both want `P0-6` already merged. |
| **17** | `P2-14` finish or delete the `Event` stub | 16 | Lowest urgency; delete is likely, and deleting is cheaper after the interface churn in PR 16. |
| **18** | `P2-15` `clang-format -i` over `modules/core`, then a CI format gate | **all** | Absolutely last. Whitespace-only, `git blame --ignore-rev`'d, and it conflicts with everything above. |

## Notes on the ordering

**The critical path is `1 → 7 → 9 → 10 → {11,12,13} → 16 → 17 → 18`.** Everything else
hangs off it. PR 10 (one owner per `Filter`) is the keystone: `P2-9`'s `getFilter`
reimplementation, `P2-7`'s `FilterThread` ownership and the deletion of the name-based
`clearBank()` teardown all fall out of that one decision, so attempting any of them first
means doing it twice.

**PR 1 was a hard gate, and it has landed.** The description below is the original rationale;
it no longer describes the tree. `enable_testing()`, 7 `add_test()` targets and a CI workflow
that configures, builds and runs `ctest` on every push all exist now — `.github/` is no longer
just Codacy, Flawfinder and Dependabot. The standing rule survives, though: every fix arrives
with a test that **fails on its parent commit**, and the PR description shows that failure.

**PRs 3, 4, 5 and 6 are mutually independent** (all depend only on PR 1) and can run in
parallel. They touch different files: `filterbank.cpp`, `filter.{hpp,cpp}`, `frame.{hpp,cpp}`
and the null-check sites respectively. This is the only stretch of the plan where
parallelism is safe.

**Formatting is last on purpose.** `P2-15` rewrites all 22 files in `modules/core`. Landing
it early would put a wall of whitespace between `main` and every correctness diff still in
flight, and would guarantee conflicts in all fourteen remaining PRs. Landing it last means
one whitespace-only commit, one `git blame --ignore-rev` entry, and a CI gate that prevents
the tree drifting again. It is the only PR in the plan that should be reviewed by
`--ignore-all-space`.

**The C++ standard decision (PR 2) is cheap to take and expensive to reverse.** `P2-11`
wants `std::optional` for `findPos`, `P2-9` wants `std::string_view` for the `btaFrame.hpp`
slot-name constants, and header-scope `const std::string` objects want inline variables. All
three are C++17. If the bump is refused, PRs 9 and 13 need a fallback shape — so find out
in PR 2, not in PR 13.

**Two PRs are explicitly "decide, then act"**: PR 2 (C++ standard) and PR 8 (apply `-DMSVC`
to the library, or delete the branches). Both should open with a one-paragraph decision
record in the PR body before code is written. Leaving the `#ifdef MSVC` branches as they are
— compiled never, tested never, but present — is the one option that is not acceptable.

**PR 12 should run under sanitizers.** It is the only PR changing concurrency behaviour
(`std::atomic<bool>`, replacing `interprocess_semaphore`, `FilterThread` ownership). A green
`ctest` is weak evidence for a data race; a green TSan run is not.
