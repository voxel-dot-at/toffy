# PR order and dependencies

This is the plan. What has actually landed is [`status.md`](status.md); item titles are in
[`plan/README.md`](plan/README.md).

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
| **19** | `P3-1` widen the verification scope; delete `libraries/sensor/` | — | Nothing depends on it and everything is measured behind it. Do it first so the other PRs are counted honestly |
| **20** | `P3-4` `using Filter::filter;` in `Mux`, then `-Werror` on `modules/core` | 1 | One line, takes core to zero warnings, and stops the count growing for every PR after it |
| **21** | `P3-3` `override` on the four `const`-only filters + `P3-9` the test that a body ran | 1 | Must precede PR 16, or `P2-12` becomes a silent behaviour change instead of a compile error |
| **22** | `P3-7`, `P3-6`, `P3-5`, `P3-8`, `P3-11` — the small build / robustness / CI fixes | — | Independent of everything above; batch them or run them alongside the structural work |
| **23** | `P3-2` delete the dead installed `toffy/web/` headers; `P3-10` drop `TOFFY_EXPORT` | 21 | Both touch installed headers, so both go with the same tag decision |
| **28** | `P3-2` + `P3-12` (`X1`, `X2` of [`plan/controller-extraction.md`](plan/controller-extraction.md)): delete the controller residue, make every installed header compile standalone, gate it in CI | 21 | Landed as the `P3-2` half of the planned PR 23. `P3-10` is the other half of that row and is still open, so it inherits this tag decision rather than opening a new one |
| **29** | `X2b` (`N9`): include guards, a double-inclusion pass in the installed-header check, and a source-tree residue gate for the tree CI cannot build | 28 | Found while gating `X2`: the check compiled each header once and so could not see a header that breaks on the second inclusion. Tagged with PR 28 as `v1.11.0` |
| **30** | `X3`: drop `--host/--port/--html` from `toffyRunner` | — | CLI-visible, not API: no library code reads them. Lands after `v1.11.0`, so it rides the next tag |
| **31** | `X4`: rewrite `use.dox` around `toffyRunner`, delete the three screenshots | — | The page was in the Doxyfile's `EXCLUDE`, so un-excluding it is half the fix. Docs only, no tag needed |
| **33** | `docs` CI job (counter 30) + `Doxyfile.cfg` hygiene | 31 | The docs target exited 0 through 15 diagnostics; now it is warning-free and gated. Docs only, no tag needed |

PRs **19–22 are off the critical path** and none of them depends on the ownership work, so
they can run alongside 9–13. 19 and 20 are worth doing before anything else precisely
because they change how much of a counter can be trusted.

**These PR numbers are the plan's, not the delivery's.** Work arrived out of the order above
and was numbered as it landed, so `PR 23` in this table (`P3-2` + `P3-10`) is not `PR 23` in
[`status.md`](status.md) (`P3-11`). [`status.md`](status.md) is authoritative for what
shipped; this table is only authoritative for what depends on what. Cite items by their `P`
or `X` id, never by PR number, when the two could be confused.

## Notes on the ordering

**The critical path is `1 → 7 → 9 → 10 → {11,12,13} → 16 → 17 → 18`.** Everything else
hangs off it. PR 10 (one owner per `Filter`) is the keystone: `P2-9`'s `getFilter`
reimplementation, `P2-7`'s `FilterThread` ownership and the deletion of the name-based
`clearBank()` teardown all fall out of that one decision, so attempting any of them first
means doing it twice.

**PR 1 was a hard gate, and it has landed.** The row above is the original rationale; it no
longer describes the tree. `enable_testing()`, **8** `add_test()` targets and a CI workflow
that configures, builds and runs `ctest` on every push all exist now — `.github/` is no
longer just Codacy, Flawfinder and Dependabot. The standing rule survives, though: every fix
arrives with a test that **fails on its parent commit**, and the PR description shows that
failure.

**PR 15 landed out of order, deliberately.** `P2-8` is independent of the ownership chain
(PR 10) it nominally follows, and leaving three global logging reconfigurations per filter
per frame in place made every other hot-path change harder to measure. It was the first PR
after `P0` to arrive with a test that fails on its parent commit for the *behaviour* rather
than for a crash.

**PRs 3, 4, 5 and 6 are mutually independent** (all depend only on PR 1) and can run in
parallel. They touch different files: `filterbank.cpp`, `filter.{hpp,cpp}`, `frame.{hpp,cpp}`
and the null-check sites respectively. This is the only stretch of the plan where
parallelism is safe.

**Formatting is last on purpose.** `P2-15` rewrites all 20 files in `modules/core` (22 before
`P2-14` deleted `event.hpp`/`event.cpp`). Landing it early would put a wall of whitespace
between `main` and every correctness diff still in flight, and would guarantee conflicts in
every remaining PR. Landing it last means
one whitespace-only commit, one `git blame --ignore-rev` entry, and a CI gate that prevents
the tree drifting again. It is the only PR in the plan that should be reviewed by
`--ignore-all-space`.

**The C++ standard decision (PR 2) is cheap to take and expensive to reverse.** `P2-11`
wants `std::optional` for `findPos`, `P2-9` wants `std::string_view` for the `btaFrame.hpp`
slot-name constants, and header-scope `const std::string` objects want inline variables. All
three are C++17. If the bump is refused, PRs 9 and 13 need a fallback shape — so find out
in PR 2, not in PR 13.

**Two PRs are explicitly "decide, then act"**: PR 2 (C++ standard — taken, C++17) and PR 8
(apply `-DMSVC` to the library, or delete the branches). Both should open with a
one-paragraph decision record in the PR body before code is written. Leaving the
`#ifdef MSVC` branches as they are — compiled never, tested never, but present — is the one
option that is not acceptable. The asymmetry is what makes PR 8 urgent rather than cosmetic:
`apps/` *does* get `-DMSVC` on a Windows build and the library does not, so the two halves
of one program disagree about `Frame`'s layout. See
[`findings/build.md`](findings/build.md).

**The controller residue (PR 28) was deliberately split from `P3-10`.** Both touch installed
headers, and the plan paired them so that one tag decision covered both. `P3-2` turned out to
be the start of a stage (`X1`), and doing it exposed two more broken installed headers (`X2`),
so it landed alone with the new CI gate. `P3-10` still rides the same tag — see
[`plan/controller-extraction.md`](plan/controller-extraction.md) for why the residue is
ABI-neutral and what `toffy-oatpp` needs from here.

**PR 16 cannot start before PR 21.** `P2-12` deletes the `const` `filter()` overload; four
filters outside core implement only that overload, and three of them never said `override`.
Without `P3-3` first, the change compiles, passes every existing gate, and stops three
filters processing frames — [`findings/api.md`](findings/api.md) has the reproduction.

**PR 12 should run under sanitizers.** It is the only PR changing concurrency behaviour
(`std::atomic<bool>`, replacing `interprocess_semaphore`, `FilterThread` ownership). A green
`ctest` is weak evidence for a data race; a green TSan run is not.
