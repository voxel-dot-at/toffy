# Status

What has landed, what is in flight, and what the release situation is. Counters are in
[`counters.md`](counters.md); the order the rest goes in is in [`order.md`](order.md).

State as of `8689d68` plus PRs 28 and 29 (`P3-2`, `P3-12`, `X2b` — stages `X1`–`X2b` of
[`plan/controller-extraction.md`](plan/controller-extraction.md)), re-checked against the tree
rather than carried over from the previous round.

## Landed

| PR | Item | State | Evidence |
|---|---|---|---|
| **1** | test harness + CI | ✅ done | `enable_testing()` (`CMakeLists.txt:436`), `add_test()` targets — 8 at the time, **11** now (counters 24) — `.github/workflows/ci.yml` (PCL matrix + ASan/UBSan/LSan job) |
| **2** | build flags | ✅ done | C++17 bumped; global `-O2` gone (Debug → `-O0`, Release back to `-O3`); redundant `-Wall` gone; dead `BOOST_VERSION` branch gone; `-Werror` on `toffy_core` via `P3-4` (`option(CORE_WERROR ON)`) |
| **3** | `stop()` / `remove()` / `findPos()` | ✅ done | `std::optional<int> findPos`, `_pipe.begin() + i`, `stop()` calls `stop()`; 4 tests pin it |
| **4** | member init + null checks | ✅ done | in-class initialisers in `filter.hpp`; `Controller` ctor throws |
| **5** | `Frame` metadata | ✅ done | `operator = default`; all three maps handled together |
| **6** | `creators.find()`, `Player::stop()`, guarded `get_child` | ✅ done | `Player::stop()` defined in `player.cpp` |
| **7** | debug leftovers, `modules/core` | ✅ core done | 0 prints, 0 `#warning` in core. **128 remain in `modules/filters`/`modules/bta`, 97 more in `libraries/`** |
| **7a** | dead macros | ✅ done | 22 `DLLExport` deleted, `WIN`/`UNIX` gone, `RAWFILE` 3 headers → 1. Side effect: 7 of the 19 `MSVC` branches went with them |
| **15** | logging policy | ✅ done | `setLoggingLvl()` is data-only; 3 hot-path calls removed; `Filter::setGlobalLogLevel()` added; `~Player` no longer kills process logging. 7 new tests, 4 of them failing pre-fix |
| **17** | delete `Event` | ✅ done | `event.hpp`/`event.cpp` gone; tagged `v1.10.0` — **local only, not pushed**, and it points at `7fea594`, not the branch tip |
| **19** | `P3-1` verification scope + `libraries/sensor/` orphan | ✅ done | 2 unreferenced headers deleted; counters 7a 3→0, 8c 2→0, 14a 16→15, 18 8→7. Counter 21's command was also broken (grepped `_pattern`, the variables are `_depthPattern`/`_amplPattern`) |
| **20** | `P3-4` `using Filter::filter;` + `-Werror` on core | ✅ done | core 3 → **0** warnings in all four configurations; `toffy_core` builds `-Werror`; gate proven by injecting an unused variable into `filter.cpp` and watching the build stop; tree 32 → 28 warnings |
| **21** | `P3-7` `if( ${VAR} )` sweep + CI check | ✅ done | 4 sites → 0; `cmake-hygiene` job added, scoped to the 29 tracked CMake files, verified red on a re-introduced site and green on the fixed tree |
| **22** | `P3-3` `override` on the four `const`-only filters | ✅ done | counter 25 3 → **0**; `const` deliberately kept (dropping it is warning-free but opens a `const Filter&` → `return false` window on installed headers) — the drop moves into `P2-12`, where the base change closes it again in the same diff |
| **23** | `P3-11` CI warning report | ✅ done | `tools/warning_report.sh` + a CI step after the build; buckets warnings per area, fails only on `modules/core`. Verified in both directions: exit 1 on a pre-`P3-4` log (4 core warnings), exit 0 on the current tree, and a vacuous-report notice instead of a false zero when the log contains no compilation |
| **24** | `P3-9` a test that a filter's body actually ran | ✅ done | `tests/test_filter_overloads.cpp` (ctest target `filter_overloads`, 9 targets): both override shapes asserted through `FilterBank::filter()` on the frame slot, plus a negative control for a `filter()` that overrides nothing. Fence verified by stubbing the base's delegation to `return false` — the two const-only tests fail, the other three stay green — then `filter.hpp` restored byte-identical |
| **25** | `A24` `CSVSource` frame counter / sequence flag | ✅ done | `loadConfig` assigned `<options/sequence>` into the *counter* and never set `useSequence`; `getConfig` wrote the counter back under the flag's key; neither member was initialised. 4 tests, all 4 failing pre-fix (first frame played was 1; with the flag *off* it advanced 1, 2, 3 on uninitialised memory) |
| **26** | `P3-6` (first half) `csv_source` format strings and `fscanf` | ✅ done | patterns validated at config time, expansions checked for truncation, both `fscanf` loops checked; 5 tests, 4 observed failing pre-fix — one of them a **segfault** on `%s%s`. Warnings 28 → 26. `exportcsv`'s 3 sites stay open under the same item |
| **27** | `P3-6` (second half) `exportcsv` format strings | ✅ done | `options/pattern` validated at config time, the config-sized VLA replaced by a fixed buffer, all three expansions bounded; the validator and `formatPath` moved to `toffy/filter_helpers.hpp` and `csv_source` switched to them rather than keeping a copy. 5 tests, 2 observed failing pre-fix — one by **segfault**. Warnings unchanged at 26. Counter 21 → **0**, and its command was tightened to match the format position (verified: 2 on each pre-fix file, 0 now). Filed `A25` on the way: `options/skipZeroes` is plumbed through four functions and ignored |
| **28** | `P3-2` + `P3-12` the controller residue (`X1`, `X2`) | ✅ done | 4 installed `toffy/web/` headers deleted and the `#ifdef WITH_CONTROL` hooks with them (counter 26 5 → **0**); 2 more installed headers *fixed* — `graphs/graph_utils.hpp` and `graphs/contour_utils.hpp` used `cv::line`/`cv::Point` and compiled only behind another header. Counter 17 **0 of 86**, by `tools/installed_header_check.sh`, now a CI job (`installed-headers`, verified red on a re-added `toffy/web/` header and green on the tree). ABI gate measured: exported symbol set **identical**, 2 889 before and after. **API removal — the tag is owed**, see *Release* below |
| **29** | `X2b` the two holes `X2` left (`N9`) | ✅ done | `X2` compiled each header **once**, which is the shape of `N1` and blind to its complement: `toffy/bta/FrameHeader.hpp`, an installed `typedef struct` with no include guard, compiles once and fails twice (`conflicting declaration`). 3 of 86 installed headers had no guard; all 3 get `#pragma once`. The check gains a `twice` pass (86 × 2 = **172** checks, observed 1 broken before the guard and 0 after), and the CI job gains a `No controller residue in the source tree` step — a grep of tracked code and build files for `toffy_web/`/`toffy/web/`/`WITH_CONTROL`, which is the only residue check that reaches `modules/bta`, whose headers no runner can stage without the SDK. Verified red on a re-added `#ifdef WITH_CONTROL`, green after. ABI unchanged by construction and re-measured: **2 889** symbols, `diff` empty |
| **30** | `X3` the three dead `toffyRunner` options | ✅ done | `--host/-h`, `--port/-p`, `--html/-d` deleted with the `cout` lines that printed them — counter 27 3 → **0**. Verified on the binary, not the source: `toffyRunner --help` now lists `--help`, `-c/--config`, `-s/--sleepDelay`, `-f/--output2File` and nothing else, and a 5-second run against `tests/xml/cond_main.xml` loads and runs. No new compiler warnings (the two `apps/main.cpp` warnings are pre-existing, present in the `8689d68` base build). The `Usage:` line said `options_description [options]` — a boost example placeholder — and now says `toffyRunner`. **CLI-visible change, no API/ABI change**: nothing in the library reads these options, and the values were never propagated anywhere |
| **31** | `X4` the Use guide, and the screenshots | ✅ done | The three `control_*.png` screenshots deleted (outdated, referenced only from the dead page). `use.dox` rewritten around `toffyRunner` — real options, the checked-in example config, how to stop, and a pointer to toffy-oatpp for whoever is looking for the browser UI. **The part that made it a real fix:** the page was in `EXCLUDE` in `docs/Doxyfile.cfg`, so it never rendered — a doxygen run read 12 files and `use.dox` was not one of them. Un-excluded, added to `page_order.dox`, and verified by building the docs: `use.html` exists, is linked from `pages.html`, the `\include examples/config.xml` resolves, and the only remaining non-obsolete warning is the pre-existing `descDocs` path. Counter 16 8 → **0**, with the `Player` `@todo` that started this whole extraction rewritten to say what a UI still cannot get (→ `X5`). Filed `A26`: the Ctrl-C handler in `main.cpp` is compiled and never installed on Linux |
| **32** | `docs/Doxyfile.cfg` hygiene (found by `X4`) | ✅ done | The `docs` target went from **15 warnings/errors to 0**: 14 obsolete tags (`Doxyfile.cfg` still declares `# Doxyfile 1.8.12`), an `EXAMPLE_PATH` entry pointing at `<root>/descDocs` when the directory is `docs/descDocs` (deleted, not corrected — correcting it would add a path to the `\include` search that has never been in effect), and an `HTML_EXTRA_FILES` entry for a `roi_guide.odt` that `git log --all` shows was never committed. `docs` exited 0 through all 15, which is why they survived since `59b6bcf`. **One warning lied:** `HTML_TIMESTAMP` is reported obsolete but still controls the footer — removing it changes 22 of 22 generated pages, verified by diffing the whole tree. The timestamp is gone deliberately (reproducible output); `TIMESTAMP = YES` puts it back. Counter 30 added |
| **33** | `docs` CI job: the gate for counter 30 | ✅ done | A fifth CI job runs the `docs` target and fails on any `warning:`/`error:` in its log — the exit code cannot go red, which is how 15 diagnostics survived since `59b6bcf`. Deliberately PCL-less and bta-less: measured, that configuration generates the same 22 pages with 0 warnings, so `libpcl-dev` buys documentation nothing. Verified red by re-adding one obsolete `PERL_PATH` line, green on the tree; YAML and the step's shell both parse-checked |

**Twenty-three PRs are done** (1, 2, 3, 4, 5, 6, 15, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27,
28, 29, 30, 31, 32, 33 — the numbering has no gaps, but PR 7 is core-only and the PRs beyond
18 are the second-pass work, so "n of 18" stopped being a meaningful fraction once `P3`
started landing). PR 2 is
complete rather than 4-of-5. That is the whole `P0` block, the test/CI fence, the build-flag
cleanup and core's warning gate: the critical path's first stretch, plus every cheap
second-pass item — a false green removed, a class of silent build breakage closed, a warning
gate landed, `P2-12` made compile-checked, both config-as-format-string sites closed, and the
public header surface proved compilable rather than assumed.

`P1`/`P2` items closed: **4 of 16** (`P1-1`, `P2-8`, `P2-14`, `P2-16`), with `P2-3`
core-only. `P2-16` closed when `P3-4` landed the `-Werror` half of it.

## Open

| PR | Item | State |
|---|---|---|
| **8** | `P2-2` Windows paths — fix or delete | ❌ 12 `MSVC` branches in `modules/`, 15 tree-wide |
| **9** | `P2-4` factory → registration | ❌ 37 branches |
| **10** | `P2-5` one owner per `Filter` | ❌ the keystone |
| **11** | `P2-6` + `P2-13` run methods, state, typos | ❌ |
| **12** | `P2-7` threading | ❌ `bool keepRunning`, 2 `interprocess` uses |
| **13** | `P2-9` module layering | ❌ `imageview.hpp` still included by `controller.cpp` |
| **14** | `P2-10` three plugin loaders | ❌ all three still exist |
| **16** | `P2-11` + `P2-12` `Frame` API, `filter()` overload | ❌ see `P3-9` before touching `P2-12` |
| **18** | `P2-15` formatting | ❌ 11 of 20 core files have hard tabs; clang-format 18.1.3 **is** installed |

The second-pass work (`P3-1` … `P3-12`) is listed in
[`plan/second-pass.md`](plan/second-pass.md). All four of its cheap items are done (`P3-1`,
`P3-3`, `P3-4`, `P3-7`) and so are `P3-11`, `P3-9` — the latter is the fence `P2-12` cannot
be merged without — `P3-6` in full, and `P3-2`/`P3-12` (PR 28). What is left there is `P3-5`
(the `system()` call), the mechanical `P3-10` and the `P3-8` note. `P3-10` and `P2-12` are the
remaining API-surface changes and belong with `P3-2`'s tag.

The controller extraction (`X1` … `X6`,
[`plan/controller-extraction.md`](plan/controller-extraction.md)) is the other half of PR 28:
the web control UI is `toffy-oatpp`'s now, and `X1`–`X2` are done. `X3` (three dead CLI
options), `X4` (`use.dox` and its three screenshots) and `X5` (the C++ API oatpp binds to,
blocked on five open items) are open; `X6` is a taken decision — keep the filter plugin ABI,
do not rehost `initUI` here.

## Gates

- **`DOD 1.1` green: all four dependency configurations build and pass.** PCL on/off ×
  BTA on/off, each configured from scratch, built and run — 11/11 in every cell, 26/25/24/23
  warnings (re-measured on PR 28: 26 default, 25 PCL-off, 24 BTA-off, 23 both off). Both axes
  were broken before this branch: PCL-off on unguarded typedefs and on CMake `if()`
  clauses that expanded to nothing, BTA-off on an unconditional `add_subdirectory(bta)`.
- **CI builds, tests and reports warnings on every push and PR** — five jobs: the PCL
  matrix, the sanitizer build, `installed-headers` (`P3-12`, two passes since `X2b`, plus the
  source-tree residue step), `docs` (counter 30) and `cmake-hygiene`; the build
  step pipes its log through `tools/warning_report.sh` (`P3-11`). Two limits when citing a
  green run:
  runners cannot have the proprietary bta SDK, so only the **BTA-off** axis is covered
  automatically — `X2b`'s grep step is what covers `modules/bta`'s 7 installed headers in the
  meantime — and the `docs` job gates *warnings*, not correctness: a `\ref` to a page that
  does not exist is a warning doxygen does emit, but a page that is simply wrong, like the
  `use.dox` of `X4`'s before-state, has always been silent. It also runs PCL-less and
  bta-less, which was measured to produce the same 22 pages.
- **`A23` (no consistent error convention) blocks further correctness work** in config
  loading — see [`findings/correctness.md`](findings/correctness.md).
- **PRs 28 and 29 remove installed public headers; `v1.11.0` is the tag that carries them**
  (see *Release* below). `api_change_report.sh v1.10.0` exits 1, as it must — it is still
  exit 1 against `v1.10.0` after the tag exists, because the base ref is what you diff
  against; run it as `api_change_report.sh v1.11.0` from the tagged commit onwards. The ABI evidence is measured rather than asserted: the
  exported dynamic symbol set of `libtoffy.so` is identical before and after (2 889 symbols,
  `diff` empty, Release/PCL-on/BTA-on both sides, base built from `8689d68` in a separate
  tree). Three of the four deleted headers could not be included at all, so no compiling
  program can break; `toffy/web/actions/action.hpp` could, and its removal is the one real —
  if improbable — API break. See [`dod/stage-gates.md`](dod/stage-gates.md) §2.4.

## Release / tag situation

- `v1.10.0` is tagged (annotated) for the `Event` removal and the vtable change it caused.
  Before it this tree built `libtoffy.so.1.7.1` while exporting none of 1.7.1's symbols —
  the SONAME advertised a compatibility it did not have.
- The tag is **local; it has not been pushed**, and it points at `7fea594` rather than the
  branch tip. Anything shipped from here needs the tag moved or re-cut.
- It is `1.10.0` rather than the convention-implied `1.8.0` so that it does not sort below
  the `v1.9.0` that exists on `origin/next`. Those two numbering lines still need
  reconciling when the branches meet.
- `SOVERSION` is the **full** version, not the major alone, so *every* patch bump changes
  the SONAME and forces a downstream relink. Version decisions here are never routine —
  see [`dod/stage-gates.md`](dod/stage-gates.md) §2.4.
- **`v1.11.0` is the agreed number and it is cut at the PR 29 tip.** `P3-2` changed the
  installed header surface and `P3-12`/`X2b` changed three more headers, so the tag covers
  PRs 28 and 29 together; `P3-10` will need the one after it. `1.11.0` rather than `1.8.0`
  keeps clearing the `v1.9.0` on `origin/next`, and a removal plus a header-hygiene release is
  a minor bump, not a patch, under the reading `v1.10.0` already set for the `Event`
  deletion. From the tagged commit `git describe` reports `1.11.0`, so the SONAME stops
  claiming `1.10.0` while the header surface has already moved. **The tag is local until it
  is pushed**, like `v1.10.0` before it.
