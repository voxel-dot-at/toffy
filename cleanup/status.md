# Status

What has landed, what is in flight, and what the release situation is. Counters are in
[`counters.md`](counters.md); the order the rest goes in is in [`order.md`](order.md).

State as of PR 36 (`P2-5`, one owner per `Filter` — the keystone), re-checked against the
tree rather than carried over from the previous round. The *Release / tag situation* section
was re-checked against `origin` separately on 2026-10-27, after the tags were pushed and
`origin/next` was deleted; the code state above it is `feature/code-cleanup`.

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
| **17** | delete `Event` | ✅ done | `event.hpp`/`event.cpp` gone; tagged `v1.10.0` (annotated, **on `origin`**), at `7fea594` rather than the branch tip — see *Release / tag situation* |
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
| **34** | `P2-2` the Windows paths — **deleted**, and the platform policy says so | ✅ done | The decision the item asked for, not a grep: toffy is POSIX-only. All **15** `MSVC` branches gone (counters 14/14a → **0**) — the `LoadLibrary`/`GetProcAddress` plug-in loaders in `controller.cpp` and `filterbank.cpp`, the `boost::posix_time` epoch arithmetic in `bta.cpp`/`capturerFilter.cpp`, the `RAWFILE` `".rw"` branch, the `<Windows.h>` include in `cloudviewpcl.cpp`, and the `SetConsoleCtrlHandler` half of `main.cpp` — plus the `-DMSVC` / `-D_WIN32_WINNT` definitions that only ever reached `apps/`, which is the asymmetry that made the two halves of one program disagree. `CMakeLists.txt` now fails a Windows configure with a message naming the reason instead of dying on a missing `<dlfcn.h>`. **Nothing was enabled**: every deleted branch was the one the library never compiled, so the output of every build this tree can produce is unchanged — measured, not asserted: 4 dependency configurations from scratch, 11/11 `ctest` each, warnings 26/25/24/23 exactly as before, the warning *set* identical to a base worktree build (only the two `apps/main.cpp` line numbers move, because lines were deleted above them), and `readelf --dyn-syms` unchanged at **2 889** symbols, `diff` empty. Gate: a second step in the `cmake-hygiene` job fails on any `#if`/`#ifdef`/`#ifndef`/`#elif` naming `MSVC` or `_MSC_VER`, or on `-DMSVC` in the build — verified red on four probe shapes and on a re-added definition, green on the tree, and it does **not** fire on the comments left behind. `RAWFILE` stays `".r"`. The vendored `apps/tst_pb.cpp` keeps the vendor's three `PLAT_WINDOWS` branches (counter **14b**, new), and the now-unreachable MSVC scaffolding in `CMakeLists.txt` is filed as **`P3-13`** |
| **35** | `P3-5` the `system()` call in `ObjectTrack` — **gone**, so a config file is a program name again | ✅ done | `publishCount()` concatenated `options/script` — read straight out of XML — with two numbers and a trailing `&`, and handed the result to `/bin/sh`. A config file containing `count.sh; rm -rf …` therefore ran the second command with this process's privileges, and because the `&` was *inside* the string, `system()` waited for the shell and never for the script. Replaced with `fork` + `execvp` on an argv array: the configured value is a program name, its arguments are arguments, and no shell is anywhere in the path. The double fork is what replaces the `&` — the script is reparented to init, so a zone change does not leave a script nobody waits for, and the *intermediate* child is waited for, so the function leaks no zombie of its own and still does not block the frame loop. Only async-signal-safe functions run between `fork` and `exec`, so the child reports failure with `write()`/`_exit()` and not with `BOOST_LOG`, whose locks another thread may hold at `fork()` time. An unrunnable script is reported and survived, because a filter returning `false` aborts the whole bank. **4 tests** (`tests/test_objecttrack.cpp`, ctest target `objecttrack`); the fence `ShellMetacharactersInScriptAreNotInterpreted` was observed failing on the parent commit for all six payloads (`;`, `&&`, `\|`, `$(…)`, backticks, newline) — the marker file appeared *and* `count.sh` had been executed with no arguments. Warnings 26 → **25**: `-Wunused-result` on `system()` was the compiler describing this defect on every build. Re-measured in all four configurations from scratch, **12/12** `ctest` each, 25/24/23/22 warnings — exactly one less in every cell, which is the expected shape of removing one call. No installed header and no exported symbol is touched (`runDetached` is in an anonymous namespace): `readelf --dyn-syms | grep -v UND | wc -l` is **2 889** on both sides of a `0262fcd` worktree build and the exported set `diff`s empty, and `api_change_report.sh` exits 0. **The import set does move, and that is worth recording rather than glossing:** `system` is gone and `fork`, `execvp`, `waitpid`, `access`, `write` and `_exit` arrive — six libc symbols in, one out. Not an ABI break (nothing downstream binds to an import of ours), but it is the first time `libtoffy.so` itself depends on `fork`/`execvp`, i.e. the library is now POSIX-only at the *symbol* level too, which is the policy `P2-2` stated. Filed `A27`: the published object count is the *previous* frame's, because `filter()` publishes before it updates `numObjects` — pinned as it behaves, not changed |
| **36** | `P2-5` one owner per `Filter` — **the keystone**, and the reason `v2.0.0` exists | ✅ done | `FilterBank::_pipe` is a `std::vector<FilterPtr>`, `FilterFactory::createFilter()` returns the filter's **only** strong reference, and the factory's registry holds `std::weak_ptr` — it *indexes*, it does not own. `add()`/`insert()` take a `FilterPtr` (the raw-pointer overload is gone, and `add(new X())` does not compile: `shared_ptr`'s raw-pointer constructor is `explicit`), `instantiateFilter()` returns a `FilterPtr`, `FilterThread` shares its filter instead of deleting it, `Controller::baseFilterBank` is a `FilterPtr`, and `~Controller` releases it by letting it go. **The name-keyed teardown is gone**: `clearBank()`, `remove()` and `~Controller` no longer ask the factory to `delete` a filter by name (counter 13a 4 → **0**), so a renamed filter is no longer leaked and a filter reachable from two banks is no longer destroyed by the first one to go. `deleteFilter()` survives as "forget this name" and destroys nothing. `insert()` also gained the bounds check it never had (counter 13's neighbour: `_pipe.insert(_pipe.begin() + pos, f)` unchecked). `CreateFilterFn` is **deliberately still `Filter* (*)(void)`** — that is the plug-in ABI (`init(FilterFactory*)` + `registerCreator`), and `X6` decided to keep it; `createFilter()` wraps what a creator returns, and the header now says the creator must hand ownership over. **9 tests** (`tests/test_ownership.cpp`, ctest target `ownership`) count Filter destructors, because "was this destroyed, and exactly how many times" is otherwise invisible: **8 of the 9 fail on the parent commit**, five of them with a sanitizer report: heap-use-after-free reading through the second bank, heap-use-after-free reading the filter `remove()` had just deleted, `SEGV` inside `FilterBank::clearBank()` once a null entered `_pipe`, a heap-buffer-overflow in the vector backing `_pipe` from the unchecked `insert()`, and a **direct leak of 184 bytes** out of the full `Player` session — real XML file, real config path, frames run, `Player` destroyed — and pre-fix it reports exactly one filter alive afterwards: the one renamed by its own `<name>`, which is `C1` reached through a config file. `BankReleasesWhatItHolds` passes on both sides and stays as the pin that a release *does* happen, just not through the bank. Measured: **13/13 `ctest` in all four dependency configurations** from scratch, warnings **25/24/23/22** — identical to PR 35, so nothing was added; ASan + UBSan + LSan (`detect_leaks=1`) **13/13 with zero reports**; installed headers **172 checks, 0 broken**; `docs` **0 warnings** after `introduction.dox` stopped telling readers the factory deletes filters. Counters: 13 → **0**, 13a → **0**, 24 12 → **13**. **This is an ABI break, and the interesting part is that `readelf` cannot see all of it** — see *Release* below. |

**Twenty-six PRs are done** (1, 2, 3, 4, 5, 6, 15, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27,
28, 29, 30, 31, 32, 33, 34, 35, 36 — the numbering has no gaps, but PR 7 is core-only and the
PRs beyond 18 are the second-pass work, so "n of 18" stopped being a meaningful fraction once
`P3` started landing). PR 2 is
complete rather than 4-of-5. That is the whole `P0` block, the test/CI fence, the build-flag
cleanup and core's warning gate: the critical path's first stretch, plus every cheap
second-pass item — a false green removed, a class of silent build breakage closed, a warning
gate landed, `P2-12` made compile-checked, both config-as-format-string sites closed, the
public header surface proved compilable rather than assumed, one long-standing "decide,
then act" item actually decided, and the ownership model the rest of the structural work was
blocked on.

`P1`/`P2` items closed: **6 of 16** (`P1-1`, `P2-2`, `P2-5`, `P2-8`, `P2-14`, `P2-16`), with
`P2-3` core-only. `P2-16` closed when `P3-4` landed the `-Werror` half of it, `P2-2` when the
Windows branches were deleted rather than enabled.

## Open

| PR | Item | State |
|---|---|---|
| **9** | `P2-4` factory → registration | ❌ 37 branches |
| ~~**10**~~ | ~~`P2-5` one owner per `Filter`~~ | ✅ **landed as PR 36** — the keystone, so 11, 12 and 13 are no longer blocked on it |
| **11** | `P2-6` + `P2-13` run methods, state, typos | ❌ unblocked by `P2-5` |
| **12** | `P2-7` threading | ❌ `bool keepRunning`, 2 `interprocess` uses. `FilterThread`'s ownership half is **done** (`P2-5` made it share a `FilterPtr`); what is left is the atomic flag, the semaphore and the copy ctor, and `delete lanes[i]` — the last `delete` in core |
| **13** | `P2-9` module layering | ❌ `imageview.hpp` still included by `controller.cpp`. `getFilter(name)` can now be reimplemented against `_pipe`, which is what `P2-5` was waiting on |
| **14** | `P2-10` three plugin loaders | ❌ all three still exist |
| **16** | `P2-11` + `P2-12` `Frame` API, `filter()` overload | ❌ see `P3-9` before touching `P2-12` |
| **18** | `P2-15` formatting | ❌ 11 of 20 core files have hard tabs; clang-format 18.1.3 **is** installed |

The second-pass work (`P3-1` … `P3-12`) is listed in
[`plan/second-pass.md`](plan/second-pass.md). All four of its cheap items are done (`P3-1`,
`P3-3`, `P3-4`, `P3-7`) and so are `P3-11`, `P3-9` — the latter is the fence `P2-12` cannot
be merged without — `P3-6` in full, `P3-2`/`P3-12` (PR 28) and `P3-5` (PR 35). What is left
there is the mechanical `P3-10` and the `P3-8` note. `P3-10` and `P2-12` are the
remaining API-surface changes and belong with `P3-2`'s tag.

The controller extraction (`X1` … `X6`,
[`plan/controller-extraction.md`](plan/controller-extraction.md)) is the other half of PR 28:
the web control UI is `toffy-oatpp`'s now, and `X1`–`X4` are done (`X3` the three dead CLI
options, PR 30; `X4` the `use.dox` rewrite and its screenshots, PR 31). `X5` (the C++ API
oatpp binds to, blocked on five open items) is the only open stage; `X6` is a taken decision —
keep the filter plugin ABI, do not rehost `initUI` here. `P2-5` honoured that decision:
`CreateFilterFn` and `init(FilterFactory*)` are unchanged, so a filter plug-in built against
an older `libtoffy` still registers. What it does *not* honour is anything a plug-in did with
the bank's pipeline — see counter 31.

## Gates

- **`DOD 1.1` green: all four dependency configurations build and pass.** PCL on/off ×
  BTA on/off, each configured from scratch, built and run — **13/13** in every cell, 25/24/23/22
  warnings (re-measured on PR 36: 25 default, 24 PCL-off, 23 BTA-off, 22 both off —
  *identical* to PR 35 cell for cell, which is the expected shape of an ownership change that
  adds no code the compiler can complain about; `P3-5` had already been measured to take
  exactly one warning out of every cell, and `P2-2` to change no build output at all).
  Both axes were broken before this branch: PCL-off on unguarded typedefs and on CMake `if()`
  clauses that expanded to nothing, BTA-off on an unconditional `add_subdirectory(bta)`.
- **`DOD 2.3`, the ownership gate, is now automated rather than hand-run.** It asked for "a
  full session, including shutdown", zero leaks and zero double-frees, no `delete` of a
  `Filter` in core, no name-based teardown, and `FilterPtr` used consistently. The first two
  are `FilterOwnership.PlayerSessionReleasesEverythingOnShutdown` — a real XML file through
  `Player::loadConfig`, frames run, `Player` destroyed, destructor counter back to zero — and
  it runs inside the `sanitizers` job on every push, which is strictly better than the
  one-off `toffyRunner` run it replaces (that was also done: a 6-second session against
  `tests/xml/cond_main.xml` under ASan/UBSan, zero reports; `toffyRunner` cannot be made to
  exit cleanly headless because of `A26`, which is why the *test* is the gate and the
  hand-run is not). The other three are counters 13, 13a and the `_pipe`/registry types.
  Measured on this tree: ASan + UBSan + LSan `detect_leaks=1`, **13/13, zero reports**.
- **CI builds, tests and reports warnings on every push and PR** — five jobs: the PCL
  matrix, the sanitizer build, `installed-headers` (`P3-12`, two passes since `X2b`, plus the
  source-tree residue step), `docs` (counter 30) and `cmake-hygiene`, which since `P3-5` has
  three steps: the `if( ${VAR} )` sweep (`P3-7`), the no-`MSVC`-branches gate (`P2-2`) and the
  no-`system()` gate (counter 20, comment-stripped for the same reason the residue gate skips
  prose; verified red on a re-added `system("true");` and green on the tree). The build
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
- **PR 36 (`P2-5`) is an ABI break and `v2.0.0` is the tag that carries it** (see *Release*
  below). `tools/api_change_report.sh v1.11.0` exits **1** and lists the changed declarations
  in six installed headers; that is the gate working, not a failure to fix.
- **PRs 28 and 29 removed installed public headers, and `v1.11.0` carries them** (tagged at
  the PR 29 tip; see *Release* below). `api_change_report.sh v1.10.0` exits 1, as it must — it
  stays exit 1 against `v1.10.0` after the tag exists, because the base ref is what you diff
  against; run it as `api_change_report.sh v1.11.0` from the tagged commit onwards.
  For those two PRs the ABI evidence was measured rather than asserted: the exported dynamic
  symbol set of `libtoffy.so` was identical before and after (2 889 symbols, `diff` empty,
  Release/PCL-on/BTA-on both sides, base built from `8689d68` in a separate tree). Three of
  the four deleted headers could not be included at all, so no compiling program can break;
  `toffy/web/actions/action.hpp` could, and its removal was the one real — if improbable — API
  break of that pair. **That measurement no longer holds across `P2-5`** — see counter 31 and
  the `v2.0.0` bullet below. See [`dod/stage-gates.md`](dod/stage-gates.md) §2.4.

## Release / tag situation

**Published state, re-checked against `origin` on 2026-10-27** with `git ls-remote` and a fresh
clone into a scratch directory. Everything below this table used to say the tags were local;
that is no longer true and the section was rewritten rather than annotated.

| tag | points at | on `origin` | note |
|---|---|---|---|
| `v1.5.1` … `v1.6.0` | lightweight | ☑ | the old line |
| `v1.7.0`, `v1.7.1` | annotated | ☑ | |
| `v1.10.0` | `7fea594` | ☑ | pushed; not a branch tip, and it stays there — see below |
| `v1.11.0` | `5fbb65a` | ☑ | pushed |
| `v2.0.0` | `47e4cd4` | ☑ | pushed 2026-10-27, the last of the three to go |
| `v1.9.0` | `bc921ab` | ☐ | **deliberately not pushed** — see below |

Heads on `origin`, re-checked 2026-10-27 **after `main` was moved onto the 2.0 line**:
`main` and `feature/code-cleanup` carry the same code — the move was a **fast-forward**, because
`main` was a strict ancestor of the cleanup line (59 commits behind, `git log
feature/code-cleanup..main` empty), so there is no merge commit and nothing was dropped. The two
heads are kept identical; read that as "the same code", not as a SHA, because a SHA in this
sentence went stale twice in one day and that is the whole reason for the rule below.
`stable-1.x` is `b6d7665` — the C++14 1.x line, parked exactly where `main` was, which is now
that line's home rather than a branch waiting to be reconciled. `origin/HEAD` is `main`, and a
fresh clone of it describes as `v2.0.0-5-g2a8bf19` and configures as
`version: v2.0.0-5-g2a8bf19` (`git describe` at the moment of the check; it moves with the tip).

`feature/code-cleanup` is where the 2.0 work happened; **`main` is where it lives now**, and new
work goes there. The branch stays for the moment because the PR history in these documents
references it.
**`next` is gone**: `git fetch --prune` reports `[deleted] (none) -> origin/next`.

Gate, measured on the fresh clone: `git ls-remote --tags origin | grep -E
'v1\.10\.0|v1\.11\.0|v2\.0\.0'` returns three; checked out at `v2.0.0`, `git describe --tags`
is `v2.0.0` and a fresh configure reports `version: v2.0.0-0-g47e4cd4`. The
`-128-NOTFOUND`-style version failure that consumers hit is gone for toffy.

**What the gate found, and what closed it.** The first pass of this re-check showed
`origin/HEAD` pointing at a `main` that described as `v1.6.0-1-gb6d7665`: tags published, the
default checkout still 1.6, and `stable-1.x` parked on the same commit. Carried as **P0-3** in
the consumer-side upgrade notes and **closed the same day** — `main` is the 2.0 line, the 1.x
line has `stable-1.x`, and a clone that names no branch now gets 2.0.

What that does *not* change is the rule the tag push exists to enforce: **pin a tag, never a
branch.** `main` moves — it just did, twice, one of them under a sentence describing it — and
`v2.0.0` does not. The difference after P0-3 is only that a consumer who ignores the rule now
gets a 2.0 tree instead of a 1.6 one, which is a better accident but the same accident.

- `v1.10.0` is tagged (annotated) for the `Event` removal and the vtable change it caused.
  Before it this tree built `libtoffy.so.1.7.1` while exporting none of 1.7.1's symbols —
  the SONAME advertised a compatibility it did not have.
- **`v1.10.0` stays at `7fea594` — decision, 2026-10-27.** It was cut mid-branch, and this
  file used to say "anything shipped from here needs the tag moved or re-cut". That was true
  while the tag was local and became false the moment it was published: moving a published
  annotated tag means two clones of one tag name build two different `libtoffy`s, which is
  precisely the failure the "one clean rebuild of everything" rule exists to prevent. Nothing
  ships from `v1.10.0`; `v1.11.0` and `v2.0.0` carry the later tips and are what anyone should
  pin. Rule for the future: **a published tag is never moved — cut a new number instead.**
- It is `1.10.0` rather than the convention-implied `1.8.0` so that it does not sort below
  the `v1.9.0` that existed on `origin/next`.
- **`v1.9.0` stays unpublished — decision, 2026-10-27.** It is a **lightweight** tag on
  `bc921ab` ("Next (#23)", the old `next` tip). That commit diverged from `b6d7665` and is not
  an ancestor of `v1.10.0`, `v1.11.0` or `v2.0.0` (`git merge-base --is-ancestor v1.9.0
  feature/code-cleanup` → false; `git log feature/code-cleanup..v1.9.0` → two commits,
  `bc921ab` and the `Feature/cxx 17 (#22)` commit under it — both single-parent squash merges,
  so there is no merge base to recover from either). With `next` deleted it is the only
  ref to that line anywhere. It is not published because it was never annotated, never built as
  a release and never consumed, and because a published tag is a pin: `find_package(toffy 1.9)`
  or `git checkout v1.9.0` would name a tree that predates every fix in the cleanup line while
  sorting *below* the releases that supersede it — the "history sorts wrong forever" failure
  this item was supposed to close, published. Its one purpose, the move to C++17, is already in
  the 2.0 line on its own terms (`1f4921c`, "… move to C++17"; `main` is still
  `CMAKE_CXX_STANDARD 14`). Two consequences worth writing down: **keep the local tag** —
  `bc921ab` is reachable from no ref on `origin` (`refs/pull/23/head` is `6f2cda2`, not
  `bc921ab`), so deleting it destroys the only copy of that commit; and **`1.8` and `1.9` are
  burned** — never reuse either number, so that anything cached from the `next` era keeps
  meaning exactly one thing.
- `SOVERSION` is the **full** version, not the major alone, so *every* patch bump changes
  the SONAME and forces a downstream relink. Version decisions here are never routine —
  see [`dod/stage-gates.md`](dod/stage-gates.md) §2.4.
- **`v2.0.0` is cut at the `P2-5` commit (`47e4cd4`), and `P2-5` is why it is a major and not
  the next minor.** It is on `origin` as of 2026-10-27. A fresh configure from the tag reports
  `version: v2.0.0-0-g47e4cd4` and builds
  `libtoffy.so.2.0.0` (`readelf -d` → `SONAME libtoffy.so.2.0.0`), and a consumer asking for
  `find_package(toffy 1.11)` is now rejected — *"considered but not accepted:
  toffyConfig.cmake, version: 2.0.0"* — while `find_package(toffy 2.0)` resolves. Both
  measured, not inferred from the `SameMajorVersion` line. Doc corrections landed after the
  tag (the tag marks the code, and `git describe` from them still reports `2.0.0`, so the
  SONAME does not move); they are not a reason to re-cut it. The decision, in the order the
  evidence arrived:
  1. Three **strong** exported symbols change name (`counters.md` 31): `FilterBank::add
     (Filter*)` and `FilterFactory::renameFilter(Filter*, …)` are gone, replaced by their
     `FilterPtr` forms, and `FilterBank::insert(int, FilterPtr)` moved out-of-line. Those
     alone make an old binary fail at load time, which is the *loud* kind of break and the
     kind `v1.10.0` already set the precedent for.
  2. A fourth change is invisible to `readelf`, and that is the one that forces the major.
     `FilterFactory::createFilter()` returns `FilterPtr` **by value** now. Under the Itanium
     ABI a by-value class return prepends a hidden `sret` pointer to the argument list, and
     the mangled name does **not** encode the return type — verified on this toolchain, where
     `T f(int)` returning a class holding a `shared_ptr` and `int h(int)` both mangle to a
     name carrying only `(int)`. `readelf --dyn-syms` shows `createFilter` with a
     byte-identical name before and after this PR. A binary linked against `v1.11.0` therefore
     resolves the symbol, loads, and calls it with every argument one register off. No SONAME
     check catches that; only a version that downstream can read as "not compatible" does.
  3. `SOVERSION` is the full version (`CMakeLists.txt`, `set_property(... SOVERSION
     ${PROJECT_VERSION})`), so the SONAME moves on every release regardless. The major number
     is not what forces the relink here — it is what tells a downstream maintainer *why*.
  A minor bump (`1.12.0`) would keep the reading that `1.x` releases are compatible with each
  other, which after `P2-5` is false in a way a linker will not detect. `v1.7.0` — "breaking
  API change in FilterBank", the last time this class of thing moved — is the precedent for
  treating a `FilterBank` signature change as a release boundary; this one is larger.
  **What is deliberately *not* broken:** the filter plug-in ABI. `CreateFilterFn` is still
  `Filter* (*)(void)`, `init(FilterFactory*)` is unchanged, and `registerCreator()` keeps its
  symbol, so a `.so` built against `v1.11.0` still loads and registers (the `X6` decision).
  A plug-in that touched a bank's pipeline — `add()`, `instantiateFilter()` — must be rebuilt.
- **`v1.11.0` is the agreed number and it is cut at the PR 29 tip.** `P3-2` changed the
  installed header surface and `P3-12`/`X2b` changed three more headers, so the tag covers
  PRs 28 and 29 together. `P3-10` was queued for "the one after it" — which is now `v2.0.0`,
  but `P3-10` had not landed when that was cut, so it waits for `v2.1.0` rather than
  retro-attaching itself to a tag that does not contain it. `1.11.0` rather than `1.8.0`
  keeps clearing the `v1.9.0` that existed on the since-deleted `origin/next` (see the `v1.9.0`
  decision above), and a removal plus a header-hygiene release is a minor bump, not a patch,
  under the reading `v1.10.0` already set for the `Event`
  deletion. From the tagged commit `git describe` reports `1.11.0`, so the SONAME stops
  claiming `1.10.0` while the header surface has already moved. **On `origin`**, like
  `v1.10.0` and `v2.0.0`.
- **`v2.0.0` carries everything after `v1.11.0`:** PRs 30 (`X3`), 31 (`X4`), 32/33 (the
  `Doxyfile.cfg` fix and the `docs` gate), 34 (`P2-2`, POSIX-only), 35 (`P3-5`) and 36
  (`P2-5`). Of those, only 36 forces the major; the rest were riding "the next tag" and are
  now riding this one. `P3-10` (dropping `TOFFY_EXPORT`) is still open, so it needs the next
  tag: it changes no exported symbol, which by the rule in `dod/stage-gates.md` §2.4 makes it
  a minor (`v2.1.0`) — a change to a *return type* would not be, and that is now written down
  where the next person will read it before making one.
- **PR 34 (`P2-2`) opens no tag decision of its own.** It touches no installed header and no
  exported symbol — measured against an `ac1108a` worktree: `readelf --dyn-syms` identical,
  2 889 symbols, `diff` empty — so it rides `v2.0.0`. What it does change is a **configure-time** behaviour: a
  Windows configure now stops with a message naming the plug-in loader as the reason, instead
  of reaching the compiler and dying on a missing `<dlfcn.h>`. That is a support-policy
  statement about a configuration that has never built, not an API change.
