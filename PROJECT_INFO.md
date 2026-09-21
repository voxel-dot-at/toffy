# Toffy — Project Notes (evidence)

Scope of this document: an evidence-based summary of the codebase with a focus on
`modules/core`. All findings cite `file:line` so they can be verified independently.
The checked-in Doxygen docs were **not** used as a source of truth (see
[Documentation](#g-documentation)) — everything below comes from the headers and sources.

This file holds the **findings only**. The numbered work is in
[`CLEANUP_PLAN.md`](CLEANUP_PLAN.md), the PR order in
[`CLEANUP_SEQUENCE.md`](CLEANUP_SEQUENCE.md) and the gates in
[`DOD.md`](DOD.md).

## Status

**P0 is complete: 12 of 12 items closed.** Also done: `P1-1` (ctest harness, now 8 tests,
plus CI), `P2-8` (logging policy out of `Filter`), `P2-14` (the `Event` stub was deleted, not
finished), and 4 of the 5 sub-items of `P2-16`. `P2-3` is done for `modules/core` but its
scope turned out to be ~3× larger than planned once bare `cout` was counted — see row 5 below.
That is **3 of the 16** `P1`/`P2` items; the structural work is still ahead.

Findings below are a record, not a to-do list, so several describe code that no longer
exists. They are marked rather than deleted so that the `A`-number citations used by the
plan keep resolving. Where a fix was partial this is stated — notably `A12` (only the
`joinable()` guard landed; the `std::terminate` risk in the four run methods is open),
`A21` (null check landed; the non-overwriting `_filters.insert` remains) and
`C3` (null guards landed; `clearCreators()` teardown is untouched).

### Re-verified against the tree

Every count in this document was re-measured rather than carried over, and the whole
`DOD 1.1` matrix was re-run from scratch (most recently on the `P2-3`/`P2-16` work: **4/4
configurations build, 8/8 `ctest` in each** — 7/7 before `P2-8` added `test_logging`). At the
 time of the first re-verification the
mechanical counters (42 `std::cout`, 1 `#warning`, 22 `DLLExport`, 3 `RAWFILE` headers,
1 viewers include, 2 `interprocess`, 37 factory branches, 21 `@todo`, 19 `MSVC` branches) and
the 3 `-Woverloaded-virtual=` warnings in core all reproduced exactly as stated.

They have since moved, because `P2-16` and the core half of `P2-3` landed: `#warning` and
`DLLExport` are now **0**, `RAWFILE` is in **1** header, `WIN`/`UNIX` are gone, the
`BOOST_VERSION` branch is gone, and `modules/core` contains **no debug prints at all**. The
`std::cout` figure of 42 turned out to be the wrong metric entirely — see row 5 of the table
below, and the note on bare `cout` in section E. The untouched counters (viewers include,
`interprocess`, factory branches, `@todo`, `MSVC` branches) still read as first measured.

Corrections made during that pass, all of them this document describing a state it had
already moved past:

| Was | Now |
|---|---|
| `CMAKE_CXX_STANDARD 14` | **17** — bumped under `P2-16` |
| `tests/` = "one file, `test_cond.cpp` (44 lines)" | 7 gtest sources, 586 lines, 7 `add_test()` targets |
| core = 4,084 lines | **3,993**; `src/frame.cpp` was missing from the inventory entirely |
| "a BTA-less build is broken" | **fixed** — `add_subdirectory(bta)` is gated by `if (HAS_BTA)`; 4/4 green |
| "docs misdescribe `findPos`/`remove`" | doc comments were corrected with the fixes |
| 23 `@todo` in core, `filterbank.hpp` 6 | **21**, `filterbank.hpp` **5** |
| findings A1–A7, A13, A19, A21 written as live | marked `FIXED (P0-n)` |

The last row matters most for future readers: the marking policy in the paragraph above was
not being applied consistently, so ten already-fixed defects read as open. That is how a
record like this gets disbelieved.

## Where the gates stand

- **`DOD 1.1` is green: all four dependency configurations build and pass.** PCL on/off
  × BTA on/off were each configured from scratch, built and run — 8/8 in every cell.
  Both axes were broken before this branch: PCL-off on unguarded typedefs and CMake `if()`
  clauses that expanded to nothing, BTA-off on an unconditional `add_subdirectory(bta)`.
- **CI builds and tests on every push and PR** (`.github/workflows/ci.yml`): a PCL-on/
  PCL-off matrix plus an ASan/UBSan/LSan job. Two limits when citing a green run — runners
  cannot have the proprietary bta SDK, so only the BTA-off axis is covered automatically,
  and **docs are not built by anything**, so broken `\ref`s and `\todo` drift stay invisible.
- **`v1.10.0` is tagged** (annotated) for the `Event` removal and vtable change. Before it,
  this tree built `libtoffy.so.1.7.1` with none of 1.7.1's symbols — the SONAME advertised
  compatibility it no longer had. The tag is **local; it has not been pushed.** It is
  `1.10.0` rather than the convention-implied `1.8.0` so it does not sort below the
  `v1.9.0` that exists on `origin/next`; those two numbering lines still need reconciling
  when the branches meet.
- **`A23` (no consistent error convention) blocks further correctness work** in config
  loading; see the empirical evidence recorded there.

## What is still open, measured

Re-measured on the current tree, not carried over from the original audit:

| | metric | now | target |
|---|---|---|---|
| | `P1`/`P2` plan items closed | 3 / 16 (`P1-1`, `P2-8`, `P2-14`; `P2-16` 4-of-5, `P2-3` core-only) | 16 / 16 |
| 5 | debug prints in `modules/` — `std::cout` **and** bare `cout` | **129** (29 + 100); **0 in core** | 0 |
| 5b | debug prints in `modules/core` | **0** (was 9 `std::cout` + bare) | 0 ✅ |
| 6 | `#warning` directives | **0** (was 1) | 0 ✅ |
| 7 | `DLLExport` occurrences | **0** live (was 22) | 0 ✅ |
| 8 | headers defining `RAWFILE` | **1** (was 3) | 1 or 0 ✅ |
| 8b | `WIN`/`UNIX` macros in public headers | **0** (were 2, in `imagesensor.hpp`/`BtaWrapper.hpp`) | 0 ✅ |
| 9 | `#include <toffy/viewers/...>` from core | 1 | 0 |
| 10 | `interprocess` primitives in core | 2 | 0 |
| 11 | hard-coded `else if (type ==` branches | 37 | 0 |
| 12 | `@todo` in `modules/core` | 21 (was 23; the `Event` deletion removed 2) | ≤ 5 |
| 12b | `@todo` in `filterThread.hpp` | 10 | 0 |
| 14 | `#ifdef MSVC` branches, never compiled | 19 | 0 |
| 15 | compiler warnings in `modules/core` | 3 with PCL on, 2 without | 0 |

**Row 5 is a correction, not a progress update.** The metric used to be `grep -rn 'std::cout'
modules/` → 42. That grep cannot see bare `cout <<`, and most of `modules/` does
`using namespace std;`, so the real figure was never 42 — counting both spellings and
excluding comments gives **129**. The old number understated the remaining work by roughly
3×, and every one of the 129 is now in `modules/filters`/`modules/bta` (core is at 0).

**Rows 6–8b are measured with comment lines stripped.** The cleanup left explanatory comments
naming the macros it removed, so a naive `grep -rn 'DLLExport' modules/` returns 8 comment
lines and looks like no work was done. Strip with
`grep -vE ':[0-9]+:[[:space:]]*(//|\*|/\*)'` — the DOD verification block now does this.

The remaining mechanical items belong to `P2-2` (MSVC branches), `P2-3` (the 129 prints),
`P2-4` (the factory chain) and `P2-15` (formatting).

---

## 1. What this project is

Toffy is a **data-flow oriented C++ library** wrapping OpenCV (and optionally PCL),
originally built for Time-of-Flight (ToF) camera pipelines, notably Becom Electronics
cameras.

The mental model is a **filter pipeline over a shared blackboard**:

- **`Frame`** — a string-keyed, type-erased blackboard (`boost::any` map). One frame is
  shared by the whole pipeline; filters read and write named slots.
- **`Filter`** — the processing unit. Contract is
  `bool filter(const Frame& in, Frame& out)`; configuration comes from an XML
  `boost::property_tree`.
- **`FilterBank`** — a *composite* Filter: an ordered pipeline of filters that itself
  satisfies the Filter interface, so banks nest.
- **`FilterFactory`** — a global singleton registry mapping type-name strings to
  constructors.
- **`Controller` / `Player`** — application drivers: own the base bank + frame, run the
  loop, load plugins.

---

## 2. Repository layout

| Path | Contents |
|---|---|
| `modules/core/` | Framework: Frame, Filter, FilterBank, Factory, Controller, Player, events, threading |
| `modules/filters/` | Actual processing filters (capture, filters, detection, tracking, smoothing, viewers) |
| `modules/bta/` | Becom BTA camera driver wrapper (optional, `HAS_BTA`) |
| `modules/commons/` | Small shared helpers (e.g. `plugins.hpp`) |
| `apps/` | Executables (`toffyRunner`, `tst_pb`) |
| `tests/` | 7 gtest sources (586 lines) + `dummy_filter.hpp` + `xml/` fixtures, wired to 7 `add_test()` targets |
| `docs/` | Doxygen sources (`.dox`), largely stale |

Note the naming trap: `modules/filters/` is a *sibling category*, while
`modules/filters/src/filters/` is one sub-category inside it. `Filter`/`FilterBank`
live in **core**, not in `modules/filters`.

## 3. Build system facts

- CMake, `CMAKE_CXX_STANDARD 17` (`CMakeLists.txt:10`) — bumped from 14 under `P2-16`, which
  is what let `findPos()` return `std::optional<int>`.
- Warnings: `-Wall -Wextra -Wno-long-long` from `add_compile_options` (`:207`, the Unix
  branch of the compiler check). The unconditional `add_definitions(-Wall)` that also sat at
  `:317` is gone — it was redundant, and being *outside* the compiler check it also reached
  MSVC, which does not accept `-Wall`. **No `-Werror`** — deliberately; see `P2-16`.
- **Optimisation is left to `CMAKE_BUILD_TYPE`.** The global `add_definitions(-O2 -fPIC)` at
  `:294` is gone. The measured behaviour is worth recording, because it was **not** what the
  original analysis assumed: `add_definitions` content lands *after*
  `CMAKE_CXX_FLAGS_<CONFIG>` on the compiler command line, so that `-O2` did not just force
  `-O2` into Debug — it also **overrode Release's `-O3` down to `-O2`**. Debug had no `-O0`
  at all (CMake's `CMAKE_CXX_FLAGS_DEBUG` is only `-g`), so "Debug" really meant
  `-O2 -g -ggdb`. Now measured directly from `flags.make`: Debug → no `-O` flag (GCC default
  `-O0`), Release → `-O3`, RelWithDebInfo → `-O2`. `-fPIC` was dropped too, but PIC is still
  applied — `CMAKE_POSITION_INDEPENDENT_CODE ON` (`:293`) makes CMake emit `-fPIC` itself.
  That satisfies the plan's "keep `-fPIC`" note through the mechanism it was actually asking
  for rather than the literal flag.
- **Core's warning count is configuration-dependent**: 3 with PCL on (the `DOD 15` figure),
  2 with `-DWITHOUT_PCL=ON` — one of the three `-Woverloaded-virtual=` sites is behind a PCL
  guard. Quote the config when quoting the number.
- Optional deps gated by preprocessor macros added globally: `-DPCL_FOUND=1` (`:267`),
  `-DHAS_BTA=1` (`:307`), `-DOCV_VERSION_*` (`:280`).
- **Test harness and CI now exist.** `enable_testing()` plus 7 `add_test()` targets
   (gtest), and `.github/workflows/ci.yml` builds and runs them on every push and PR.
  Previously `.github/` held only Codacy, Flawfinder and Dependabot — static analysis
  bots and a dependency updater, nothing that compiled or tested anything.
- **`BUILD_DOC` does not exist.** It is referenced nowhere in the build; `docs/CMakeLists.txt`
  is `include()`'d unconditionally (`CMakeLists.txt:591`) and its `docs` target only ever
  runs on demand. Any earlier claim that it "defaults to OFF" was wrong — the flag is inert.
- **`CMAKE_BUILD_TYPE` defaults to Release** (`CMakeLists.txt:92-94`), and Debug defines
  `CM_DEBUG`, which changes `Filter`'s initial log level. Behaviour differs between the two,
  so both must be built — CI does.

---

## 4. `modules/core` inventory

**3,993 lines** total (headers + sources), re-counted on the current tree.

| File | LOC | Responsibility |
|---|---|---|
| `src/filterbank.cpp` | 473 | Pipeline execution, XML config instantiation, plugin loading |
| `include/toffy/frame.hpp` | 421 | `Frame` blackboard + all inline typed accessors |
| `src/controller.cpp` | 405 | Run loop, threading, plugin loading |
| `include/toffy/filter.hpp` | 398 | `Filter` base, `filterState`, `FilterListener`, `Port` |
| `include/toffy/filterbank.hpp` | 364 | Composite bank API |
| `src/filterfactory.cpp` | 335 | Singleton registry + type-name dispatch |
| `src/filter.cpp` | 194 | Base config/log/state plumbing |
| `include/toffy/controller.hpp` | 156 | Controller API |
| `include/toffy/filterfactory.hpp` | 154 | Factory API |
| `src/player.cpp` / `include/toffy/player.hpp` | 148 / 134 | App facade + logging setup |
| `src/parallelFilter.cpp` / `.hpp` | 147 / 92 | Parallel lanes |
| `src/filterThread.cpp` / `include/toffy/filterThread.hpp` | 125 / 103 | Worker thread + frame queues |
| `src/frame.cpp` | 105 | `Frame` out-of-line members |
| ~~`include/toffy/event.hpp`, `src/event.cpp`~~ | ~~156~~ | ~~Event bus (incomplete)~~ — deleted, `P2-14` |
| `include/toffy/filter_helpers.hpp` | 84 | `LOG` macro, ptree option getters |
| `include/toffy/mux.hpp` / `src/mux.cpp` | 67 / 38 | Fan-in of frames |
| `include/toffy/btaFrame.hpp` | 50 | BTA-flavoured Frame subclass + slot-name constants |

## 5. How a run actually happens

1. `Player` ctor installs Boost.Log sinks and a **global** severity filter. Sink install is
   guarded to once per process (`P2-8`) — `~Player` no longer removes sinks, so without the
   guard sequential `Player`s would each add a file sink and duplicate every record. Boost.Log
   as built here cannot enumerate the core's sinks, so the guard has to be a local flag.
2. `Player::loadConfig()` reads XML, loads `<plugins>`, then
   `FilterBank::loadConfig()` walks the `<toffy>` node; each child name is looked up in
   `FilterFactory::createFilter()`, `bank()` is set, and `loadConfig()` is called on it.
3. `Controller::forward()` spawns a thread running `loopFilters()`, which repeatedly
   calls `baseFilterBank->filter(f, f)` — **the same `Frame` is both input and output**.
4. `FilterBank::filter()` iterates `_pipe`, times each filter, catches exceptions, and
   `ready.post()`s a semaphore at the end.

---

## 6. Findings

### A. Correctness bugs (fix first — these are not style issues)

1. **~~`FilterBank::stop()` starts the filters.~~ FIXED (`P0-1`)** — the loop body called
   `_pipe[i]->start()`, a copy-paste of `start()` two functions above, so stopping a bank
   left every child in `filterRunning`. Now calls `stop()`; pinned by
   `FilterBankStop.StopStopsChildren`.

2. **~~`FilterBank::remove(size_t)` erases from the wrong end.~~ FIXED (`P0-2`)** — was
   `_pipe.erase(_pipe.end() + i)`, out-of-range iterator arithmetic for *every* `i`
   (including `0`) → undefined behaviour / heap corruption. The bounds check immediately
   above it was correct, which is what made it survive review. Now `_pipe.begin() + i`;
   pinned by `FilterBankRemove.ByIndexRemovesTheElementAtThatIndex`.

3. **~~`findPos()` cannot express "not found".~~ FIXED (`P0-4`)** — returned `size_t` with
   `return -1` on failure (→ `SIZE_MAX`) while the header documented "negative if not
   found". Now returns `std::optional<int>`; pinned by
   `FilterBankFindPos.MissingNameYieldsNoValue`. The header comment was corrected in the
   same change.

4. **~~`remove(std::string)` had no bounds check.~~ FIXED (`P0-3`)** — it narrowed
   `findPos()` into `int pos` (so `SIZE_MAX` → `-1`) and erased at `begin() + pos`
   unconditionally, always returning 1. Now checks the optional, logs, and returns 0;
   pinned by `FilterBankRemove.MissingNameLeavesBankIntact`. `remove(size_t)` likewise
   returns 0 out of range.

5. **~~`Frame` silently lost metadata.~~ FIXED (`P0-6`)** — the copy ctor copied only `data`
   and `meta`, `clearData()` cleared only `data`, and `removeData()` erased only from
   `data`, so `getDataType()`/`getDescription()` kept reporting slots that were gone while
   `hasKey()` correctly said they were. All three maps are now handled together
   (ctor, dtor, `removeData`); `operator=` is `= default`. Pinned by `FrameMetadata.*`.

6. **~~`Filter` constructors left members uninitialised.~~ FIXED (`P0-5`)** — see the
   original analysis below, which is why the fix used in-class initialisers.
   `src/filter.cpp:41` is `Filter::Filter() : _type("...") {}` — `_bank`, `_log_lvl`,
   `dbg`, `update` and `state` are all uninitialised. `bank()` then returns garbage,
   which `loadGlobals()` casts and dereferences (see A9), and `getState()` returns
   garbage before `init()`.

   Worse than first recorded: the *typed* constructor (`src/filter.cpp:43`) initialises
   `_bank`, `_log_lvl`, `dbg` and `update` but **also omits `state`**, so no construction
   path ever set it. Confirmed empirically when the regression test was written —
   `getState()` returned `861460480` and `-1214622000` on the two construction paths.
   Fixed with in-class member initialisers rather than constructor init lists, so a
   future constructor cannot forget again.

7. **~~`Filter::loadConfig()` threw right after diagnosing the problem.~~ FIXED
   (`P0-12`)** — it logged a helpful message when the type node was missing, then
   unconditionally executed `pt.get_child(_type)`, which throws `ptree_bad_path`. Now
   returns `-1` before the throw; pinned by
   `FilterLoadConfig.MissingTypeNodeReportsFailureInsteadOfThrowing` and
   `MatchingTypeNodeStillSucceeds`.

8. **~~`Event::data()` dereferences a null pointer.~~ RESOLVED BY REMOVAL** — the whole
   `Event` class was deleted (`P2-14`) rather than repaired, so this and the uninitialised
   `_re_type`/`_sender` in the default ctor no longer exist. Kept numbered so the `A8`
   citations in `CLEANUP_PLAN.md` keep resolving.

9. **`loadGlobals()` casts a possibly-null `_bank` to `FilterBank*`.**
   `src/filter.cpp:118-120` uses an unchecked `static_cast<FilterBank*>(_bank)` and
   calls `getBaseFilterbank()`. For a filter with no bank this is a call through a null
   pointer.

10. **`getBaseFilterbank()` relies on a fragile external invariant.**
    `FilterBank::FilterBank()` sets `bank(this)` (`filterbank.cpp`, ctor), so
    `getBaseFilterbank()`'s `if (bank() == NULL)` base case is never reached — it only
    terminates because `Controller`'s ctor manually calls `baseFilterBank->bank(NULL)`
    (`src/controller.cpp:57`). Any root bank not created that way recurses forever.

11. **`<filterGroup>` configuration is broken.** `FilterBank::handleConfigItem()`
    (`src/filterbank.cpp:98-111`) calls `ff->createFilter("filterGroup")`, but
    `FilterFactory::createFilter()` has **no** `"filterGroup"` branch
    (`src/filterfactory.cpp:148-265`) → returns `NULL` → every `<filterGroup>` node
    errors out.

12. **`Controller::stop()` joins a possibly non-joinable thread.** *Partially fixed:*
    the unconditional `_thread.join()` is now guarded by `joinable()`, because defining
    `Player::stop()` (A16) would otherwise have exposed the throw on a path no caller could
    previously reach. **Still open:** `src/controller.cpp:84` and its three siblings assign
    `_thread = boost::thread(...)` — assigning to an already-joinable `boost::thread` calls
    `std::terminate()`. That, and the four near-duplicate run methods that let the bug hide
    in four places, remain `P2-6`/`P2-7`.

13. **~~`Frame::operator=` dropped the type metadata.~~ FIXED (`P0-6`)** — it copied *only*
    `data` (`Frame& operator=(const Frame& x) { data = x.data; return *this; }`), so after
    `f1 = f2` every `getDataType()` returned `NotFound` while `hasKey()` returned `true`.
    Worse than the copy ctor and silent: `Frame` declared a dtor and a copy ctor but got
    assignment wrong — a textbook Rule-of-Three break. Now `= default`, which removes the
    hand-written bug *and* the chance of reintroducing it.

14. **`addData(long)` stores a value that no getter can read.**
    `include/toffy/frame.hpp:140-144` overloads for `long` / `unsigned long` tag the slot
    as `Int` / `Uint`, but the `boost::any` holds a `long`. `getInt()` does
    `any_cast<int>` (`:314`) → `boost::bad_any_cast` at runtime. On 64-bit Linux `long`
    is distinct from `int`, so this throws.

15. **Every typed getter throws on a missing key.** `getData()` returns an empty
    `boost::any` when the key is absent (`src/frame.cpp:40-56`), and `getBool()`,
    `getInt()`, `getMatPtr()`, etc. `any_cast` it unguarded → `bad_any_cast`. Only the
    `opt*()` variants are safe, and nothing in the header marks the plain getters as
    throwing. `BtaFrame::getDepth()` (`include/toffy/btaFrame.hpp`) inherits the hazard.

16. **~~`Player::stop()` is declared but never defined.~~ FIXED** — defined as a
    delegation to `Controller::stop()`, pinned by `PlayerStop.StopIsDefined`. Before the
    fix any caller got `undefined reference to toffy::Player::stop()` at link time, which
    is also why the fault survived: the declared API could never be called, so nothing ever
    discovered it did not exist.

17. **Typos are baked into the public API.** `getSertMatPtr` (`frame.hpp:282,392`),
    `Controller::stedBackward()` (`controller.hpp:97`, `controller.cpp:154`), and
    `Controller::CERROR` (`controller.hpp:60`). None of the three is called anywhere in
    the repo, so they can be renamed cheaply *now* — the cost only grows.

18. **`FilterFactory::~FilterFactory()` deletes the singleton it belongs to.**
    `src/filterfactory.cpp:113-118` executes `delete uniqueFactory;` from inside the
    destructor of the very object it points at, and never nulls it. Any delete leaves a
    dangling global. In practice the factory is never deleted at all, so it simply leaks.

19. **~~`creators[type]` inserted while looking up.~~ FIXED (`P0-10`)** — `operator[]` on
    the static creator map meant a typo'd filter type *mutated* a shared global registry by
    inserting a null entry instead of failing. Now `creators.find()`, with the null-entry
    case rejected too; pinned by `FilterFactoryCreate.FailedLookupDoesNotRegisterTheType`.

20. **`createFilter()` ignores its `name` argument.** `src/filterfactory.cpp:148` leaves
    the parameter deliberately unnamed (`std::string /* name */`), although
    `include/toffy/filterfactory.hpp:68-73` documents it as the filter identifier. Callers
    who pass a name silently get a generated one.

21. **A creator that returns null is dereferenced.** *Partially fixed (`P0-8`):* the null
    dereference is gone — `f = it->second()` is now followed by an explicit null check and
    an error return, where `f->name()` used to be called unconditionally. Pinned by
    `FilterFactoryCreate.CreatorReturningNullIsRejected`, which segfaulted before the fix.
    **Still open:** the following `_filters.insert(std::pair<std::string, Filter*>(f->id(), f))`
    is still an `insert`, not an assignment, so an id collision silently leaves the new
    filter unregistered and leaked. It does not bite today because `id()` embeds a
    per-construction counter, but it is a latent leak and belongs with `P2-5`.

22. **`FilterBank::insert()` has no bounds check.**
    `include/toffy/filterbank.hpp:100-103` computes `_pipe.insert(it + pos, f)` from an
    `int pos`; a negative or oversized value is undefined behaviour.

23. **No consistent error convention.** `filterbank.cpp:232,239` use `.at()` (throws),
    `remove(size_t)` returns `-1`, `remove(std::string)` always returns `1`, and
    `loadConfig` mixes `-1`, `0`, `1` and `throw std::runtime_error`. Callers cannot write
    correct error handling against this.

    Confirmed empirically while fixing `P0-12`: `FilterBank::instantiateFilter()` does not
    check `loadConfig()`'s return value, and the obvious-looking fix -- treat non-positive
    as failure -- **breaks a valid configuration**. `Cond::loadConfig()` returns `<= 0` for
    a cond with no dependent filters, which is a legitimate empty cond, not an error; with
    the check in place `ctest cond_empty` aborted with `std::runtime_error`
    ("filterBank::loadConfig() failure"). So the return code cannot be propagated until the
    convention is decided. This is why `P0-12` guards the throw but deliberately leaves the
    return value unchecked, and why `A23` must be resolved before config errors can be made
    loud.

---

### B. API & design

**Two parallel state machines.** `filterState` (`filter.hpp:52`) and `Controller::state`
(`controller.hpp:53-60`) both model run state, with no mapping between them.

**The run loop depends on enum ordering.** `while (_state > Controller::IDLE)`
(`controller.cpp:203,219`). Because `CERROR = 0xff` is the largest enumerator, entering the
error state keeps the loop spinning instead of exiting.

**`filter()` const/non-const overload trap.** `Filter` declares both a `const` and a
non-`const` virtual `filter()`, and the `const` one just `return false`. `FilterBank` and
`ParallelFilter` override only the non-`const` variant, so anything holding a
`const Filter&` silently gets `false`.

**`FilterBank::getFilter(name)` ignores its own pipeline.** `filterbank.cpp:227-237`
delegates to the global factory instead of `_pipe`, so a bank hands out filters it does not
contain. The header already flags this as deprecated.

**`Filter` does four jobs** in one 411-line header: processing interface, config plumbing,
per-filter log-level management, and observer/state machinery — while exposing mutable
public `dbg` and `update` fields.

**`Controller` exposes its internals.** `baseFilterBank` and `f` are public data members,
and `Player` both wraps `Controller` and hands it out again via `getController()`.

**~~Events are a stub.~~ REMOVED.** `event.hpp` carried `@todo Implement the event logic`
and `Filter::processEvent` logged at `info` for every unhandled event. The class, both
`processEvent()` virtuals and the header were deleted under `P2-14`; nothing outside core
used them.

**`btaFrame.hpp` lives in core.** BTA camera slot-name constants inside the framework
module invert the dependency — core should not know about one vendor's camera.

---

### C. Ownership & memory

**Three owners, one raw pointer.** `FilterFactory::_filters` (a global static map) and
`FilterBank::_pipe` both hold raw `Filter*`. `FilterBank::add()` takes a raw pointer, and
`~FilterBank` destroys filters *via the factory, by name*. A filter that was renamed, or
that was added to two banks, leaks or is double-freed.

**`clearBank()` deletes by name.** `src/filterbank.cpp:303-319` looks each filter up by
`name()` in the global factory in order to destroy it — name-keyed deletion of
pointer-owned state. Names are mutable, so this is not a stable identity.

**`~Controller` tears down process-global state.** `src/controller.cpp:64-66` calls
`deleteFilter(baseFilterBank->id())` with no null check, then `clearCreators()`, which
wipes the *process-wide* creator registry. A second `Controller` or `Player` in the same
process therefore starts with no built-in filters at all.

*Partially addressed (`P0-9`):* both the destructor's `->id()` and the constructor's
`->bank(NULL)` are now guarded — the constructor throws if the factory cannot supply the
base bank. **`clearCreators()` is unchanged**, because removing it is a behaviour change
that belongs with the ownership work in `P2-5`. It is currently benign for built-in types:
`createFilter()` resolves those through a hard-coded chain rather than the creator map, so a
second `Controller` still works (pinned by `ControllerLifecycle.SequentialControllersStillWork`).
Only *registered* creators — plugins, `Player::loadFilter()` — are lost, and nothing tests
that yet.

**`FilterThread` copies a raw owner.** `include/toffy/filterThread.hpp:42` documents that
the thread owns the `Filter` and deletes it; the copy ctor at `:53` copies `f` with no
transfer of ownership → two owners, two deletes. The Rule of Five is not applied, and the
declared copy ctor could not compile if used, because it would have to copy
`boost::thread` and `boost::mutex`. It should be `= delete`.

**`FilterThread` queues raw `Frame*`** (`:89,93`) with no documented ownership, so it is
impossible to tell whether the queue, the producer, or the consumer frees them.

**A header include is commented out.** `#include <list>` is disabled at
`filterThread.hpp:19` while `std::list` is used at `:89,93` — the file compiles only
through transitive includes.

**Plugin handles are leaked.** `dlopen()` is never paired with `dlclose()`; the calls are
explicitly commented out in both `filterbank.cpp` and `controller.cpp`. Handles are kept
as untyped `std::vector<void*> _loads` (`controller.hpp:149`), so they cannot be closed
correctly even in principle.

---

### D. Concurrency

**`keepRunning` is a plain `bool`.** `include/toffy/filterThread.hpp:85` declares the
worker-loop stop flag as a non-atomic `bool`, but it is written by the controller thread
and read by the worker every iteration.

The compiler is free to hoist the read out of the loop, so `stop()` may never be observed.
At best this is a missed wakeup; formally it is a data race, i.e. undefined behaviour.
It should be `std::atomic<bool>`.

**An inter-process semaphore is used for in-process sync.** `filterbank.hpp:23` includes
`boost/interprocess/sync/interprocess_semaphore.hpp` and `:357` declares the member.

This synchronises frames between two threads of *one* process using a System V kernel
object — heavier than needed, platform-specific, and `post()` past the semaphore's maximum
throws `bad_semaphore`. Since `FilterBank::filter()` posts once per pipeline run
(`filterbank.cpp:79`) and nothing is required to be waiting, the count drifts upward.
A `std::binary_semaphore` or `condition_variable` is the correct primitive.

**~~`setLoggingLvl()` mutates process-global logging from a per-filter method.~~ FIXED
(`P2-8`).** `src/filter.cpp` used to implement it by calling
`logging::core::get()->set_filter(...)`, so one filter's configured level silently overrode
the level for the entire application, including every other filter — and which one won
depended on pipeline order. `filter.hpp`'s `@todo` ("changing severity filter affects
everithing") admitted it.

`_log_lvl` is now data only: `setLoggingLvl()` derives `dbg` from it and touches nothing
shared. The one sanctioned way to move the process-wide level is
`static Filter::setGlobalLogLevel()`, called once by the application. Two follow-on defects
surfaced while fixing this:

- `updateConfig()` set `_log_lvl` but never refreshed `dbg`; it only looked correct because
  the per-frame hot path called `setLoggingLvl()` behind its back. Removing the hot path would
  have frozen `dbg` — a bug that depended on another bug.
- `<loglvl>99</loglvl>` was cast straight into the severity enum, yielding a value outside
  `trace..fatal`. Now clamped.

`~Player` also stopped calling `remove_all_sinks()`, which had silenced logging process-wide
the moment any `Player` was destroyed. See F for the hot-path half of this.

**Listeners are raw, never auto-unsubscribed pointers.** `filter.hpp` stores
`std::vector<FilterListener*>` and `setState()` iterates it while notifying.

A listener that destroys itself, or calls `removeListener()`, from inside a callback
invalidates the iterator being iterated. `addListener()` takes `FilterListener*` while
`removeListener()` takes `const FilterListener*` — an asymmetric API.

**`getInstance()` is unsynchronised.** `src/filterfactory.cpp:96-111` lazily creates the
singleton with no lock or `std::call_once`, so the first two concurrent calls can both
construct it.

**`FilterThread` has no defined shutdown contract.** `stop()`/`join()` interact with the
condition variables in `inCond`/`outCond`, but because the stop flag is not atomic (above)
there is no guaranteed wake-up, so joining can hang.

---

### E. Portability & build

**`#ifdef MSVC` is dead code inside the library.** This is the most consequential
build-system bug here.

`CMakeLists.txt:169` appends `-DMSVC` to the `DEFINITIONS` list, but the line that would
apply that list to the library is commented out (`CMakeLists.txt:373`). Only
`apps/CMakeLists.txt:6` consumes `DEFINITIONS`.

Consequence: inside `libtoffy` itself, `MSVC` is never defined, so every `#ifdef MSVC`
branch in `filterbank.cpp`, `controller.cpp` and `frame.hpp` compiles as POSIX. The
Windows plugin-loading paths (`LoadLibrary`/`GetProcAddress`) are unreachable, and the
POSIX paths (`dlfcn.h`) would be selected on Windows.

**~~`DLLExport` is vestigial.~~ FIXED (`P2-3`)** — all 22 occurrences deleted from **8**
headers, not the 3 this finding originally listed; the audit had only looked at core. It was
`#define`d in `frame.hpp`, `filterbank.hpp`, `filterfactory.hpp`, `capturerFilter.hpp`,
`detectedObject.hpp`, `average.hpp`, `imagesensor.hpp` and `BtaWrapper.hpp`, and actually used
on only 3 class declarations (`Average`, `ImageSensor`, `BtaWrapper`) — the rest were dead or
commented out (`class /*DLLExport*/ TOFFY_EXPORT FilterFactory`). Those 3 now use
`TOFFY_EXPORT`, the real macro from CMake's `generate_export_header`. No-op on Linux, since
`DLLExport` expanded to `/**/` off-MSVC, and the `dllexport` branch was unreachable from
inside the library anyway (see the `MSVC` finding above).

**`WIN` and `UNIX` were also going.** Not in the original audit: `imagesensor.hpp` and
`BtaWrapper.hpp` did `#define WIN true` / `#define UNIX true` in *public headers*, referenced
only by three commented-out lines in `BtaWrapper.cpp`. Two of the most generic macro names in
C, leaking into every consumer. Deleted.

**~~`RAWFILE` is defined in three separate public headers.~~ FIXED (`P2-3`)** — it is now
defined once, in `bta/BtaWrapper.hpp`, next to its only consumer (`bta.cpp:61`). It used to be
repeated in `filterbank.hpp` and `capture/capturerFilter.hpp` with their own `#ifdef`s, so a
translation unit including two of them risked a redefinition, and every consumer of the
library inherited a two-character macro. The `.rw`/`.r` value is unchanged on purpose: it keys
off `MSVC`, which the library build never defines, so it has always been `".r"` — but silently
changing an on-disk file extension is a behaviour change, not a cleanup.

**A PCL-less build now works.** `frame.hpp` used to guard the PCL *includes* but leave the
two typedefs naming those templates outside the guard, so `-DWITHOUT_PCL=ON` failed with
`'pcl' does not name a type`. Fixing that exposed that the configuration had never actually
reached the compiler, because of an `if( ${VAR} )`-vs-`if(VAR)` bug that aborted CMake first.

Full set of fixes on the PCL axis:

- `frame.hpp` — typedefs guarded; the `CloudXyz`/`CloudXyzRgb` enumerators deliberately
  left unconditional so `SlotDataType` cannot differ between two builds of the header.
- `viewers/CMakeLists.txt`, `3d/CMakeLists.txt` — `if( ${PCL_FOUND} )` style clauses
  expanded to nothing when PCL was off, so CMake parsed `if( AND OFF)` and died with
  "Unknown arguments specified". `if()` takes variable *names*.
- `3d/CMakeLists.txt` — `add_library(toffy_3d OBJECT "")` is rejected by CMake, so the
  target is now conditional and its two consumers (`CMakeLists.txt`,
  `modules/filters/CMakeLists.txt`) reference it through a `PCL_FOUND`-gated variable.
- `viewers/exportcloud.hpp` — holds a `pcl::PCDWriter` by value, so the whole class is
  guarded and `exportcloud.cpp` moved into a PCL-gated source list; `init.cpp` guards the
  include, factory function and registration to match.
- `viewers/init.cpp` — included `cloudviewpcl.hpp` unconditionally while guarding only its
  registration.
- `viewers/exportcsv.hpp` — `#include <pcl/io/pcd_io.h>` with **no** `pcl::` usage anywhere
  in the header or its source. Purely unnecessary, and it made an always-built
  PCL-independent filter fail to compile.
- `reproject/CMakeLists.txt` — `reprojectpcl.cpp` was always built; now gated. (`
  filterfactory.cpp` already guarded both the include and the `reprojectpcl` branch.)

Verified: clean `-DWITHOUT_PCL=ON` build, 74 TUs, 7/7 ctest. The default PCL-on build is
provably unaffected — configuring `HEAD` and this tree yields identical compilation-unit
lists (234 each), and the default build is clean with 7/7.

**`PCL_FOUND` is still not part of the exported interface.** It arrives via a global
`add_definitions` (`:267`), so consumers must reproduce the flag themselves or `Frame`
changes shape under them. Worth an exported compile definition.

**A BTA-less build used to be broken; it now works.** Found while testing the `DOD 1.1`
matrix: `modules/CMakeLists.txt` did `add_subdirectory(bta)` **unconditionally**, so the bta
module compiled even when `find_package(bta)` failed and linking then died with ~162
undefined `BTA*` references. It was confirmed on `HEAD` at the time, and was unrelated to the
PCL work.

**FIXED** — `add_subdirectory(bta)` is now gated by `if (HAS_BTA)` (`modules/CMakeLists.txt:8-10`)
and the single `$<TARGET_OBJECTS:toffy_bta>` consumer is gated the same way. Re-verified on the
current tree: all four `PCL_FOUND` × `HAS_BTA` cells configure, build and pass 8/8 `ctest`, so
the `DOD 1.1` matrix is 4/4 green and agrees with `DOD.md`. There is still no explicit option
to disable BTA — testing the off axis needs `-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON`.

**~~Dead version branches.~~ FIXED (`P2-16`).** `controller.cpp` kept
`#if (BOOST_VERSION > 105500)`, guarding a `char` log-severity variant from a 2017-era Boost
that no supported compiler accepts. The `#else` branch is gone.

**~~`#warning bta missing!`~~ FIXED (`P2-3`).** It fired on every build of the default
configuration, training everyone to ignore build noise. Deleted — CMake already emits
`message(WARNING "no bta library!")`, so the information was never lost, just doubled and
attached to the wrong audience.

**The debug-print metric was measuring the wrong thing — and still is, if you copy the old
command.** `grep -rn 'std::cout' modules/` returns 42, and that number has been quoted as the
size of `P2-3`. But most of `modules/` has `using namespace std;` at file scope, so the debug
prints are overwhelmingly **bare** `cout <<`, which that pattern cannot match. Counting both
spellings and stripping comments: **129 live prints**, of which only 29 are `std::cout`.

Two consequences. First, the item is ~3× bigger than planned. Second, and more subtle: the
same `using namespace std;` that hides them is *why* they are there — writing `cout` instead of
`std::cout` is a one-character saving that costs grep-ability, and it cost it here. Core's
copies are all gone and its file-scope `using namespace std;` directives went with them, so
new debug prints in core will at least say `std::cout` and be countable.

---

### F. Performance

These are not micro-optimisations; several are on the per-frame or per-config-read path.

**Config helpers copy the whole property tree on every call.**
`filter_helpers.hpp:38,54,66` declare their parameter as `boost::property_tree::ptree`
**by value**:

```cpp
template<typename T> bool pt_optional_get(const boost::property_tree::ptree pt, ...
```

Every optional config read deep-copies the tree. Changing all three to `const ptree&` is a
one-line, no-risk fix.

**`Frame::getData()` looks a key up twice.** `frame.cpp:40-56` calls `data.find(key)` and
then `data.at(key)`. A single `find()` and reuse of the iterator is enough.

**Every `opt*` accessor costs three lookups.** `hasKey()` does one `find`, then the getter
calls `getData()` which does `find` + `at`. See `frame.hpp:359-390`.

**`removeData()` searches three times.** `frame.cpp:72-79` does `find`, then `find` again
inside the `if`, then `erase(iterator)` which searches once more.

**`Frame::info()` does two extra lookups per entry.** `frame.cpp:84-95` iterates `data`
but then calls `getDataType(key)` and `getDescription(key)`, each a fresh map lookup,
instead of walking `meta`/`desc` alongside.

**~~`FilterBank::filter()` reconfigures logging three times per filter, per frame.~~ FIXED
(`P2-8`).** It called the bank's own level before the loop, each child's inside the loop, and
the bank's again after every child — three `set_filter()` calls on the shared logging core per
filter per frame, each constructing a new filter expression and taking the core's lock. All
three are gone, along with the one in `FilterBank::loadGlobals()` (see D).

**Wall-clock timing uses local time.** `filterbank.cpp:54,66` use
`microsec_clock::local_time()`, which jumps on DST and NTP adjustments. Use
`steady_clock` (or `universal_time()` if a calendar stamp is genuinely wanted).

**`createFilter()` is a ~120-line string chain.** `filterfactory.cpp:148-265` compares the
requested type against ~30 literals with `if/else if`, on every instantiation — while the
`creators` map that exists precisely to replace this sits unused for built-ins. See P2.

---

### G. Documentation

The checked-in docs are not a reliable description of the code, which is why this
document was written from the sources.

**The main pages are stubs.** `docs/mainpages/public/how_to_get.dox` consists of a heading
and `\todo do`. `modules.dox` ends with `\todo intro` for the entire Viewers section.

**Worse than a stub: `use.dox` documents a removed product.** It is a full page, but it
tells the reader to run `minimal_toffy` and open a web UI on `localhost:9999` with HTML at
`/opt/toffy/html`, and `\include`s `examples/config.xml`. No `minimal_toffy` target exists
anywhere in the repo (`grep -rn minimal_toffy` → no matches; `apps/` contains only
`toffyRunner` and `tst_pb`), and the web control was dropped in commit `54d9577`. A new
reader following this page cannot get to a running system.

**~~The docs misdescribe the buggy functions.~~ FIXED with the code.** `filterbank.hpp` once
documented `findPos()` as returning "negative if not found", which `size_t` cannot do (A3),
and documented both `remove()` overloads as "positive on success, negative or 0 in failed"
while they always `return 1` (A4). The doc comments were corrected alongside the fixes and now
match the code: `findPos()` says "or `std::nullopt` if no filter has that name", and
`remove(size_t)` says "1 if a filter was removed, 0 if `i` is out of range". This was the one
case where trusting the documentation actively hid a defect.

**Deprecated things are still used.** `filter.hpp` marks `_name` and
`Filter::loadFileConfig()` `@deprecated` ("we don't want a filter reading files, only the
filterbank"), yet `FilterBank::handleConfigItem()` still drives config loading through
`loadFileConfig()` for `<filterGroup>` recursion (`filterbank.cpp:110`).

**The most dangerous class is the least documented.** `filterThread.hpp` carries 10
`@todo document` markers — including on every queue, mutex and condition variable it uses
to synchronise across threads. Core contains **21** `@todo`s in total (down from 23; the
`Event` deletion removed 2), concentrated in `filterThread.hpp` (10), `filterbank.hpp` (5)
and `controller.hpp` (4); `filter.hpp` and `player.hpp` carry one each.

**Commented-out API sketches stand in for real docs.** `frame.hpp:262-280` is a block of
planned `insGet`/`setGet` accessors, with `optBool` etc. duplicated as comments right above
the real declarations.

**Many doc comments restate the signature.** e.g. `@brief ~Filter`, `@brief Mux`,
`@param in` / `@param out` with no content — noise that hides the comments carrying real
information.

**Top-level docs are duplicated.** `README.md` and `Welcome.txt` are byte-identical 5-line
stubs (verified with `cmp`); neither mentions the build, the module layout, or the XML
config format.

**The Doxygen config is a committed generated file.** `docs/Doxyfile.cfg` is 104 KB, of
which the vast majority is stock `#`-commented defaults; only a few dozen lines are
project settings. It should be regenerated or reduced to the settings that matter.

**Docs are generated but never gated.** `docs/CMakeLists.txt` adds a `docs` custom target
that only runs when Doxygen is found and only builds on demand. CI builds and tests the code
but does **not** build docs, so broken `\ref`s and `\todo` accumulation remain invisible —
that is the one gate the new workflow deliberately does not close yet. (`BUILD_DOC` is not
the mechanism: it is referenced nowhere in the build.)

**Config reference is XML-only.** `docs/configDocs/xmls/` holds one XML per filter, and
`bta.xml` exists twice (`docs/configDocs/bta.xml` and `docs/descDocs/bta.xml`) — duplicated
reference material that will drift.
