# Build flags, Windows paths, debug leftovers, formatting

## P2-2 — Fix or delete the Windows paths

`-DMSVC` is appended to `DEFINITIONS` (`CMakeLists.txt:169`, inside `if(MSVC)`) but the line
that would apply `DEFINITIONS` to the library is commented out
(`target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEFINITIONS})`, `CMakeLists.txt:404`);
only `apps/CMakeLists.txt` consumes it. So every `#ifdef MSVC` branch inside `modules/`
compiles as POSIX unconditionally — `FilterBank::loadPlugins()`, `Controller::loadPlugin()`
and `frame.hpp` are all dead code, and the `dlfcn.h` path is what actually ships.

**Count: 12 in `modules/`, 16 tree-wide** (4 `controller.cpp`, 3 `filterbank.cpp`,
2 `capturerFilter.cpp`, 1 each `cloudviewpcl.cpp`, `bta.cpp`, `BtaWrapper.hpp`, 1
`libraries/sensor/`, 3 `apps/main.cpp`). Two documents used to quote 19 for this; `P2-3`
deleted the seven that were wrapped round `DLLExport`/`RAWFILE`.

The asymmetry is the argument for doing this at all: on a Windows build `apps/` gets
`-DMSVC` and the library does not, so the two halves of one program disagree about
`Frame`'s members and about which plugin loader is compiled.

Pick one:

- apply `-DMSVC` to the library target so the branches become live and get tested, or
- delete the `MSVC` branches and state that Windows is unsupported.

Leaving them is the worst option: it looks like support and is neither compiled nor tested.
`TOFFY_EXPORT` is part of the same decision — see `P3-10` in
[`second-pass.md`](second-pass.md), which removes the macro rather than waiting for this.

## P2-3 — Remove debug leftovers and dead code

**The headline finding is that this item's metric was wrong, and wrong in the direction that
made the job look small.** The counter was `grep -rn 'std::cout' modules/` → 42. But most of
the codebase does `using namespace std;`, so the actual debug prints are mostly bare `cout <<`,
which that grep cannot see. Measured properly (bare `cout` included, comment lines excluded):

| metric | plan said | actually |
|---|---|---|
| `std::cout` in `modules/` | 42 | 29 remaining |
| bare `cout <<` in `modules/` | **not counted** | 100 remaining |
| live debug prints, `modules/` | ~42 | **128** |
| live debug prints, `libraries/` | **not in scope at all** | **97** |
| live debug prints, library code tree-wide | ~42 | **225** |
| live debug prints in `modules/core` | ~9 | **0 — done** |

So `P2-3` was never a 42-site job. It was a 128-site job once bare `cout` was counted, and a
**225**-site job once the scope included `libraries/`, which links into the same
`libtoffy.so` (`libraries/CMakeLists.txt:4-8`). Current figures:
[`../counters.md`](../counters.md). Anyone re-running the original grep will conclude the
tree got *worse* than the plan thought (42 → 29 reads as 13 fixed) while 196 uncounted
prints sit untouched. **Fix the metric before quoting progress on this item.**

**Done — all of `modules/core`:**

- Every `std::cout` and bare `cout` in core is gone (verified: 0 and 0). Hot-path noise was
  deleted outright — `"in thread."` fired on every single-step, four `"hola"` prints sat in
  `bilateral.cpp`, and several prints duplicated a `BOOST_LOG_TRIVIAL` on the line above
  them. Genuinely useful diagnostics were converted to `BOOST_LOG_TRIVIAL`, at `trace` where
  they sit in the worker/frame loop (`filterThread.cpp`, `parallelFilter.cpp`) so they are
  suppressible, and `debug` elsewhere.
- `#warning bta missing!` deleted. CMake already says `message(WARNING "no bta library!")`,
  so it was redundant build noise on every default compile.
- **All 22 `DLLExport` occurrences deleted** from `modules/` across 8 headers. It was live on
  exactly 3 class declarations (`Average`, `ImageSensor`, `BtaWrapper`); those went to
  `TOFFY_EXPORT`. On non-MSVC `DLLExport` expanded to nothing, so this is a no-op on Linux —
  and MSVC is never defined inside the library (see `P2-2`), so the `dllexport` branch was
  unreachable from here. **Two caveats this item shipped with:** the `libraries/sensor/`
  orphan still has its `DLLExport`/`WIN`/`UNIX` (`P3-1`), and `TOFFY_EXPORT` turned out to be
  vestigial in its own right — no translation unit ever defines `toffy_EXPORTS`
  (`P3-10`).
- **`WIN` and `UNIX` macros deleted** — not in the original list. They were `#define WIN true`
  in a *public header*, referenced only by three commented-out lines, leaking two of the most
  generic macro names imaginable into every consumer.
- **`RAWFILE` reduced from 3 headers to 1** (`bta/BtaWrapper.hpp`), which is where its single
  consumer (`bta.cpp`) lives. Value left as-is: it keys off `MSVC`, so it has always been
  `".r"` in practice, but changing an on-disk file extension is a behaviour change, not a
  cleanup.
- Commented-out code removed, including the stale `dlclose()` loops. Where the commented code
  encoded unfinished intent (plug-in unloading) it became a `TODO(P2-10)` pointing at the
  finding, so the information survives without the dead text.
- `<iostream>` and `using namespace std;` dropped from the core `.cpp` files that no longer
  need them — **except two that were missed**: `controller.cpp` and `filterbank.cpp` still
  `#include <iostream>` and neither uses a stream any more. Harmless, but it is the include
  that makes `std::cout` cheap to reach for, which is exactly the habit this item was
  breaking. One line each; fold into the next PR that touches those files.

**Still open — 128 prints in `modules/filters` and `modules/bta`, plus 97 in `libraries/`.**
Worst offenders, re-counted with the canonical pattern: `squareDetect.cpp` (25),
`groundprojection.cpp` (20), `BtaWrapper.cpp` (18), `thickTracer8.cpp` (17 of 33 in
`libraries/`), `blobs.cpp` (8), `sampleConsensus.cpp` (8), `simpleBlobs.cpp` (7),
`bilateral.cpp` (6), `csv_source.cpp` (6), `graph_utils.hpp` (19).
Recommend it as its own PR per module: none of that code is covered by `ctest`, and a
hundred-site sweep across untested filters should not share a commit with core, where there
is a test fence.

**Measurement hazard introduced by this cleanup:** the explanatory comments left where macros
used to be contain the macro names, so a naive `grep -rn 'DLLExport' modules/` returns
comment lines and looks like nothing was removed. The verification greps strip comments with
the `NC` helper in [`../counters.md`](../counters.md) — without it, criteria 6 and 7 report
the cleanup notes rather than the code.

## P2-15 — Enforce formatting, then keep it enforced

A `.clang-format` (6 KB) sits at the repo root but is neither applied nor checked.

Measured state of `modules/core` (**20** header/source files, re-counted — it was 22 before
`event.hpp`/`event.cpp` were deleted):

- **11 of 20 files contain hard tabs** (was 13 of 22; two tab-bearing files went away with
  `Event`).
- Namespace brace style is still split, now 5 headers on `namespace toffy {` and 6 on
  `namespace toffy` + newline. Same-line: `controller.hpp`, `filter.hpp`, `filterbank.hpp`,
  `frame.hpp`, `player.hpp`. Newline: `btaFrame.hpp`, `filterThread.hpp`,
  `filter_helpers.hpp`, `filterfactory.hpp`, `mux.hpp`, `parallelFilter.hpp`. (The earlier
  example here cited `frame.hpp` vs `filterbank.hpp`; those two now agree, so the split runs
  between `filter.hpp` and `parallelFilter.hpp` instead.)
- Also normalised for free: `it < v.end()` iterator comparisons, stray `;` after function
  bodies, and the whole-class 4-space-plus indent in `mux.hpp`.

Plan:

1. Pin a `clang-format` version. It is **not** installed in the dev container today, so
   nobody can verify compliance locally.
2. Land one whitespace-only commit (`clang-format -i`) over `modules/core` — **after** P0,
   so the correctness diffs stay readable — and record its SHA for
   `git blame --ignore-rev`.
3. Add a CI check (`clang-format --dry-run -Werror`) so the tree cannot drift again.

## P2-16 — Clean up the build flags

| sub-item | state |
|---|---|
| C++17 bump (`CMAKE_CXX_STANDARD 17`) | ✅ done — the decision this item wanted early; `findPos()` returns `std::optional<int>` because of it |
| `add_definitions(-O2 -fPIC)` forcing `-O2` into every build type | ✅ removed |
| redundant `add_definitions(-Wall)` | ✅ removed |
| `#if (BOOST_VERSION > 105500)` dead branch | ✅ removed from `controller.cpp` |
| `-Werror` | ✅ enabled for `modules/core` only (`P3-4`), behind `option(CORE_WERROR ON)` — still deliberately off everywhere else |

**The `-O2` was worse than described, in a second direction.** The write-up here said it
"forces `-O2` into every build type, including Debug". Measured from `flags.make`, it also
overrode **Release**: `add_definitions` content is emitted *after* `CMAKE_CXX_FLAGS_<CONFIG>`
on the command line, so Release compiled `-O3 … -O2` and got `-O2`. Nobody was ever getting
`-O3`. Debug had no `-O0` at all, because CMake's `CMAKE_CXX_FLAGS_DEBUG` is just `-g` — so
"Debug" meant `-O2 -g -ggdb`, and CI builds Debug, which means sanitizer reports were being
generated against inlined-away frames.

After the change, measured directly: Debug → no `-O` flag (GCC default `-O0`), Release →
`-O3`, RelWithDebInfo → `-O2`.

**Two deviations from the literal instructions here**, both deliberate:

- The plan said "Keep `-fPIC`". It is gone from `add_definitions`, but PIC is still applied —
  `CMAKE_POSITION_INDEPENDENT_CODE ON` sits on the line directly above and makes CMake emit
  `-fPIC` itself (verified present in `flags.make`). The property is the mechanism; the flag
  was noise on top of it.
- The plan said to delete the redundant `-Wall`, and it went — but it is worth knowing it was
  not merely redundant. It sat *outside* the `if(GNUCC/GNUCXX)` check, so it also reached
  MSVC, which does not accept `-Wall`. Deleting it fixed a latent Windows problem, not just a
  duplication.

**`-Werror` — done for core, and the stated blocker was wrong.** This item gated `-Werror` on
`P2-12` because core's 3 warnings under `-Wall -Wextra` were assumed to be the `filter()`
const/non-const overload trap. They were not: all three were `Mux` hiding `Filter::filter` by
declaring a different signature, and one `using Filter::filter;` line removed them (`P3-4`,
control/treatment in [`../findings/api.md`](../findings/api.md)). Core is now at 0 warnings in
all four dependency configurations and `toffy_core` compiles with `-Werror`, so the count
cannot grow again.

It is scoped to that one target (`target_compile_options(toffy_core PRIVATE -Werror)`, not
`add_compile_options`), to GNU/Clang only, and behind `option(CORE_WERROR ON)` so a downstream
build on a compiler this tree has not been measured with has an escape hatch. The rest of the
tree still emits 28 warnings and is reported, not gated (`P3-11`). `P2-12` stays open: the
design trap — a `const Filter&` getting `false` from `FilterBank`/`ParallelFilter` — was never
what those warnings were reporting.

Verification for this change: 4 dependency configurations × {Debug, Release} = 8 builds, all
configuring, building and passing 7/7 `ctest`; warning count unchanged before and after at 3
(PCL on) and 2 (PCL off), measured against a `HEAD` worktree with an identical method.

---

Four cheap fixes in the top-level `CMakeLists.txt`:

- **`add_definitions(-O2 -fPIC)` (`:294`)** forces `-O2` into *every* build type, including
  `Debug`. Combined with `-g -ggdb` (`:207`) this yields optimised debug builds — locals are
  optimised out and stack frames are inlined away, which is actively harmful while chasing
  the concurrency bugs in item 7. Keep `-fPIC`, but let `CMAKE_<CONFIG>_FLAGS` own
  optimisation.
- **`add_definitions(-Wall)` (`:317`)** is redundant; `-Wall -Wextra` already come from
  `add_compile_options` (`:207`). Delete it.
- **No `-Werror`.** The narrowing, sign-compare and unused-parameter warnings behind several
  A-items are already being emitted and ignored. Do not enable it globally yet (the build
  would fail), but add `-Werror` to files touched by each cleanup PR so the count stops
  growing.
- **`#if (BOOST_VERSION > 105500)`** in `controller.cpp` guards a branch for a 2017-era
  Boost whose `char` variant no longer exists. Delete it; prefer a `static_assert` on a
  minimum Boost version.

Decision worth taking early: `CMAKE_CXX_STANDARD 14` (`:10`). Several fixes read better in
C++17 — `std::optional` for `findPos` (A3), `std::string_view` for the slot-name constants
in `btaFrame.hpp`, and inline variables to replace header-scope `const std::string` objects.
If a bump is acceptable, do it *before* items 11 and 13 so they land in final form.
