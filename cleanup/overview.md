# Toffy — what the code is


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

Sizes are `.cpp` + `.hpp` only, so they understate directories with a lot of CMake or XML.

| Path | Files | Lines | Contents |
|---|---|---|---|
| `modules/core/` | 20 | 4 021 | Framework: `Frame`, `Filter`, `FilterBank`, factory, `Controller`, `Player`, threading |
| `modules/filters/` | 96 | 13 606 | The actual processing filters (capture, filters, detection, tracking, smoothing, viewers) |
| `modules/bta/` | 12 | 3 271 | Becom BTA camera driver wrapper (optional, `HAS_BTA`) |
| `libraries/` | 39 | 5 893 | Skeletonizers, tracers, graphs, `common/` helpers — **and it links into `libtoffy.so`**, see `libraries/CMakeLists.txt:4-8` |
| `apps/` | 2 | 300 | Executables: `toffyRunner` (`main.cpp`), `tst_pb` |
| `tests/` | 8 + 1 | 876 | 8 gtest sources + `dummy_filter.hpp` + `xml/` fixtures, wired to 8 `add_test()` targets |
| `tools/` | 1 | — | `api_change_report.sh` — the `DOD 2.4` gate |
| `docs/` | — | — | Doxygen sources (`.dox`), largely stale; see [`findings/documentation.md`](findings/documentation.md) |
| `cleanup/` | — | — | This document set |

There is **no `modules/commons/`** — an earlier version of this table listed it. The shared
helpers it was said to hold (`plugins.hpp`, `filenodehelper.hpp`) are in
`libraries/include/toffy/common/`.

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
  applied — `CMAKE_POSITION_INDEPENDENT_CODE ON` (`:303`) makes CMake emit `-fPIC` itself.
  That satisfies the plan's "keep `-fPIC`" note through the mechanism it was actually asking
  for rather than the literal flag.
- **Core's warning count is configuration-dependent**: 3 with PCL on (the `DOD 15` figure),
  2 with `-DWITHOUT_PCL=ON` — one of the three `-Woverloaded-virtual=` sites is behind a PCL
  guard. Quote the config when quoting the number.
- Optional deps gated by preprocessor macros added globally: `-DPCL_FOUND=1` (`:267`),
  `-DHAS_BTA=1` (`:316`), `-DOCV_VERSION_*`. `PCL_FOUND` arrives through a global
  `add_definitions`, which is its own problem — see
  [`findings/build.md`](findings/build.md).
- **Test harness and CI now exist.** `enable_testing()` (`CMakeLists.txt:436`) plus 8
  `add_test()` targets (gtest), and `.github/workflows/ci.yml` builds and runs them on every
  push and PR. Previously `.github/` held only Codacy, Flawfinder and Dependabot — static
  analysis bots and a dependency updater, nothing that compiled or tested anything.
- **`BUILD_DOC` does not exist.** It is referenced nowhere in the build; `docs/CMakeLists.txt`
  is `include()`'d unconditionally (`CMakeLists.txt:591`) and its `docs` target only ever
  runs on demand. Any earlier claim that it "defaults to OFF" was wrong — the flag is inert.
- **`CMAKE_BUILD_TYPE` defaults to Release** (`CMakeLists.txt:92-94`), and Debug defines
  `CM_DEBUG`, which changes `Filter`'s initial log level. Behaviour differs between the two,
  so both must be built — CI does.

---

## 4. `modules/core` inventory

**4 021 lines** across **20 files** (headers + sources), re-counted on the current tree.
`P2-14` deleted `event.hpp`/`event.cpp` (156 lines), which is why the previous figure,
3 993, is both stale and lower than the file count implies — the earlier table had also
missed `src/frame.cpp` when it was first drawn.

| File | LOC | Responsibility |
|---|---|---|
| `src/filterbank.cpp` | 471 | Pipeline execution, XML config instantiation, plugin loading |
| `include/toffy/frame.hpp` | 419 | `Frame` blackboard + all inline typed accessors |
| `include/toffy/filter.hpp` | 407 | `Filter` base, `filterState`, `FilterListener`, `Port` |
| `src/controller.cpp` | 394 | Run loop, threading, plugin loading |
| `include/toffy/filterbank.hpp` | 362 | Composite bank API |
| `src/filterfactory.cpp` | 337 | Singleton registry + type-name dispatch |
| `src/filter.cpp` | 220 | Base config/log/state plumbing |
| `src/player.cpp` / `include/toffy/player.hpp` | 162 / 134 | App facade + logging setup |
| `include/toffy/controller.hpp` | 156 | Controller API |
| `include/toffy/filterfactory.hpp` | 151 | Factory API |
| `src/parallelFilter.cpp` / `include/toffy/parallelFilter.hpp` | 146 / 92 | Parallel lanes |
| `src/filterThread.cpp` / `include/toffy/filterThread.hpp` | 123 / 103 | Worker thread + frame queues |
| `src/frame.cpp` | 105 | `Frame` out-of-line members |
| `include/toffy/filter_helpers.hpp` | 84 | `LOG` macro, ptree option getters |
| `include/toffy/mux.hpp` / `src/mux.cpp` | 67 / 38 | Fan-in of frames |
| `include/toffy/btaFrame.hpp` | 50 | BTA-flavoured `Frame` subclass + slot-name constants |
| ~~`include/toffy/event.hpp`, `src/event.cpp`~~ | ~~156~~ | ~~Event bus (incomplete)~~ — deleted, `P2-14` |

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
