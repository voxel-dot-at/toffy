# Toffy `modules/core` Cleanup Plan

Companion to [`PROJECT_INFO.md`](PROJECT_INFO.md), which holds the evidence. Every item
below cites the finding it comes from (`A3`, `D1`, …) so it can be checked before touching
code.

**Ground rules**

- Fix bugs before refactoring. Several P0 items are one-line changes that would be lost in
  a larger restructure.
- One commit per numbered item. Most are independent and revertable in isolation.
- Add the failing test *before* the fix where a test harness exists (see P1-1). Today none
  does, so P1-1 gates everything after it.
- Do not rename public symbols in the same commit as a behaviour fix; that keeps `git
  bisect` and review readable.

---

## P0 — Correctness, no API change

These are defects, not preferences. Each is small and independently verifiable.

| # | Item | Finding | Risk |
|---|---|---|---|
| 1 | ~~`FilterBank::stop()` calls `stop()`, not `start()`~~ **DONE** — fixed, pinned by `FilterBankStop.StopStopsChildren` | A1 | trivial |
| 2 | ~~`remove(size_t)`: `_pipe.begin() + i` not `end() + i`~~ **DONE** — pinned by `FilterBankRemove.ByIndexRemovesTheElementAtThatIndex` | A2 | trivial |
| 3 | ~~`remove(string)`: bounds-check, return real status~~ **DONE** — pinned by `FilterBankRemove.MissingNameLeavesBankIntact` | A4 | low |
| 4 | ~~`findPos()`: return `int` / `optional`, not `size_t -1`~~ **DONE** — returns `std::optional<int>` (C++17); pinned by `FilterBankFindPos.MissingNameYieldsNoValue` | A3 | low |
| 5 | Initialise all `Filter` members in both ctors | A6 | trivial |
| 6 | `Frame`: copy/assign/clear `meta` + `desc` consistently | A5, A14 | low |
| 7 | Null-check `Event::data()` | A8 | trivial |
| 8 | Null-check after `fn()` in `createFilter()` | A20 | trivial |
| 9 | Null-check `baseFilterBank` in `~Controller` | C3 | trivial |
| 10 | `creators.find()` instead of `operator[]` | A18 | trivial |
| 11 | Delete or define `Player::stop()` | A13 | trivial |
| 12 | Guard `pt.get_child(_type)` after diagnosing its absence | A7 | low |

### Notes on the non-obvious ones

**1 — `stop()` starting the pipeline** (`filterbank.cpp:351-355`) is a copy-paste of
`start()` two functions above. The whole body is correct except the method name being
called. Stopping a bank currently leaves every child reporting `filterRunning`, so no
filter ever sees a stop.

**2 — `end() + i`** (`filterbank.cpp:299`) is out-of-range iterator arithmetic for *every*
value of `i`, including `0`. The bounds check immediately above it (`:293`) is correct,
which is what makes this survive review. It is the single most likely cause of any
"random" crash after removing a filter.

**3 + 4 — the `remove`/`findPos` pair is one bug with two faces.** `findPos` returns
`SIZE_MAX` on failure (`:280`), `remove(string)` narrows that into `int pos` (`:285`) so it
becomes `-1`, then erases at `begin() - 1` with no check (`:286`). Fixing only one of the
two leaves the path broken. Recommended shape:

```cpp
// returns index in [0, size), or -1
int FilterBank::findPos(const std::string& name) const;

int FilterBank::remove(const std::string& name) {
    const int pos = findPos(name);
    if (pos < 0) return 0;          // not found
    ...
    return 1;
}
```

**5 — uninitialised `Filter`.** `Filter::Filter() : _type("...") {}` (`filter.cpp:56`)
leaves `_bank`, `_log_lvl`, `dbg`, `update` and `state` indeterminate. `_bank` matters most:
`loadGlobals()` casts it to `FilterBank*` and calls through it (A9), so an uninitialised
`_bank` is a call to an arbitrary address. Use in-class initialisers in `filter.hpp` so no
constructor can forget:

```cpp
Filter* _bank = nullptr;
bool dbg = false;
bool update = false;
filterState state = filterLoaded;
boost::log::trivial::severity_level _log_lvl = boost::log::trivial::info;
```

**6 — `Frame` metadata.** Three separate leaks of the same invariant: the copy ctor omits
`desc` (`frame.cpp:26`), `operator=` omits `meta` *and* `desc` (`frame.hpp:82-86`),
`clearData()` clears only `data` (`frame.cpp:81`), `removeData()` erases only from `data`
(`:72-79`). Fix all four together, otherwise `getDataType()` keeps reporting types for keys
that are gone.

---

## P1 — Small, behaviour-visible, low-risk

### 1 — Add a test harness first

There is no `enable_testing()`, no `add_test()`, no gtest/catch2 anywhere. `tests/` holds a
single 44-line `test_cond.cpp` built as a bare executable that nothing runs. CI
(`.github/`) is Codacy + Flawfinder only — static-analysis bots with **no build and no test
gate**, which is exactly why items A1–A22 have survived.

Suggested minimum, before any P2 work:

```cmake
enable_testing()
add_subdirectory(tests)          # tests/CMakeLists.txt
```

```cmake
# tests/CMakeLists.txt — one target per area
add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame toffy)
add_test(NAME frame COMMAND test_frame)
```

The first tests should pin the P0 fixes: `remove()` on a missing name, `findPos()` on an
empty bank, `Frame` copy/assign preserving `meta` + `desc`, and `stop()` actually stopping.
Those tests are the regression fence for everything in P2.

### 2 — Fix or delete the Windows paths

`-DMSVC` is appended to `DEFINITIONS` (`CMakeLists.txt:169`) but the line that would apply
`DEFINITIONS` to the library is commented out (`:373`); only `apps/CMakeLists.txt:6`
consumes it. So every `#ifdef MSVC` branch inside `modules/` compiles as POSIX
unconditionally — `filterbank.cpp:415`, `controller.cpp` `loadPlugin()`, and `frame.hpp:34`
are all dead code, and the `dlfcn.h` path is what actually ships.

Pick one:

- apply `-DMSVC` to the library target so the branches become live and get tested, or
- delete the `MSVC` branches and state that Windows is unsupported.

Leaving them is the worst option: it looks like support and is neither compiled nor tested.

### 3 — Remove debug leftovers and dead code

- `std::cout` in library code: `controller.cpp:223` (`"in thread."`) and `:228`
  (`"Thread ends."`) fire on every single-step; `filterbank.cpp:314` prints
  `"AU!!!! clearBank failed…"`; `filterfactory.cpp:153,248` and `filter.cpp` print config
  noise. Route through `BOOST_LOG_TRIVIAL` or delete.
- `#warning bta missing!` (`filterfactory.cpp:104`) fires on every default build.
- `DLLExport` is defined in `frame.hpp:34-38`, `filterbank.hpp` and `filterfactory.hpp`
  (commented out at `:51`) but `TOFFY_EXPORT` from `generate_export_header` is what is
  really used. Delete all `DLLExport` definitions.
- `RAWFILE` is defined in three headers (`filterbank.hpp:29,32`,
  `filters/.../capturerFilter.hpp`, `bta/BtaWrapper.hpp`) — a redefinition hazard that also
  leaks into every consumer. Move to one internal header or a `constexpr` in a `.cpp`.
- `#if (BOOST_VERSION > 105500)` in `controller.cpp` keeps a branch for a 2017 Boost; the
  alternative branch uses an API that no longer exists.
- Commented-out blocks: `controller.cpp:41-49` (a stale singleton), `frame.hpp:262-280`
  (planned `insGet`/`setGet`), `mux.hpp` (pure virtuals), `filterbank.cpp:107,144,433-436`.
- `using namespace std;` / `using namespace cv;` in `filterbank.cpp:37-38`,
  `filterfactory.cpp:37`, `controller.cpp:38-39`. `cv` is not needed at all in
  `filterbank.cpp`.

---

## P2 — Structural

Do these only after P0 is merged and P1-1 has a test fence in place.

### 4 — Replace the `createFilter()` chain with registration

`filterfactory.cpp:148-265` is a ~120-line `if/else if` over ~30 string literals, executed
on every instantiation. It also hard-codes every filter into core, which defeats the plugin
mechanism sitting in the same file.

The replacement already exists and works: `modules/filters/src/viewers/init.cpp:54` and
`modules/filters/src/tracking/init.cpp:26` define `initFilters(FilterFactory&)` and call
`factory.registerCreator(...)` (`viewers/init.cpp:59-68`, `tracking/init.cpp:31-33`).

Steps:

1. Give each sub-category its own `init.cpp` doing nothing but `registerCreator` calls
   (capture, filters, detection, smoothing — viewers and tracking are done).
2. Have core call the category init functions once, instead of branching per type. Replace
   the `extern void initFilters(FilterFactory&)` declarations currently buried inside a
   namespace block at `filterfactory.cpp:76,79` with a real header.
3. Reduce `createFilter()` to a single map lookup plus the `name`/`id` bookkeeping.

This deletes the largest function in core and makes the "unknown filter" path a single
log line instead of a fall-through `else`.

### 5 — One owner per `Filter`

Today `FilterFactory::_filters` (a global static map) and `FilterBank::_pipe` both hold raw
`Filter*`, `add()` takes a raw pointer, and destruction goes through the factory *by name*
(`clearBank()`, `filterbank.cpp:303-319`). Any rename, or any filter reachable from two
banks, leaks or double-frees.

Recommended shape, which also fixes A17, C1, C3 and the `insert()` bounds problem. Note
`FilterPtr` already exists (`filter.hpp:50`) and is already used for filter-in-filter
composition (`backgroundsubs.hpp:56`, `kalmanaverage.hpp:33`) — it is only the factory and
the bank that ignore it, so the type is not a new concept to introduce.

```cpp
using FilterPtr = std::shared_ptr<Filter>;   // declared at filter.hpp:50

std::vector<FilterPtr> _pipe;
FilterPtr createFilter(const std::string& type, const std::string& name = "");
```

Then `remove()` is `_pipe.erase(begin() + pos)` with no factory round-trip, `~Controller`
needs no `deleteFilter`, and `clearCreators()` no longer has to exist as a teardown step.

If `shared_ptr` is judged too invasive, the alternative is to make the factory a pure
creator (no registry at all) and let banks own `std::unique_ptr<Filter>` — but then
`getFilter(name)` must be reimplemented against `_pipe`, which is what its deprecation note
already asks for.

### 6 — Collapse the four run methods

`Controller::forward()`, `backward()`, `stepForward()` and `stedBackward()`
(`controller.cpp:74-184`) are ~90% the same code: look up `parallelFilter`s, `init()` them,
set `_state`, spawn `loopFilters`/`loopFiltersOnce`, join. Four copies of a thread lifecycle
is four places for the A12 join bug to hide.

Suggested: one private `runLoop(State, bool once)` plus thin public wrappers.

While in there, fix the `loopFilters()` policy itself (`controller.cpp:198-212`): the
viewer count is computed **once before the loop** (`:202`), so filters added later are not
seen; the loop condition `while (_state > Controller::IDLE)` depends on enum ordering, and
`CERROR = 0xff` (`controller.hpp:60`) is the largest value — so an error state spins the
loop forever instead of exiting; and `cv::waitKey(10)` puts an OpenCV GUI call inside the
core run loop.

### 7 — Make the threading honest

Three separate problems, all in the parallel path:

- `keepRunning` is a plain `bool` (`filterThread.hpp:85`) written by the controller and read
  by the worker loop. Make it `std::atomic<bool>` (or guard it). As written the compiler may
  hoist the read out of the loop, so `stop()` can be ignored.
- `interprocess_semaphore` (`filterbank.hpp:23,357`) is a System V *inter-process* primitive
  used to pass frames between threads of one process. Replace with
  `std::binary_semaphore`/`std::condition_variable`. It is heavier, platform-specific, and
  `post()` past the maximum throws — relevant because `FilterBank::filter()` posts once per
  frame and nothing guarantees a matching `wait()`.
- `FilterThread` declares a copy ctor (`filterThread.hpp:53`) that copies the owned `Filter*`
  with no ownership transfer, while the doc at `:42` says the thread deletes it. Either
  `= delete` the copy operations or switch the member to `std::unique_ptr<Filter>`. As
  declared the ctor cannot even compile (it would copy `boost::thread` and `boost::mutex`),
  so deleting it costs nothing.

Also restore `#include <list>` (`filterThread.hpp:19`, commented out) and add the missing
`<boost/thread/mutex.hpp>` / `<boost/thread/condition_variable.hpp>` / `<memory>` includes
that `filterThread.hpp`, `event.hpp` and `mux.hpp` currently get by transitive luck.

### 8 — Move logging policy out of `Filter`

`Filter` owns a `_log_lvl` member and `setLoggingLvl()` reconfigures the **global**
`boost::log` core from a per-filter method — so the last filter to run dictates the level
for the whole process. `FilterBank::filter()` calls it three times per filter per frame
(`filterbank.cpp:49,53,67`). This is the `@todo` in `filter.hpp` ("changing severity filter
affects everithing").

Plan: keep a per-filter level as *data*, but apply it at the log call site (or drop
per-filter levels entirely and keep one application-level filter). Remove the
`setLoggingLvl()` calls from the hot path. Replace the `_log_lvl <= 1` magic number in
`filter.cpp` with `boost::log::trivial::debug`.

Related: `Player`'s ctor installs sinks and a global severity filter, and `~Player` calls
`remove_all_sinks()` — so destroying one `Player` silences logging for the whole
application. Logging setup belongs to `main()`, not to a library object's lifetime.

### 9 — Fix the module layering

- `controller.cpp:30` includes `<toffy/viewers/imageview.hpp>` — **core depends on the
  viewers module**, purely to reach `ImageView::id_name` at `:202`. Core also links viewers
  as a result. Replace with a "needs GUI event pump" flag on the Filter interface, or a
  string constant owned by core.
- `btaFrame.hpp` lives in core but defines BTA camera slot names (`btaMf`, `btaIt`,
  `btaFc`, `btaDepth`, `btaAmpl`). Camera-specific vocabulary belongs in `modules/bta/`.
  Those `const std::string` objects at namespace scope in a header also get a separate
  instance per translation unit (C++14 has no inline variables) — move them to the `.cpp`
  with an `extern` declaration.
- `FilterBank::getFilter(name)` (`filterbank.cpp:227-237`) delegates to the global factory
  instead of `_pipe`, so a bank returns filters it does not contain. Reimplement against
  `_pipe` (its own `@deprecated` note asks for exactly this).
- `Controller` exposes `baseFilterBank` and `f` as **public data members**, and `Player`
  both wraps `Controller` and hands it out via `getController()` — so the facade protects
  nothing. Make them private with accessors.

### 10 — Consolidate the three plugin loaders

The same feature is implemented three times, with three different failure modes:

| Implementation | Node looked up | Missing `<plugins>`? |
|---|---|---|
| `Player::loadPlugins` (`player.cpp:123-139`) | `toffy.plugins` | caught, logged as warning |
| `Controller::loadPlugins` (`controller.cpp:306-320`) | `toffy.plugins` | **throws `ptree_bad_path`** |
| `FilterBank::loadPlugins` (`filterbank.cpp:404-472`) | `plugins` | **throws `ptree_bad_path`** |

`Controller::loadPlugins` is the identical loop to Player's but without the
`try`/`catch`, and `get_child()` throws when the path is absent. So
`Controller::loadConfigFile()` (`controller.cpp:294`) aborts on any config that has no
`<plugins>` node, while `Player::loadConfig()` (`player.cpp:87`) survives it. Both entry
points also each trigger their own loader, so the same node can be processed twice.

`FilterBank::loadPlugins` additionally uses a different node name (`plugins`, not
`toffy.plugins`), so it never finds the node the other two expect.

Plan: one `loadPlugins(ptree&)` in one place, taking the node name as a parameter or
locating it with `get_child_optional`. Delete the other two. Keep `dlopen` handles in a
single owner so they can actually be closed.

### 11 — Fix the `Frame` accessor API

- **The `SlotDataType` tag is decorative.** `addData(long)` (`frame.hpp:140`) stores a
  `long` under the `Int` tag, but `getInt()` (`:314`) does `any_cast<int>`, which checks
  the *real* C++ type — so it throws `boost::bad_any_cast`. Same for `unsigned long` /
  `Uint` (`:144`). Either drop the `long` overloads or add matching `getLong()` /
  `getULong()` and tag them honestly.
- **`optString` cannot take a literal.** `frame.hpp:259,386` declare
  `std::string& dfault` (non-const ref) while every sibling `opt*` takes its default by
  value. Change to `const std::string&`.
- **`getSertMatPtr` is a typo in the public API** (`frame.hpp:282,392`). Rename to
  `getOrInsertMatPtr`; nothing in the repo calls it, so a deprecated alias is cheap.
- **`getSertMatPtr` is the only mutating `const` member.** It inserts into `data`/`meta`
  from a `const` method. When `Frame` gains any concurrency this is the first thing to
  break — decide whether the accessor maps should be `mutable` + guarded, or whether the
  method should be non-`const`.

### 12 — Resolve the `filter()` const/non-const overload trap

`Filter` declares two virtuals (`filter.hpp`):

```cpp
virtual bool filter(const Frame& in, Frame& out) const { return false; }   // :~200
virtual bool filter(const Frame& in, Frame& out)      { return constF.filter(in, out); }
```

`FilterBank` and `ParallelFilter` override **only** the non-`const` one. So anything
holding a `const Filter&` — a `const`-correct caller, a shared `const` handle — gets a
silent `false` with no log line and no error state.

Plan: delete the `const` overload from the base and keep one non-`const` virtual. Filters
that genuinely do not modify state can still be `const` internally; the interface does not
need to express it. If the `const` variant is kept, every composite must `override` it and
the base should be pure virtual so a new filter cannot forget.

Also add `override` consistently: `mux.hpp` uses it, `filterbank.hpp` and
`parallelFilter.hpp` do not. `override` on every reimplementation would have made the
missing `const` override a compile error.

### 13 — Unify run state and rename the remaining typos

Two independent state machines model the same thing:

- `filterState` (`filter.hpp:52`) — `filterLoaded/Idle/Running/Paused/Error`, per filter.
- `Controller::state` (`controller.hpp:53-60`) — `IDLE/.../CERROR = 0xff`, per application.

`Controller::state` is compared with `>` (`controller.cpp:203,219`), so its numeric order is
load-bearing, and `CERROR = 0xff` being the largest value means an error keeps the run loop
spinning (see item 6).

Plan: keep one enum, make `Controller` hold a `filterState` plus a direction
(`FORWARD`/`BACKWARD`) rather than encoding direction into the state ordinal, and replace
`>` comparisons with explicit checks.

Renames, all in the public API and all currently uncalled elsewhere, so do them now:

| Current | Should be | Location |
|---|---|---|
| `Controller::CERROR` | `ERROR` | `controller.hpp:60` |
| `Controller::stedBackward()` | `stepBackward()` | `controller.hpp:97`, `controller.cpp:154` |
| `Frame::getSertMatPtr()` | `getOrInsertMatPtr()` | `frame.hpp:282,392` |

Leave a deprecated inline alias for each for one release.

### 14 — Finish or delete the `Event` stub

`event.hpp` carries `@todo Implement the event logic`, and the class is not used by the
run loop at all. While it stays:

- `data()` returns `*_data` with no null check — calling it before `data(any)` was set
  dereferences null (A8).
- The default `Event()` leaves `_re_type` and `_sender` uninitialised.
- `receiver()` and `event()` return `std::string` by value; should be `const&`.
- `receiverType()` is not `const`.
- `Filter::processEvent` logs "Filter does not have events declared" at `info` for every
  unhandled event — log spam the moment events are used.

Plan: either implement the bus (typed payloads, `std::function` subscribers, no raw
`Filter*` sender) or remove the class until there is a consumer. A half-implemented event
system in a header every filter includes is worse than none.


### 15 — Enforce formatting, then keep it enforced

A `.clang-format` (6 KB) sits at the repo root but is neither applied nor checked.

Measured state of `modules/core` (22 header/source files):

- **13 of 22 files contain hard tabs.**
- Namespace brace style is split almost evenly: 6 files use `namespace toffy {`, 5 use
  `namespace toffy` + newline. `frame.hpp` and `filterbank.hpp` disagree with each other.
- Also normalised for free: `it < v.end()` iterator comparisons, stray `;` after function
  bodies (`frame.hpp:362-390`), and the whole-class 4-space-plus indent in `mux.hpp`.

Plan:

1. Pin a `clang-format` version. It is **not** installed in the dev container today, so
   nobody can verify compliance locally.
2. Land one whitespace-only commit (`clang-format -i`) over `modules/core` — **after** P0,
   so the correctness diffs stay readable — and record its SHA for
   `git blame --ignore-rev`.
3. Add a CI check (`clang-format --dry-run -Werror`) so the tree cannot drift again.

### 16 — Clean up the build flags

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
