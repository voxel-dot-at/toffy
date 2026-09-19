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

**P0 is complete: 12 of 12 items closed.** Also done: `P1-1` (ctest harness, 7 tests),
`P2-14` (the `Event` stub was deleted, not finished), and the C++17 half of `P2-16`.

Findings below are a record, not a to-do list, so several describe code that no longer
exists. They are marked rather than deleted so that the `A`-number citations used by the
plan keep resolving. Where a fix was partial this is stated — notably `A12` (only the
`joinable()` guard landed; the `std::terminate` risk in the four run methods is open) and
`C3` (null guards landed; `clearCreators()` teardown is untouched).

Still open and worth knowing before anything else:

- **No CI builds or tests this project.** `.github/` is Codacy, Flawfinder and Dependabot
  only. The harness exists; nothing runs it automatically. `DOD 1.2` says "CI green" must
  not be cited as evidence, and it still cannot be.
- **A PCL-less build now works** (`-DWITHOUT_PCL=ON`: clean build, 7/7 ctest; default
  PCL-on build provably unchanged). `DOD 1.1` is still not fully green, but the remaining
  gap is a **different axis**: a BTA-less build is broken by an unconditional
  `add_subdirectory(bta)`, pre-existing and confirmed on `HEAD`. See section E.
- **A version tag is outstanding.** `P2-14` removed a public virtual from `Filter`, so the
  vtable shrank; `SOVERSION` comes from `git describe`. This branch tops out at `v1.7.1`
  while `origin/next` carries `v1.9.0`, so the number is a merge decision — but it cannot
  ship untagged.
- **`A23` (no consistent error convention) blocks further correctness work** in config
  loading; see the empirical evidence recorded there.

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
| `tests/` | **One** file, `test_cond.cpp` (44 lines) |
| `docs/` | Doxygen sources (`.dox`), largely stale |

Note the naming trap: `modules/filters/` is a *sibling category*, while
`modules/filters/src/filters/` is one sub-category inside it. `Filter`/`FilterBank`
live in **core**, not in `modules/filters`.

## 3. Build system facts

- CMake, `CMAKE_CXX_STANDARD 14` (`CMakeLists.txt:10`).
- Warnings: `-Wall -Wextra` (`:207`) and a redundant `-Wall` (`:317`). **No `-Werror`.**
- Global `add_definitions(-O2 -fPIC)` (`:294`) — optimisation level hard-coded for all
  build types, including Debug.
- Optional deps gated by preprocessor macros added globally: `-DPCL_FOUND=1` (`:267`),
  `-DHAS_BTA=1` (`:307`), `-DOCV_VERSION_*` (`:280`).
- **No test harness.** No `enable_testing()`, no `add_test()`, no gtest/catch2.
  `tests/test_cond.cpp` is a bare executable. CI is Codacy + Flawfinder only
  (`.github/`), i.e. static analysis bots with no build or test gate.

---

## 4. `modules/core` inventory

4,084 lines total (headers + sources).

| File | LOC | Responsibility |
|---|---|---|
| `src/filterbank.cpp` | 472 | Pipeline execution, XML config instantiation, plugin loading |
| `include/toffy/filter.hpp` | 411 | `Filter` base, `filterState`, `FilterListener`, `Port` |
| `include/toffy/frame.hpp` | 402 | `Frame` blackboard + all inline typed accessors |
| `src/controller.cpp` | 382 | Run loop, threading, plugin loading |
| `include/toffy/filterbank.hpp` | 370 | Composite bank API |
| `src/filterfactory.cpp` | 323 | Singleton registry + type-name dispatch |
| `src/filter.cpp` | 196 | Base config/log/state plumbing |
| `include/toffy/controller.hpp` | 156 | Controller API |
| `src/parallelFilter.cpp` | 147 | Parallel lanes |
| `include/toffy/filterfactory.hpp` | 142 | Factory API |
| `src/player.cpp`, `include/toffy/player.hpp` | 273 | App facade + logging setup |
| `src/filterThread.cpp` / `.hpp` | 228 | Worker thread + frame queues |
| ~~`include/toffy/event.hpp`, `src/event.cpp`~~ | ~~156~~ | ~~Event bus (incomplete)~~ — deleted, `P2-14` |
| `include/toffy/filter_helpers.hpp` | 84 | `LOG` macro, ptree option getters |
| `include/toffy/mux.hpp`, `src/mux.cpp` | 105 | Fan-in of frames |
| `include/toffy/btaFrame.hpp` | 50 | BTA-flavoured Frame subclass + slot-name constants |

## 5. How a run actually happens

1. `Player` ctor installs Boost.Log sinks and a **global** severity filter.
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

1. **`FilterBank::stop()` starts the filters.** `src/filterbank.cpp:351-355` iterates
   `_pipe` and calls `_pipe[i]->start()`. Copy-paste of `start()`. Stopping a bank
   leaves every child in `filterRunning`.

2. **`FilterBank::remove(size_t)` erases from the wrong end.**
   `src/filterbank.cpp:299`: `_pipe.erase(_pipe.end() + i)`. Should be
   `_pipe.begin() + i`. `end() + i` is out-of-range iterator arithmetic for every `i`
   (including `i == 0`) → undefined behaviour / heap corruption. The bounds check just
   above (`:293`) is correct, which makes this easy to miss.

3. **`findPos()` cannot express "not found".** `src/filterbank.cpp:275-281` returns
   `size_t` but `return -1` on failure → `SIZE_MAX`, while the header documents
   "negative if not found" (`filterbank.hpp:266-272`).

4. **`remove(std::string)` then has no bounds check.** `src/filterbank.cpp:283-289`
   narrows `findPos()` into `int pos` (so `SIZE_MAX` → `-1`) and calls
   `_pipe.erase(_pipe.begin() + pos)` unconditionally. Removing a name that does not
   exist is UB. It also always `return 1` (success).

5. **`Frame` silently loses metadata.** `src/frame.cpp:26` copy-constructs only
   `data` and `meta`, never `desc`. `clearData()` (`:81`) clears only `data`;
   `removeData()` (`:72-79`) erases only from `data`. Consequence: `getDataType()` and
   `getDescription()` keep reporting types/descriptions for keys that are gone, and
   `hasKey() == false` while `getDataType() != NotFound`.

6. **`Filter` constructors leave members uninitialised.**
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

7. **`Filter::loadConfig()` throws right after diagnosing the problem.**
   `src/filter.cpp:92-101` logs a helpful message when the type node is missing, then
   unconditionally executes `pt.get_child(_type)`, which throws `ptree_bad_path`.

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

13. **`Frame::operator=` drops the type metadata.** `include/toffy/frame.hpp:82-86`
    copies *only* `data`:

    ```cpp
    Frame& operator=(const Frame& x) { data = x.data; return *this; }
    ```

    `meta` and `desc` are not assigned, so after `f1 = f2` every `getDataType()` returns
    `NotFound` while `hasKey()` returns `true`. This is worse than the copy ctor (which
    at least copies `meta`) and it is silent. `Frame` declares a dtor and a copy ctor but
    gets assignment wrong — a textbook Rule-of-Three break.

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

19. **`creators[type]` inserts while looking up.** `src/filterfactory.cpp:249` uses
    `operator[]` on the static creator map. A typo'd filter type therefore *mutates* a
    shared global registry by inserting a null entry, instead of failing. Use `find()`.

20. **`createFilter()` ignores its `name` argument.** `src/filterfactory.cpp:148` leaves
    the parameter deliberately unnamed (`std::string /* name */`), although
    `include/toffy/filterfactory.hpp:68-73` documents it as the filter identifier. Callers
    who pass a name silently get a generated one.

21. **A creator that returns null is dereferenced.** `src/filterfactory.cpp:257-262`
    calls `f->name()` with no null check after `f = fn()`. The following
    `_filters.insert(...)` also does not overwrite, so an id collision leaks the new
    filter and leaves it unregistered.

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

**`setLoggingLvl()` mutates process-global logging from a per-filter method.**
`src/filter.cpp` implements it by calling `logging::core::get()->set_filter(...)`.

That means one filter's configured level silently overrides the level for the entire
application, including other filters. `filter.hpp` already carries an `@todo` admitting
this ("changing severity filter affects everithing"). See also F — it is called on the
hot path.

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

**`DLLExport` is vestigial.** It is defined in `frame.hpp:34-38`, `filterbank.hpp` and
`filterfactory.hpp` — where it is even commented out at `:51` (`class /*DLLExport*/
TOFFY_EXPORT FilterFactory`).

The real export macro is `TOFFY_EXPORT`, generated by CMake's `generate_export_header`.
`DLLExport` is dead weight in three public headers and should be deleted.

**`RAWFILE` is defined in three separate public headers.**
`filterbank.hpp:29,32`, `filters/include/toffy/capture/capturerFilter.hpp:26,29` and
`bta/include/toffy/bta/BtaWrapper.hpp:23,28` each `#define RAWFILE`.

Each has its own `#ifdef` for the `.rw`/`.r` variant. Any translation unit that
transitively includes two of them gets a redefinition warning at best. It also leaks a
two-character macro into every consumer of the library.

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

**Separate pre-existing bug, found while testing the `DOD 1.1` matrix: a BTA-less build is
broken.** `modules/CMakeLists.txt` does `add_subdirectory(bta)` **unconditionally**, so the
bta module compiles even when `find_package(bta)` fails, and linking then fails with ~162
undefined `BTA*` references. Confirmed on `HEAD`, so it is unrelated to the PCL work. The
`DOD 1.1` matrix is therefore: PCL-on/BTA-on ✅, PCL-off/BTA-on ✅, and both BTA-off cells
❌ until `add_subdirectory(bta)` and its consumers are gated the same way the PCL ones now
are. There is no option to disable BTA — testing it needs
`-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON`.

**Dead version branches.** `controller.cpp` keeps `#if (BOOST_VERSION > 105500)`, guarding
a `char` log-severity variant from a 2017-era Boost that no supported compiler accepts.

**`#warning bta missing!`** (`filterfactory.cpp:104`) fires on every build of the default
configuration, training everyone to ignore build noise.

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

**`FilterBank::filter()` reconfigures logging three times per filter, per frame.**
`filterbank.cpp:49` before the loop, `:53` for every filter, and `:67` again after every
filter. Each call can touch the global logging core (see D).

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

**The docs misdescribe the buggy functions.** `filterbank.hpp` documents `findPos()` as
returning "negative if not found", which `size_t` cannot do (A3), and documents both
`remove()` overloads as "positive on success, negative or 0 in failed" while they always
`return 1` (A4). A reader trusting the docs would not suspect the bugs.

**Deprecated things are still used.** `filter.hpp` marks `_name` and
`Filter::loadFileConfig()` `@deprecated` ("we don't want a filter reading files, only the
filterbank"), yet `FilterBank::handleConfigItem()` still drives config loading through
`loadFileConfig()` for `<filterGroup>` recursion (`filterbank.cpp:110`).

**The most dangerous class is the least documented.** `filterThread.hpp` carries 10
`@todo document` markers — including on every queue, mutex and condition variable it uses
to synchronise across threads. Core contains 23 `@todo`s in total, concentrated in
`filterThread.hpp` (10), `filterbank.hpp` (6) and `controller.hpp` (4).

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
that only runs when Doxygen is found and only builds on demand; `BUILD_DOC` defaults to
`OFF`. No CI job builds docs, so broken `\ref`s and `\todo` accumulation are invisible.

**Config reference is XML-only.** `docs/configDocs/xmls/` holds one XML per filter, and
`bta.xml` exists twice (`docs/configDocs/bta.xml` and `docs/descDocs/bta.xml`) — duplicated
reference material that will drift.
