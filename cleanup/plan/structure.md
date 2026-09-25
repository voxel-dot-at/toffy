# Structural: factory, ownership, run loop, threading, layering, plugins

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

### 8 — Move logging policy out of `Filter` — **DONE**

Implemented the plan's second option: per-filter levels are **data only**, and there is one
application-level filter.

- **`setLoggingLvl()` no longer touches the global core.** It now derives `dbg` from the
  filter's own `_log_lvl` and nothing else. The name is kept (5 filters and `FilterBank` call
  it) but the doc comment says plainly that it no longer "sets the boost log filter severity",
  which is exactly what made it dangerous.
- **All three hot-path calls removed** from `FilterBank::filter()`, plus the one in
  `FilterBank::loadGlobals()`. That was three reconfigurations of the process-wide logging
  core per filter per frame.
- **New `static Filter::setGlobalLogLevel(severity_level)`** — the single sanctioned entry
  point to the shared core. Called once by the application (`Player`'s ctor already sets it),
  never per filter and never from the frame loop.
- **`_log_lvl <= 1` replaced with `<= boost::log::trivial::debug`.**
- **`@todo` in `filter.hpp` resolved** and the member documented as data-only.

Two defects found while implementing, both fixed:

- **`updateConfig()` set `_log_lvl` but never refreshed `dbg`.** It only appeared to work
  because `FilterBank::filter()` called `setLoggingLvl()` every frame — so removing the hot
  path would have silently frozen `dbg` at its constructor value. `updateConfig()` now calls
  `setLoggingLvl()` itself. This is the classic case of a bug depending on another bug.
- **`<loglvl>99</loglvl>` was `static_cast` straight into the severity enum**, producing a
  value outside `trace..fatal`. Now clamped.

**`Player` lifetime (the "Related" paragraph), done as far as it can be here:**

- `~Player` no longer calls `remove_all_sinks()`. Destroying one `Player` silenced logging
  for the entire process — any filter or second `Player` still running lost its output.
- That removal *requires* a guard, otherwise sequential `Player`s accumulate a file sink each
  and duplicate every log record. `add_file_log()` builds a new sink per call, so Boost's own
  "already registered, call ignored" rule does not help, and **the Boost.Log as built here
  exposes no way to enumerate the core's sinks** (only `add_sink`/`remove_sink`/`remove_all_sinks`
  — `sinks()` and `registered_sinks()` are both absent). Installation is therefore tracked with
  a function-local static on this side.
- Moving sink setup to `main()` entirely is still open — it is a public-API change
  (`Player`'s ctor signature), not a cleanup.

**Behaviour change, deliberate:** setting `<loglvl>` on one filter no longer changes global
output. Previously it did, nondeterministically — whichever filter ran last won. The level is
now set once by the application via `Filter::setGlobalLogLevel()` or `Player`'s ctor.

**Verified against the DOD's own gate.** `tests/test_logging.cpp` adds 7 tests; **4 fail on
the pre-fix code** (checked in a `HEAD` worktree with only `setGlobalLogLevel` back-ported so
the file would compile), including the headline `RunningABankDoesNotRelaxTheGlobalLevel`,
which captures log output through a sink and asserts a trace-level child cannot leak debug
records past an `info` threshold. The other 3 pass before *and* after on purpose —
`OptionsLogLevelStillOverridesLogLevel` pins the original precedence, which an earlier draft
of this change silently inverted by collapsing the two `get<int>()` calls into one nested
expression. The headline test also asserts a warning *is* captured, so it cannot pass
vacuously on a broken sink. 8/8 `ctest` after the change.

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
