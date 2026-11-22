# Structural: factory, ownership, run loop, threading, layering, plugins

### 4 — Replace the `createFilter()` chain with registration — **DONE** (`P2-4`)

All three steps landed as written. `createFilter()` is now one `creators.find()`, a null check
on the creator, one on its result, and the `name`/`id` bookkeeping; the categories are
`base`, `capture`, `smoothing`, `detection`, `import`, `3d` alongside the `viewers` and
`tracking` that already registered themselves, each an `init.cpp` that is nothing but
`registerCreator` calls; and `modules/filters/include/toffy/filter_init.hpp` replaced the
`extern` block. Counters 11 and 11a carry the numbers (37 branches → 0; 35 includes of modules
above core → 2).

Three things the plan did not know, because they only show up when you do it:

**The chain had three branches that could never be taken.** `reprojectopencv` and
`kalmanAverage` were guarded on `WITH_VISUALIZATION` and `OPENCV_TRACKING`, and `reprojectpcl`
on `PCL_VISUALIZATION` — macros no build defines for `modules/core`, because the filters' own
directories define them for themselves. So the filters are compiled, linked into `libtoffy.so`
and unreachable by name from any configuration file. `tests/test_filter_registry.cpp`
`ThreeChainNamesAreUnreachableToday` pins all three, and the move leaves them unreachable on
purpose: making them resolvable changes which configs load, which is not what a commit whose
claim is "the set did not move" gets to do. Filed as its own change.

Two corrections to that paragraph, both from building on the target machine
(`toreo-vxl`, Ubuntu 18.04, OpenCV 4.1 without the contrib `tracking` module) rather
than reading the host tree — see [`../findings/build.md`](../findings/build.md):

- *"compiled, linked into `libtoffy.so`"* is configuration-dependent and was stated as
  unconditional. `kalmanaverage.cpp` is compiled only `if (OPENCV_TRACKING_FOUND)`, and on a
  target without OpenCV's `tracking` module it is not in the library at all — `nm` finds zero
  matching symbols. The pinning test therefore passed there with nothing behind it: it asserted
  a property of the build and reported it as a property of the factory. It now asserts only
  where the class is compiled, names the skips, and fails at zero assertions.
- **`cvTracker` was never in the fence at all.** `tracking/init.cpp` registers it under
  `#if OPENCV_TRACKING`, a macro that directory does see, so it has been a working registered
  type the whole time (`hasCreator=1 creatable=1` measured) with no test requiring it to stay
  so. It is in `expectedTypes()` now, under the same guard the build uses.

`OPENCV_TRACKING` is also added at the top level now, next to the `find_package(OpenCV ...)`
that decides it, instead of only inside the two filter directories — which is the same defect
that made the chain's guard dead, still live in the flag's scope.

**`~Controller` called `clearCreators()`, and the chain was the only reason that was
survivable.** Finding `C3` had it recorded as wiping plug-in creators process-wide; with the
built-ins registered too, the same call erased every type name in the product, so the second
`Controller` in a process could not create its base bank. `SequentialControllersStillWork`
said so in a comment — "the coupling is real but currently benign" — and stopped being benign
on schedule. The call is gone, not the built-ins re-registered behind it: a `Controller` does
not own the registry (`P2-5` settled that).

**Two of the chain's guards are vestigial.** `squareDetect` and `exportYaml` are gated on
`PCL_FOUND`, but both are built unconditionally and neither contains pcl (exportYaml's pcl
includes are commented out), so a PCL-less build compiles the filter and refuses to name it.
Preserved as-is so this commit moves types rather than adding them; noted at each site.

The evidence is a set comparison, not a diff read: the probe over every type name resolves
**byte-identical** sets before and after — 35 names with PCL and BTA on, and in the
`WITHOUT_PCL=ON` and `WITHOUT_BTA=ON` cells exactly the expected 10 and 1 names absent with
nothing appearing that was not there before. `EveryExpectedTypeIsARegisteredCreator` is the
assertion that was red before the move and green after it.

### 5 — One owner per `Filter` — **DONE** (`P2-5`, PR 36)

The recommended shape below is what landed, with two differences worth naming:

- **The factory registry is `weak_ptr`, not `shared_ptr`.** The plan implied the factory
  could keep holding filters; it now indexes them. That is what makes "one owner" literally
  true — the caller of `createFilter()` holds the only strong reference — and it is what lets
  `getFilter(name)` answer "no such filter" instead of returning freed memory once the owner
  has gone. A `shared_ptr` registry would have kept the old leak and given it a GC feel.
- **`clearCreators()` is no longer called by `~Controller`.** The paragraph below said it "no
  longer has to exist as a teardown step", which `P2-5` made true and `P2-4` made enforced: the
  call is gone. It could not survive the move — with the built-ins registered, wiping the map
  in a destructor wiped every type name in the product. The behaviour change `P2-5` deferred
  (a second `Controller` keeps the plug-ins the first registered) is the correct behaviour and
  is what both consumers already assume. Findings `C1`, `C3` and `A17` are closed.

Everything else went as written: `_pipe` is `std::vector<FilterPtr>`, `createFilter()` returns
`FilterPtr`, `remove()` is an `erase` with no factory round-trip, `insert()` gained its bounds
check, `FilterThread` shares instead of deleting, and nothing in `modules/core` calls `delete`
on a `Filter` (counter 13). `CreateFilterFn` deliberately did **not** change — it is the
plug-in ABI (`X6`). The cost is an ABI break, recorded in [`../status.md`](../status.md) and
counter 31: `createFilter()` returns by value, which changes the calling convention without
changing the mangled name, so this is the release that has to be `v2.0.0`.

Evidence: `tests/test_ownership.cpp` (9 tests, ctest target `ownership`) counts destructors;
8 of the 9 fail on the parent commit, five of them with an ASan report, and one of those is a
full `Player` session whose shutdown leaks exactly the one filter that had been renamed by
its own config.

---

*Original item text, kept because `C1`/`A17`/`C3` cite it:*

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
needs no `deleteFilter` (done), and `clearCreators()` no longer has to exist as a teardown
step — true under `P2-5`, enforced under `P2-4`, which is where the call came out.

If `shared_ptr` is judged too invasive, the alternative is to make the factory a pure
creator (no registry at all) and let banks own `std::unique_ptr<Filter>` — but then
`getFilter(name)` must be reimplemented against `_pipe`, which is what its deprecation note
already asks for.

### 6 — Collapse the four run methods — **done**

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

**Landed in `f623259`.** One private `run(state, bool singleStep)`; the four public
methods are one-line wrappers. The two real differences were the state and whether the
bank runs once on the calling thread or on a worker until `stop()`. A step deliberately
never touches `_state` — which is what lets a step follow a step — and that is preserved
and tested rather than tidied away.

**The loop condition was the actual bug.** `while (_state > Controller::IDLE)` reads as
"anything but idle" and is not: `CERROR = 0xff` is the largest enumerator, so an error
state is greater than `IDLE` and the loop spins forever instead of leaving. Now
`Controller::isRunning()`, an explicit list, so reordering the enumerators cannot silently
change what the loop does either. `CERROR` is never assigned anywhere in toffy or either
consumer, so this was unreachable — the same shape as `P2-10`'s throwing loaders, and
worth fixing for the same reason: the enumerator is public API, so the defect is one line
of someone else's code away.

**These four methods had no test coverage at all.** Eight `ControllerRun` tests now cover
them. The red was a genuine runtime red, not a compile error: the 7 characterisation tests
pass against the *old* code — which is what makes them a faithful baseline for a collapse —
and the `CERROR` test fails against the old predicate, stubbed in temporarily to prove it.

**Two of the four flagged sub-issues were deliberately not changed**, recorded in
`findings/correctness.md` rather than quietly dropped:

- The viewer count is computed **once before the loop**. Recomputing per iteration costs a
  bank walk per frame on a production line-scan machine, so "just recompute" is not free.
  It needs a decision about whether filters can be added during a run at all — if they
  cannot, the current code is correct and should say so.
- `cv::waitKey(10)` inside the core run loop. Removing it means a "needs GUI event pump"
  hook on `Filter`, which adds a virtual — and `P2-12` pinned `Filter`'s vtable layout
  precisely because inserting a virtual shifts every later slot. Even appended at the end
  it changes the vtable size, so it is a plug-in ABI break and belongs in major 3, not
  smuggled into a collapse commit.

Counters 54–56.

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

### 9 — Fix the module layering — **done**

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

**Landed in four commits** (`7b044f4`, `359d103`, `95b4fff`, and the `Controller`
encapsulation commit). Each item is its own commit because each is a separate behaviour
question; item 3 in particular would have been two behaviour changes in one commit if the
lookup key had been changed along with the scope.

Two of this item's claims were wrong, continuing a run: `P2-11` had two, this has two.

- **"Core also links viewers as a result" — it does not.** `modules/core` and
  `toffy_viewers` are both `OBJECT` libraries whose objects are merged into a single
  `libtoffy` (`CMakeLists.txt:599` and `:611`). There is no link step between them. The
  dependency was compile-time only: core's TU needed viewers' header on the include path.
  Invisible in the build graph, no less real, but not a link dependency and not fixable by
  anything that touches linking.
- **`getSertMatPtr`-style "belongs in `modules/bta/`" for `btaFrame.hpp` is not doable as
  written.** The three filters that include it (`blobs`, `blobsDetector`, `simpleBlobs`)
  are built unconditionally, so moving the header into `modules/bta/` would make them
  depend on bta and break `-DWITHOUT_BTA=ON`. The real defect there — five `const
  std::string` objects *defined* in an installed header, hence one copy, one dynamic
  initialiser and one address per including TU — was fixed in place instead, with
  `extern` + one `.cpp`. Placement is left as a finding.

Notable consequences, both measured:

- The five `bta*` constants are now **exported ABI** (type `B`) where before each TU had a
  private copy and nothing was exported. `abi_surface.sh` reports exactly +5, no removals;
  baseline re-blessed from a verified BTA-off build. They are frozen from here.
- `FilterBank::getFilter(name)` keeps its `id()` key. `findPos()`/`remove()` in the same
  class match on `name()`. That split predates this work and is recorded as **suggestion
  S5** in `findings/api.md` with a route to fixing it; fixing scope and key together would
  have been two behaviour changes in one commit.
- Making `Controller`'s members private is a **source** change and **not an ABI** change —
  the layout is identical — which is why it can land inside major 2 under the rule the
  `v2.1.0` tag set. Measured first: neither member is referenced by either consumer; the
  only external users were `Player` (which got `baseBank()` / `getFrame()`) and toffy's own
  tests. A `const Frame& getFrame() const` overload was needed because `Player::hasKey()`
  is const and holds a const `Controller`.

Gates added: a hygiene step stops `modules/core` including anything from a module above it
(allowlisting only the `HAS_BTA` hook, which `P2-10` owns), and `tests/test_layering.cpp`
pins core's copy of the HighGUI-pump type name against `ImageView::id_name`. Both verified
capable of failing — the gate only after I caught my first check of it being vacuous (a
`sed` anchored on a line `controller.cpp` does not contain, so nothing was inserted and the
gate correctly stayed quiet). Counters 47–50.

### 10 — Consolidate the three plugin loaders — **done**

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

**Landed in `9fcfd40`.** One implementation in `toffy::commons::plugins` —
`loadOne()` for the `dlopen`/`dlsym`/`init` sequence, `loadFromConfig()` for walking the
node, `loadedHandles()` as the single owner — with all three entry points delegating.
Node found with `get_child_optional`, so an absent node is "nothing to do". `Player`'s
`try`/`catch` around the whole loop went away with it, which also stops it swallowing
genuine failures from a node that *was* present.

Three things the plan did not say, all measured:

- **The worst defect was not in the table.** `FilterBank::loadPlugins` kept the `dlopen`
  handle in a local that was never stored anywhere. Every plug-in it managed to open was
  leaked with no record at all — unclosable even in principle. Counter 52.
- **`FilterBank::loadPlugins` is `protected`, not public, and nothing calls it.** The only
  `FilterBank` subclasses in the tree are `Cond` and `ParallelFilter`, neither of which
  calls it, and no consumer subclasses `FilterBank` at all. Consolidating it was not the
  API break the plan assumed; it is kept anyway, because a downstream subclass could.
- **The throwing path was unreachable in practice.** `Controller::loadPlugins` is called
  only from `Controller::loadConfigFile()` and `Controller::loadConfig()`, and neither is
  called by toffy or either consumer. `Player::loadConfig()` is the live path and is the
  one that catches — which is exactly why production works and the other two never showed
  up as failures.

**Why the handles are still never `dlclose()`d.** The plan's last line — "keep `dlopen`
handles in a single owner so they can actually be closed" — assumed closability was the
goal and ownership the blocker. Ownership *was* the blocker, and fixing it makes the real
blocker visible: a plug-in's `init()` registers creators into the process-wide
`FilterFactory`, and those creators are function pointers into the `.so`. Unloading it
while the registry still holds them makes the next `createFilter()` jump into unmapped
memory. Closing safely needs `FilterFactory` to give creators back first, which it cannot
do. So the handles are now a documented process-lifetime registry rather than a TODO. The
old comment asked for `_loads` to stop being an untyped `std::vector<void*>` as if that
were the obstacle; it was not.

Also fixed on the way: the `dlsym`-failure path now `dlclose()`es the library it
successfully opened. None of the three copies did that.

There is now a **real plug-in in the tree** (`tests/plugin_fixture.cpp`, built as a
`MODULE`). Until this, the plug-in path was only ever exercised with a library that does
not exist — which tests the failure branch and nothing else. Counters 51–53.

Plan: one `loadPlugins(ptree&)` in one place, taking the node name as a parameter or
locating it with `get_child_optional`. Delete the other two. Keep `dlopen` handles in a
single owner so they can actually be closed.
