# A — Correctness bugs

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
   citations in [`../plan/`](../plan/) keep resolving.

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

24. **~~`CSVSource` conflated its frame counter with its sequence flag.~~ FIXED** — found
    while implementing `P3-6`, and outside `modules/core`, but the same shape as `P0-1`
    (copy-paste between two adjacent functions) and `P0-5` (uninitialised members).

    `csv_source.hpp` declares `int sequence` — the index substituted into the file-name
    pattern — and `bool useSequence` — whether that index advances. Neither had an
    initialiser, and the three config functions disagreed about which was which:

    | site | did |
    |---|---|
    | `loadConfig()` | `sequence = pt.get<bool>("options.sequence", sequence)` — read the *counter* as the default for a *bool* get (reading uninitialised memory), then stored the flag in the counter |
    | `updateConfig()` | `useSequence = pt.get<bool>("options.sequence", useSequence)` — the correct member |
    | `getConfig()` | `pt.put("options.sequence", sequence)` — wrote the counter under the flag's key, so a config round-trip moved the playback position into the flag |
    | `filter()` | `if (useSequence) sequence++` — branched on the member `loadConfig()` never set |

    Net effect on the documented use of the class: with `<options><sequence>true</sequence>`,
    the first frame played was frame **1**, not 0, and whether playback advanced at all was
    whatever the heap had left in `useSequence`. Measured before the fix: all four tests of
    `tests/test_csv_source.cpp` failed, `SequencedPlaybackStartsAtFrameZero` reading frame 1,
    and `WithoutTheFlagTheSameFrameIsReRead` advancing 1, 2, 3 with the flag off because the
    uninitialised `useSequence` happened to be true. Pinned by `CsvSourceSequence.*` (ctest
    target `csv_source`).

    **Not fixed, same file:** `filter()` calls `cv::waitKey(500)` — a GUI event-loop call in
    a capture filter's read path — and `loadConfig()`/`updateConfig()` append `options/fcs`
    to `fcs` without clearing it, so a second `updateConfig()` duplicates the list. Both are
    `P2-3`/`P2-13` sized and neither is a correctness defect of this shape.

25. **`ExportCSV`'s `options/skipZeroes` is plumbed through the whole filter and then
    ignored.** Found while fixing `N5` in the same file, and the compiler has been saying so
    on every build: `exportcsv.cpp:156` is `saveMatCSV(fileName, mat, bool skipZeroes)` and
    the body never reads the parameter — `-Wunused-parameter`, one of the 26 warnings the
    tree emits and `P3-11` reports without gating.

    The value is not lost in one place, it is carried through all of them, which is what makes
    it a defect rather than an unfinished corner:

    | site | does |
    |---|---|
    | `updateConfig()` | `_skip0s = pt.get<bool>("options.skipZeroes", _skip0s)` |
    | `getConfig()` | writes it back, so a config round-trip preserves it |
    | `filter()` | `saveMatCSV(std::string(path), *input, _skip0s)` |
    | `saveMatCSV()` | ignores it; every one of the six `mat.type()` branches writes all pixels |

    The `uint32_t j` that `saveMatCSV` increments once per pixel and never reads is the other
    half of the same unfinished feature — a counter for values that were meant to be skipped.

    A user setting `<options><skipZeroes>true</skipZeroes>` gets a full CSV and no diagnostic,
    and `getConfig()` reports the option as configured, so the round-trip looks like success.
    (`docs/` does not mention the option at all — the only documentation of it is the config
    reader itself.) Either implement it — skip zero-valued pixels per type, which also gives
    `j` a purpose — or delete the option and its `getConfig()` line. Not fixed here: `P3-6` is
    the format-string item, this is a behaviour change, and one numbered item per commit is
    [`../dod/per-pr.md`](../dod/per-pr.md) #5.

**A26 — `toffyRunner`'s Ctrl-C handler is compiled and never installed on Linux.** Found
while writing the stop section of `use.dox` (`X4`), which had to document what actually
happens rather than what the code appears to do. `apps/main.cpp` defines `my_handler(int)`,
which clears the `keepRunning` flag so the frame loop can exit cleanly, and then never
registers it: the `sigaction(SIGINT, &sigIntHandler, NULL)` call is commented out a few lines
below the `struct sigaction` it would have used. The registration that does exist,
`SetConsoleCtrlHandler()`, is inside `#ifdef MSVC`. Two consequences, both measured:

- Ctrl-C takes the default disposition, so the process dies mid-frame — no `Stopped...`, no
  unwinding of the player, whatever the OS does to the OpenCV windows.
- `sigIntHandler` is `set but not used` and `my_handler`'s parameter is unused. Those are two
  of the warnings `P3-11` reports per area; the compiler has been describing this defect on
  every build.

Verified by running the binary under `timeout -s INT` and grepping its output for the
handler's message and for `Stopped...`: neither appears. Not fixed here — installing the
handler changes shutdown behaviour (the loop would then exit through `keepRunning`, and the
player's teardown gets run for the first time in anger), which is a behaviour change with its
own test, and it sits in the same code as the `#ifdef MSVC` paths that `P2-2` owns.
