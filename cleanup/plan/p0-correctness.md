# P0 — Correctness, no API change

These are defects, not preferences. Each is small and independently verifiable.

| # | Item | Finding | Risk |
|---|---|---|---|
| 1 | ~~`FilterBank::stop()` calls `stop()`, not `start()`~~ **DONE** — fixed, pinned by `FilterBankStop.StopStopsChildren` | A1 | trivial |
| 2 | ~~`remove(size_t)`: `_pipe.begin() + i` not `end() + i`~~ **DONE** — pinned by `FilterBankRemove.ByIndexRemovesTheElementAtThatIndex` | A2 | trivial |
| 3 | ~~`remove(string)`: bounds-check, return real status~~ **DONE** — pinned by `FilterBankRemove.MissingNameLeavesBankIntact` | A4 | low |
| 4 | ~~`findPos()`: return `int` / `optional`, not `size_t -1`~~ **DONE** — returns `std::optional<int>` (C++17); pinned by `FilterBankFindPos.MissingNameYieldsNoValue` | A3 | low |
| 5 | ~~Initialise all `Filter` members in both ctors~~ **DONE** — in-class initialisers; pinned by `FilterConstruction.*` | A6 | trivial |
| 6 | ~~`Frame`: copy/assign/clear `meta` + `desc` consistently~~ **DONE** — `operator=` defaulted; pinned by `FrameMetadata.*` | A5, A14 | low |
| 7 | ~~Null-check `Event::data()`~~ **OBSOLETE** — the whole `Event` class was deleted under `P2-14`, so the null dereference no longer exists. Do not fix in place. | A8 | — |
| 8 | ~~Null-check after `fn()` in `createFilter()`~~ **DONE** — pinned by `FilterFactoryCreate.CreatorReturningNullIsRejected` (segfaulted before the fix) | A21 | trivial |
| 9 | ~~Null-check `baseFilterBank` in `~Controller`~~ **DONE (defensive)** — the reachable fault was in the *constructor*, which called `->bank(NULL)` on an unchecked `createFilter()` result; it now throws. Guarded in the destructor too. `clearCreators()` teardown is **not** fixed (behaviour change, belongs with `P2-5`). Pinned by `ControllerLifecycle.*`, which pass on the parent commit — the null path is unreachable, see the note there. | C3 | trivial |
| 10 | ~~`creators.find()` instead of `operator[]`~~ **DONE** — pinned by `FilterFactoryCreate.FailedLookupDoesNotRegisterTheType` | A19 | trivial |
| 11 | ~~Delete or define `Player::stop()`~~ **DONE** — defined, delegating to `Controller::stop()`; pinned by `PlayerStop.*`. Citation corrected (`A13` → `A16`; `A13` is `Frame::operator=`). Also guarded the unconditional `join()` that defining it would have newly exposed — the joinable() half of `A12`, the rest stays in `P2-6`/`P2-7`. | A16 | trivial |
| 12 | ~~Guard `pt.get_child(_type)` after diagnosing its absence~~ **DONE** — returns `-1`; pinned by `FilterLoadConfig.MissingTypeNodeReportsFailureInsteadOfThrowing` and `MatchingTypeNodeStillSucceeds`. Note: the ignored return value in `instantiateFilter()` was deliberately *not* made fatal — see `A23`. | A7 | low |

## Notes on the non-obvious ones

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
