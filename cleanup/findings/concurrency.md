# D — Concurrency

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
