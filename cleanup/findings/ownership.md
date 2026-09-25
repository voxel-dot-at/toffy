# C — Ownership & memory

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
