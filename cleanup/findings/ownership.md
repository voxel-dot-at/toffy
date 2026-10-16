# C — Ownership & memory

`P2-5` (PR 36) fixed the ownership half of this chapter and left the rest alone. The
original wording is kept where it still describes open work, and marked where it does not,
because the plan and the commit messages cite these entries by name (`C1`, `C3`, `A17`).

**Three owners, one raw pointer — FIXED (`P2-5`).** `FilterFactory::_filters` (a global
static map) and `FilterBank::_pipe` both held raw `Filter*`. `FilterBank::add()` took a raw
pointer, and `~FilterBank` destroyed filters *via the factory, by name*. A filter that was
renamed, or that was added to two banks, leaked or was double-freed.

Now: `_pipe` is a `std::vector<FilterPtr>`, the registry is
`flat_map<std::string, std::weak_ptr<Filter>>`, `add()`/`insert()` take a `FilterPtr`, and
`createFilter()` returns the filter's only strong reference. Nothing calls `delete` on a
`Filter` (counter 13) and nothing looks one up by name in order to destroy it (counter 13a).
Both failure modes are pinned by `tests/test_ownership.cpp`: `RenamedFilterIsReleasedWithTheBank`
and `FilterInTwoBanksSurvivesTheFirstOne` fail on the pre-`P2-5` tree, the second of them with
an ASan heap-use-after-free.

**`clearBank()` deletes by name — FIXED (`P2-5`).** It looked each filter up by `name()` in
the global factory in order to destroy it — name-keyed deletion of pointer-owned state. Names
are mutable (`Filter::name()` is public and `updateConfig()` writes it from `<name>`), so this
was not a stable identity, and the lookup was done with the *new* name against a registry
keyed by *id*. `clearBank()` is now `_pipe.clear()`. The reachable-through-a-config-file
version of this is `FilterOwnership.PlayerSessionReleasesEverythingOnShutdown`, which pre-fix
leaves exactly one filter alive: the one the config renamed.

**`~Controller` tears down process-global state — HALF FIXED (`P2-5`).** It used to call
`deleteFilter(baseFilterBank->id())`, then `clearCreators()`. The first half is gone:
`baseFilterBank` is a `std::shared_ptr<FilterBank>` and the teardown is letting it go, which
destroys the bank and everything pipelined into it, in this object, without a registry
round-trip (`ControllerLifecycle.*` still pass, and `P2-5` added the `Player` session test
that exercises the same path through a config file).

**`clearCreators()` is still called**, and `P2-5` is what makes it *optional* rather than
load-bearing: it has nothing to do with ownership any more. Removing it is a behaviour change
(a second `Controller` would start retaining the plug-ins the first registered) and belongs
with `P2-10`, which owns the `dlopen`/`dlclose` lifetime that makes it tempting to keep. It
is currently benign for built-in types: `createFilter()` resolves those through a hard-coded
chain rather than the creator map, so a second `Controller` still works (pinned by
`ControllerLifecycle.SequentialControllersStillWork`). Only *registered* creators — plugins,
`Player::loadFilter()` — are lost, and nothing tests that yet.

**`FilterThread` copies a raw owner — HALF FIXED (`P2-5`).** The thread used to document that
it owns the `Filter` and `delete` it in its destructor while the `FilterBank` that created the
same filter also held it and released it through the factory; `~ParallelFilter` ran the
thread's `delete` first, then `~FilterBank` cleared `_pipe` through the factory. The member is
a `FilterPtr` now and the destructor does not delete, so the two holders share and the object
dies once — `ParallelFilter`'s lanes are still destroyed before the base bank clears `_pipe`,
but that is now just the thread letting go early instead of a double free.

What is left is the **copy ctor**: still declared, still uncompilable if used (`boost::thread`
and `boost::mutex` are not copyable), and still the wrong idea — it would hand a second
`FilterThread` the same filter. Deleting it is `P2-7`'s, and `P2-5` deliberately did not do it
because the copy it describes changed meaning (share, not double-own) rather than going away.

**`FilterThread` queues raw `Frame*`** with no documented ownership, so it is impossible to
tell whether the queue, the producer, or the consumer frees them. Open; `P2-7`.

**A header include is commented out.** `#include <list>` is disabled in `filterThread.hpp`
while `std::list` is used there — the file compiles only through transitive includes. Still
open, and still `P2-7`'s; `P2-5` added the `<memory>` it needs for its own reason and left a
note saying which include belongs to whom.

**Plugin handles are leaked.** `dlopen()` is never paired with `dlclose()`; the calls are
explicitly commented out in both `filterbank.cpp` and `controller.cpp`. Handles are kept
as untyped `std::vector<void*> _loads`, so they cannot be closed correctly even in principle.
Open; `P2-10`. `P2-5` did not touch the loader, and its `~Controller` comment records that
`_loads` is still never drained.

## What the fix cost

`FilterPtr` in the public API is an ABI break, and one part of it is invisible to the tool
that is supposed to catch ABI breaks (`createFilter()` keeps its mangled name while changing
its calling convention). See counter 31 and the *Release* section of
[`../status.md`](../status.md): this is why the tree goes to `v2.0.0` rather than `1.12.0`.
The filter plug-in ABI (`CreateFilterFn`, `init(FilterFactory*)`, `registerCreator`) was kept
on purpose — the `X6` decision.
