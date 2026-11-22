# Public API: `Frame` accessors, the `filter()` overload, run state, `Event`

### 11 — Fix the `Frame` accessor API — **done**

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

**Landed.** Three of the four items were real; the fourth was not, and the two claims
that turned out to be wrong are worth recording because both were confident.

- **`getSertMatPtr` is not `const` in the current tree.** The item says it "inserts into
  `data`/`meta` from a `const` method"; the declaration is
  `inline matPtr getSertMatPtr(const std::string&, cv::Size, int)` with no trailing
  `const`, and no version of the concurrency question applies. Nothing was done for it.
  A plan item that describes a defect the code does not have is still a defect — in the
  plan — and the fix is to say so rather than to quietly skip it.
- **"nothing in the repo calls it, so a deprecated alias is cheap" was true of toffy and
  false of the stack.** `becom-knapp-wurthbox` calls it twice
  (`src/toffy/colorbox.cpp`, `src/toffy/colorblob.cpp`). So this rename took a
  `[[deprecated]]` alias where `stedBackward` → `stepBackward` deliberately did not.
  That is a reversal of the recorded precedent, and the reason is evidence rather than
  taste: nothing outside toffy called `stedBackward`, so a bare rename broke nobody;
  this one had a live caller, so a bare rename would make the next toffy tag fail
  someone else's build over a spelling mistake of ours. boxes has since moved to the
  new name (`boxes@8d49cc2`), and the alias can be deleted at the next major.

The tag fix is the substantive one. `addData(long)` stored a `long` in the `boost::any`
while labelling it `Int`; every getter does `any_cast<T>` on the **real** type, so
`addData(k, 42L)` then `getInt(k)` threw `bad_any_cast` while `getDataType()` answered
`Int`. The tag was decorative. `Long`/`ULong` were added **at the end** of `SlotDataType`
— renumbering existing enumerators would silently re-label data written by an older
build, the same reason `CloudXyz`/`CloudXyzRgb` are not behind `#ifdef PCL`.

Real shape of the bug, in-tree: `modules/filters/src/tracking/tracker.cpp:265` does
`out.addData(_out_count, detObjs.size())`. `size()` is `size_t` = `unsigned long` on LP64,
so it went through the `unsigned long` overload and was tagged `Uint`. Nothing in toffy or
either consumer reads that slot back today, which is the only reason this is a docs
finding rather than a crash report.

`abi_surface.sh` reports no change at 554 symbols: the new getters and the alias are all
inline in the header, so nothing new is exported. Counter 46.

### 12 — Resolve the `filter()` const/non-const overload trap — **done**

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

**Both fences now exist.** `P3-3` put `override` on the four `const`-only filters in
`modules/filters`, so deleting the base's `const` overload fails to compile at those four
sites instead of silently dropping them out of the pipeline. `P3-9` added
`tests/test_filter_overloads.cpp`, which asserts a filter's *body* ran for each override
shape — including a negative control proving a filter that overrides nothing is detected as
not running. Expect this to be a compile-error-driven change with a test that has to be
updated deliberately, not a silent one; run the `DOD 2.4` gate, not just a build.

**Landed, in the two commits `N2` asked for.** Step 1 (`661424d`) dropped `const` from the four
classes; step 2 deleted the base's `const` overload. Both predictions came true — the only
fallout was `tests/test_filter_overloads.cpp`, which failed exactly where designed, and the
change was compile-error-driven rather than silent.

**The plan's own gate was not enough, though.** `abi_surface.sh` reported 4 changes (step 1's
qualifier renames) and said nothing about the thing that actually mattered: deleting the base's
`const` virtual shifts every later virtual down a slot, and the tool filters weak symbols out, so
it cannot see a vtable at all. Honouring `X6`'s plug-in-ABI promise needed a reserved slot in
place of the deleted virtual, which this plan did not anticipate. See
[`../findings/api.md`](../findings/api.md) N4 and `DOD 2.4` rule 3. Counters 44, 45.

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
| `Controller::CERROR` | `ERROR` | `controller.hpp:63` — open |
| ~~`Controller::stedBackward()`~~ | ~~`stepBackward()`~~ | **done**, `b397f44` — `controller.hpp:115` declares `stepBackward()` and the old name is gone from the tree |
| `Frame::getSertMatPtr()` | `getOrInsertMatPtr()` | `frame.hpp:299,409` — open |

Leave a deprecated inline alias for each for one release.

**The alias rule was not followed for `stedBackward`, deliberately** (`b397f44`): an alias
keeps in the public surface of the class the very thing the item exists to remove, and it
would not have preserved the ABI anyway — the alias is *added* and the old name stays,
which is not a reduction. This is the branch where breaking is allowed, and the break is
the loud kind: `nm -D` shows `- toffy::Controller::stedBackward()` /
`+ toffy::Controller::stepBackward()` and nothing else. Nothing in the repository called
either name. Whoever does the other two inherits the same choice, and `getSertMatPtr` is
the one to think about first: it is `inline` in a public header, so it has no exported
symbol to watch and an out-of-tree caller cannot be found with `nm`.

### 14 — Finish or delete the `Event` stub — **DONE (deleted)**

Deleted rather than implemented. `toffy::Event`, `event.hpp`, `event.cpp`,
`Filter::processEvent()` and `FilterBank::processEvent()` are gone. Verified before
removal that nothing outside `modules/core` referenced them: no use in
`modules/filters`, `modules/bta`, `apps/` or `tests/`. The `__infoEvent` XML tags in
`apps/configs/` and the `infoEvent` field in `bta.xml` are unrelated BTA driver callback
fields and were left alone.

This also removed a latent null dereference that was never catalogued: when the receiver
was a single `FILTER`, `FilterBank::processEvent()` did `ff->getFilter(e.receiver())` and
called through the result without a null check, so an event naming an unknown filter
faulted.

**This is a breaking change**, unlike the rest of the programme so far: a public virtual
was removed from `Filter`, so the vtable loses a slot, and an installed header is gone.
`tools/api_change_report.sh v1.7.1` exits 1. Out-of-tree filters that override
`processEvent` will not compile, and any plugin built against the old header is ABI-broken.
See the version-tag note in the commit.

**Original rationale, kept for the record.** Everything below described the stub as live;
the class is gone, so none of it is actionable any more.

`event.hpp` carried `@todo Implement the event logic`, and the class was not used by the
run loop at all. While it stayed:

- `data()` returned `*_data` with no null check — calling it before `data(any)` was set
  dereferenced null (A8).
- The default `Event()` left `_re_type` and `_sender` uninitialised.
- `receiver()` and `event()` returned `std::string` by value; should have been `const&`.
- `receiverType()` was not `const`.
- `Filter::processEvent` logged "Filter does not have events declared" at `info` for every
  unhandled event — log spam the moment events were used.

The plan was: either implement the bus (typed payloads, `std::function` subscribers, no raw
`Filter*` sender) or remove the class until there is a consumer. A half-implemented event
system in a header every filter includes is worse than none. **Delete was chosen.** If a
real event bus is ever wanted, start from that sentence rather than resurrecting the
deleted code.
