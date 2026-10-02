# B — API & design

**Two parallel state machines.** `filterState` (`filter.hpp:52`) and `Controller::state`
(`controller.hpp:53-60`) both model run state, with no mapping between them.

**The run loop depends on enum ordering.** `while (_state > Controller::IDLE)`
(`controller.cpp:203,219`). Because `CERROR = 0xff` is the largest enumerator, entering the
error state keeps the loop spinning instead of exiting.

**`filter()` const/non-const overload trap.** `Filter` declares both a `const` and a
non-`const` virtual `filter()`, and the `const` one just `return false`. `FilterBank` and
`ParallelFilter` override only the non-`const` variant, so anything holding a
`const Filter&` silently gets `false`.

Two things make this harder to fix than it looks, and both are measured rather than
opinion: four filters outside core implement **only** the `const` form, three of them
without `override`, so deleting the base overload makes them compile and stop running
(**N2** below → `P3-3`, `P3-9`); and the three warnings core emits are *not* the trap, they
are `Mux` hiding the base name (**N3** below → `P3-4`). Fixing the warning and fixing the
design are separate jobs.

**`FilterBank::getFilter(name)` ignores its own pipeline.** It delegates to the global
factory instead of `_pipe`, so a bank hands out filters it does not contain. The header
already flags this as deprecated, and `P2-5` is what makes the rewrite possible.

**`Filter` does four jobs** in one 407-line header: processing interface, config plumbing,
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

---

## From the second-pass audit

Second-pass audit, measured on `e943a37`. The audit record itself — the re-verification table and the scope bug behind the two false zeros — is in [`audit-2024-09.md`](audit-2024-09.md).

### N2 — `P2-12` as written would silently disable three filters

[`../plan/api.md`](../plan/api.md) `P2-12` says: *"delete the `const` overload from the
base and keep one non-`const` virtual"*. Four classes outside core implement **only** the `const` form:

| class | declaration | uses `override`? | outcome if the base `const` overload is deleted |
|---|---|---|---|
| `CloudViewOpenCv` | `viewers/cloudviewopencv.hpp:35` | **yes** | compile error — loud, safe |
| `OffSet` | `base/offset.hpp:48` | no | **compiles, stops running** |
| `Transform` | `3d/transform.hpp:49` | no | **compiles, stops running** |
| `Merge` | `3d/merge.hpp:44` | no | **compiles, stops running** |

None of the four declares a non-`const` `filter()`. Today they work through the base's
delegation (`filter.hpp:196`: the non-`const` virtual calls `constF.filter(in, out)`). Delete
the `const` overload and their declaration becomes a *new* virtual that overrides nothing, so
a call through `Filter*` lands on the base default — `return false`.

Reproduced, minimal, with the repository's own flags:

```cpp
struct Filter { virtual bool filter(const Frame&, Frame &out) { out.v = -1; return false; } };
class OffSet : public Filter {                       // const-only, no `override`
public: virtual bool filter(const Frame&, Frame &out) const { out.v = 42; return true; }
};
Filter *f = new OffSet(); Frame in, out; bool r = f->filter(in, out);
```

```
$ g++ -std=c++17 -Wall -Wextra p212.cpp -o p212 && ./p212
  -> Filter::filter() base ran (returns false)
OffSet through Filter*: returned 0, out.v = -1
```

The filter does not run. `-Woverloaded-virtual=` does emit *"‘Filter::filter(...)’ was hidden"*,
but `modules/filters` is not warning-gated, so nothing stops the build. `-Wsuggest-override`
does **not** catch it (verified — no diagnostic).

`DOD 2.4`'s gate for API-changing PRs is *"`modules/filters`, `modules/bta` and `apps/` all
still compile"*. That gate passes while three filters silently stop processing frames, which is
the exact failure mode `P2-12` exists to eliminate.

**Suggestion S3 — split `P2-12` into two commits, and change the gate.**

1. First commit, mechanical and safe: drop `const` from the four declarations and four
   definitions in `modules/filters`, and add `override` to all of them. This changes nothing at
   runtime (the delegation still resolves to the same body) and it makes the second commit a
   compile error rather than a behaviour change if any site is missed.
2. Second commit: delete the `const` overload from `Filter`, per the plan.

Add to `DOD 2.4`: *"a filter that overrides only one `filter()` overload is exercised by a
test that asserts its body ran"* — one test, and it is the only thing that distinguishes
"compiles" from "works" here.

### N3 — ~~core can be warning-free today, without waiting for `P2-12`~~ **DONE (`P3-4`)**

Landed: the `using Filter::filter;` line is in `mux.hpp`, core measures 0 warnings in all
four dependency configurations, and `toffy_core` compiles `-Werror` behind
`option(CORE_WERROR ON)`. The control/treatment table below is kept because it is the
evidence that the one line — not `P2-12` — was what the warnings were reporting. One
addition to the measurement: the same line also silenced a **fourth** warning outside core,
in `modules/filters/include/toffy/3d/muxMerge.hpp`, so a full default build went from 32
warnings to 28.

The `DOD` criterion 15 note (0 warnings, `-Werror` on core) gated itself on `P2-12` and
described the three warnings as *"all one root cause: the `filter()` const/non-const overload
trap"*. The root cause is narrower than that, and reachable today — which is `P3-4`. All three warnings name the same class:

```
filter.hpp:177: warning: ‘virtual bool toffy::Filter::filter(const Frame&, Frame&) const’
    was hidden by ‘toffy::Mux::filter’        [mux.cpp, parallelFilter.cpp, filterfactory.cpp]
```

`Mux` (`mux.hpp:63`) declares `filter(const std::vector<Frame*>&, Frame&)`, a different
signature, which hides the base's overload set. One line fixes it:

```cpp
class Mux: public Filter {
public:
    using Filter::filter;          // <-- add this
```

Control and treatment, same command, only the one line differing:

```sh
FM=build/modules/core/src/CMakeFiles/toffy_core.dir/flags.make
FLAGS=$(grep '^CXX_FLAGS'   $FM | cut -d= -f2-)
DEFS=$(grep '^CXX_DEFINES'  $FM | cut -d= -f2-)
INCS=$(grep '^CXX_INCLUDES' $FM | cut -d= -f2-)
for f in mux parallelFilter filterfactory; do
  g++ $FLAGS $DEFS $INCS -Werror -fsyntax-only modules/core/src/$f.cpp; echo "$f -> $?"
done
```

| | `mux.cpp` | `parallelFilter.cpp` | `filterfactory.cpp` |
|---|---|---|---|
| as-is | **fail** | **fail** | **fail** |
| + `using Filter::filter;` in `mux.hpp` | pass | pass | pass |

Full `make toffy_core` after the line: **0 warnings** (was 3).

**Suggestion S4 — land the `using` line as its own PR and turn on `-Werror` for
`modules/core`.** It is one line, not an API change, not an ABI change, and it closes DOD
criterion 15 without untangling `P2-12`. It does **not** fix the design trap — a `const
Filter&` still gets `false` from `FilterBank`/`ParallelFilter`, which are the classes that
actually need `P2-12` — so keep `P2-12` open. What changes is that the warning gate and the API
fix stop being one indivisible job, and that `-Werror` — the thing that stops the count growing
— becomes available now rather than after four structural PRs.
