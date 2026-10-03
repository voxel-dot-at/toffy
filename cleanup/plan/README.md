# The plan

The numbered work. Findings and evidence live in [`../findings/`](../findings/), the order
in [`../order.md`](../order.md), the gates in [`../dod/`](../dod/).

## Ground rules

1. **Fix bugs before refactoring.** Several `P0` items were one-line changes that would
   have been lost in a larger restructure.
2. **One commit per numbered item.** Most are independent and revertable in isolation.
3. **The failing test comes first.** A fix PR shows its test failing on the parent commit
   in the PR body. This has been enforceable since `P1-1` landed and every fix PR so far
   has done it; a test that has never been observed to fail is not a regression fence.
4. **Never mix a public rename with a behaviour change in one commit.** Keeps `git bisect`
   and review readable.
5. **Docs change in the same PR as the behaviour** (`../dod/per-pr.md` #6). Several
   findings exist because the doc comment and the code disagreed.
6. **Numbers are quoted from [`../counters.md`](../counters.md)**, with their scope.

## Priority bands

| Band | Meaning |
|---|---|
| `P0` | correctness, no API change. Defects, not preferences |
| `P1` | small, behaviour-visible, low risk — the test harness |
| `P2` | structural. Only after `P0` merged and `P1-1` had a fence |
| `P3` | second pass: findings from re-measuring the `P0`–`P2` record, plus what the wider scope exposed |

## Items

### `P0` — correctness ([`p0-correctness.md`](p0-correctness.md))

| Item | Title | State |
|---|---|---|
| `P0-1` | `FilterBank::stop()` calls `stop()`, not `start()` | ✅ |
| `P0-2` | `remove(size_t)`: `_pipe.begin() + i`, not `end() + i` | ✅ |
| `P0-3` | `remove(string)`: bounds-check, return real status | ✅ |
| `P0-4` | `findPos()`: `std::optional<int>`, not `size_t -1` | ✅ |
| `P0-5` | initialise all `Filter` members | ✅ |
| `P0-6` | `Frame`: copy/assign/clear `meta` + `desc` consistently | ✅ |
| `P0-7` | null-check `Event::data()` | ⚫ obsolete — `Event` was deleted under `P2-14` |
| `P0-8` | null-check after `fn()` in `createFilter()` | ✅ |
| `P0-9` | null-check `baseFilterBank` in `~Controller` | ✅ defensive; `clearCreators()` left to `P2-5` |
| `P0-10` | `creators.find()` instead of `operator[]` | ✅ |
| `P0-11` | define `Player::stop()` | ✅ |
| `P0-12` | guard `pt.get_child(_type)` | ✅ |

### `P1` — harness ([`test-harness.md`](test-harness.md))

| Item | Title | State |
|---|---|---|
| `P1-1` | `enable_testing()`, `ctest` targets, CI that runs them | ✅ |

### `P2` — structural

| Item | Title | File | State |
|---|---|---|---|
| `P2-2` | fix or delete the Windows paths | [`build-and-hygiene.md`](build-and-hygiene.md) | ❌ |
| `P2-3` | debug leftovers and dead code | [`build-and-hygiene.md`](build-and-hygiene.md) | 🟡 core done |
| `P2-4` | `createFilter()` → `registerCreator()` | [`structure.md`](structure.md) | ❌ |
| `P2-5` | one owner per `Filter` | [`structure.md`](structure.md) | ❌ **keystone** |
| `P2-6` | collapse the four run methods | [`structure.md`](structure.md) | ❌ |
| `P2-7` | make the threading honest | [`structure.md`](structure.md) | ❌ |
| `P2-8` | logging policy out of `Filter` | [`structure.md`](structure.md) | ✅ |
| `P2-9` | module layering | [`structure.md`](structure.md) | ❌ |
| `P2-10` | consolidate the three plugin loaders | [`structure.md`](structure.md) | ❌ |
| `P2-11` | `Frame` accessor API | [`api.md`](api.md) | ❌ |
| `P2-12` | the `filter()` const/non-const overload trap | [`api.md`](api.md) | ❌ see `P3-9` |
| `P2-13` | unify run state, rename the typos | [`api.md`](api.md) | ❌ |
| `P2-14` | finish or delete the `Event` stub | [`api.md`](api.md) | ✅ deleted |
| `P2-15` | enforce formatting | [`build-and-hygiene.md`](build-and-hygiene.md) | ❌ |
| `P2-16` | build flags | [`build-and-hygiene.md`](build-and-hygiene.md) | 🟡 4 of 5 |

### `P3` — second pass ([`second-pass.md`](second-pass.md))

| Item | Title | Size | State |
|---|---|---|---|
| `P3-1` | widen the verification scope; delete the `libraries/sensor/` orphan | 2 files | ✅ |
| `P3-2` | delete the dead, installed `toffy/web/` headers | 4 files | ❌ API |
| `P3-3` | make the four `const`-only filters `override` before `P2-12` | 4 sites | ✅ |
| `P3-4` | `using Filter::filter;` in `Mux`; `-Werror` on core | 1 line | ✅ |
| `P3-5` | `objectTrack`: `system()` on config data | 1 function | ❌ |
| `P3-6` | `csv_source`: check `fscanf`, validate the pattern | ~10 lines | ❌ |
| `P3-7` | `if( ${VAR} )` → `if(VAR)`, 4 sites | 4 lines | ✅ |
| `P3-8` | `toffy_tracking` layering inversion | CMake | ❌ |
| `P3-9` | a test that a filter's body actually ran | 1 test | ✅ |
| `P3-10` | drop `TOFFY_EXPORT` and the generated export header | ~30 lines | ❌ |
| `P3-11` | CI warning counter, without changing the build | CI | ✅ |
