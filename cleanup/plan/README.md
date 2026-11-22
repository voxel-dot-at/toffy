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
| `P2-2` | fix or delete the Windows paths | [`build-and-hygiene.md`](build-and-hygiene.md) | ✅ deleted — POSIX only |
| `P2-3` | debug leftovers and dead code | [`build-and-hygiene.md`](build-and-hygiene.md) | 🟡 core done |
| `P2-4` | `createFilter()` → `registerCreator()` | [`structure.md`](structure.md) | ✅ **landed** — 0 branches, 35 includes → 2; forced `~Controller`'s `clearCreators()` out (finding `C3`) and found three type names the chain could never resolve |
| `P2-5` | one owner per `Filter` | [`structure.md`](structure.md) | ✅ **the keystone — landed, and it is why `v2.0.0` exists** |
| `P2-6` | collapse the four run methods | [`structure.md`](structure.md) | ❌ |
| `P2-7` | make the threading honest | [`structure.md`](structure.md) | ✅ landed as PR 37 (`b9e3032`) — `std::thread`/`mutex`/`condition_variable`/`atomic`, `interprocess_semaphore` gone, and `<boost/thread.hpp>` no longer reaches a consumer |
| `P2-8` | logging policy out of `Filter` | [`structure.md`](structure.md) | ✅ |
| ✅ `P2-9` | module layering | **0** core includes from a module above it (allowlisted BTA hook excepted); 0 header-defined `const std::string`; 0 registry-backed `FilterBank` lookups; 0 public `Controller` data members | ✅ |
| ✅ `P2-10` | consolidate the three plugin loaders | **1** loader (was 3), 0 discarded `dlopen` handles, 0 loaders throwing on a missing `<plugins>` node | ✅ |
| ✅ `P2-11` | `Frame` accessor API | **0** defects left (was 3 real + 1 already-not-true); `SlotDataType` gains `Long`/`ULong` at the end; `abi_surface.sh` unchanged at 554 | ✅ |
| ✅ `P2-12` | the `filter()` const/non-const overload trap | **1** virtual `filter()` (was 2); `Filter`'s vtable slots pinned at 2/3/7 by a test; `abi_surface.sh` re-blessed at 554 symbols with 4 qualifier renames | ✅ |
| `P2-13` | unify run state, rename the typos | [`api.md`](api.md) | ◐ one of the three renames landed (`stedBackward` → `stepBackward`, `b397f44`); `CERROR`, `getSertMatPtr` and the state unification are open |
| `P2-14` | finish or delete the `Event` stub | [`api.md`](api.md) | ✅ deleted |
| `P2-15` | enforce formatting | [`build-and-hygiene.md`](build-and-hygiene.md) | ❌ |
| `P2-16` | build flags | [`build-and-hygiene.md`](build-and-hygiene.md) | ✅ — the `-Werror` half landed as `P3-4` |
| `P2-17` | `toffy::log`, so the logging backend is a choice | [`logging.md`](logging.md) | ◐ **new item, step A landed** — 658 `BOOST_LOG_TRIVIAL` sites → 0, no installed header includes `boost/log`, no consumer symbol names a Boost type. log4cplus backend and `boxes`' own 378 sites still open |
| `P2-18` | Boost.Filesystem out of `libtoffy` | [`../findings/build.md`](../findings/build.md) | ✅ **landed** — `libtoffy.so` has no `libboost_filesystem` in `NEEDED` (was 1) and no installed header includes `boost/filesystem` (was 1). Nine shipped files included it; seven used nothing from it. `filesystem` out of `find_package(Boost)` |

### `P3` — second pass ([`second-pass.md`](second-pass.md))

| Item | Title | Size | State |
|---|---|---|---|
| `P3-1` | widen the verification scope; delete the `libraries/sensor/` orphan | 2 files | ✅ |
| `P3-2` | delete the dead, installed `toffy/web/` headers | 4 files + 2 sources | ✅ API, tag owed |
| `P3-12` | make every installed header compile; gate it | 2 headers + tool + CI | ✅ |
| `P3-3` | make the four `const`-only filters `override` before `P2-12` | 4 sites | ✅ |
| `P3-4` | `using Filter::filter;` in `Mux`; `-Werror` on core | 1 line | ✅ |
| `P3-5` | `objectTrack`: `system()` on config data | 1 function | ✅ |
| `P3-6` | `csv_source` + `exportcsv`: check `fscanf`, validate the pattern | 3 files | ✅ |
| `P3-7` | `if( ${VAR} )` → `if(VAR)`, 4 sites | 4 lines | ✅ |
| `P3-8` | `toffy_tracking` layering inversion | CMake | ❌ |
| `P3-9` | a test that a filter's body actually ran | 1 test | ✅ |
| ✅ `P3-10` | drop `TOFFY_EXPORT` and the generated export header | **0** annotation sites (was 16 in 15 headers), **0** `toffy_export.h` includes (was 6), and `toffy_export.h` is no longer installed — 88 → 87 installed headers. Gate: `readelf --dyn-syms` byte-identical, 2499 symbols | ✅ |
| `P3-11` | CI warning counter, without changing the build | CI | ✅ |
| `P3-13` | delete the Windows scaffolding `P2-2` left unreachable in the build | CMake | ❌ |

### `X` — the controller extraction ([`controller-extraction.md`](controller-extraction.md))

A separate id space, because it is a decision about where a product lives rather than a
cleanup item: the web control UI is now `toffy-oatpp`'s, and these are the stages of moving
what is left of it out of this repository. `X1` is `P3-2` and `X2` is `P3-12`; `X2b` and
`X3`–`X6` have no `P` number because they were only identified while doing those two.

| Stage | Title | State |
|---|---|---|
| `X1` | delete the four `toffy/web/` headers and the `WITH_CONTROL` hooks (= `P3-2`) | ✅ |
| `X2` | every installed header compiles standalone, and CI keeps it that way (= `P3-12`) | ✅ |
| `X2b` | guards + a double-inclusion pass (`N9`), and a residue gate for the tree CI cannot build | ✅ |
| `X3` | drop `--host/--port/--html` from `toffyRunner` | ✅ |
| `X4` | rewrite `use.dox` around `toffyRunner`; screenshots deleted; page un-excluded | ✅ |
| `X5` | the C++ API `toffy-oatpp` binds to, and the five items it waits on | ❌ |
| `X6` | plugin ABI: keep the filter half, do not rehost the controller half | decision, taken |
