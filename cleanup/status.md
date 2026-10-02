# Status

What has landed, what is in flight, and what the release situation is. Counters are in
[`counters.md`](counters.md); the order the rest goes in is in [`order.md`](order.md).

State as of `569f071` (`feature/code-cleanup`), re-checked against the tree rather than
carried over from the previous round.

## Landed

| PR | Item | State | Evidence |
|---|---|---|---|
| **1** | test harness + CI | ✅ done | `enable_testing()` (`CMakeLists.txt:436`), **8** `add_test()` targets, `.github/workflows/ci.yml` (PCL matrix + ASan/UBSan/LSan job) |
| **2** | build flags | ✅ done | C++17 bumped; global `-O2` gone (Debug → `-O0`, Release back to `-O3`); redundant `-Wall` gone; dead `BOOST_VERSION` branch gone; `-Werror` on `toffy_core` via `P3-4` (`option(CORE_WERROR ON)`) |
| **3** | `stop()` / `remove()` / `findPos()` | ✅ done | `std::optional<int> findPos`, `_pipe.begin() + i`, `stop()` calls `stop()`; 4 tests pin it |
| **4** | member init + null checks | ✅ done | in-class initialisers in `filter.hpp`; `Controller` ctor throws |
| **5** | `Frame` metadata | ✅ done | `operator = default`; all three maps handled together |
| **6** | `creators.find()`, `Player::stop()`, guarded `get_child` | ✅ done | `Player::stop()` defined in `player.cpp` |
| **7** | debug leftovers, `modules/core` | ✅ core done | 0 prints, 0 `#warning` in core. **128 remain in `modules/filters`/`modules/bta`, 97 more in `libraries/`** |
| **7a** | dead macros | ✅ done | 22 `DLLExport` deleted, `WIN`/`UNIX` gone, `RAWFILE` 3 headers → 1. Side effect: 7 of the 19 `MSVC` branches went with them |
| **15** | logging policy | ✅ done | `setLoggingLvl()` is data-only; 3 hot-path calls removed; `Filter::setGlobalLogLevel()` added; `~Player` no longer kills process logging. 7 new tests, 4 of them failing pre-fix |
| **17** | delete `Event` | ✅ done | `event.hpp`/`event.cpp` gone; tagged `v1.10.0` — **local only, not pushed**, and it points at `7fea594`, not the branch tip |
| **19** | `P3-1` verification scope + `libraries/sensor/` orphan | ✅ done | 2 unreferenced headers deleted; counters 7a 3→0, 8c 2→0, 14a 16→15, 18 8→7. Counter 21's command was also broken (grepped `_pattern`, the variables are `_depthPattern`/`_amplPattern`) |
| **20** | `P3-4` `using Filter::filter;` + `-Werror` on core | ✅ done | core 3 → **0** warnings in all four configurations; `toffy_core` builds `-Werror`; gate proven by injecting an unused variable into `filter.cpp` and watching the build stop; tree 32 → 28 warnings |
| **21** | `P3-7` `if( ${VAR} )` sweep + CI check | ✅ done | 4 sites → 0; `cmake-hygiene` job added, scoped to the 29 tracked CMake files, verified red on a re-introduced site and green on the fixed tree |

**Ten of the eighteen PRs are done** (1, 3, 4, 5, 6, 15, 17, 19, 20, 21), and PR 2 is now
complete rather than 4-of-5. That is the whole `P0` block, the test/CI fence, the build-flag
cleanup and core's warning gate: the critical path's first stretch, plus the three cheap
second-pass items that removed a false green, closed a class of silent build breakage and
made a later change compile-checked.

`P1`/`P2` items closed: **4 of 16** (`P1-1`, `P2-8`, `P2-14`, `P2-16`), with `P2-3`
core-only. `P2-16` closed when `P3-4` landed the `-Werror` half of it.

## Open

| PR | Item | State |
|---|---|---|
| **8** | `P2-2` Windows paths — fix or delete | ❌ 12 `MSVC` branches in `modules/`, 15 tree-wide |
| **9** | `P2-4` factory → registration | ❌ 37 branches |
| **10** | `P2-5` one owner per `Filter` | ❌ the keystone |
| **11** | `P2-6` + `P2-13` run methods, state, typos | ❌ |
| **12** | `P2-7` threading | ❌ `bool keepRunning`, 2 `interprocess` uses |
| **13** | `P2-9` module layering | ❌ `imageview.hpp` still included by `controller.cpp` |
| **14** | `P2-10` three plugin loaders | ❌ all three still exist |
| **16** | `P2-11` + `P2-12` `Frame` API, `filter()` overload | ❌ see `P3-9` before touching `P2-12` |
| **18** | `P2-15` formatting | ❌ 11 of 20 core files have hard tabs; clang-format 18.1.3 **is** installed |

The second-pass work (`P3-1` … `P3-11`) is listed in
[`plan/second-pass.md`](plan/second-pass.md). Three of its four cheap items are done (`P3-1`,
`P3-4`, `P3-7`); `P3-3` is the remaining one — three lines, no behaviour change, and it
makes `P2-12` compile-checked. `P3-2` and `P2-12` are API changes and belong with a tag.

## Gates

- **`DOD 1.1` green: all four dependency configurations build and pass.** PCL on/off ×
  BTA on/off, each configured from scratch, built and run — 8/8 in every cell. Both axes
  were broken before this branch: PCL-off on unguarded typedefs and on CMake `if()`
  clauses that expanded to nothing, BTA-off on an unconditional `add_subdirectory(bta)`.
- **CI builds and tests on every push and PR.** Two limits when citing a green run:
  runners cannot have the proprietary bta SDK, so only the **BTA-off** axis is covered
  automatically, and **nothing builds the Doxygen docs**, so broken `\ref`s and `\todo`
  drift stay invisible.
- **`A23` (no consistent error convention) blocks further correctness work** in config
  loading — see [`findings/correctness.md`](findings/correctness.md).

## Release / tag situation

- `v1.10.0` is tagged (annotated) for the `Event` removal and the vtable change it caused.
  Before it this tree built `libtoffy.so.1.7.1` while exporting none of 1.7.1's symbols —
  the SONAME advertised a compatibility it did not have.
- The tag is **local; it has not been pushed**, and it points at `7fea594` rather than the
  branch tip. Anything shipped from here needs the tag moved or re-cut.
- It is `1.10.0` rather than the convention-implied `1.8.0` so that it does not sort below
  the `v1.9.0` that exists on `origin/next`. Those two numbering lines still need
  reconciling when the branches meet.
- `SOVERSION` is the **full** version, not the major alone, so *every* patch bump changes
  the SONAME and forces a downstream relink. Version decisions here are never routine —
  see [`dod/stage-gates.md`](dod/stage-gates.md) §2.4.
