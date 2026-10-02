# `P3` — the second pass

Work that came out of re-measuring the `P0`–`P2` record rather than from the original
audit. The audit itself is [`../findings/audit-2024-09.md`](../findings/audit-2024-09.md);
the new defects were filed into the subject chapters. `S` ids are the suggestion numbers
from that round and are kept so the mapping stays traceable.

**None of this is on the critical path** (`1 → 7 → 9 → 10 → {11,12,13} → 16 → 17 → 18`)
and none of it depends on the ownership work, so it can all run alongside.

| Item | Was | Title | Size | Risk | State | Unblocks |
|---|---|---|---|---|---|---|
| `P3-1` | `S1` | widen the verification scope; delete the orphan | 2 files | none | ✅ | honest counters 7, 8c, 14a |
| `P3-4` | `S4` | `using Filter::filter;` in `Mux`, then `-Werror` on core | 1 line + CMake | none | ✅ | `DOD 15`, `P2-16`'s last sub-item |
| `P3-3` | `S3a` | `override` on the four `const`-only filters | 8 sites | none | ❌ | makes `P2-12` compile-checked |
| `P3-7` | `S7` | `if( ${VAR} )` → `if(VAR)` | 4 lines | none | ✅ | a class of silent build breakage |
| `P3-6` | `S6` | `csv_source`: check `fscanf`, validate the pattern | ~10 lines | low | ❌ | 2 `-Wunused-result`, silently corrupt frames |
| `P3-5` | `S5` | `objectTrack`: `system()` → `execv`, or delete | 1 function | low | ❌ | the only `system()` in the tree |
| `P3-9` | `S3` (gate) | a test that a filter's body actually ran | 1 test | none | ❌ | separates "compiles" from "works" |
| `P3-2` | `S2` | delete the dead, installed `toffy/web/` headers | 4 files | API — needs a tag | ❌ | an installed header that cannot compile |
| `P3-8` | `S8` | `toffy_tracking` layering inversion | CMake | low | ❌ | — |
| `P3-10` | — | drop `TOFFY_EXPORT` and the generated export header | ~30 lines | none on the shipped ABI | ❌ | `DOD 18`; one less generated header in the public API |
| `P3-11` | `S9` | CI warning counter, without changing the build | CI | none | ❌ | visibility of the 28 non-core warnings |

`P3-1`, `P3-4`, `P3-3` and `P3-7` are together roughly fifteen lines, carry no behaviour
change, and each one either removes a false green or makes a later change compile-checked.
They are the part of this document worth doing this week. `P3-2` and `P2-12` are API
changes and belong with a tag.

---

## `P3-1` — widen the verification scope, then delete the orphan

Every check in the old `DOD` verification block was scoped to `modules/`. The tree is
bigger, and `libraries/` is not decoration — its object libraries are linked into
`libtoffy.so` (`libraries/CMakeLists.txt:4-8`).

Two consequences, both already measured in [`../counters.md`](../counters.md):

- `libraries/sensor/include/toffy/imagesensor.hpp` still contains the exact `DLLExport`,
  `WIN` and `UNIX` macros that `P2-3` reports deleted. It is a stale pre-`P2-3` copy of
  `modules/bta/include/toffy/io/imagesensor.hpp`, has **no `CMakeLists.txt`**, appears in
  no `add_subdirectory`, is on no include path and is not installed.
- `libraries/` holds **97** debug prints that criterion 5 has never counted — 33 of them
  in `thickTracer8.cpp`.

**Do:** change the greps to `modules/ libraries/ apps/` (done in
[`../counters.md`](../counters.md)), and delete `libraries/sensor/` outright — two
headers, nothing references them. Until then, criteria 7 and 8c report green while the
pattern they hunt is in the tree.

## `P3-2` — delete the dead `toffy/web/` headers

`make install` ships four headers, one of which cannot be included at all:
`libraries/include/toffy/web/common/plugins.hpp` includes
`toffy/web/controllerFactory.hpp`, which exists nowhere in the repository. They are
leftovers of the web control UI removed in `54d9577`, and they are *installed*, i.e. part
of the shipped public API surface. Evidence and the reproduction:
[`../findings/build.md`](../findings/build.md).

Deleting them is an API removal, so `../dod/stage-gates.md` §2.4 applies (tag +
`tools/api_change_report.sh`) even though the risk is close to nil. It also removes the
need for the `use.dox` rewrite in the same area — the page and the headers describe the
same dead product.

## `P3-3` — make the four `const`-only filters `override` first

`P2-12` says "delete the `const` overload from the base and keep one non-`const` virtual".
Four classes outside core implement **only** the `const` form, and three of them do not
say `override`, so deleting the base overload turns them into new virtuals that override
nothing: they keep compiling and stop running. Evidence, including the minimal
reproduction: [`../findings/api.md`](../findings/api.md).

**Commit 1 (this item):** drop `const` from the four declarations and four definitions in
`modules/filters` and add `override` to all of them. Nothing changes at runtime — the
base's delegation resolves to the same body — and it turns any missed site from a silent
behaviour change into a compile error.
**Commit 2:** `P2-12` proper.

## `P3-4` — `using Filter::filter;` in `Mux`, then `-Werror` on core

All three of core's `-Woverloaded-virtual=` warnings name the same hiding declaration in
`Mux`, not the `P2-12` design trap. One line takes core to **0 warnings** — control and
treatment measured in [`../findings/api.md`](../findings/api.md).

Land the `using` line on its own, then enable `-Werror` for `modules/core`. This does
**not** fix the design trap (a `const Filter&` still gets `false` from `FilterBank` and
`ParallelFilter`), so `P2-12` stays open — what changes is that the warning gate and the
API fix stop being one indivisible job, and that the thing which stops the count growing
becomes available now rather than after four structural PRs.

## `P3-5` — `objectTrack`: stop passing configuration to a shell

`system()` on a string assembled from an XML `<options/script>` value, with a trailing `&`
inside the string so it does not even wait. Any shell metacharacter in a config file runs
with the process's privileges. Evidence: [`../findings/security.md`](../findings/security.md).

Replace with `posix_spawn` or `fork`+`execv` on an argv array, or delete the feature.
`objectTrack` is in `modules/filters`, outside the programme's stated scope, but this is a
one-function change and the only `system()` call in the tree.

## `P3-6` — `csv_source`: check the return values

A config string is used as a `snprintf` format (undefined behaviour if it is not a valid
format string, and `-Wformat` cannot help because it is not a literal), and two `fscanf`
calls discard their results so a short or malformed CSV writes uninitialised values into
the image and reports success. Evidence: [`../findings/security.md`](../findings/security.md).

Validate the pattern once at config time and check the `fscanf` return. Small, local, and
it turns a silently corrupt frame into a logged error.

## `P3-7` — `if( ${VAR} )` → `if(VAR)` — **DONE**

The bug class that hid the PCL-less build failure was fixed on the PCL axis but never
swept. Four sites, all now `if(VAR)`: `bta_FOUND`, `BUILD_TESTS`, and two
`OPENCV_TRACKING_FOUND`. `MATCHES` comparisons (`if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")`)
are correct as written and stayed.

The CI line went in too, scoped more tightly than originally suggested — over the repository
root the suggested grep reports `build/generated/toffyConfig.cmake`, a generated file, so the
shipped check covers only the tracked CMake files and excludes `MATCHES`/`STREQUAL`. It was
verified to fail on a re-introduced site. Evidence, the measured CMake behaviour and the
reason the four sites sat unnoticed: [`../findings/build.md`](../findings/build.md) N6.

## `P3-8` — the `toffy_tracking` layering inversion

`libraries/CMakeLists.txt` lists `$<TARGET_OBJECTS:toffy_tracking>`, a target defined in
`modules/filters/src/tracking/`, and `add_subdirectory(libraries)` runs *before*
`add_subdirectory(modules)`. It works because generator expressions resolve at generate
time, but the dependency points downwards. Same inversion as `P2-9`, one layer below it —
record it as a `P2-9` follow-on.
[`../findings/build.md`](../findings/build.md).

## `P3-9` — a test that a filter's body actually ran

Add to `../dod/stage-gates.md` §2.4: *a filter that overrides only one `filter()` overload
is exercised by a test that asserts its body ran.* One test, and it is the only thing in
the gate that distinguishes "compiles" from "works" — the current API gate passes while
three filters silently stop processing frames.

## `P3-10` — drop `TOFFY_EXPORT`

`P2-3` replaced the vestigial `DLLExport` with `TOFFY_EXPORT`, "the real macro from
`generate_export_header`". It is also vestigial, for a reason that is harder to see
because it *is* wired up correctly-looking:

- Nothing in the build ever compiles with `toffy_EXPORTS` defined. `libtoffy.so` is
  assembled from object libraries (`toffy_core`, `toffy_filters`, `toffy_lib_*`, …) plus
  `$<TARGET_OBJECTS:>`, and the `toffy` target itself has no sources — so the
  "we are building this library" branch is never taken by any translation unit.
- Visibility is default everywhere: no `-fvisibility=hidden` anywhere in the build, and
  the generated header emits `__attribute__((visibility("default")))` on **both** sides of
  the `toffy_EXPORTS` test. On the only supported platform the macro is a no-op.
- On MSVC it is worse than a no-op: with no TU defining `toffy_EXPORTS`, every annotated
  class in the library would be compiled as `__declspec(dllimport)`.
- It drags a *generated* header into the public include path — 8 public headers do
  `#include <toffy/toffy_export.h>`, so consuming `toffy/frame.hpp` requires CMake's
  generated dir, and `make install` has to ship a build artifact to keep the rest usable.

**Do:** delete the 27 `TOFFY_EXPORT` uses and the 8 includes, drop
`include(GenerateExportHeader)` and the `GENERATE_EXPORT_HEADER()` call, and keep
`${CMAKE_CURRENT_BINARY_DIR}/generated/` on the include path — `toffy_config.h` is
generated there too and is genuinely used.

**Gate:** the exported symbol set of `libtoffy.so` must be byte-identical before and after
(`readelf --dyn-syms`), which is what makes this a no-op on the shipped ABI rather than an
assertion. `tools/api_change_report.sh` will still exit 1, because installed headers
changed; see the note in that item about the tag decision.

## `P3-11` — a CI warning counter that does not change the build

Nothing outside core measures warnings: a full build emits 28 warnings today and nobody
looks. Print `grep -c warning:` per target on every PR and fail only if `modules/core`
goes above 0 — core is already held there by `-Werror` (`P3-4`), so what this item adds is
visibility for the other 28 instead of silently re-emitting them.
[`../findings/audit-2024-09.md`](../findings/audit-2024-09.md) has the breakdown; the
`-Wmaybe-uninitialized` cluster in the K3M skeletonizers is the part worth a human look
before it is dismissed.
