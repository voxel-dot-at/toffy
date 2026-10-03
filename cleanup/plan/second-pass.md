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
| `P3-3` | `S3a` | `override` on the four `const`-only filters | 8 sites | none | ✅ | makes `P2-12` compile-checked |
| `P3-7` | `S7` | `if( ${VAR} )` → `if(VAR)` | 4 lines | none | ✅ | a class of silent build breakage |
| `P3-6` | `S6` | config string as `printf` format; check `fscanf` | 2 files | low | 🟡 `csv_source` done, `exportcsv` open | 2 `-Wunused-result`, silently corrupt frames |
| `P3-5` | `S5` | `objectTrack`: `system()` → `execv`, or delete | 1 function | low | ❌ | the only `system()` in the tree |
| `P3-9` | `S3` (gate) | a test that a filter's body actually ran | 1 test | none | ✅ | separates "compiles" from "works" |
| `P3-2` | `S2` | delete the dead, installed `toffy/web/` headers | 4 files | API — needs a tag | ❌ | an installed header that cannot compile |
| `P3-8` | `S8` | `toffy_tracking` layering inversion | CMake | low | ❌ | — |
| `P3-10` | — | drop `TOFFY_EXPORT` and the generated export header | ~30 lines | none on the shipped ABI | ❌ | `DOD 18`; one less generated header in the public API |
| `P3-11` | `S9` | CI warning counter, without changing the build | CI | none | ✅ | visibility of the 26 non-core warnings |

`P3-1`, `P3-4`, `P3-3` and `P3-7` were together roughly fifteen lines, carried no behaviour
change, and each one either removed a false green or made a later change compile-checked.
All four are done, as are `P3-9` and `P3-11`. What is left of this document is the two
`modules/filters` safety items (`P3-6`'s `exportcsv` half and `P3-5`), the mechanical
`P3-10`, the `P3-8` note, and `P3-2` — which, like `P2-12`, is an API change and belongs
with a tag.

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

## `P3-3` — make the four `const`-only filters `override` first — **done**

`P2-12` says "delete the `const` overload from the base and keep one non-`const` virtual".
Four classes outside core implement **only** the `const` form, and three of them did not
say `override`, so deleting the base overload turned them into new virtuals that override
nothing: they kept compiling and stopped running. Evidence, including the minimal
reproduction: [`../findings/api.md`](../findings/api.md).

**What landed:** `override` on all four declarations — `OffSet`, `Transform`, `Merge`
(the three that were unmarked) and `CloudViewOpenCv`, which already had it and now carries
the same comment as the other three. Counters 25: 3 → 0.

**What was *not* done, on purpose:** the `const` drop from the original S3 wording. It moves
into `P2-12`, and two measurements are why:

- **Dropping `const` is warning-free, so no `using` declarations are needed here.** Measured
  by dropping `const` from `OffSet`'s declaration *and* definition and rebuilding: no
  `-Woverloaded-virtual`. `Mux` warns because its `filter()` overrides nothing; a derived
  declaration that *is* an override of the surviving base virtual does not trigger it. That
  removes the one thing that could have made `P2-12` need per-site fixes.
- **Dropping `const` here would open a behaviour window that `P2-12` cannot close.** Once a
  class overrides only the non-`const` virtual, a caller holding a `const Filter&` gets the
  base's `return false`. Inside this tree that caller does not exist — the only
  `const`-qualified call of `filter()` is the base's own delegation (`filter.hpp:197`);
  `FilterBank`, `ParallelFilter`, `FilterThread` and `Controller` all call through
  non-`const` `Filter*`. But these are *installed* headers, and `P3-3` is not a tagged API
  change, so the window would be real for anyone downstream for as long as the two commits
  sit apart.

`override` alone gets the whole benefit: when `P2-12` deletes the base's `const` overload,
all four sites fail to compile, which is exactly the outcome the finding asked for. The
`const` drop then happens in the same commit as the base change, so the two cannot drift.

**Verified:** full `DOD 1.1` matrix — 4/4 configurations build, 8/8 `ctest` in each, warning
counts unchanged (28 / 27 / 26 / 25). No test is added by this item, because it changes no
runtime path — the fence that distinguishes "compiles" from "runs" is `P3-9`, still open.

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

## `P3-6` — config strings used as `printf` formats — **csv_source done, exportcsv open**

A config string is used as a `snprintf` format (undefined behaviour if it is not a valid
format string, and `-Wformat` cannot help because it is not a literal), and two `fscanf`
calls discard their results so a short or malformed CSV writes uninitialised values into
the image and reports success. Evidence: [`../findings/security.md`](../findings/security.md).

**What landed (`csv_source`).** A file-local validator accepts no conversion, or exactly
one signed-decimal conversion (`%d`/`%i` with flags and width — the documented `%05d`), and
rejects everything else with the reason in the log; `loadConfig` and `updateConfig` both
run it and keep the previously configured pattern on rejection. `loadConfig` returns 0 when
it rejects something, which is a report and not a halt: `FilterBank::instantiateFilter`
deliberately does not check `loadConfig`'s return value until `A23` settles what the number
means. Both expansions now go through one helper that checks `snprintf`'s return for
truncation, and both `fscanf` loops check for `!= 1`, log the file, the expected count and
how far they got, and stop writing — the remaining pixels keep the content they already had
instead of an uninitialised variable.

Five tests, four of them observed failing on the parent commit:

| test | pre-fix behaviour |
|---|---|
| `FormatStringThatIsNotAnIntConversionIsRejected` | returned 1 for `%s%s` |
| `RejectedPatternIsNotUsed` | **segfault** — `%s%s` read a `char*` out of the stack slot holding the frame counter |
| `ShortFileStopsInsteadOfWritingUninitialisedValues` | pixels 2 and 3 held `8`, the last value that *was* there |
| `MalformedFirstValueStopsTheRead` | every pixel held `9.1834095e-41`, i.e. an uninitialised float bit pattern |
| `TruncatedPathIsReportedAndTheFrameIsLeftAlone` | passed before the fix too — a truncated name is also not a readable file, so this one pins behaviour and buys a diagnostic; it is not a fence |

Tree warnings 28 → **26**: the two `-Wunused-result` reports on the `fscanf` calls are gone.

**What is left.** `exportcsv.cpp:92,100,103` — same finding, three sites, plus a
`char path[_filePattern.length() + 64]` VLA whose size comes from the config file. The
validator should move to `filter_helpers.hpp` (its stated purpose is helpers for filter
implementations) and be shared rather than duplicated. Counter 21: 5 → 3, target 0.

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

## `P3-9` — a test that a filter's body actually ran — **done**

The gate already said *a filter that overrides only one `filter()` overload is exercised by
a test that asserts its body ran* (`../dod/stage-gates.md` §2.4); what was missing was the
test, so the gate passed while a filter that resolves to the base default would not have.

**What landed:** `tests/test_filter_overloads.cpp`, ctest target `filter_overloads` (9
targets now). Five tests, all asserting the *body* ran — the filter writes a slot into the
`Frame`, and the assertion is on the slot, not on a return value the base default also
supplies:

| test | what it pins |
|---|---|
| `ConstOnlyOverrideRunsThroughTheBank` | the `OffSet`/`Transform`/`Merge`/`CloudViewOpenCv` shape: reached through `FilterBank::filter()` via the base's delegation |
| `NonConstOnlyOverrideRunsThroughTheBank` | the common shape, so the const case cannot pass by accident |
| `BothShapesRunInOneBank` | a mixed pipeline runs both bodies and does not stop at the const-only filter |
| `ConstReferenceStillReachesTheConstOverride` | the contract `P2-12` proposes to remove; it stops compiling when `P2-12` lands, which is the point |
| `NegativeControlDetectsAnOverrideThatOverridesNothing` | a `filter()` that overrides nothing keeps compiling, keeps its place in the bank, and never runs — the bank must report failure and the slot must stay empty |

The negative control is what makes the other four evidence rather than decoration: it is
the same shape as the `P2-12` accident, and it demonstrates that "compiles and is in the
pipeline" and "runs" are different claims.

**Verified, in both directions.** With the base's non-`const` delegation replaced by
`return false` — exactly the `P2-12` hazard — `ConstOnlyOverrideRunsThroughTheBank` and
`BothShapesRunInOneBank` fail and the other three stay green, because they do not depend on
the delegation. `filter.hpp` was then restored byte-identical (`git diff --quiet` clean),
the tree rebuilt warning-free and `ctest` is 9/9. The first attempt at that experiment did
not compile at all — `-Werror=unused-parameter` on the stubbed-out core — and the stale
test binary from before it reported 5/5 green: a reminder to check that the build actually
rebuilt before quoting a test result.

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

## `P3-11` — a CI warning counter that does not change the build — **done**

Nothing outside core measured warnings: a full build emitted 28 warnings and nobody
looked. [`../findings/audit-2024-09.md`](../findings/audit-2024-09.md) had the breakdown;
the `-Wmaybe-uninitialized` cluster in the K3M skeletonizers is the part worth a human look
before it is dismissed.

**What landed:** `tools/warning_report.sh`, run as a step in the `build-test` CI job after
the build. It reads the build log (not the source tree), attributes every warning line to
the area the compiler named — `modules/core`, `modules/filters`, `modules/bta`,
`libraries`, `apps`, `external/` — and prints the per-area table, the worst files and a
by-message histogram. It exits 1 **only** for `modules/core`, which `-Werror` (`P3-4`)
already holds at 0; everything else is reported. The Build step now `tee`s its output to
`build.log` under `set -eo pipefail` — without `pipefail` the pipeline's status is `tee`'s
and a failed build would reach the Test step looking fine — and `build.log` is added to the
failure artifact.

**Why only core gates:** gating `modules/filters` would fail every run on 20-odd
pre-existing warnings, and a job that always fails gets disabled rather than acted on. The
number becomes actionable by being visible, which is the whole item.

**Measured, not asserted:**

| input | core | total | exit |
|---|---|---|---|
| pre-`P3-4` default build log | 4 | 32 | 1 |
| current tree, Release, PCL off, BTA off | 0 | 25 | 0 |
| current tree, Debug, PCL on, BTA off (the CI configuration) | 0 | 16 | 0 |
| empty log | 0 | 0 | 0 |

The first row is the check that it can fail; the last is the check that it does not fail on
nothing. Note the last two rows: **the tree-wide count is configuration-dependent** — the
same tree emits 16 in Debug and 25 in Release (PCL off, BTA off in both), because
`-Wmaybe-uninitialized` (7 of them, the K3M cluster) is only diagnosed with optimisation
on. Quote the configuration with 15a.
