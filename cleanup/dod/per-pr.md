# Definition of done — every PR

1. **Builds in all four dependency configurations**, not just the developer's. This gate is
   now **green in all four cells**, which it was not when this document was written:

   | `PCL_FOUND` | `HAS_BTA` | status | how to reproduce |
   |---|---|---|---|
   | on | on | ✅ builds, 8/8 ctest | default (needs the bta SDK) |
   | on | off | ✅ builds, 8/8 ctest | `-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON` |
   | off | on | ✅ builds, 8/8 ctest | `-DWITHOUT_PCL=ON` |
   | off | off | ✅ builds, 8/8 ctest | both flags |

   The original note blamed only the unguarded PCL typedefs in `frame.hpp`. That was the
   first blocker of several, and the interesting part is that the typedefs had never been
   *seen* to fail: two CMake `if( ${VAR} )` clauses expanded to nothing when the dependency
   was absent, so configuration aborted before reaching the compiler. Fixing the header
   exposed the CMake bugs, which exposed an empty `add_library(toffy_3d OBJECT "")`, which
   exposed `exportcloud.hpp` holding a `pcl::PCDWriter` by value and an unnecessary
   `<pcl/io/pcd_io.h>` in `exportcsv.hpp`. Separately, `add_subdirectory(bta)` was
   unconditional, so every BTA-off configuration failed to link with ~162 undefined `BTA*`
   references.

   A PCL-less build is not optional to check: it is the configuration most downstream
   packagers will hit first. Note that CI can only ever cover the **BTA-off** axis (the SDK
   is proprietary), so the two BTA-on cells still require a machine that has it.

2. **CI is green, and CI actually builds and tests.** This gate now exists:
   `.github/workflows/ci.yml` configures, compiles and runs `ctest` on every push and PR,
   as a PCL-on/PCL-off matrix plus a separate ASan/UBSan/LSan job. The original wording —
   that `.github/` held only Codacy, Flawfinder and Dependabot and nothing compiled
   anything — no longer applies.

   Two limits to keep in mind when citing a green run:
   - Runners cannot have the proprietary Becom bta SDK, so CI only ever covers the
     **BTA-off** axis. A BTA-on build still has to be checked on a machine that has the SDK.
   - CI does **not** build the Doxygen docs, so broken `\ref`s and `\todo` drift are not
     caught by anything.

3. **`ctest` passes**, and any new test fails on the parent commit for the right reason.
   For a fix PR, paste the pre-fix failure into the PR description. A test that has never
   been observed to fail is not a regression fence.

4. **No new compiler warnings** under the existing `-Wall -Wextra`
   (`CMakeLists.txt:207`) in any file the PR touches. Do not enable `-Werror` globally — the
   28 warnings outside `modules/core` would fail the build — but do not add to them either.
   `modules/core` is the exception and `P3-4` has landed: `toffy_core` compiles with
   `-Werror` (`option(CORE_WERROR ON)`, GNU/Clang only, `target_compile_options` PRIVATE so
   it cannot leak), so a new warning in core is a build break, not a review comment. If a
   future compiler release makes that unbuildable, say so in the PR rather than turning the
   option off silently. Every CI build also runs `tools/warning_report.sh` over the build
   log (`P3-11`): it prints the count per area — `modules/core`, `modules/filters`,
   `modules/bta`, `libraries`, `apps`, `external/` — and fails only on `modules/core`. Locally,
   run it on a **clean** build (`cmake --build build -j"$(nproc)" 2>&1 | tools/warning_report.sh -`);
   an up-to-date tree compiles nothing and the report says so instead of reporting a zero.

5. **One numbered plan item per commit, one PR-stage per group of commits.** The plan's
   ground rule: never mix a public rename with a behaviour change in one commit.

6. **Docs updated in the same PR** as the behaviour or API change. If a PR changes what a
   documented function returns, the doc comment changes in that PR — not later. Several
   findings exist precisely because the docs and the code disagreed (`findPos`, `remove`).
   In this document set that means: the item's status in
   [`../plan/README.md`](../plan/README.md), the measured figures in
   [`../counters.md`](../counters.md), and anything the finding said about the code being
   open. A number quoted anywhere else is a number waiting to disagree.

7. **No debug output added to library code.** Library code means `modules/` **and**
   `libraries/` — both link into `libtoffy.so`. Neither may write to `std::cout` *or* to
   bare `cout`; use `BOOST_LOG_TRIVIAL`. (`apps/` talking to its user is fine and is not
   counted.)

8. **Revertable in isolation.** If reverting the PR also requires reverting another, say so
   in the PR body and reconsider the split.

9. **A check states its scope.** Any grep, CI step or counter this PR adds names the
   directories it covers. Two criteria sat green for weeks because their greps were scoped
   to `modules/` while the pattern lived in `libraries/` — see
   [`../findings/audit-2024-09.md`](../findings/audit-2024-09.md). A check that cannot fail
   is not a gate.
