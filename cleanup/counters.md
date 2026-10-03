# Canonical counters

**This is the only place a measured number is written down.** Other chapters link here.

Re-measured on `feature/code-cleanup` at `8689d68` with PRs 28 and 29 (`P3-2`, `P3-12`,
`X2b` — the controller-extraction stages X1–X2b) applied, i.e. after `P3-1`, `P3-3`, `P3-4`,
`P3-7`, `P3-9`, `P3-11`, `A24`, `P3-6`, `P3-2`, `P3-12` and `X2b`; GCC 13.3, CMake 3.28.3,
default configuration (PCL on, BTA on). Every command below is copy-pasteable from the
repository root; run them before quoting a figure.

An earlier revision of this header cited `4739900`, which is not an ancestor of `HEAD` — it
was the pre-amend version of the README commit and is unreachable from this branch. Cite a
commit that `git log` can actually show.

**Scope is part of the number.** `modules/core`, `modules/` and the whole tree are three
different answers for most of these, and the programme has twice quoted one for another —
see [`findings/audit-2024-09.md`](findings/audit-2024-09.md). `libraries/` is not a
scratch directory: its object libraries are linked into `libtoffy.so`
(`libraries/CMakeLists.txt:4-8`), so it ships.

| # | metric | scope | now | target | |
|---|---|---|---|---|---|
| 1 | `P0` correctness items closed, each with a regression test | — | **12/12** | 12/12 | ✅ |
| 2 | `P1`/`P2` plan items closed | 16 items | **4/16** (`P1-1`, `P2-8`, `P2-14`, `P2-16`) | 16/16 | |
| 3 | CI configures, builds and runs `ctest` on every push | `.github/` | matrix + sanitizers | required | ✅ |
| 4 | Builds in all four `PCL_FOUND`/`HAS_BTA` configurations | build | **4/4**, 11/11 ctest each | 4/4 | ✅ |
| 5 | debug prints in library code | `modules/` | **128** | 0 | |
| 5a | | `modules/core` | **0** | 0 | ✅ |
| 5b | | `libraries/` | **97** | 0 | ⚠️ never counted |
| 5c | | tree, library code | **225** | 0 | |
| 6 | `#warning` directives | tree | **0** (was 1) | 0 | ✅ |
| 7 | `DLLExport` occurrences | `modules/` | **0** (was 22) | 0 | ✅ |
| 7a | | tree | **0** (was 3, all in the `libraries/sensor/` orphan, deleted under `P3-1`) | 0 | ✅ |
| 8 | headers defining `RAWFILE` | tree | **1** (was 3) | ≤ 1 | ✅ |
| 8b | `WIN` / `UNIX` defines | `modules/` | **0** | 0 | ✅ |
| 8c | | tree | **0** (was 2, both in the deleted orphan) | 0 | ✅ |
| 9 | `#include <toffy/viewers/…>` from core | `modules/core` | **1** | 0 | |
| 10 | `interprocess` primitives | `modules/core` | **2** | 0 | |
| 11 | `else if (type ==` branches in the factory | `filterfactory.cpp` | **37** | 0 | |
| 12 | `@todo` markers | `modules/core` | **20** | ≤ 5 | |
| 12b | `@todo` markers | `filterThread.hpp` | **10** | 0 | |
| 13 | `delete` of a `Filter*` | `modules/core` | **4 sites, 3 owners** | 1 owner | |
| 14 | `MSVC` preprocessor branches | `modules/` | **12** (was 19) | 0 | |
| 14a | | tree | **15** (was 16; one went with the orphan) | 0 | |
| 15 | warnings in core under `-Wall -Wextra` | `modules/core` | **0** in all four configurations, and `toffy_core` is compiled with `-Werror` (`P3-4`) | 0 | ✅ |
| 15a | warnings in a full default build (Release, PCL on, BTA on) | tree | **26** (was 32; −4 from the `Mux` using-declaration, −2 from `csv_source`'s `fscanf` sites now being checked, `P3-6`) — reported per area by `tools/warning_report.sh` on every CI build, only `modules/core` gates (`P3-11`) | reported, not gated | |
| 16 | artefacts describing the removed web control product | tree | **0** (was 12, then 8: 4 installed headers `P3-2`, 3 screenshots and the `use.dox` page `X4`, 3 CLI options `X3`, 1 `Player` `@todo` `X4`) — enumerated below, because "≥ 3" was not a number | 0 | ✅ |
| 17 | installed public headers that do not compile standalone | install tree | **0** (was 4 by hand, 5 once the check was written: 3 deleted by `P3-2`, 2 fixed by `P3-12`) — 86 of 86 pass, 0 skipped, and the `installed-headers` CI job runs the check on every push. The check is now **two passes**: each header once, and each header twice (`X2b`), so 172 checks | 0 | ✅ |
| 18 | public headers including the CMake-generated `toffy_export.h` | tree | **7** (was 8; one went with the orphan) — `P3-10` targets 0 | 0 | |
| 18a | `TOFFY_EXPORT` annotation sites | tree | **17** in 16 headers | 0 | |
| 19 | CMake `if( ${VAR} )` sites | build | **0** (was 4) — CI now fails on any new one | 0 | ✅ |
| 20 | `system()` on configuration data | `modules/filters` | **1** | 0 | |
| 21 | config string used as a `printf` format | `modules/filters` | **0** — `csv_source`'s 2 and `exportcsv`'s 2 are closed (`P3-6`); both filters validate their pattern at config time and expand through `toffy::formatPath`, the one place in the tree where snprintf's format argument is a variable | 0 | ✅ |
| 22 | files containing hard tabs / files in core | `modules/core` | **11 / 20** | 0 / 20 | |
| 23 | lines of code | `modules/core` | **4 113** (was 4 032 at `84d1b63`; +81 from `filter_helpers.hpp`, which the previous figure claimed to already count — it was measured before that commit landed) | — | |
| 24 | `ctest` targets | `tests/` | **11** (was 8; `filter_overloads` by `P3-9`, `csv_source` by `A24`, `exportcsv` by `P3-6`) | ≥ 8 | ✅ |
| 25 | `const`-only `filter()` overrides that do not say `override` | `modules/`, `libraries/` | **0** (was 3, fixed by `P3-3`; 4 sites now carry it) | 0 | ✅ |
| 26 | `WITH_CONTROL` sites — a symbol no build file defines | tree | **0** (was 5: 3 in `initPlugin.cpp`, 2 in `initPlugin.hpp`; `P3-2`) | 0 | ✅ |
| 27 | `toffyRunner` options describing a server that never starts | `apps/` | **0** (was 3: `--host/-h`, `--port/-p`, `--html/-d` — parsed, printed, used by nothing; `X3`) | 0 | ✅ |
| 28 | installed headers that cannot be included **twice** | install tree | **0** (was 1: `toffy/bta/FrameHeader.hpp`, a `typedef struct` with no guard; 3 of 86 had no guard at all) — `N9`, fixed by `X2b`, gated by the `twice` pass | 0 | ✅ |
| 29 | controller residue in tracked code and build files | `*.cpp`/`*.hpp`/`*.h`/`*.in`/`*CMakeLists.txt`/`*.cmake`/`*.dox` | **0** (`toffy_web/`, `toffy/web/`, `WITH_CONTROL`) — the gated superset of counter 26, and the only residue check that reaches `modules/bta`, whose headers CI cannot stage without the SDK | 0 | ✅ |

## The commands

```sh
# Comments name the things that used to be here, so strip comment lines first.
NC() { grep -vE ':[0-9]+:[[:space:]]*(//|\*|/\*)'; }
SRC='modules libraries'          # library code that ships in libtoffy.so

# 5 — debug prints. Count `std::cout` AND bare `cout`: most of the tree does
#     `using namespace std;`, so the old `grep 'std::cout'` pattern saw 29 of 128.
grep -rnE '(^|[^:a-zA-Z_.])(std::)?cout *<<' $SRC --include=*.cpp --include=*.hpp | NC | wc -l
#   per-directory:  modules/ 128   libraries/ 97   modules/core 0
#   per-file (worst first):
grep -rnE '(^|[^:a-zA-Z_.])(std::)?cout *<<' $SRC --include=*.cpp --include=*.hpp | NC \
  | cut -d: -f1 | sort | uniq -c | sort -rn | head

# 6, 7, 8 — build noise and vestigial macros. Scope is the TREE, not modules/.
grep -rn "#warning" modules/ libraries/ apps/ | NC
grep -rn "DLLExport" modules/ libraries/ apps/ | NC
# 8 counts HEADERS, and the one header defines the macro twice (one per branch of an
# #ifdef), so `grep -n` returns 2 lines for a counter of 1. Use -l.
grep -rl "define RAWFILE" modules/ libraries/ apps/
grep -rnE "define (WIN|UNIX)\b" modules/ libraries/ apps/

# 9, 10 — layering and the wrong synchronisation primitive
grep -rn "include <toffy/viewers" modules/core
grep -rn "interprocess" modules/core

# 11, 12 — factory and documentation debt
grep -c "else if (type ==" modules/core/src/filterfactory.cpp
grep -rc "@todo" modules/core | grep -v ":0$"

# 13 — who deletes a Filter
grep -rn "delete " modules/core/src | grep -vE ':[0-9]+:[[:space:]]*//'

# 14 — dead Windows branches. Must cover #if, #ifdef AND #ifndef; a pattern
#      matching only "#if"/"#ifdef" under-counts.
grep -rnE '^[[:space:]]*#[[:space:]]*if(n?def)?[[:space:]].*MSVC' modules/ | wc -l
grep -rnE '^[[:space:]]*#[[:space:]]*if(n?def)?[[:space:]].*MSVC' modules/ libraries/ apps/ | wc -l

# 15 — core warnings. 0 since P3-4, and toffy_core now builds -Werror, so a
#      non-zero answer here is a build failure, not a count.
cd build && touch ../modules/core/src/*.cpp && make toffy_core 2>&1 | grep -c "warning:"

# 15a — tree-wide warnings, bucketed, from a build log (what CI runs). Counting
#       `grep -c warning:` on a log is NOT the same as counting unique sites: a
#       header warning included by 10 TUs is emitted 10 times, and that is what
#       the developer sees. State the configuration — see the note below. Run it
#       on a CLEAN build: an up-to-date tree emits nothing and reports 0.
cmake --build build -j"$(nproc)" 2>&1 | tools/warning_report.sh -

# 17 — installed headers that cannot compile (needs a build dir).
# The hand version of this check found 4 and missed 1; the script is the counter now.
# Scope: the *installed* tree, not the source include dirs - only what `install()` ships
# is a public header. Third-party include flags are required or every OpenCV/PCL header is
# reported as "skipped" and the run proves nothing (the script exits 2 if it tested none).
cmake --build build --target install -- DESTDIR=/tmp/ti >/dev/null
tools/installed_header_check.sh /tmp/ti/usr/local/include \
    $(pkg-config --cflags opencv4 pcl_common)
#   86 total, 86 ok, 0 skipped, 0 broken. Drop the PCL flags for a WITHOUT_PCL build and
#   the pcl-dependent headers move from "ok" to "skipped", which the run reports.
#   pkg-config, not hardcoded -I flags: the PCL directory is versioned (/usr/include/pcl-1.14
#   today) and bta/ni/openni2 are only on a machine that has the SDK. The CI job stages with
#   `cmake --install build --prefix $PWD/stage` after building only the `toffy` target, which
#   is the cheapest tree install() accepts - the apps and tests are not installed.

# 28 — NOT a separate command: it is the second pass of counter 17's script, and the two
#      counters move together. Every header is also included twice in one translation unit
#      (86 x 2 = 172 checks); HEADER_CHECK_SINGLE=1 runs the once pass only. A [twice]
#      failure is a missing include guard, and no build in this repository can produce one:
#      each of these headers has a single consumer, so every build includes it exactly once.
#      CI cannot produce one either - the runner has no bta SDK, so modules/bta's 7 headers
#      are never staged - which is what counter 29 is for.
#   172 checks, 0 broken. Before X2b: 1 broken (toffy/bta/FrameHeader.hpp).
grep -c "pragma once" /tmp/ti/usr/local/include/toffy/bta/FrameHeader.hpp \
                        /tmp/ti/usr/local/include/toffy/bta/initPlugin.hpp   # 1 each

# 29 — the residue gate CI can actually run. Scoped to code and build files: the cleanup
#      docs and tools/installed_header_check.sh itself name these strings to record that
#      they are gone, and a gate that is always red is not a gate. Verified capable of
#      failing: re-adding one `#ifdef WITH_CONTROL` line to modules/bta/src turns it red.
git ls-files -z '*.cpp' '*.hpp' '*.h' '*.in' '*.txt' '*.cmake' '*.dox' \
  | xargs -0 -r grep -nHE 'toffy_web/|toffy/web/|WITH_CONTROL' | wc -l

# 16 — artefacts describing the removed web control product. Countable, unlike the ">= 3"
#      it replaces: 4 installed headers + use.dox + 3 screenshots + 3 CLI options
#      + 1 Player @todo = 12 before, 8 after P3-2, 0 after X3/X4.
grep -rn "toffy/web" modules libraries apps | wc -l                       # 0 (P3-2)
ls docs/extraDocs/images/control_*.png 2>/dev/null | wc -l                # 0 (X4)
grep -cE '"(host|port|html),[a-z]"' apps/main.cpp                         # 0 (X3)
grep -rn "web control" modules/core | wc -l                               # 0 (X4)
#      use.dox is deliberately NOT in the list above: the file still exists, and it still
#      contains the strings "minimal_toffy" and "localhost:9999" — in the one paragraph
#      that tells a reader who typed them in where the UI went. Grepping the page for those
#      strings counts the redirect as the artefact. The check that means something is that
#      the page is generated and describes the binary that exists:
grep -n "use.dox" docs/Doxyfile.cfg                    # only in the comment saying it is
                                                       # no longer excluded
grep -c "toffyRunner" docs/mainpages/public/use.dox    # 5 — the page is about the real CLI

# 26 — the build symbol nothing builds. `WITH_CONTROL` was never defined by any CMake file
#      in any configuration, and the toffy_web/ headers it guarded do not exist, so the
#      branch could not be compiled even by defining it by hand.
grep -rn "WITH_CONTROL" modules libraries apps | wc -l

# 27 — CLI options that advertise a server which never starts. Match the *definition*
#      sites (`"host,h"`, not the bare word host) and check the uses separately: each of
#      the three used to be read exactly once, in the `cout` line that printed it.
#      Both greps now return 0, and `grep -c` exits 1 on no match - that is the pass.
grep -cE '"(host|port|html),[a-z]"' apps/main.cpp     # 0 definitions (was 3)
grep -nE 'vm\["(host|port|html)"\]' apps/main.cpp     # 0 uses (was 3)
#      The check that actually matters is the binary's own help text:
#      ./build/apps/toffyRunner --help   # --help, -c/--config, -s/--sleepDelay,
#                                        # -f/--output2File, and nothing else

# 18 — the generated export header
grep -rn "^[[:space:]]*#[[:space:]]*include *[<\"]toffy/toffy_export.h" modules libraries apps
# 18a — count annotation sites, not mentions: P2-3 left eight "DLLExport is gone,
#       TOFFY_EXPORT is the real macro" comments behind (raw grep: 25, sites: 17).
grep -rn TOFFY_EXPORT modules libraries apps | grep -vE ':[0-9]+:[[:space:]]*(//|\*)' | wc -l

# 19 — the CMake bug class that hid the PCL-less build failure. Scope: the tracked
#      CMake files. A plain recursive grep also hits build/generated/toffyConfig.cmake,
#      which is generated and not ours - that is a false positive, not a finding.
git ls-files -z '*.cmake' '*CMakeLists.txt' \
  | xargs -0 -r grep -HE '(if|elseif|while)[[:space:]]*\([[:space:]]*\$\{' \
  | grep -vE 'MATCHES|STREQUAL'

# 20, 21
grep -rn "system(" modules/filters --include=*.cpp
# 21 — a config string passed as snprintf's format argument. The variables are
#      _depthPattern/_amplPattern/_filePattern, so the "_pattern" grep this counter used to
#      carry matched nothing at all, and the figure was being carried on trust. Match on the
#      *format* position (third argument), not on the string "pattern" appearing anywhere in
#      the call, or the checked `snprintf(buf, size, "%s", pattern.c_str())` fallback counts
#      as a finding when it is exactly what the fix asks for.
grep -rnE 'snprintf\([^,]*,[^,]*,[[:space:]]*_[A-Za-z0-9_]*[Pp]attern\.c_str\(\)' modules libraries --include=*.cpp
#   0 sites. The one remaining variable-format snprintf is the choke point itself,
#   toffy::formatPath in filter_helpers.hpp, whose parameter is deliberately called `fmt`;
#   every caller has validated the pattern before reaching it.

# 25 — const-only filter() overrides with no override keyword. Each one is a filter
#      P2-12 would silently turn into a new virtual that overrides nothing: compiles,
#      stops processing frames. 0 since P3-3; the second command lists what it became.
grep -rn "filter(const Frame" modules libraries --include=*.hpp | grep "const;" | grep -v override | wc -l
grep -rn "out) const override" modules libraries --include=*.hpp

# 22, 23 — formatting and size
grep -rlP '\t' modules/core --include=*.cpp --include=*.hpp | wc -l
find modules/core \( -name '*.cpp' -o -name '*.hpp' \) | xargs wc -l | tail -1

# 3, 4 — the gates
ctest --test-dir build --output-on-failure
for f in "" "-DWITHOUT_PCL=ON" "-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON" \
         "-DWITHOUT_PCL=ON -DCMAKE_DISABLE_FIND_PACKAGE_bta=ON"; do
  d=$(mktemp -d); cmake -S . -B "$d" $f >/dev/null 2>&1 \
    && cmake --build "$d" -j"$(nproc)" >/dev/null 2>&1 \
    && (cd "$d" && ctest 2>&1 | grep -E "tests passed|tests failed")
  rm -rf "$d"
done
```

## Notes on some of these numbers

**17 (0, was 4, was 5).** The hand check that produced the 4 compiled one header and read
its error. The script compiles all 86 and classifies every failure, and it found a fifth the
hand check had missed: `libraries/graphs/graph_utils.hpp` uses `cv::line` and
`contour_utils.hpp` uses `cv::Point`, both of which compiled only when some other header
happened to pull OpenCV in first. A downstream user including `graph_utils.hpp` alone got an
undeclared-identifier error, and no build in this repository could ever have noticed, because
nothing in it includes those two headers without including OpenCV beforehand. Two of the five
were therefore *fixed* (`P3-12`) and three *deleted* (`P3-2`). The counter is now a CI gate
rather than a figure: see `tools/installed_header_check.sh` and the `installed-headers` job in
`.github/workflows/ci.yml`.

**16 (8, was "≥ 3").** "≥ 3" is not a number and it was not reproducible. The artefacts are
now enumerated one by one in the command block: 4 installed headers, `use.dox`, 3
screenshots, 3 CLI options, 1 `Player` `@todo`. `P3-2` closed the first four; `X3` and `X4`
close the rest.

**5 (128 vs 129).** Both figures have been quoted. The strict pattern above gives 128;
counting `std::cout` (29) and bare `cout` (100) separately gives 129. They differ at
`modules/filters/src/detection/sampleConsensus.cpp:501`, where `std::cout` sits alone on a
line and the `<<` continues below it. **128 is the canonical figure** — it is what the
command in this file returns. Stop quoting the two spellings as independent counts.

**5b (97 in `libraries/`).** A third false zero, larger than the two already recorded.
Criterion 5 has always been scoped to `modules/`, and `libraries/` — which is linked into
the same `libtoffy.so` — holds 97 more prints, 33 of them in `thickTracer8.cpp` alone.
Tree-wide, library code prints **225** times, not 128. `apps/main.cpp` also prints, but
that is a CLI talking to its user and is not counted.

**15 (0 warnings, was 3).** All three named the same declaration and were one root cause —
but not the one the plan blamed. `Mux` declares `filter(const std::vector<Frame*>&, Frame&)`,
which *hides* `Filter::filter(...)`; the fix was a `using Filter::filter;` line, not the
`P2-12` API change ([`findings/api.md`](findings/api.md), `P3-4`). `toffy_core` is now
compiled with `-Werror` behind `option(CORE_WERROR ON)`, scoped to that target and to
GNU/Clang; the gate was tested by injecting an unused variable into `filter.cpp` and
watching the build stop with `all warnings being treated as errors`.

The count used to be configuration-dependent (one site behind a PCL guard); at zero that no
longer matters, and all four `PCL_FOUND` × `HAS_BTA` cells were measured at 0 core warnings.
The tree-wide figure (15a) is **not** gated: 28 warnings remain outside core and
`tools/warning_report.sh` prints them per area on every CI build, exiting 1 only for
`modules/core` (`P3-11`). It is a build-log reader, so its scope is whatever the compiler
was told to compile — and its number is configuration-dependent in a way core's is not:
the same tree emits **28** (Release, PCL on, BTA on), **25** (Release, PCL off, BTA off)
and **16** (Debug, PCL on, BTA off — the CI configuration; counter 16 is a different 16),
because `-Wmaybe-uninitialized`,
7 of the 28, is only diagnosed with optimisation on. Quote the configuration.
