# Canonical counters

**This is the only place a measured number is written down.** Other chapters link here.

Re-measured on `feature/code-cleanup` at `5ec61d7`, i.e. after `P3-1`, `P3-4`, `P3-7` and
`P3-3` landed; GCC 13.3, CMake 3.28.3, default configuration (PCL on, BTA on). Every command
below is copy-pasteable from the repository root; run them before quoting a figure.

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
| 4 | Builds in all four `PCL_FOUND`/`HAS_BTA` configurations | build | **4/4**, 9/9 ctest each | 4/4 | ✅ |
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
| 15a | warnings in a full default build (Release, PCL on, BTA on) | tree | **28** (was 32; the `Mux` using-declaration also silenced a 4th, in `3d/muxMerge.hpp`) — reported per area by `tools/warning_report.sh` on every CI build, only `modules/core` gates (`P3-11`) | reported, not gated | |
| 16 | docs describing a product that does not exist | `docs/`, installed headers | **≥ 3** | 0 | |
| 17 | installed public headers that do not compile | install tree | **4** | 0 | |
| 18 | public headers including the CMake-generated `toffy_export.h` | tree | **7** (was 8; one went with the orphan) — `P3-10` targets 0 | 0 | |
| 18a | `TOFFY_EXPORT` annotation sites | tree | **17** in 16 headers | 0 | |
| 19 | CMake `if( ${VAR} )` sites | build | **0** (was 4) — CI now fails on any new one | 0 | ✅ |
| 20 | `system()` on configuration data | `modules/filters` | **1** | 0 | |
| 21 | config string used as a `printf` format | `modules/filters` | **5** in 2 files — `csv_source.cpp` 2, `exportcsv.cpp` 3 (was recorded as 2, because only `csv_source` had been read) | 0 | |
| 22 | files containing hard tabs / files in core | `modules/core` | **11 / 20** | 0 / 20 | |
| 23 | lines of code | `modules/core` | **4 021** | — | |
| 24 | `ctest` targets | `tests/` | **9** (was 8; `filter_overloads` added by `P3-9`) | ≥ 8 | ✅ |
| 25 | `const`-only `filter()` overrides that do not say `override` | `modules/`, `libraries/` | **0** (was 3, fixed by `P3-3`; 4 sites now carry it) | 0 | ✅ |

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
grep -rn "define RAWFILE" modules/ libraries/ apps/
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

# 17 — installed headers that cannot compile (needs a build dir)
cmake --build build --target install -- DESTDIR=/tmp/ti >/dev/null
printf '#include <toffy/web/common/plugins.hpp>\n' | g++ -std=c++17 -fsyntax-only \
    -I/tmp/ti/usr/local/include -x c++ -

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
#      carry matched nothing at all, and the figure was being carried on trust.
grep -rnE "snprintf\([^;]*[Pp]attern" modules libraries --include=*.cpp
#   5 sites: csv_source.cpp:162,183 and exportcsv.cpp:92,100,103

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

## Notes on three of these numbers

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
and **16** (Debug, PCL on, BTA off — the CI configuration), because `-Wmaybe-uninitialized`,
7 of the 28, is only diagnosed with optimisation on. Quote the configuration.
