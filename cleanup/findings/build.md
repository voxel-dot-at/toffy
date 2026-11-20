# E — Portability & build

**`#ifdef MSVC` is dead code inside the library.** This is the most consequential
build-system bug here.

`CMakeLists.txt:169` appends `-DMSVC` to the `DEFINITIONS` list, but the line that would
apply that list to the library is commented out (`target_compile_definitions(${PROJECT_NAME}
PUBLIC ${DEFINITIONS})`, `CMakeLists.txt:404`). Only `apps/CMakeLists.txt` consumes
`DEFINITIONS`.

Consequence: inside `libtoffy` itself, `MSVC` is never defined, so every `#ifdef MSVC`
branch in `filterbank.cpp`, `controller.cpp` and `frame.hpp` compiles as POSIX. The
Windows plugin-loading paths (`LoadLibrary`/`GetProcAddress`) are unreachable, and the
POSIX paths (`dlfcn.h`) would be selected on Windows.

The asymmetry is the part that will actually hurt: `-DMSVC` is appended *inside*
`if(MSVC)`, so on a Windows build **`apps/` gets `MSVC` defined and the library does not**.
Two halves of one program then disagree about `Frame`'s members and about which plugin
loader is compiled. That is why `P2-2` is a decision rather than a grep: either apply
`DEFINITIONS` to the library so the branches go live and get tested, or delete them and
say Windows is unsupported.

**Count: 12 in `modules/`, 16 tree-wide** — 4 in `controller.cpp`, 3 in `filterbank.cpp`,
2 in `capturerFilter.cpp`, 1 each in `cloudviewpcl.cpp`, `bta.cpp`, `BtaWrapper.hpp`, 1 in
the `libraries/sensor/` orphan and 3 in `apps/main.cpp`. It was 19 before `P2-3` deleted
the seven wrapped round `DLLExport`/`RAWFILE`; two documents kept quoting 19 after that.
Count with a pattern that covers `#if`, `#ifdef` **and** `#ifndef` — the usual two-term
pattern reports 10.

**~~`DLLExport` is vestigial.~~ FIXED (`P2-3`)** — all 22 occurrences deleted from **8**
headers, not the 3 this finding originally listed; the audit had only looked at core. It was
`#define`d in `frame.hpp`, `filterbank.hpp`, `filterfactory.hpp`, `capturerFilter.hpp`,
`detectedObject.hpp`, `average.hpp`, `imagesensor.hpp` and `BtaWrapper.hpp`, and actually used
on only 3 class declarations (`Average`, `ImageSensor`, `BtaWrapper`) — the rest were dead or
commented out (`class /*DLLExport*/ TOFFY_EXPORT FilterFactory`). No-op on Linux, since
`DLLExport` expanded to `/**/` off-MSVC, and the `dllexport` branch was unreachable from
inside the library anyway (see the `MSVC` finding above).

**The replacement was not an improvement — see `TOFFY_EXPORT is vestigial too` below.**
The 3 classes went to `TOFFY_EXPORT` because it was the macro the build actually
generates; that turned out to be a weaker argument than it looked.

**…and the deletion was incomplete — FIXED (`P3-1`).** `libraries/sensor/include/toffy/imagesensor.hpp`
still defined `DLLExport`, `WIN` and `UNIX` and still annotated `class DLLExport
ImageSensor`. It was a pre-`P2-3` copy of `modules/bta/include/toffy/io/imagesensor.hpp`,
had no `CMakeLists.txt`, was in no `add_subdirectory`, was on no include path and was not
installed — so the counters read 0 while the pattern sat in the tree. Both headers in
`libraries/sensor/` are now deleted; `diff` of the deleted file against the live one showed
exactly the `P2-3` changes plus the `#include <toffy/sensor.hpp>` path, i.e. the same file
at two vintages, which is what made it safe to remove rather than to fix. Counters 7a and
8c are now true zeros rather than false ones.

**`WIN` and `UNIX` were also going.** Not in the original audit: `imagesensor.hpp` and
`BtaWrapper.hpp` did `#define WIN true` / `#define UNIX true` in *public headers*, referenced
only by three commented-out lines in `BtaWrapper.cpp`. Two of the most generic macro names in
C, leaking into every consumer. Deleted.

**~~`RAWFILE` is defined in three separate public headers.~~ FIXED (`P2-3`)** — it is now
defined once, in `bta/BtaWrapper.hpp`, next to its only consumer (`bta.cpp:61`). It used to be
repeated in `filterbank.hpp` and `capture/capturerFilter.hpp` with their own `#ifdef`s, so a
translation unit including two of them risked a redefinition, and every consumer of the
library inherited a two-character macro. The `.rw`/`.r` value is unchanged on purpose: it keys
off `MSVC`, which the library build never defines, so it has always been `".r"` — but silently
changing an on-disk file extension is a behaviour change, not a cleanup.

**`TOFFY_EXPORT` is vestigial too, and it is wired up in a way that only looks right.**
`P2-3` replaced `DLLExport` with `TOFFY_EXPORT` on the 3 class declarations that used it,
on the grounds that it is the macro CMake's `generate_export_header` really generates. It
is generated — and it does nothing:

- **No translation unit in the build ever defines `toffy_EXPORTS`.** `libtoffy.so` is
  assembled from object libraries (`toffy_core`, `toffy_filters`, `toffy_lib_*`, …) via
  `$<TARGET_OBJECTS:>`, and the `toffy` target itself has no sources — so the generated
  header's "we are building this library" branch is never taken. Check:
  `grep -l toffy_EXPORTS build/*/flags.make` names only `build/CMakeFiles/toffy.dir/`, the
  target that compiles nothing.
- **Visibility is default anyway.** No `-fvisibility=hidden` anywhere in the build, and the
  generated header emits `__attribute__((visibility("default")))` on *both* sides of the
  `toffy_EXPORTS` test. On Linux the macro cannot change anything.
- **On MSVC it would be actively wrong:** with nothing defining `toffy_EXPORTS`, every
  annotated class in the library would be compiled `__declspec(dllimport)`.
- **It drags a build artifact into the public API.** 7 public headers do
  `#include <toffy/toffy_export.h>`, so including `toffy/frame.hpp` requires CMake's
  generated include dir, and `make install` has to ship `toffy_export.h` for the rest of
  the installed headers to parse.

17 annotation sites across 16 headers, 7 includes (counters 18/18a; the raw
`TOFFY_EXPORT` grep says 25 because eight of the mentions are `P2-3`'s own
"DLLExport is gone" comments). `P3-10` deletes them and keeps
`${CMAKE_CURRENT_BINARY_DIR}/generated/` on the include path for `toffy_config.h`, which
*is* used. The gate is that `readelf --dyn-syms` on `libtoffy.so` is unchanged.

**A PCL-less build now works.** `frame.hpp` used to guard the PCL *includes* but leave the
two typedefs naming those templates outside the guard, so `-DWITHOUT_PCL=ON` failed with
`'pcl' does not name a type`. Fixing that exposed that the configuration had never actually
reached the compiler, because of an `if( ${VAR} )`-vs-`if(VAR)` bug that aborted CMake first.

Full set of fixes on the PCL axis:

- `frame.hpp` — typedefs guarded; the `CloudXyz`/`CloudXyzRgb` enumerators deliberately
  left unconditional so `SlotDataType` cannot differ between two builds of the header.
- `viewers/CMakeLists.txt`, `3d/CMakeLists.txt` — `if( ${PCL_FOUND} )` style clauses
  expanded to nothing when PCL was off, so CMake parsed `if( AND OFF)` and died with
  "Unknown arguments specified". `if()` takes variable *names*.
- `3d/CMakeLists.txt` — `add_library(toffy_3d OBJECT "")` is rejected by CMake, so the
  target is now conditional and its two consumers (`CMakeLists.txt`,
  `modules/filters/CMakeLists.txt`) reference it through a `PCL_FOUND`-gated variable.
- `viewers/exportcloud.hpp` — holds a `pcl::PCDWriter` by value, so the whole class is
  guarded and `exportcloud.cpp` moved into a PCL-gated source list; `init.cpp` guards the
  include, factory function and registration to match.
- `viewers/init.cpp` — included `cloudviewpcl.hpp` unconditionally while guarding only its
  registration.
- `viewers/exportcsv.hpp` — `#include <pcl/io/pcd_io.h>` with **no** `pcl::` usage anywhere
  in the header or its source. Purely unnecessary, and it made an always-built
  PCL-independent filter fail to compile.
- `reproject/CMakeLists.txt` — `reprojectpcl.cpp` was always built; now gated. (`
  filterfactory.cpp` already guarded both the include and the `reprojectpcl` branch.)

Verified at the time: clean `-DWITHOUT_PCL=ON` build, 74 TUs, 7/7 `ctest` (8/8 since
`P2-8` added `test_logging`). The default PCL-on build is
provably unaffected — configuring `HEAD` and this tree yields identical compilation-unit
lists (234 each), and the default build is clean with 8/8.

**`PCL_FOUND` is still not part of the exported interface.** It arrives via a global
`add_definitions` (`:267`), so consumers must reproduce the flag themselves or `Frame`
changes shape under them. Worth an exported compile definition.

**A BTA-less build used to be broken; it now works.** Found while testing the `DOD 1.1`
matrix: `modules/CMakeLists.txt` did `add_subdirectory(bta)` **unconditionally**, so the bta
module compiled even when `find_package(bta)` failed and linking then died with ~162
undefined `BTA*` references. It was confirmed on `HEAD` at the time, and was unrelated to the
PCL work.

**FIXED** — `add_subdirectory(bta)` is now gated by `if (HAS_BTA)` (`modules/CMakeLists.txt:8-10`)
and the single `$<TARGET_OBJECTS:toffy_bta>` consumer is gated the same way. Re-verified on the
current tree: all four `PCL_FOUND` × `HAS_BTA` cells configure, build and pass 8/8 `ctest`, so
the `DOD 1.1` matrix is 4/4 green and agrees with
[`../dod/per-pr.md`](../dod/per-pr.md). There is still no explicit option
to disable BTA — testing the off axis needs `-DCMAKE_DISABLE_FIND_PACKAGE_bta=ON`.

**~~Dead version branches.~~ FIXED (`P2-16`).** `controller.cpp` kept
`#if (BOOST_VERSION > 105500)`, guarding a `char` log-severity variant from a 2017-era Boost
that no supported compiler accepts. The `#else` branch is gone.

**~~`#warning bta missing!`~~ FIXED (`P2-3`).** It fired on every build of the default
configuration, training everyone to ignore build noise. Deleted — CMake already emits
`message(WARNING "no bta library!")`, so the information was never lost, just doubled and
attached to the wrong audience.

**The debug-print metric was measuring the wrong thing — and still is, if you copy the old
command.** `grep -rn 'std::cout' modules/` returns 42, and that number has been quoted as the
size of `P2-3`. But most of `modules/` has `using namespace std;` at file scope, so the debug
prints are overwhelmingly **bare** `cout <<`, which that pattern cannot match. Counting both
spellings and stripping comments: **129 live prints**, of which only 29 are `std::cout`.

Two consequences. First, the item is ~3× bigger than planned. Second, and more subtle: the
same `using namespace std;` that hides them is *why* they are there — writing `cout` instead of
`std::cout` is a one-character saving that costs grep-ability, and it cost it here. Core's
copies are all gone and its file-scope `using namespace std;` directives went with them, so
new debug prints in core will at least say `std::cout` and be countable.

---

## From the second-pass audit

Second-pass audit, measured on `e943a37`. The audit record itself — the re-verification table and the scope bug behind the two false zeros — is in [`audit-2024-09.md`](audit-2024-09.md).

### N1 — `make install` ships a public header that does not compile — **FIXED (`P3-2`, `P3-12`)**

`libraries/CMakeLists.txt:11` is `install(DIRECTORY "include/" DESTINATION "include/")`, which
installs `libraries/include/toffy/web/` wholesale. One of those headers is broken:

```sh
$ cmake --build build --target install -- DESTDIR=/tmp/ti2
$ find /tmp/ti2 -path '*toffy/web*' -type f
/tmp/ti2/usr/local/include/toffy/web/common/plugins.hpp
/tmp/ti2/usr/local/include/toffy/web/actions/action.hpp
/tmp/ti2/usr/local/include/toffy/web/btaController.hpp
/tmp/ti2/usr/local/include/toffy/web/btaGroupController.hpp

$ printf '#include <toffy/web/common/plugins.hpp>\n' | g++ -std=c++17 -fsyntax-only \
      -I/tmp/ti2/usr/local/include -x c++ -
fatal error: toffy/web/controllerFactory.hpp: No such file or directory
```

`toffy/web/controllerFactory.hpp` exists nowhere in the repository (`find . -name
'controllerFactory*'` → nothing), so the installed header cannot be included by anyone. These
four files are leftovers of the web control UI removed in `54d9577`. The record already
noted that `docs/use.dox` still documents that removed product
([`documentation.md`](documentation.md)), but not the headers themselves — and not the fact
that they are *installed*, i.e. part of the shipped public API surface.

**Suggestion S2 — delete `libraries/include/toffy/web/` and `modules/bta/include/toffy/web/`**
— **done**, together with the `#ifdef WITH_CONTROL` hooks in `modules/bta` that were the only
in-tree references, and with the version decision recorded against `DOD 2.4`.

Two things the hand check in this section got wrong, both fixed by
`tools/installed_header_check.sh` (`P3-12`):

- **4 was really 5.** Compiling *all* 90 installed headers rather than the one named above
  found `libraries/graphs/graph_utils.hpp` (`'line' was not declared`) and
  `libraries/include/toffy/graphs/contour_utils.hpp` (`'cv' was not declared`), which
  compiled only when another header had pulled OpenCV in first. Both are fixed by adding the
  include, not by deleting: `libraries/src/graphs` uses them.
- **Two `install(DIRECTORY …)` rules ship that tree, not one.** `libraries/CMakeLists.txt:11`
  and `modules/bta/CMakeLists.txt:3` both install their `include/` wholesale, which is why
  the four files appear under two different source directories. The reproduction above listed
  all four but compiled one of them. The check walks the *installed* tree, so it does not
  matter which `install()` rule put a header there — that is the scoping bug class of
  [`audit-2024-09.md`](audit-2024-09.md), one level down.

The residue inventory, the ABI evidence and what `toffy-oatpp` needs from this side is in
[`../plan/controller-extraction.md`](../plan/controller-extraction.md).

### N9 — an installed header that cannot be included twice — **FIXED (`X2b`)**

`P3-12` gated the wrong shape of the bug. It compiles each installed header as the *only*
include of a translation unit, which is exactly the N1 failure — and it therefore cannot see
its complement: a header that compiles once and **not twice**, because it has no include
guard and contains a definition rather than a declaration.

`modules/bta/include/toffy/bta/FrameHeader.hpp` is one, and it ships:

```sh
$ printf '#include <toffy/bta/FrameHeader.hpp>\n#include <toffy/bta/FrameHeader.hpp>\n' \
    | g++ -std=c++17 -fsyntax-only -I<install>/include -x c++ -
error: conflicting declaration ‘typedef struct FrameHeader FrameHeader’
```

Three of the 86 installed headers had no guard at all; only this one actually breaks.
`toffy/bta/initPlugin.hpp` declares functions, and re-declaration is legal. The generated
`toffy/common/toffy_config.h` is all `#define`s with identical values, so re-inclusion is
harmless. `FrameHeader.hpp` holds the `typedef struct` for the on-disk `.rw` raw format —
the *binary* interface — and its only consumer in the whole tree is one `.cpp`, so every
build this repository can run includes it exactly once and stays green. CI cannot see it
either: the runners have no proprietary bta SDK, so `modules/bta` is not configured and its
7 installed headers are never staged (see the scope note on the `installed-headers` job).

**Fix — check the second inclusion too, and gate what CI cannot build** (stage `X2b`; not
minted as an `S` id, because `S1`–`S9` is the closed second-pass set and `S9` is taken):
`#pragma once` on all three headers, a `twice` pass in
`tools/installed_header_check.sh` (86 headers × 2 passes = 172 checks), and a source-tree
step failing on `toffy_web/` / `WITH_CONTROL` in tracked code and build files — the half of
the residue gate that does not need the SDK. Measured: 1 broken before, 0 after, exported
symbol set unchanged (2 889, `diff` empty) since a guard adds no symbols.

### N6 — ~~the `if( ${VAR} )` bug class is still in the build~~ **FIXED (`P3-7`)**

This document set documents this bug class well: two CMake `if( ${VAR} )` clauses expanded to nothing
when the variable was absent, which is why a PCL-less build never reached the compiler. The
class was fixed on the PCL axis but not swept. Still present:

| location | line |
|---|---|
| `CMakeLists.txt` — `if (${bta_FOUND})` | 309 |
| `CMakeLists.txt` — `if( ${BUILD_TESTS})` | 435 |
| `modules/filters/src/smoothing/CMakeLists.txt` — `if ( ${OPENCV_TRACKING_FOUND} )` | 1 |
| `modules/filters/src/tracking/CMakeLists.txt` — `if ( ${OPENCV_TRACKING_FOUND} )` | 3 |

`if( ${BUILD_TESTS})` is the one that bites: `BUILD_TESTS` is `option(... ON)` (`:54`), so it
works today, but any value containing a space turns into a CMake error rather than a false.
Demonstrated on this CMake:

```cmake
if( ${MAYBE} )          # MAYBE empty  -> silently false, no error
if( ${MAYBE} AND FOO )  # MAYBE empty  -> CMake Error: if given arguments: "AND" "FOO"
```

**Suggestion S7 — one mechanical commit: `if( ${VAR} )` → `if(VAR)`** — **done**, all four
sites (`MATCHES` comparisons like `if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")` are correct as
written and stayed).

The CI line was worth adding, but not in the form suggested: `grep -rn 'if[ ]*([ ]*\${' \
--include=CMakeLists.txt` over the repository root also matches
`build/generated/toffyConfig.cmake`, a generated file that legitimately contains
`if(${_NAME}_FIND_REQUIRED_${comp})`. A check that reports a finding nobody owns gets ignored,
which is how a gate dies. The shipped version scopes to the 29 **tracked** CMake files
(`git ls-files`) and excludes `MATCHES`/`STREQUAL`; it was verified to go red when a site is
re-introduced and green when it is removed. See the `cmake-hygiene` job in
`.github/workflows/ci.yml`.

Measured behaviour of the bug class on CMake 3.28, for the record:

```cmake
if( ${MAYBE} )          # empty      -> silently FALSE, no diagnostic
if( ${MAYBE} AND ON )   # empty      -> CMake Error: "AND" "ON" — Unknown arguments
if( ${MAYBE} AND ON )   # "a b"      -> dev warning, evaluates FALSE
```

The first row is the reason four sites sat unnoticed for years: they were not broken, they
were silently always-false, which for `bta_FOUND` and `OPENCV_TRACKING_FOUND` happened to
agree with the truth whenever the dependency was missing.

### N7 — `libraries/` depends on a target defined in `modules/filters/`

`libraries/CMakeLists.txt:8` lists `$<TARGET_OBJECTS:toffy_tracking>`, but `toffy_tracking` is
defined in `modules/filters/src/tracking/CMakeLists.txt:13` — and the top level does
`add_subdirectory(libraries)` at `:336`, *before* `add_subdirectory(modules)` at `:338`. It
works because generator expressions resolve at generate time, but the dependency points
downwards: the lower-level library directory consumes a target owned by the module above it.
This is the same inversion `P2-9` is fixing between core and viewers, in the layer below it.

**Suggestion S8 — record it as a `P2-9` follow-on**, not a separate item: `toffy_tracking`
belongs in `libraries/` next to the code it builds, or the `$<TARGET_OBJECTS:>` reference comes
out of `libraries/CMakeLists.txt`.

---

## Ubuntu's `liblog4cplus-dev` exports a CMake target that cannot link (found by `P2-17` step C)

`find_package(log4cplus)` succeeds and then every link fails:

```
No rule to make target '/usr/lib/aarch64-linux-gnu/libnsl.so', needed by 'libtoffy.so.2.0.0'
```

`/usr/lib/aarch64-linux-gnu/cmake/log4cplus/log4cplusTargets.cmake` exports

```cmake
INTERFACE_LINK_LIBRARIES "/usr/lib/aarch64-linux-gnu/librt.a;/usr/lib/aarch64-linux-gnu/libnsl.so"
```

with absolute paths to libraries the package does not depend on. `apt-cache
depends liblog4cplus-dev` lists exactly one dependency, `liblog4cplus-2.0.5t64`.
So a machine that has the dev package and not `libnsl-dev` — which is what
`apt-get install liblog4cplus-dev` gives you — has an imported target naming a
file that is not there. This machine has `libnsl.so.1` and `libnsl.so.2` and no
`libnsl.so`; the `-dev` symlink is what is missing, and nothing asked for it.

Two things make it worse than a missing dependency, and both are the reason it is
written down rather than worked around silently:

* It fails at **build time with a CMake generator error**, not at configure with
  a "could not find dependency" message. The imported target was accepted; the
  file is only needed when something tries to link. So `find_package` reports
  success and the failure lands somewhere far away from the thing that was wrong.
* It is **absolute-pathed**, so it is not even a name the linker could resolve
  some other way.

`P2-17` works around it by finding log4cplus with `pkg_search_module` and linking
`-llog4cplus`, and `toffyConfig.cmake` correspondingly has no
`find_dependency(log4cplus)` — there is no imported target for a consumer to have
needed first. Both halves switch back together if the package is fixed; the
imported target is the better mechanism and this is a workaround, commented as
one where it is used.

Not toffy's problem to fix, and it will bite anyone: the CI runner installs from
the same archive, and an embedded sysroot that installs a dev package without its
transitive dev symlinks is the normal shape of the thing rather than the
exception. If log4cplus is going to be a real dependency of toffy deployments,
this is worth reporting upstream or pinning around deliberately.

---

## A build-configuration-dependent test that could not fail on half the fleet (found on the target, fixed by hoisting `OPENCV_TRACKING`)

`tests/test_filter_registry.cpp` is the `P2-4` fence. One of its tests asserts
that three type names — `reprojectopencv`, `kalmanAverage`, `reprojectpcl` — are
*not* creatable, pinning a known defect (their classes are compiled and linked,
but no configuration can name them) so that fixing it is a deliberate act.

On the 2026-11-20 target build (`toreo-vxl`, Ubuntu 18.04, OpenCV **4.1** without
the contrib `tracking` module) that test **passed with nothing behind it**:

- `modules/filters/src/smoothing/CMakeLists.txt` compiles `kalmanaverage.cpp`
  only `if (OPENCV_TRACKING_FOUND)`; the configure log says
  `opencv tracking module not found - will NOT build kalmanaverage`.
- `nm -D` on the resulting `libtoffy.so`: **0** symbols matching
  `kalmanaverage|cvTracker`.
- So `createFilter("kalmanAverage") == nullptr` was true because the class was
  never built, not because the registry declines the name. The test asserted a
  property of the build and reported it as a property of the factory.

Two things came out of writing the version that distinguishes them.

**`cvTracker` was missing from the fence entirely.** `tracking/init.cpp`
registers `CVTracker::id_name` under `#if OPENCV_TRACKING` — a macro that
directory *does* see — so `cvTracker` has been a working, registered,
name-creatable filter type throughout, and no test required it to stay that way.
Measured on a PCL-on/BTA-on host build: `hasCreator=1 creatable=1`. The fence
only protects the names it names, and this one was found by running the build on
a second machine rather than by reading the first one.

**`OPENCV_TRACKING` was directory-scoped, which is the same bug class it was
there to expose.** `add_definitions(-DOPENCV_TRACKING=1)` sat in
`modules/filters/src/{smoothing,tracking}/CMakeLists.txt` only. That is exactly
why the deleted `createFilter()` chain could never reach `kalmanAverage`: the
guard tested a macro `modules/core` never sees. It is now added at the top level
next to the `find_package(OpenCV ...)` that decides it, so one flag means one
thing tree-wide — including in `tests/`, which needs it to tell "not registered"
from "not compiled". No shipped code changes: the only sources testing the macro
are the two `init.cpp` files that already saw it.

The test now asserts non-creatability **only where the class is compiled**, prints
`[skipped] '<name>': <why>` where it is not, and fails if that leaves it with zero
assertions. Verified on both machines: on the host all three are asserted; on the
target it reports `[skipped] 'kalmanAverage': not built: OpenCV 'tracking' module
absent` and still asserts the other two; in `WITHOUT_PCL=ON` it reports
`[skipped] 'reprojectpcl': not built: PCL is off`.

Also measured while doing this, and worth knowing about anything else that asks
the factory a question: **`FilterFactory::hasCreator()` before the first
`getInstance()` answers `false` for every name in the product**, because
`getInstance()` is what populates the registry. A probe that asked `hasCreator`
first reported `hasCreator=0 creatable=1` for `cvTracker` — a contradiction that
was entirely an artefact of call order. The helpers in the fence warm the factory
first so the answer does not depend on which test ran before it.

---

## Boost.Filesystem was a dependency of `libtoffy` for two files, one of which needed it (fixed by `P2-18`)

`libtoffy.so` linked `libboost_filesystem` — `NEEDED libboost_filesystem.so.1.83.0`
on the host, `.so.1.81.0` on the target. Counting the users:

```
modules/filters/src/base/rectify.cpp            include + alias, uses nothing
modules/filters/src/base/polar2cart.cpp         include + alias, uses nothing
modules/filters/src/base/focus.cpp              include + alias, uses nothing
modules/filters/src/base/amplitudeRange.cpp     include + alias, uses nothing
modules/filters/src/capture/csv_source.cpp      include, uses nothing
modules/filters/src/smoothing/bilateral.cpp     include + alias, uses nothing
modules/filters/include/toffy/base/distAmpl.hpp include, uses nothing  <-- INSTALLED HEADER
modules/filters/src/capture/capturerFilter.cpp  uses it
modules/bta/src/bta.cpp                         uses it
```

**Seven of nine used nothing from it at all.** Each had `#include
<boost/filesystem.hpp>` and, in six cases, a `namespace fs = boost::filesystem;`
alias that named nothing either — the residue of edits that removed the calls and
left the include. One of the seven is an **installed header**, so toffy was
putting `boost/filesystem.hpp` into every consumer's include path to provide
nothing. That is the same defect `P2-17` removed for Boost.Log, found by counting
uses rather than includes.

The actual surface in the two real users was eight operations: `path`, `exists`,
`is_directory`, `is_regular_file`, `directory_iterator`, `create_directories`,
`absolute`, `filesystem_error`, plus three `path` members (`.string()`,
`.extension()`, `.parent_path()`). All of it is `stat` / `opendir` / `mkdir`, so
it is now `libraries/internal/toffy_internal/fs_posix.hpp` and
`filesystem` is out of `find_package(Boost)`.

Measured after: `libtoffy.so` `NEEDED` has **zero** boost entries; installed
headers including `boost/filesystem` **0** (was 1); `find_package(Boost)` asks for
5 components instead of 6; 16/16 `ctest` in Release, `WITHOUT_PCL=ON` and
`WITHOUT_BTA=ON`; 25 tree warnings unchanged; `abi_surface.sh` matches at 554
symbols; both consumers rebuild against a fresh install.

### Why the helper is not installed, and why that needed a new directory

`libraries/CMakeLists.txt:11` is `install(DIRECTORY "include/" DESTINATION
"include/")` — everything under `libraries/include/` becomes shipped API. A
helper used by two `.cpp` files is not API, and this programme's direction for
export surface is down, not up (see `P3-10`). So it lives in a new
`libraries/internal/`, which is on the include path and has no install rule, and
the include reads `<toffy_internal/fs_posix.hpp>` — the path itself tells a
consumer whether they are allowed to depend on it. Verified: `find
<stage> -name fs_posix.hpp` returns nothing.

### Three things the conversion found, deliberately not fixed in the same commit

* **`CapturerFilter::savePath()` computed a parent path and threw it away.** It
  warned `"is not a directory. Removing file name to get the path"`, assigned
  `fsPath = fsPath.parent_path()`, and then stored `_savePath = newPath` — the
  original string. The warning described something that never happened. Behaviour
  preserved; the dead call is gone. (`loadPath()`'s identical-looking line *is*
  live — its `fsPath` feeds the directory scan — so that one was ported, not
  deleted.)
* **`Bta::loadPath()` has an unreachable `exists()` check.** `if
  (fs::is_regular_file(fsPath) && ...)` and then, inside, `if
  (!fs::exists(fsPath))`. `is_regular_file` has already stat'ed it successfully.
  Kept, with a comment, so the commit stays a backend swap.
* **`boost::lexical_cast<int>` on a file stem becomes `std::stoi`.** Both throw on
  a non-numeric stem, so `loadPath()` still propagates rather than silently
  reading 0 and reordering the begin/end file selection — but the exception
  *type* changed (`bad_lexical_cast` → `std::invalid_argument`). Nothing in toffy,
  `toffy-oatpp` or `boxes` catches the old one (measured), so this is not a
  consumer-visible break; it is noted because it is the one place the swap is not
  literally identical.

One genuine behaviour difference, and it is an improvement: `boost::filesystem::
directory_iterator` **throws** if handed a non-directory, so `loadPath("foo.r")`
at the top level of a tree (`parent_path()` → `""`) threw. `opendir("")` just
fails, the listing comes back empty, and `loadPath()` returns `-1` — which is
what its signature has always been for.

### Sanitizer build trees live *inside* the source tree, and 1.5 GB of them went to the target because of it

`ci.yml` configures its sanitizer builds at `build-san/` and `build-tsan/`
**relative to the source directory**, so after any local CI-equivalent run the
source tree contains 931 MB + 621 MB of build output. They are not gitignored
away from a copy: `rsync -a --exclude='build' src/ target:` does **not** match
`build-san` or `build-tsan`, and that is exactly what happened when toffy was
copied to `toreo-vxl` — 1.5 GB landed on a device with 2.6 GB free and took it to
100%.

Two things worth changing, neither of which is urgent and both of which are cheap:

* move the sanitizer build dirs out of the source tree (`/tmp/toffy-san`, or
  `${CMAKE_BINARY_DIR}` under a sibling directory), so the source tree is the
  size it claims to be; and
* if they stay, make the ignore cover them — the pattern that matters is
  `build*/`, not `build`.

The general lesson is the one this programme keeps relearning: anything that lives
in the source tree is going to be copied, tarballed and shipped by someone
someday, and "it is only build output" is not a reason to put it where a copy
cannot tell it apart from source.
