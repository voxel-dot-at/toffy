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

**…and the deletion was incomplete.** `libraries/sensor/include/toffy/imagesensor.hpp`
still defines `DLLExport`, `WIN` and `UNIX` and still annotates `class DLLExport
ImageSensor`. It is a pre-`P2-3` copy of `modules/bta/include/toffy/io/imagesensor.hpp`,
has no `CMakeLists.txt`, is in no `add_subdirectory`, is on no include path and is not
installed — so the counters read 0 while the pattern sat in the tree. `P3-1` deletes it.

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
- **It drags a build artifact into the public API.** 8 public headers do
  `#include <toffy/toffy_export.h>`, so including `toffy/frame.hpp` requires CMake's
  generated include dir, and `make install` has to ship `toffy_export.h` for the rest of
  the installed headers to parse.

27 uses across 15 headers, 8 includes. `P3-10` deletes them and keeps
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

### N1 — `make install` ships a public header that does not compile

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

**Suggestion S2 — delete `libraries/include/toffy/web/` and `modules/bta/include/toffy/web/`.**
This is an API removal, so per `DOD 2.4` it needs a version tag and a run of
`tools/api_change_report.sh`. It is a removal of headers that cannot compile, so the risk is
close to nil, but the gate still applies. Doing it also removes the need for the `use.dox`
rewrite in the same area: the page and the headers describe the same dead product.

### N6 — the `if( ${VAR} )` bug class is still in the build

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

**Suggestion S7 — one mechanical commit: `if( ${VAR} )` → `if(VAR)`** for the four sites above
(`MATCHES` comparisons like `if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")` are correct as written
and should stay). Worth a CI line too: `grep -rn 'if[ ]*([ ]*\${' --include=CMakeLists.txt`
excluding `MATCHES`.

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
