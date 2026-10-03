# Moving the controller layer to `toffy-oatpp`

The web control UI is not this project's job any more: it lives in the separate
**`toffy-oatpp`** project. It left `toffy` in `54d9577` ("dropped deprecated web
control"), which deleted `modules/control/` — 84 files: an asio HTTP server, a URI
grammar, nine controller classes and the `html/` front-end. `requestController.cpp` alone
was 776 lines.

What it left behind here is a residue: four installed headers, a hook in the bta plugin
entry point, three CLI options that describe a server that does not start, and a Doxygen
page that still tells the user to open `http://localhost:9999/`. This document is the
inventory, the evidence that removing the residue is ABI-neutral, and the contract
`toffy-oatpp` has to be able to build on.

## Two controllers, one name

| | | |
|---|---|---|
| `toffy::Controller` | `modules/core` | the **pipeline** controller: a `FilterBank` subclass, a run-state enum, one `boost::thread`. **Stays in toffy.** |
| `toffy::control::` | deleted | the **web** controller layer: `ControllerFactory`, `FilterController`, `Action`, per-filter controllers. **Lives in toffy-oatpp.** |

"Move the controller code" is ambiguous between these two, which is the first thing this
document exists to settle. Nothing in `modules/core` moves. `P2-9` wants
`Controller`'s members made private and `P2-13` wants its run methods collapsed — that is
core cleanup, not extraction.

## What is still in the tree

| residue | what it is | why it is dead |
|---|---|---|
| `libraries/include/toffy/web/common/plugins.hpp` | the `initUI(ControllerFactory*)` C entry point + `initUI_t` | includes `toffy/web/controllerFactory.hpp`, which exists nowhere → **cannot be included by anyone** |
| `libraries/include/toffy/web/actions/action.hpp` | `toffy::Actions::Action`, a `std::string id` + JSON serialiser | compiles, ships, and nothing in the tree includes it. The only one of the four a consumer could be using |
| `modules/bta/include/toffy/web/btaController.hpp` | `toffy::control::BtaController : FilterController` | base class deleted; `doAction()` is declared and defined nowhere → any user gets a link error |
| `modules/bta/include/toffy/web/btaGroupController.hpp` | `…BtaGroupController` | as above |
| `modules/bta/src/initPlugin.cpp`, `include/toffy/bta/initPlugin.hpp` | the `#ifdef WITH_CONTROL` half of the bta plugin entry point | `WITH_CONTROL` is defined by no build file, in any configuration, on any platform — and the `toffy_web/` headers it includes do not exist, so the branch cannot be compiled even if it were defined |
| `apps/main.cpp` | `--host/-h`, `--port/-p`, `--html/-d` options, parsed and printed | the values go nowhere. Every run prints `Host: localhost / Port: 9999 / Html path: /opt/toffy/html/`, i.e. advertises a server that never starts |
| `docs/mainpages/public/use.dox` | the whole "Use guide" page | documents `minimal_toffy` (no such binary; the app is `toffyRunner`), `WITH_EXAMPLES` (no such option), the web interface and port 9999 |
| `docs/extraDocs/images/control_{main,single,capturer}.png` | screenshots of the deleted UI | referenced only from `use.dox` |
| `Player`'s `@todo` | "Player is still not independent of the web control module… maybe creating a c++ Api that may be also use by the controller" | the design note that started this. The C++ API it asks for is stage **X5** below |

All four headers are *installed* — `install(DIRECTORY "include/" …)` in both
`libraries/CMakeLists.txt` and `modules/bta/CMakeLists.txt` ships their trees wholesale —
so they are part of the published public API surface and their removal is an API change
(`../dod/stage-gates.md` §2.4).

## Why removing it is ABI-neutral, and where it is not

`libtoffy.so` exports **no** symbol from the web layer: `nm -D` finds 0 matches for
`N5toffy7control`, 0 for `initUI`, 0 for `N5toffy7Actions`, 0 for `BtaController`. The
`WITH_CONTROL` blocks are never compiled, so nothing they declare ever reached an object
file. The gate is the `readelf --dyn-syms` snapshot diff from `DOD 2.4`: identical before
and after, not asserted to be — measured on `8689d68` and on this tree, Release/PCL-on/
BTA-on, **2 889** defined symbols on each side and an empty `diff`.

**One honest exception.** `toffy/web/actions/action.hpp` *does* compile standalone (it is
not in counter 17's failure list), so a downstream consumer could be including it today.
The other three cannot be included at all. So: three of the four removals cannot break a
compiling program; the fourth is a real, if improbable, API removal and needs the tag and
a release note like any other.

## The pathway

| stage | what | depends on | state |
|---|---|---|---|
| **X1** | delete the four `toffy/web/` headers and the `WITH_CONTROL` hooks (`P3-2`) | — | ✅ |
| **X2** | make every installed header compile, and keep it that way (`P3-12`) — 86 of 86 pass, gated by the `installed-headers` CI job | X1 | ✅ |
| **X2b** | close the two holes `X2` left: include guards + a double-inclusion pass (`N9`), and a source-tree gate for the `modules/bta` tree CI cannot build | X2 | ✅ |
| **X3** | drop `--host/--port/--html` from `toffyRunner` | — | ❌ |
| **X4** | rewrite `use.dox` around `toffyRunner`; move the three screenshots to toffy-oatpp | X3 | ❌ |
| **X5** | the C++ API `toffy-oatpp` binds to, and the fixes it needs first | `A23`, `P2-9`, `P2-7`, `P2-11`, `P2-13` | ❌ |
| **X6** | the plugin ABI: keep the filter half, do not rehost the controller half | — | decision, below |

### X2b — the two holes X2 left

`X2` compiled every installed header as the only include of a translation unit. That is the
shape of `N1`, and it is blind to the complementary shape: a header that compiles once and
not twice. `toffy/bta/FrameHeader.hpp` — the `typedef struct` for the on-disk `.rw` format —
was exactly that, and it is still installed (`findings/build.md`, `N9`).

The second hole is scope, and `X2` documented it without closing it: the runners have no
proprietary bta SDK, so `modules/bta` is never configured and its 7 installed headers are
never staged. Two of the four headers `X1` deleted lived there. `X2b` closes both:

- `#pragma once` on the three installed headers that had no guard (`FrameHeader.hpp`,
  `bta/initPlugin.hpp`, and the generated `toffy_config.h` template),
- a `twice` pass in `tools/installed_header_check.sh` — 86 headers × 2 passes = 172 checks,
  observed red on `FrameHeader.hpp` before the guard and green after,
- a `No controller residue in the source tree` step in the same CI job, grepping tracked
  code and build files for `toffy_web/`, `toffy/web/` and `WITH_CONTROL`. It needs no SDK,
  so it covers the bta tree; it is scoped to code, because the cleanup docs and the check
  script itself name those strings in order to record that they are gone.

ABI-neutral by construction and measured: a guard adds no symbols. Re-diffed against the
`8689d68` base after this change — still 2 889 defined symbols, `diff` empty.

X1–X4 are off the critical path and independent of the ownership work. X5 is *not*: it is
downstream of five open items, and pretending otherwise is how `toffy-oatpp` ends up
forking `toffy` instead of linking it.

### X5 — what `toffy-oatpp` needs, and what blocks it

The deleted layer's actual toffy usage was small: `FilterBank::{size, getFilter,
getFiltersByType}`, `Filter::{name, type, id, getState, getConfig, updateConfig}`,
`Controller::{forward, backward, stepForward, stedBackward, stop, getState, getFrame,
saveRunConfig, loadRuntimeConfig, getLoadedFilters}`, `Player::{loadConfig, runOnce,
getData, hasKey}`. Every one of those exists today. Five of them are not safe to bind to:

| need | today | blocker |
|---|---|---|
| report *why* a config was rejected | `updateConfig()` returns `void`; `loadConfig()` returns an `int` with no agreed meaning | `A23` — a remote UI cannot show an error message it cannot obtain |
| list the filters by name | `size()` + `getFilter(int)` works; `getFilter(name)` is a linear scan of `_pipe` | `P2-9` — a UI that lists *n* filters does *n²* scans |
| read the frame while the pipeline runs | `Controller::f` is public and `getFrame()` hands out a mutable reference to the one live blackboard | `P2-11` + `P2-7`. The old layer did `_c->f.hasKey(name)` and `_c->f.getData("cloud")` from the HTTP thread. That is a data race, and oatpp will reproduce it unless toffy offers a snapshot or a documented lock |
| run-state from another thread | `getState()` returns `_state`, a plain enum written by the pipeline thread | `P2-7` |
| call the run methods | `stedBackward()` | `P2-13` — the typo will be compiled into a binding and then be load-bearing |

**Practical rule until X5 lands:** `toffy-oatpp` should bind to `Player`,
`FilterBank::{size, getFilter, getFiltersByType}` and `Filter::{name, type, getConfig,
updateConfig}` only, and treat `Frame` as readable on the pipeline thread only. Those
signatures should not change without a tag; everything else in `Controller` is still in
flight.

The wire format is a separate decision and belongs to oatpp, but it inherits one contract
from here: the old layer read `Frame` slots **by name** (`"cloud"`, `"actions"`) and
serialised `cv::Mat` with `imencode` + base64 and `pcl::PCLPointCloud2Ptr` with
`boost::archive`. Those slot names and types are the interface. They are documented
nowhere, which is a `P2-11` problem, not an oatpp problem.

### X6 — the plugin ABI

Keep `extern "C" void init(toffy::FilterFactory*)` in `toffy/common/plugins.hpp`: it is
used, it works, and `toffy::bta::init(FilterFactory*)` is exported from the library.

Do **not** rehost `initUI(ControllerFactory*)` here. `ControllerFactory` was a copy of
`FilterFactory` for UI controllers, and its own comment said *"Right now it is not used
anywhere. Its the best way for expand control?, where to put it…"* — an unanswered design
question from the deleted layer. If oatpp needs a second registration point it should own
it; a header in toffy that names a class toffy does not define is exactly the artefact X1
removed.

## What does not move

`toffy::Controller`, `FilterBank`, `Player`, `Frame`, `FilterFactory` and the filter
plugin ABI are the product. A UI is one consumer of them. The mistake the old layout made
was putting the HTTP server and the per-filter UI adapters in the same repository, so that
`Player` grew a `@todo` about sending itself JSON over its own socket.
