# G — Documentation

The checked-in docs are not a reliable description of the code, which is why this
document was written from the sources.

**The main pages are stubs.** `docs/mainpages/public/how_to_get.dox` consists of a heading
and `\todo do`. `modules.dox` ends with `\todo intro` for the entire Viewers section.

**Worse than a stub: `use.dox` documents a removed product.** It is a full page, but it
tells the reader to run `minimal_toffy` and open a web UI on `localhost:9999` with HTML at
`/opt/toffy/html`, and `\include`s `examples/config.xml`. No `minimal_toffy` target exists
anywhere in the repo (`grep -rn minimal_toffy` → no matches; `apps/` contains only
`toffyRunner` and `tst_pb`), and the web control was dropped in commit `54d9577`. A new
reader following this page cannot get to a running system.

**~~The docs misdescribe the buggy functions.~~ FIXED with the code.** `filterbank.hpp` once
documented `findPos()` as returning "negative if not found", which `size_t` cannot do (A3),
and documented both `remove()` overloads as "positive on success, negative or 0 in failed"
while they always `return 1` (A4). The doc comments were corrected alongside the fixes and now
match the code: `findPos()` says "or `std::nullopt` if no filter has that name", and
`remove(size_t)` says "1 if a filter was removed, 0 if `i` is out of range". This was the one
case where trusting the documentation actively hid a defect.

**Deprecated things are still used.** `filter.hpp` marks `_name` and
`Filter::loadFileConfig()` `@deprecated` ("we don't want a filter reading files, only the
filterbank"), yet `FilterBank::handleConfigItem()` still drives config loading through
`loadFileConfig()` for `<filterGroup>` recursion (`filterbank.cpp:110`).

**The most dangerous class is the least documented.** `filterThread.hpp` carries 10
`@todo document` markers — including on every queue, mutex and condition variable it uses
to synchronise across threads. Core contains **20** `@todo`s in total (down from 23; the
`Event` deletion removed 2 and `P2-8` resolved `filter.hpp`'s), concentrated in
`filterThread.hpp` (10), `filterbank.hpp` (5), `controller.hpp` (4) and `player.hpp` (1).
The previous sentence in this document claimed `filter.hpp` carried one after `P2-8` had
deleted it — the kind of half-update that makes a reader distrust the whole table.

**Commented-out API sketches stand in for real docs.** `frame.hpp:262-280` is a block of
planned `insGet`/`setGet` accessors, with `optBool` etc. duplicated as comments right above
the real declarations.

**Many doc comments restate the signature.** e.g. `@brief ~Filter`, `@brief Mux`,
`@param in` / `@param out` with no content — noise that hides the comments carrying real
information.

**Top-level docs were duplicated.** `README.md` and `Welcome.txt` were byte-identical 5-line
stubs (verified with `cmp`); neither mentioned the build, the module layout, nor the XML
config format. `README.md` now carries a documentation map into this directory;
`Welcome.txt` is still the old stub, and is still byte-identical to what `README.md` used to
be — deleting it is a one-line cleanup nobody has claimed yet.

**The Doxygen config is a committed generated file.** `docs/Doxyfile.cfg` is 104 KB, of
which the vast majority is stock `#`-commented defaults; only a few dozen lines are
project settings. It should be regenerated or reduced to the settings that matter.

**Docs are generated but never gated.** `docs/CMakeLists.txt` adds a `docs` custom target
that only runs when Doxygen is found and only builds on demand. CI builds and tests the code
but does **not** build docs, so broken `\ref`s and `\todo` accumulation remain invisible —
that is the one gate the new workflow deliberately does not close yet. (`BUILD_DOC` is not
the mechanism: it is referenced nowhere in the build.)

**Config reference is XML-only.** `docs/configDocs/xmls/` holds one XML per filter, and
`bta.xml` exists **three** times — `docs/configDocs/bta.xml`, `docs/descDocs/bta.xml` and
`docs/configDocs/xmls/bta.xml` — duplicated reference material that will drift.

**~~The dead product is installed, not just documented.~~ Headers FIXED (`P3-2`); the page
is `X4`.** `use.dox` was the documentation half of the problem; the other half *shipped*.
`make install` installed `libraries/include/toffy/web/` and `modules/bta/include/toffy/web/`
— four headers from the web control UI removed in `54d9577`, one of which includes a file
that does not exist anywhere in the repository, so it cannot be compiled by anyone. Evidence
and the reproduction: [`build.md`](build.md). `P3-2` deleted them.

That closes the shipping half, and it is the half that mattered: a doc page can be ignored,
an installed header cannot. What is left of the product in the docs is now counted rather
than estimated — counter 16 in [`../counters.md`](../counters.md) enumerates `use.dox`, the
three `control_*.png` screenshots, the three `toffyRunner` options and the `Player` `@todo`.
`X3` and `X4` in
[`../plan/controller-extraction.md`](../plan/controller-extraction.md) close them; the
screenshots belong to `toffy-oatpp`, which is where the UI they show now lives.
