# G — Documentation

The checked-in docs are not a reliable description of the code, which is why this
document was written from the sources.

**The main pages are stubs.** `docs/mainpages/public/how_to_get.dox` consists of a heading
and `\todo do`. `modules.dox` ends with `\todo intro` for the entire Viewers section.

**~~Worse than a stub: `use.dox` documents a removed product.~~ FIXED (`X4`).** It was a
full page telling the reader to run `minimal_toffy` and open a web UI on `localhost:9999`
with HTML at `/opt/toffy/html`. No `minimal_toffy` target exists anywhere in the repo
(`apps/` contains `toffyRunner` and `tst_pb`), and the web control was dropped in
`54d9577`. The page is now a `toffyRunner` guide: real options, the checked-in example
config, where the configs live, how to stop — and one paragraph for whoever typed
`localhost:9999` into a search engine to say where that UI went (toffy-oatpp).

**One correction to this finding, and it matters.** The page could not be reached through
the generated documentation at all: `docs/Doxyfile.cfg` had it in `EXCLUDE` alongside the
`how_to_get.dox` stub. Measured — a doxygen run read 12 files and `use.dox` was not among
them, and no `use.html` was emitted. So "a new reader following this page cannot get to a
running system" was too generous: the page was not a page, it was a source file that looked
like one. That is why `X4` had to delete it from `EXCLUDE` and add `\page use` to
`page_order.dox`, not just rewrite the prose — a doc fix that leaves the page excluded fixes
nothing a reader will notice.

**The docs build is broken in three places and nothing runs it.** `status.md` already
listed "nothing builds the Doxygen docs" as a CI limit; the cost is that the breakage below
has been there since the `docs/` reorganisation in `59b6bcf`. Measured with doxygen 1.9.8 on
the `docs/Doxyfile.cfg` as checked in:

| symptom | cause |
|---|---|
| `warning: source '…/descDocs' is not a readable file or directory… skipping.` | `EXAMPLE_PATH` lists `@PROJECT_SOURCE_DIR@/descDocs`; the directory is `docs/descDocs` |
| `error: Extra file '…/docs/extraDocs/filters/roi_guide.odt' specified in HTML_EXTRA_FILES does not exist!` | the file is not in the repo; that directory holds `roi_extra.dox` |
| 13 × `warning: Tag '…' has become obsolete.` | the config header still says `# Doxyfile 1.8.12`; `PERL_PATH`, `MSCGEN_PATH`, `HTML_TIMESTAMP`, `RTF_SOURCE_CODE`, `DOT_FONTNAME` … |

None of these stop the build (`docs` exits 0), which is exactly why they are still there.
The fix is a CI job that runs the `docs` target and reads its warnings — the same lesson as
`P3-11` and `P3-12`: a check nobody runs is a check nobody passes.

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
