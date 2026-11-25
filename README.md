# toffy 

This is a data-flow oriented library that encapsulates functionality from [OpenCV](https://opencv.org/) and [PointClouds](https://pointclouds.org/).

It has been initially developed for handling ToF based data and interfaces easily to from [Becom Electronics](https://www.becom-group.com/whatwedo/3dsensoren/) cameras, but can be adapted to other use cases easily.

## Documentation

The Doxygen sources in `docs/` are **stale** — `docs/mainpages/public/use.dox` still
describes a `minimal_toffy` binary and a web UI that no longer exist. Read the headers,
and these, first:

| | |
|---|---|
| [`cleanup/overview.md`](cleanup/overview.md) | What the code is: the `Frame` / `Filter` / `FilterBank` / `FilterFactory` model, repository layout, build facts, what happens on a run |
| [`cleanup/counters.md`](cleanup/counters.md) | Every measured number in one place, with the command that produces it |
| [`cleanup/`](cleanup/) | The `modules/core` cleanup programme: findings, plan, order, definition of done |

The mental model, in four nouns: a `Frame` is a string-keyed type-erased blackboard, a
`Filter` is `bool filter(const Frame& in, Frame& out)` configured from XML, a
`FilterBank` is a Filter that is also an ordered pipeline of Filters, and a
`FilterFactory` maps type-name strings to constructors. `Controller` and `Player` drive
it. `modules/core` holds all of that; `modules/filters` holds the processing filters and
`modules/bta` the Becom driver wrapper.

## Building

```sh
cmake -S . -B build                 # -DWITHOUT_PCL=ON, -DCMAKE_DISABLE_FIND_PACKAGE_bta=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

`CMAKE_BUILD_TYPE` defaults to `Release`; `Debug` defines `CM_DEBUG`, which changes
`Filter`'s initial log level, so both are worth building. Optional dependencies are
detected by `find_package` and switch on `PCL_FOUND` / `HAS_BTA` — see
[`cleanup/overview.md`](cleanup/overview.md) for what each configuration does and does not
cover.

| Option | Default | Effect |
|---|---|---|
| `WITHOUT_PCL` | `OFF` | exclude the point-cloud classes; supported, and built in CI |
| `WITH_VISUALIZATION` | `ON` | viewer objects, needs a GUI |
| `WITH_PCL_CLOUDVIEW` | `OFF` | the PCL cloud viewer |
| `BUILD_TESTS` | `ON` | the `ctest` suite in `tests/` |
| `BUILD_STATIC` | `OFF` | static library instead of the shared one |

All four PCL × BTA combinations are expected to configure, build and pass `ctest`; see
[`cleanup/dod/per-pr.md`](cleanup/dod/per-pr.md).

`Welcome.txt` is a verbatim duplicate of this file's opening five lines (verified with
`diff`, not assumed). It is generated from nothing, so it drifts the moment one of those
lines is edited here and not there.

## Formatting

```sh
cmake --build build --target format         # rewrite modules/core in place
cmake --build build --target format-check   # check, do not write; non-zero on any deviation
```

Both come from [`cmake/Formatting.cmake`](cmake/Formatting.cmake) and cover
**`modules/core` only** — the same scope as `CORE_WERROR` (P3-4), because that is the
part of the tree the cleanup programme is actually holding to a standard. The rest of the
tree is not gated and is not formatted.

**Pinned version: clang-format 18.** The target looks for `clang-format-18` first and
falls back to whatever `clang-format` is on `PATH`, and warns at configure time if the
major differs. Different majors disagree about output, so a gate that accepted any version
would flip depending on which machine ran it.

Formatting your own changes, if you are working outside `modules/core`:

```sh
clang-format-18 -i path/to/file.cpp     # or: git clang-format --diff  (staged changes only)
```

### `format-check` is red today, and that is the accurate answer

The tree has never been formatted. Measured on `bcad264`: **22 of the 23 files in
`modules/core` deviate, 687 changed lines.** The gate is not wired into CI for exactly
this reason — a gate added before the code complies is a permanently red build, and
nobody keeps a permanently red build.

### Before you run `make format`: it is not whitespace-only

`P2-15` assumed a `clang-format -i` commit would be whitespace-only. It is not. Checked
by comparing the token stream of every file before and after, not by eyeballing the diff:

- `#define UNUSED(expr) do { (void)(expr); } while (0)` in `filter_helpers.hpp` gets split
  across four lines with backslash continuations.
- A string literal in the same header gets split: `"the only argument supplied"` becomes
  `"the only argument" "supplied"`. Adjacent-literal concatenation makes the runtime string
  identical, but it is a content change and it breaks grepping for that message.
- Block comments get their `*` realigned, and `// namespace toffy` closing comments are
  added where they were absent.

All of it is semantically inert — verified, not assumed: after formatting, `format-check`
converges (rc 0), a second `format` run changes nothing, the tree builds with its warning
count unchanged, `19/19 ctest` passes, and `abi_surface.sh` reports the same **563**
symbols against the BTA-off baseline.

So the reformat is a decision about whether those three rewrites are acceptable, not a
checkbox. If they are, the commit should be recorded in `.git-blame-ignore-revs` so
`git blame` skips it, and the CI gate should land **in that same commit**:

```yaml
- name: formatting (P2-15)
  run: cmake --build build --target format-check
```

See [`cleanup/plan/build-and-hygiene.md`](cleanup/plan/build-and-hygiene.md) (P2-15).
