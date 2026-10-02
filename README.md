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
