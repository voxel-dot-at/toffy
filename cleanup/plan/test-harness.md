# P1-1 — Add a test harness first — **DONE**

Complete. `enable_testing()` plus 7 `add_test()` targets (gtest) across `frame`, `filter`,
`filterbank`, `filterfactory`, `controller`, `player` and `cond`, and
`.github/workflows/ci.yml` now builds and runs them on every push and PR — a PCL-on/PCL-off
matrix plus a separate ASan/UBSan/LeakSanitizer job. The first tests pin the P0 fixes as
specified here: `remove()` on a missing name, `findPos()` on an empty bank, `Frame`
copy/assign preserving `meta` + `desc`, and `stop()` actually stopping.

Two things this caught that were not predicted: a `FilterConstruction` assertion that was
only ever run in Release and fails in Debug (the constructor sets `debug` under `CM_DEBUG`),
and `test_cond` leaking its `Player` — found by the sanitizer job, not by review.

The paragraphs below are kept as the original rationale.

---

There is no `enable_testing()`, no `add_test()`, no gtest/catch2 anywhere. `tests/` holds a
single 44-line `test_cond.cpp` built as a bare executable that nothing runs. CI
(`.github/`) is Codacy + Flawfinder only — static-analysis bots with **no build and no test
gate**, which is exactly why items A1–A22 have survived.

Suggested minimum, before any P2 work:

```cmake
enable_testing()
add_subdirectory(tests)          # tests/CMakeLists.txt
```

```cmake
# tests/CMakeLists.txt — one target per area
add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame toffy)
add_test(NAME frame COMMAND test_frame)
```

The first tests should pin the P0 fixes: `remove()` on a missing name, `findPos()` on an
empty bank, `Frame` copy/assign preserving `meta` + `desc`, and `stop()` actually stopping.
Those tests are the regression fence for everything in P2.
