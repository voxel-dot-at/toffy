# Definition of done — stage gates

Only the gates matching the PR kind apply.

### 2.1 Correctness PRs (PRs 3–6, the `P0` items)

- The regression test is committed **before** the fix commit, and the PR body shows it
  failing on the parent commit.
- PR 3 (`_pipe` mutation: `remove()`, `findPos()`, `stop()`) additionally passes an
  **ASan/UBSan** build. That path was live undefined behaviour, so a green normal build
  proved very little. This is now automated: the `sanitizers` job in
  `.github/workflows/ci.yml` runs the whole suite under ASan + UBSan + LeakSanitizer with
  `detect_leaks=1` on every push, so it no longer has to be run and pasted by hand.
- `stop()` is asserted to actually stop: a test drives a bank, calls `stop()`, and asserts
  every child reports `filterIdle`. **Done** — `FilterBankStop.StopStopsChildren`.
- The other PR-3 fixes are pinned too: `FilterBankRemove.ByIndexRemovesTheElementAtThatIndex`,
  `FilterBankRemove.MissingNameLeavesBankIntact`, `FilterBankFindPos.MissingNameYieldsNoValue`.
  Each was confirmed to fail on its parent commit before the fix landed.

### 2.2 Threading PR (PR 12)

- **TSan clean** on a run that exercises `ParallelFilter` lanes, and **ASan clean** on the
  same run. A green `ctest` is weak evidence for a data race; a green TSan run is not.
- The stop path is exercised repeatedly (≥100 iterations of start/stop) rather than once —
  the `keepRunning` race and the `join()`-on-non-joinable-thread bug are both timing-dependent.
- No `boost::thread` object is ever assigned while joinable.

### 2.3 Ownership PR (PR 10, the keystone)

- A full `toffyRunner` session under ASan/valgrind reports **zero leaks and zero
  double-frees**, including shutdown.
- `grep -rn "delete " modules/core/src` returns no `delete` of a `Filter`.
- The name-based teardown is gone: no `deleteFilter(name())` call remains in
  `clearBank()` or `~Controller`.
- `FilterPtr` is used consistently — the factory and `_pipe` no longer hold raw `Filter*`.

### 2.4 API-changing PRs (PRs 2, 16, 17, and `P3-2`, `P3-10`)

- Every renamed or removed public symbol has a deprecated alias, or an explicit note that
  this is a breaking release.
- `modules/filters`, `modules/bta`, `libraries/` and `apps/` all still compile — an API
  change that only builds `modules/core` is not done.
- **Compiling is not the gate.** Three filters in `modules/filters` would keep compiling and
  silently stop processing frames if `P2-12` were applied as written, because they override
  only the `const` `filter()` overload and never said `override`
  ([`../findings/api.md`](../findings/api.md), N2). So: **a filter that overrides only one
  `filter()` overload is exercised by a test that asserts its body ran** (`P3-9`). Add the
  equivalent for any other change whose failure mode is "resolves to the base default".
- The Doxygen comment for each changed symbol is updated in the same PR (per-PR gate 6).
- **A change that claims to be ABI-neutral has to prove it.** Snapshot
  `readelf --dyn-syms -W build/libtoffy.so | awk '$7!="UND"{print $8}' | sort` before and
  after and diff. That is how `P3-10` (deleting `TOFFY_EXPORT`) is shown to ship an
  identical exported symbol set rather than asserted to.
- **A new version tag is pushed.** This is not ceremony: the library version *and*
  `SOVERSION` are derived from `git_describe` (`CMakeLists.txt:31`, parsed at `:35-39`;
  `SOVERSION` set at `:388`). An
  API change with no new tag therefore ships under the previous `SOVERSION`, so the
  SONAME keeps advertising the old release while the exported mangled symbols have
  already changed — downstream binaries then fail at load time against a library that
  claims to be the version they linked.

  Two worked examples, both real:
  - **`v1.7.0`** — exists only to mark the `findPos()` / `remove()` signature change.
  - **`v1.10.0`** — marks the deletion of `toffy::Event` and the removal of the public
    virtual `Filter::processEvent()`, which shrinks `Filter`'s vtable by one slot. Before
    the tag, this tree built `libtoffy.so.1.7.1` with SONAME `libtoffy.so.1.7.1` while
    exporting **none** of the `Event`/`processEvent` symbols the real 1.7.1 exported — the
    hazard above, demonstrated with `readelf` rather than asserted. After the tag the SONAME
    is `libtoffy.so.1.10.0` and a stale binary refuses to load.

  Numbering note: `1.10.0` rather than the convention-implied `1.8.0`, so the number does
  not sort below `v1.9.0`, which exists on `origin/next`. That branch diverged from
  `b6d7665` and is not an ancestor of the cleanup line, so the two schemes still need
  reconciling when they meet.

  Also worth knowing before the next bump: `SOVERSION` is set to the **full** version, not
  the major alone, so every patch bump changes the SONAME and forces a downstream relink.
  That is why version decisions here cannot be routine.
- Run `tools/api_change_report.sh [BASE]` and put its output in the PR description. It
  exits 1 when any installed header under `*/include/` changed, 2 if the base ref is
  invalid (it fails closed on purpose), and 0 otherwise. The declaration listing is
  advisory — text diffing cannot be exact — but the exit code is the gate.

```sh
tools/api_change_report.sh "$(git describe --tags --abbrev=0)"   # exit 1 => tag required
```

### 2.5 Layering PRs (PR 13, `P3-8`, `P3-10`)

- `grep -rn "include <toffy/viewers" modules/core` returns **nothing**: core must not include
  from the viewers module.
- `grep -rn "bta" modules/core/include/toffy` returns nothing after `btaFrame.hpp` moves.
- The link line for the core library no longer references the viewers library.
- No directory lower in the stack consumes a target owned by one above it:
  `libraries/CMakeLists.txt` must not name `toffy_tracking` (`P3-8`).
- **No public header includes a generated one.** `grep -rn "toffy_export.h" --include=*.hpp
  modules libraries` returns nothing (`P3-10`). `toffy_config.h` stays generated — it is
  genuinely used — but it is included where it is needed, not from the API surface.

### 2.6 Formatting PR (PR 18)

- Pin the version: `clang-format 18.1.3` is what the dev container has, and `.clang-format`
  sets `UseTab: Never`, so the 11 tab-bearing core files are non-conforming by the repo's own
  config rather than by taste. (The item used to say clang-format was not installed at all;
  it is.)
- The diff is provably whitespace-only:
  `git diff --ignore-all-space --exit-code <base> HEAD` exits **0**.
- The commit SHA is added to `.git-blame-ignore-revs`.
- The CI format check is added **in this same PR**, so the tree cannot drift afterwards.
