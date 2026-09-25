# Findings

Evidence, by subject. Every finding cites `file:symbol` so it can be checked independently.
The checked-in Doxygen docs were **not** used as a source of truth — see
[`documentation.md`](documentation.md) for why.

| File | Subject | Contents |
|---|---|---|
| [`correctness.md`](correctness.md) | **A** — correctness bugs | `A1`…`A23`, the ones that are defects rather than preferences |
| [`api.md`](api.md) | **B** — API & design | state machines, the `filter()` overload trap, layering, + `N2`, `N3` |
| [`ownership.md`](ownership.md) | **C** — ownership & memory | three owners one pointer, name-keyed teardown, leaked `dlopen` handles |
| [`concurrency.md`](concurrency.md) | **D** — concurrency | non-atomic stop flag, inter-process semaphore, unsynchronised singleton |
| [`build.md`](build.md) | **E** — portability & build | dead `MSVC` paths, the PCL/BTA axes, + `N1`, `N6`, `N7` |
| [`performance.md`](performance.md) | **F** — performance | per-frame and per-config-read costs |
| [`documentation.md`](documentation.md) | **G** — documentation | stubs, a page for a removed product, doc/code disagreement |
| [`security.md`](security.md) | **H** — security & robustness | `system()` on config data, config string as `printf` format |
| [`audit-2024-09.md`](audit-2024-09.md) | **I** — audit record | what the second pass re-measured, the two false zeros, the scope bug |

## Reading conventions

- A finding that is **fixed is marked, not deleted**: `FIXED (P0-n)` and a strike-through
  on the headline. The ids are cited by commit messages and by the plan, so the entry
  stays so the citation resolves.
- **Partially** fixed findings say exactly which half landed — `A12` (only the `joinable()`
  guard), `A21` (null check landed, the non-overwriting `_filters.insert` remains),
  `C3` (null guards landed, `clearCreators()` untouched).
- Where a fix was deliberately *not* made, the reason is recorded with the finding —
  `A23` is the clearest case: propagating `loadConfig()`'s return value breaks a valid
  empty `<cond>`, so the return value stays unchecked until the convention is decided.
- Findings are a record, not a to-do list. The work is in [`../plan/`](../plan/).
