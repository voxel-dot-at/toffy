# Toffy `modules/core` cleanup — document map

This directory holds the whole cleanup programme. Each file is one chapter and is meant
to be read on its own; nothing here is longer than ~200 lines.

| Read this | When |
|---|---|
| [`counters.md`](counters.md) | **You want a number.** The single source of measured figures. |
| [`status.md`](status.md) | What has landed, what is in flight, what the tag situation is. |
| [`order.md`](order.md) | What order to do the rest in, and why. |
| [`overview.md`](overview.md) | What the code is: layout, build facts, the run path. |
| [`plan/`](plan/) | The numbered work items. |
| [`findings/`](findings/) | The evidence behind them, by subject. |
| [`dod/`](dod/) | The gates a PR has to pass. |

## Rules that keep these files from rotting

1. **Numbers live in [`counters.md`](counters.md) only.** Every other chapter links to
   it. Three documents restating the same counter is how the last round ended up with
   `19` in one file and `12` in another for the same grep.
2. **Every number carries its scope.** `modules/core`, `modules/`, and the whole tree are
   three different answers to most of these questions, and two of them have been quoted
   as if they were one ([`findings/audit-2024-09.md`](findings/audit-2024-09.md)).
3. **Cite by id, not by line number.** `P2-5`, `A12`, `DOD 2.4`, `S4`. File:line citations
   drift the moment someone edits the file they point at — several in the previous
   documents already had.
4. **A finding that is fixed is marked, not deleted.** The `A`/`B`/`C`… ids are cited by
   the plan and by commit messages; removing an entry breaks those references.
5. **Docs change in the PR that changes the behaviour** (`dod/per-pr.md` #6).

## Ids

| Prefix | Meaning | Defined in |
|---|---|---|
| `P0-n`, `P1-n`, `P2-n`, `P3-n` | numbered work item | [`plan/`](plan/) |
| `A1`…`A23`, `B`, `C`, `D`, `E`, `F`, `G`, `H`, `I` | finding, by subject | [`findings/`](findings/) |
| `S1`…`S9` | second-pass suggestion; `P3-n` is its work item | [`plan/second-pass.md`](plan/second-pass.md) |
| `X1`…`X6` | controller-extraction stage; `X1` = `P3-2`, `X2` = `P3-12` | [`plan/controller-extraction.md`](plan/controller-extraction.md) |
| `N1`…`N8` | second-pass finding; filed into the subject chapters | [`findings/audit-2024-09.md`](findings/audit-2024-09.md) |
| `DOD 1.1`, `DOD 2.4`, … | gate | [`dod/`](dod/) |
| `PR n` | the delivery unit | [`order.md`](order.md) |

## Where the old documents went

`PROJECT_INFO.md`, `CLEANUP_PLAN.md`, `CLEANUP_SEQUENCE.md`, `DOD.md` and
`CLEANUP_SUGGESTIONS.md` at the repository root were split into these chapters and
merged, because they had grown to 2 100 lines between them and were disagreeing with each
other. The mapping:

| Was | Now |
|---|---|
| `PROJECT_INFO.md` §status, §measured | [`counters.md`](counters.md), [`status.md`](status.md) |
| `PROJECT_INFO.md` §1–§5 | [`overview.md`](overview.md) |
| `PROJECT_INFO.md` §6 A–G | [`findings/`](findings/) (one file per letter) |
| `CLEANUP_PLAN.md` P0 / P1-1 | [`plan/p0-correctness.md`](plan/p0-correctness.md), [`plan/test-harness.md`](plan/test-harness.md) |
| `CLEANUP_PLAN.md` P2-2, P2-3, P2-15, P2-16 | [`plan/build-and-hygiene.md`](plan/build-and-hygiene.md) |
| `CLEANUP_PLAN.md` P2-4 … P2-10 | [`plan/structure.md`](plan/structure.md) |
| `CLEANUP_PLAN.md` P2-11 … P2-14 | [`plan/api.md`](plan/api.md) |
| `CLEANUP_SEQUENCE.md` | [`order.md`](order.md), [`status.md`](status.md) |
| `DOD.md` | [`dod/`](dod/) |
| `CLEANUP_SUGGESTIONS.md` §1, §N8 | [`findings/audit-2024-09.md`](findings/audit-2024-09.md) |
| `CLEANUP_SUGGESTIONS.md` §N1–N7 | [`findings/build.md`](findings/build.md), [`findings/api.md`](findings/api.md), [`findings/security.md`](findings/security.md) |
| `CLEANUP_SUGGESTIONS.md` §4 | [`plan/second-pass.md`](plan/second-pass.md) |

Item numbering note: the old plan put *Windows paths* and *debug leftovers* under a `## P1`
heading while every other document cited them as `P2-2` and `P2-3`. The numbering used
everywhere else is the one kept — `P1` has exactly one item, the test harness.
