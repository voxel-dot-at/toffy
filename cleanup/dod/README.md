# Definition of done

Completion criteria for the `modules/core` cleanup. Items:
[`../plan/`](../plan/). Order: [`../order.md`](../order.md). Numbers:
[`../counters.md`](../counters.md).

Three levels:

| File | Applies to |
|---|---|
| [`per-pr.md`](per-pr.md) | every PR, no exceptions |
| [`stage-gates.md`](stage-gates.md) | extra gates by kind of PR — correctness, threading, ownership, API, layering, formatting |
| [`programme.md`](programme.md) | when the cleanup as a whole is finished |

**A PR is not done because the code compiles. It is done when the thing it claims to fix
cannot silently come back.**

Two rules the last round of this document did not follow, and which its own verification
block then reported as passing:

1. **A check that cannot fail is not a gate.** Two criteria reported `0` for weeks while the
   pattern they hunted sat in `libraries/`, because the greps were scoped to `modules/`.
   Every check states its scope, and the scope is part of the claim.
2. **A number in a gate is a number in [`../counters.md`](../counters.md).** Gate documents
   quote targets; the measured value is written down once. That is the only way the two
   stay honest with each other.
