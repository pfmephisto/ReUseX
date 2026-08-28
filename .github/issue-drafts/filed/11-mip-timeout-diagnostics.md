title: MIP solver timeout + infeasibility diagnostics in Solidifier
labels: robustness, phase-2

## Problem
`Solidifier.cpp:93-96`: `solver.solve()` has no time limit and failure yields
only "MIP solver failed to find solution" + empty return. On complex
buildings the solver can hang indefinitely or fail with zero insight into
why (infeasible? timeout? too many cells?).

## Tasks
- [ ] Add `time_limit_seconds` (default ~120 s) and expose it up through
      `MeshOptions` and the `rux create mesh` CLI
- [ ] On failure, log the solver status (infeasible / time limit / error),
      problem size (cells, faces, variables, constraints), and a hint
      (e.g. "412 cells — consider stronger plane merging")
- [ ] Expose `alpha` (hardcoded 0.04 at `Solidifier.cpp:37`) as an option
- [ ] Unit test: infeasible toy problem reports status, oversized problem
      respects timeout

## Acceptance
A failed `rux create mesh` tells the user *why* and *what to try*.

category=Geometry estimate=1d
