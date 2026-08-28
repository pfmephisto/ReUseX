title: Cell complex guards: cell-count limit + non-manifold detection
labels: robustness, phase-2

## Problem
`CellComplexConstructor.cpp` builds a 2D arrangement whose complexity grows
~quadratically with plane count; over-fragmented input (noisy scans, 60+
planes) explodes it until the MIP is intractable. Non-manifold topology is
guarded only by `assert(twin != self)` (~line 257), which vanishes in
Release. Non-vertical planes throw or are dropped without a summary.

## Tasks
- [ ] Configurable `max_cells` (fail early with actionable message:
      plane count, suggestion to merge/filter planes)
- [ ] Replace topology asserts with runtime checks that throw descriptive
      errors
- [ ] Count and report dropped non-vertical planes once per run
- [ ] Log arrangement statistics (planes in, faces, cells) at info level

## Acceptance
A pathological input fails in seconds with a diagnosis instead of hanging in
the solver.

category=Geometry estimate=2d
