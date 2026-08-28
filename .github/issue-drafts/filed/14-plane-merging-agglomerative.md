title: Replace greedy plane merging with agglomerative clustering
labels: robustness, phase-2

## Problem
`geometry/utils.cpp:163-206` merges planes greedily (first match wins) and
requires 0.8 overlap — two segments of the same wall scanned from different
viewpoints (overlap ~0.7, angle diff < 1°) stay separate, producing false
walls and inflating the cell complex.

## Tasks
- [ ] Agglomerative merging over a plane-distance matrix (angle + offset +
      inlier-weighted), merge until no pair is below threshold
- [ ] Reconsider the overlap criterion: coplanarity + adjacency may suffice
- [ ] Unit test: split-wall scenario merges; perpendicular planes never merge
- [ ] Benchmark on synthetic room with duplicated/offset wall patches

## Acceptance
The split-wall case merges to a single plane; plane count into the cell
complex drops on noisy scans.

category=Geometry estimate=2d
