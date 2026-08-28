title: Noise-adaptive plane segmentation thresholds
labels: robustness, phase-2

## Problem
All `SegmentPlanesOptions` defaults are absolute constants
(`segment_planes.hpp:26-31`: angle 25°, dist 0.07 m, min_inliers 1000). On
noisy scans this over-fragments (dozens of tiny planes -> cell complex
explosion); on sparse scans `min_inliers=1000` silently discards real
surfaces. This is a primary root cause of solid-model failures on complex
buildings.

## Tasks
- [ ] Estimate per-cloud noise (point-to-plane residual histogram on local
      PCA patches) and density
- [ ] Scale `plane_dist_threshold` (~3 sigma) and `min_inliers` (by density x
      minimum surface area) from the estimates; keep explicit overrides
- [ ] Warn when the output plane count is anomalous (e.g. > 60 or mean plane
      area below threshold)
- [ ] Benchmark + integration test on noisy synthetic room
      (`tests/support/synthetic_scene.hpp` with higher sigma)

## Acceptance
The same CLI invocation produces reasonable plane counts on clean and noisy
versions of the synthetic room.

category=Geometry estimate=3d
