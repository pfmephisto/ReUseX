title: Header hygiene: split types.hpp, slim ProjectDB.hpp, observer out of core
labels: refactor, build-speed, phase-3

## Problem
Compile-time coupling islands slow every agent iteration (5-15 min verify
cycles):
- `types.hpp` force-includes PCL + Eigen into nearly every TU
- `core/ProjectDB.hpp` exposes OpenCV Mat + PCL mesh types to all DB clients
- `core/processing_observer.hpp` pulls PCL/Eigen into core, coupling core to
  visualization concerns
- `geometry/CellComplex.hpp` instantiates Boost.Graph templates in a public
  header

## Tasks
- [ ] Split `types.hpp` (point types vs Eigen helpers); include only what's
      needed per module
- [ ] Forward-declare cv::Mat / pcl mesh types in `ProjectDB.hpp` where
      possible (Pimpl already hides sqlite)
- [ ] Abstract observer interface in core; PCL-typed payloads move to a
      visualization-side header
- [ ] Measure: record clean + incremental build time before/after (ccache on)
- [ ] Add ccache to devshell/CI if not present

## Acceptance
Incremental rebuild after touching a geometry .cpp drops measurably;
`docs/STANDARDS.md` §1-2 boundaries hold for core.

category=Geometry estimate=1w
