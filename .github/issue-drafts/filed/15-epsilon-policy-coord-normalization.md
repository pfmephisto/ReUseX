title: Unified epsilon policy + coordinate normalization before CGAL
labels: robustness, phase-2

## Problem
Degenerate-geometry epsilons vary from 1e-15 (`CoplanarPolygon.cpp`) through
1e-9 (`Solidifier_to_mesh.cpp`) to absolute 0.1 (`utils.cpp` orthogonality),
with no unit/relative convention. Georeferenced coordinates (UTM-scale) are
fed to CGAL's inexact-construction kernel without recentering, degrading
precision. `dist_plane_point` divides by `squaredNorm()` with no zero guard.

## Tasks
- [ ] Named constants in `utils/` (e.g. `kEpsilonLength` [m],
      `kEpsilonAngle` [rad], `kEpsilonRelative`) + usage guidance in
      docs/STANDARDS.md §4
- [ ] Migrate the scattered literals
- [ ] Recenter clouds to their bounding-box centroid before cell-complex
      construction; restore offset on output
- [ ] Guard divisions by squared norms; validate plane normals are normalized
      at module boundaries

## Acceptance
Pipeline produces the same mesh (within tolerance) for a scan at origin and
the same scan translated by 500 km.

category=Geometry estimate=2d
