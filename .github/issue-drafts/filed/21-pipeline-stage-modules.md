title: Restructure library into pipeline-stage modules with enforced boundaries
labels: refactor, epic, phase-3

## Problem
`libs/reusex/geometry/` conflates four pipeline stages (registration,
segmentation, reconstruction, meshing) in one module compiled into one CMake
target. Module layering is convention-only, swapping a stage implementation
(notably SLAM) requires touching a grab-bag module, and parallel agent work
collides in the same directories.

## Proposed structure
```
core            ProjectDB, logging, types, label semantics (stage contracts)
slam            registration/, pose refinement, densify
vision          (unchanged)
segmentation    segment_planes, segment_rooms, segment_instances
reconstruction  CellComplex*, Solidifier*, mesh, texture_mesh, create_windows
io              format conversion only
visualize       (unchanged)
```

## Mechanisms (the actual value)
- [ ] One CMake target per module, explicit `target_link_libraries` per the
      layer diagram in docs/STANDARDS.md §1 — illegal dependencies become
      link errors
- [ ] Stage contracts documented in core: named clouds/tables each stage
      consumes/produces; `rux validate --stage <x>` checks them
- [ ] Within-stage swappability via strategy interfaces (pattern:
      vision::IMLBackend), NOT more modules
- [ ] Behavior-neutral: integration test + benchmarks must be identical
      before/after (requires Phase 0 harness green first)

## Notes
- Absorbs the header-hygiene issue (types.hpp split, observer out of core)
  where natural — do moves and include-trims together per module
- Do one module at a time (suggested order: segmentation, reconstruction,
  slam), each its own PR

category=Geometry estimate=1w
