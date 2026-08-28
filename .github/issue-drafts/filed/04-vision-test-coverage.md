title: Vision module test coverage (Dataloader, backends, annotation)
labels: testing, phase-0

## Problem
The vision module has ~11 test cases in 2 files (`tests/unit/vision/`) for the
entire ML stack: Dataloader, backend factory, YOLO/SAM post-processing, NMS,
annotation pipeline, 3D-2D projection. `nms.cpp`, `project.cpp`,
`Dataloader.cpp` have zero coverage. Agent changes to vision code are
effectively unverifiable.

## Tasks
- [ ] Tests for `BackendFactory::detect_backend()` (extension mapping, errors)
- [ ] Tests for NMS with hand-constructed boxes
- [ ] Tests for `project.cpp` label projection with a synthetic camera + known
      geometry (no GPU needed)
- [ ] Tests for Dataloader batching/ordering (needs seedable shuffle — see
      deterministic-dataloader issue)

## Acceptance
Vision unit tests cover the CPU-reachable logic; GPU-only paths documented as
untested.

category=Vision estimate=2d
