title: Central label-semantics module (one encoding contract)
labels: refactor, data-model, phase-1

## Problem
Four different "unlabeled" encodings coexist with inline casts between them:
- storage PNG: CV_16U, 0 = background (label+1) — `ProjectDB.cpp:1325-1340`
- segmentation API: CV_32S, -1 = background
- point clouds: uint32_t, 0 = unlabeled — but `segment_rooms.cpp:113`
  initializes to -1 (wraps to 0xFFFFFFFF)
- reconstruction casts int -1 to uint32_t inline (`reconstruct.cpp:205`)

A label >= 65535 silently wraps when stored as CV_16U. Off-by-one hazards at
every `label - 1` vector index.

## Tasks
- [ ] New `core/label_semantics.hpp`: named constants (`kUnlabeled`), typed
      conversion helpers storage<->api<->point, range-checked (throw on
      overflow instead of wrapping)
- [ ] Migrate `ProjectDB`, `reconstruct`, `project`, `segment_*` to use it
- [ ] Fix `segment_rooms` to use 0-as-unlabeled per docs/STANDARDS.md §3
- [ ] Unit tests incl. boundary labels (0, 1, 65534, 65535)

## Acceptance
No inline label offset arithmetic remains outside the helper module
(`grep -rn "label - 1\|label + 1" libs/` reviewed).

category=Geometry estimate=2d
