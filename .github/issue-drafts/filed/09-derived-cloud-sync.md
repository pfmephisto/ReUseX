title: Keep derived clouds in sync on downsample/filter (or invalidate loudly)
labels: bug, data-model, phase-1

## Problem
`rux edit downsample` rewrites "cloud" but leaves "labels", "normals",
"planes", "rooms", "instances" and `instance_materials` untouched
(`apps/rux/src/edit/downsample.cpp`), silently desynchronizing every derived
product. The `VoxelAssignment` mechanism (`geometry/downsample.hpp`) already
supports downsampling parallel clouds consistently — it just isn't applied
across the board.

## Tasks
- [ ] `edit downsample`/filter operations enumerate all same-size sibling
      clouds and either transform them with the same assignment/indices or
      refuse with a clear message listing what would be invalidated
- [ ] Labels: majority vote per bucket (as reconstruct.cpp does); normals:
      existing `downsample(CloudN, ...)` overload
- [ ] Record cloud lineage (source cloud + operation) in ProjectDB so
      staleness is detectable by `rux validate`
- [ ] Integration test: downsample -> instances/materials still consistent

## Acceptance
No CLI operation can silently desynchronize parallel clouds.

category=Geometry estimate=2d
