title: Stable instance identity — regeneration orphans material links
labels: bug, data-model, phase-1

## Problem
`segment_instances` assigns sequential IDs starting at 1 on every run
(`libs/reusex/src/geometry/segment_instances.cpp:117,160`). Re-running
`rux create instances` silently invalidates every row in `instance_materials`
— all material passport links break with no warning. This is the root cause
of the fragile passport <-> point cloud association.

## Proposed design
- Give each instance a stable GUID at creation, stored in a new
  `instances(guid, cloud_id, instance_id, semantic_class, point_count)` table
- On regeneration, match new instances to old ones (semantic class + spatial
  overlap of point sets) and carry GUIDs over; unmatched old GUIDs are
  reported, not silently dropped
- `instance_materials` links `material_guid <-> instance_guid` instead of the
  raw label value

## Acceptance
Re-running `rux create instances` on an unchanged cloud preserves all
material links; on a changed cloud it reports exactly which links could not
be migrated.

category=Geometry estimate=1w
