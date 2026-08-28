title: Link BuildingComponents to source instances (provenance)
labels: data-model, phase-1

## Problem
Components created by `rux create windows` etc. get independent GUIDs with no
reference to the instance (point cloud subset) they were derived from
(`geometry/BuildingComponent.hpp`). "Which component came from instance #5"
is unanswerable; CSV export shows components and passports as disconnected
lists.

## Tasks
- [ ] Add optional `source_instance_guid` to BuildingComponent (JSON + schema)
      — depends on stable instance identity (#06)
- [ ] Populate it in `create windows` / component-generating commands
- [ ] Surface the link in CSV export/import round-trip
- [ ] `rux validate` checks it resolves

## Acceptance
Component -> instance -> material passport is traversable in both directions.

category=Geometry estimate=1d
