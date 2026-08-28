title: Referential integrity + validation for instance_materials
labels: bug, data-model, phase-1

## Problem
`instance_materials` (`ProjectDB.cpp:708-713`) has no FK on `material_guid`
and `instance_id` is validated against nothing — `set_instance_material()`
silently succeeds for non-existent instances and non-existent passports.
Deleting a cloud CASCADEs the links away while passports remain, with no way
to detect orphans.

## Tasks
- [ ] `set_instance_material()` validates the instance exists in the target
      cloud's label definitions and the passport GUID exists; throws otherwise
- [ ] Add FK `material_guid REFERENCES material_passports(document_guid)`
      (schema v10 migration)
- [ ] New `rux validate` subcommand: checks orphaned passports, dangling
      links, size-mismatched parallel clouds; reports a fixable summary
- [ ] Unit tests for each failure mode

## Acceptance
It is impossible to create a dangling link via the API, and `rux validate`
finds pre-existing ones.

category=I/O estimate=2d
