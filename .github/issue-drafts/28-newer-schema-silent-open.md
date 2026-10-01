title: Older rux builds open newer-schema .rux projects silently
labels: backend, bug, data-model

## Problem
`ProjectDB::Impl::configure()` (`libs/reusex/src/core/ProjectDB.cpp`) warns
when a project's on-disk schema is *older* than the build's
`LATEST_SCHEMA_VERSION` (read-only open: "Opened read-only, so no migration
was applied and newer fields are unavailable"; read-write open migrates).
There is no symmetric check for the opposite case: a project written by a
*newer* build than the one now opening it (`onDisk > LATEST_SCHEMA_VERSION`
/ `current > LATEST_SCHEMA_VERSION`). Both checks in `ProjectDB.cpp` only
test `>= 0 && < LATEST_SCHEMA_VERSION`; nothing fires when the stored
version exceeds what this binary understands.

Concretely: open a v23 project (this phase added `report_pdfs
.blocking_types`) with a `rux` built before that migration landed, and the
older build neither warns nor refuses — it just reads the project as if the
newer columns did not exist, silently dropping data a later read-write
session could then overwrite with stale assumptions.

## Proposed fix
- [ ] On open, compare `onDisk`/`current` against `LATEST_SCHEMA_VERSION`
      in both directions.
- [ ] Read-only: warn the same way as the "older schema" case, naming both
      versions, so an operator knows to use a newer build.
- [ ] Read-write: refuse to open (throw), since writing through an older
      schema's lens risks silently discarding or corrupting newer columns
      it doesn't know about — this is the destructive direction.
- [ ] Unit test: a database row with `schema_version` ahead of
      `LATEST_SCHEMA_VERSION` (easy to construct by writing the row
      directly in a test fixture) triggers the warn-on-RO /
      throw-on-RW behaviour.

category=I/O estimate=4h
