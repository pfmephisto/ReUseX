title: Case identity metadata (BFE, case number, MRK, deadline) and a BBR line
labels: gui, enhancement, backend, frontend

## Problem
The prototype's Overblik hero carries a case number (`RX-2026-0047`), an
MRK (miljøregistrering/-koordinator) name, a demolition deadline, and a BBR
("Bygnings- og Boligregistret") line sourced from the building's BFE number.
None of that exists in `ProjectInfo`/`ProjectDB`, so Phase 5 (R5, R6) drops
all four and does not draw the BBR line at all — `docs/design/
gui-kortlaegning-redesign.md` marks it "shown only when project metadata
carries a BFE number", a condition that can currently never hold.

## Proposed fix
- [ ] Add `bfe_number`, `case_number`, `mrk`, `demolition_deadline` (date)
      to the project metadata schema (bump `LATEST_SCHEMA_VERSION`), the
      `PATCH /projects/{id}` body, and `rux set`/`rux get` paths.
- [ ] Extend `ProjectMetaForm`'s field list and `META_FIELDS` to edit them,
      following the existing sparse-PATCH-on-blur convention.
- [ ] Add them to the hero subline (`heroSubline`) once stored.
- [ ] Build the BBR line: given a BFE number, a verified link format into
      BBR's public lookup (confirm the URL scheme before shipping — the
      prototype marks the row "Demodata", i.e. not a real link). Decide
      whether to fetch any BBR data server-side or just link out.

category=I/O estimate=1d
