title: Store the client (bygherre) in case metadata
labels: gui, enhancement, backend, frontend

## Problem
The prototype's Sager card reads `2760 Måløv · Bygherre: KBH Ejendomme A/S`.
`ProjectInfo` has no client field, so Phase 6 (R2) shows the address and
`udarbejdet af <organisation>` (the surveying firm, which is not the client).
Draft 24 covers the other missing case identity fields (BFE, case number,
MRK, deadline); the client was not in it.

## Proposed fix
- [ ] Add `client` to the project metadata schema, `PATCH /projects/{id}`
      and `rux set`/`rux get`, together with draft 24's fields (one schema
      bump for all of them).
- [ ] Add it to `ProjectMetaForm`, Overblik's hero line and the Sager card's
      sub line (`cardSubline`), as `Bygherre: <name>`.

category=I/O estimate=4h
